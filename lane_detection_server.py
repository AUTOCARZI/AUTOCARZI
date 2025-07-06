import socket
import cv2
import numpy as np
import struct
import threading
import time
import os
import sys
from datetime import datetime

class LaneDetectionServer:
    def __init__(self, host='127.0.0.1', port=65432):
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Terminal output settings (macOS/Linux compatible)
        try:
            sys.stdout.reconfigure(encoding='utf-8', errors='replace')
        except AttributeError:
            # Python 3.7 and below don't have reconfigure
            pass
        
        # Image saving settings
        self.save_images = True
        self.save_all_frames = False  # Save all frames
        self.save_interval = 30  # Regular save interval
        self.save_detection_changes = True  # Save when detection status changes
        self.save_no_detection = True  # Save when no lanes detected
        self.show_preview = False  # Disable OpenCV window preview
        self.frame_count = 0
        self.last_detection_status = None  # Previous detection status
        self.output_dir = "lane_detection_results"
        
        # Statistics variables
        self.total_saved = 0
        self.detected_count = 0
        self.not_detected_count = 0
        
        # Create output folder
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            full_path = os.path.abspath(self.output_dir)
            print(f"Created results folder: {full_path}")
        else:
            full_path = os.path.abspath(self.output_dir)
            print(f"Using existing folder: {full_path}")
        
    def log_message(self, message):
        """Log message that outputs immediately to terminal"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        print(f"[{timestamp}] {message}")
        sys.stdout.flush()  # Force immediate output
        
    def start_server(self):
        self.socket.bind((self.host, self.port))
        self.socket.listen(5)
        self.log_message(f"Lane detection server started: {self.host}:{self.port}")
        self.log_message(f"Image saving enabled: {self.save_images}")
        
        # Output absolute path of save folder
        full_output_path = os.path.abspath(self.output_dir)
        print(f"Save folder: {full_output_path}")
        
        while True:
            try:
                client_socket, address = self.socket.accept()
                print(f"Unity client connected: {address}")
                
                client_thread = threading.Thread(
                    target=self.handle_client, 
                    args=(client_socket,)
                )
                client_thread.daemon = True
                client_thread.start()
                
            except Exception as e:
                self.log_message(f"Server error: {e}")
                break
    
    def handle_client(self, client_socket):
        try:
            while True:
                # Receive image size
                size_data = self.recv_all(client_socket, 4)
                if not size_data:
                    break
                
                image_size = struct.unpack('I', size_data)[0]
                
                # Receive image data
                image_data = self.recv_all(client_socket, image_size)
                if not image_data:
                    break
                
                # Decode image
                nparr = np.frombuffer(image_data, np.uint8)
                image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                
                if image is not None:
                    # Lane detection and visualization
                    processed_image, lane_deviation, detected, confidence = self.detect_and_visualize_lanes(image)
                    
                    # Update statistics
                    if detected:
                        self.detected_count += 1
                    else:
                        self.not_detected_count += 1
                    
                    # Output statistics every 10 frames
                    if self.frame_count % 10 == 0:
                        self.log_message(f"Frame {self.frame_count}: detected={detected}, deviation={lane_deviation:.3f}, confidence={confidence:.3f}")
                    
                    # Check image save conditions
                    should_save = self.should_save_image(detected, confidence)
                    if self.save_images and should_save:
                        self.save_detection_result(image, processed_image, lane_deviation, detected, confidence)
                    
                    self.frame_count += 1
                    
                    # Send results to Unity
                    result = struct.pack('fff', lane_deviation, 1.0 if detected else 0.0, confidence)
                    client_socket.send(result)
                    
                    # Real-time preview (optional)
                    if self.show_preview:
                        try:
                            cv2.imshow('Lane Detection Live', processed_image)
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                break
                        except Exception as cv_error:
                            # Ignore OpenCV window errors and continue
                            self.log_message(f"OpenCV window error (ignored): {cv_error}")
                            pass
                
        except Exception as e:
            self.log_message(f"Client handling error: {e}")
        finally:
            try:
                client_socket.close()
            except:
                pass
            cv2.destroyAllWindows()
            self.log_message("Client connection closed")
    
    def recv_all(self, socket, size):
        """Receive data completely up to specified size"""
        data = b''
        while len(data) < size:
            packet = socket.recv(size - len(data))
            if not packet:
                return None
            data += packet
        return data
    
    def detect_and_visualize_lanes(self, image):
        """Main lane detection with gray-white lane support"""
        try:
            result_image = image.copy()
            
            # Try gray-white lane detection first (for Unity roads)
            gray_white_result = self.detect_gray_white_lanes(image)
            
            # Try color-based detection
            color_result = self.detect_color_lanes(image)
            
            # Try basic edge detection as fallback
            basic_result = self.detect_basic_lanes(image)
            
            # Choose best result based on confidence
            results = [gray_white_result, color_result, basic_result]
            best_result = max(results, key=lambda x: x['confidence'])
            
            if best_result['detected']:
                return best_result['image'], best_result['deviation'], True, best_result['confidence']
            else:
                self.add_info_text(result_image, 0.0, False, 0.0)
                return result_image, 0.0, False, 0.0
                
        except Exception as e:
            self.log_message(f"Lane detection error: {e}")
            self.add_info_text(image, 0.0, False, 0.0)
            return image, 0.0, False, 0.0
    
    def detect_gray_white_lanes(self, image):
        """Detect gray-white lanes optimized for Unity roads - Full 7-strategy approach"""
        try:
            height, width = image.shape[:2]
            
            # Convert to different color spaces
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # STRATEGY 1: Lower threshold white detection for gray-white
            # Relaxed RGB thresholds for gray-white
            rgb_gray_white1 = cv2.inRange(image, np.array([120, 120, 120]), np.array([255, 255, 255]))
            rgb_gray_white2 = cv2.inRange(image, np.array([140, 140, 140]), np.array([220, 220, 220]))
            rgb_gray_white3 = cv2.inRange(image, np.array([100, 100, 100]), np.array([180, 180, 180]))
            
            # STRATEGY 2: HSV for low-saturation detection
            # Low saturation, medium to high value (brightness)
            hsv_gray_white1 = cv2.inRange(hsv, np.array([0, 0, 100]), np.array([180, 60, 255]))
            hsv_gray_white2 = cv2.inRange(hsv, np.array([0, 0, 120]), np.array([180, 40, 200]))
            hsv_gray_white3 = cv2.inRange(hsv, np.array([0, 0, 80]), np.array([180, 80, 255]))
            
            # STRATEGY 3: LAB L-channel for lightness
            l_channel = lab[:,:,0]
            lab_gray_white1 = cv2.inRange(l_channel, 120, 255)
            lab_gray_white2 = cv2.inRange(l_channel, 140, 200)
            lab_gray_white3 = cv2.inRange(l_channel, 100, 180)
            
            # STRATEGY 4: Grayscale thresholds
            gray_mask1 = cv2.inRange(gray, 120, 255)
            gray_mask2 = cv2.inRange(gray, 140, 200)
            gray_mask3 = cv2.inRange(gray, 100, 180)
            
            # STRATEGY 5: Adaptive threshold for varying lighting
            adaptive_thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                                  cv2.THRESH_BINARY, 15, -5)
            
            # STRATEGY 6: Edge-enhanced detection
            # Enhance edges first, then apply threshold
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, 30, 100)
            
            # Dilate edges to create thicker lane candidates
            kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            edges_dilated = cv2.dilate(edges, kernel_dilate, iterations=1)
            
            # Combine edges with intensity detection
            edge_intensity_combined = cv2.bitwise_and(gray_mask1, edges_dilated)
            
            # STRATEGY 7: Relative brightness detection
            # Find pixels that are brighter than their local neighborhood
            kernel_avg = np.ones((15, 15), np.float32) / 225
            local_avg = cv2.filter2D(gray.astype(np.float32), -1, kernel_avg)
            relative_bright = (gray.astype(np.float32) - local_avg) > 20
            relative_bright_mask = (relative_bright * 255).astype(np.uint8)
            
            # Combine all strategies
            # Primary combinations
            combined1 = cv2.bitwise_or(rgb_gray_white1, hsv_gray_white1)
            combined1 = cv2.bitwise_or(combined1, lab_gray_white1)
            combined1 = cv2.bitwise_or(combined1, gray_mask1)
            
            # Secondary combinations  
            combined2 = cv2.bitwise_or(rgb_gray_white2, hsv_gray_white2)
            combined2 = cv2.bitwise_or(combined2, lab_gray_white2)
            combined2 = cv2.bitwise_or(combined2, gray_mask2)
            
            # Tertiary combinations
            combined3 = cv2.bitwise_or(rgb_gray_white3, hsv_gray_white3)
            combined3 = cv2.bitwise_or(combined3, lab_gray_white3)
            combined3 = cv2.bitwise_or(combined3, gray_mask3)
            
            # Final combination
            final_combined = cv2.bitwise_or(combined1, combined2)
            final_combined = cv2.bitwise_or(final_combined, combined3)
            final_combined = cv2.bitwise_or(final_combined, edge_intensity_combined)
            final_combined = cv2.bitwise_or(final_combined, relative_bright_mask)
            final_combined = cv2.bitwise_or(final_combined, adaptive_thresh)
            
            # Apply smart ROI
            roi_mask = self.create_smart_roi(height, width)
            final_roi = cv2.bitwise_and(final_combined, roi_mask)
            
            # Morphological operations for lane-like shapes
            # Close gaps in dashed lines
            kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (20, 5))
            closed = cv2.morphologyEx(final_roi, cv2.MORPH_CLOSE, kernel_close)
            
            # Remove small noise
            kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            final_mask = cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel_open)
            
            # Detect lines using Hough transform with relaxed parameters (same as working code)
            lines = cv2.HoughLinesP(final_mask, 1, np.pi/180, threshold=8, minLineLength=15, maxLineGap=40)
            
            if lines is not None and len(lines) > 0:
                result_image = self.draw_detected_lanes(image.copy(), lines, "GRAY-WHITE")
                deviation, confidence = self.calculate_lane_deviation(lines, width)
                self.add_info_text(result_image, deviation, True, confidence)
                
                return {
                    'image': result_image,
                    'deviation': deviation,
                    'confidence': confidence,
                    'detected': True
                }
            else:
                return {
                    'image': image.copy(),
                    'deviation': 0.0,
                    'confidence': 0.0,
                    'detected': False
                }
                
        except Exception as e:
            self.log_message(f"Gray-white lane detection error: {e}")
            return {
                'image': image.copy(),
                'deviation': 0.0,
                'confidence': 0.0,
                'detected': False
            }
    
    def detect_color_lanes(self, image):
        """Detect white and yellow lanes using color filtering"""
        try:
            height, width = image.shape[:2]
            
            # Convert to HSV
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # White lane mask (expanded range for Unity)
            white_mask1 = cv2.inRange(hsv, np.array([0, 0, 180]), np.array([180, 40, 255]))
            white_mask2 = cv2.inRange(hsv, np.array([0, 0, 220]), np.array([180, 20, 255]))
            rgb_white = cv2.inRange(image, np.array([180, 180, 180]), np.array([255, 255, 255]))
            
            white_mask = cv2.bitwise_or(white_mask1, white_mask2)
            white_mask = cv2.bitwise_or(white_mask, rgb_white)
            
            # Yellow lane mask
            yellow_mask = cv2.inRange(hsv, np.array([15, 80, 80]), np.array([35, 255, 255]))
            
            # Apply ROI to both masks
            roi_mask = self.create_smart_roi(height, width)
            white_roi = cv2.bitwise_and(white_mask, roi_mask)
            yellow_roi = cv2.bitwise_and(yellow_mask, roi_mask)
            
            # Detect lines for each color
            white_lines = cv2.HoughLinesP(white_roi, 1, np.pi/180, threshold=15, minLineLength=20, maxLineGap=50)
            yellow_lines = cv2.HoughLinesP(yellow_roi, 1, np.pi/180, threshold=20, minLineLength=25, maxLineGap=40)
            
            # Combine all detected lines
            all_lines = []
            line_colors = []
            
            if white_lines is not None:
                for line in white_lines:
                    all_lines.append(line)
                    line_colors.append("WHITE")
            
            if yellow_lines is not None:
                for line in yellow_lines:
                    all_lines.append(line)
                    line_colors.append("YELLOW")
            
            if len(all_lines) > 0:
                result_image = self.draw_color_coded_lanes(image.copy(), all_lines, line_colors)
                deviation, confidence = self.calculate_lane_deviation(all_lines, width)
                self.add_info_text(result_image, deviation, True, confidence)
                
                return {
                    'image': result_image,
                    'deviation': deviation,
                    'confidence': confidence,
                    'detected': True
                }
            else:
                return {
                    'image': image.copy(),
                    'deviation': 0.0,
                    'confidence': 0.0,
                    'detected': False
                }
                
        except Exception as e:
            self.log_message(f"Color lane detection error: {e}")
            return {
                'image': image.copy(),
                'deviation': 0.0,
                'confidence': 0.0,
                'detected': False
            }
    
    def detect_basic_lanes(self, image):
        """Basic edge-based lane detection as fallback"""
        try:
            height, width = image.shape[:2]
            
            # Convert to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Enhance contrast
            enhanced = cv2.equalizeHist(gray)
            
            # Gaussian blur
            blurred = cv2.GaussianBlur(enhanced, (5, 5), 0)
            
            # Canny edge detection
            edges = cv2.Canny(blurred, 50, 150)
            
            # Apply ROI
            roi_mask = self.create_smart_roi(height, width)
            masked_edges = cv2.bitwise_and(edges, roi_mask)
            
            # Hough transform
            lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, threshold=30, minLineLength=25, maxLineGap=80)
            
            if lines is not None and len(lines) > 0:
                result_image = self.draw_detected_lanes(image.copy(), lines, "BASIC")
                deviation, confidence = self.calculate_lane_deviation(lines, width)
                self.add_info_text(result_image, deviation, True, confidence)
                
                return {
                    'image': result_image,
                    'deviation': deviation,
                    'confidence': confidence * 0.7,  # Lower confidence for basic detection
                    'detected': True
                }
            else:
                return {
                    'image': image.copy(),
                    'deviation': 0.0,
                    'confidence': 0.0,
                    'detected': False
                }
                
        except Exception as e:
            self.log_message(f"Basic lane detection error: {e}")
            return {
                'image': image.copy(),
                'deviation': 0.0,
                'confidence': 0.0,
                'detected': False
            }
    
    def create_smart_roi(self, height, width):
        """Create smart Region of Interest for lane detection"""
        roi_mask = np.zeros((height, width), dtype=np.uint8)
        
        # Trapezoidal ROI that covers typical lane area
        vertices = np.array([
            [(0, height),
             (width//6, height//2),
             (5*width//6, height//2),
             (width, height)]
        ], np.int32)
        
        cv2.fillPoly(roi_mask, vertices, 255)
        return roi_mask
    
    def draw_detected_lanes(self, image, lines, detection_type):
        """Draw detected lanes with Unity-style visualization"""
        left_lines = []
        right_lines = []
        
        # Classify lines as left or right
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 - x1 == 0:
                continue
            slope = (y2 - y1) / (x2 - x1)
            length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            
            # Filter out very short or horizontal lines
            if length < 20 or abs(slope) < 0.2:
                continue
            
            center_x = (x1 + x2) / 2
            
            if slope < -0.2 and center_x < image.shape[1] * 0.6:  # Left lane
                left_lines.append((line[0], length, slope))
            elif slope > 0.2 and center_x > image.shape[1] * 0.4:  # Right lane
                right_lines.append((line[0], length, slope))
        
        # Draw left lanes (green)
        if left_lines:
            best_left = max(left_lines, key=lambda x: x[1])  # Longest line
            x1, y1, x2, y2 = best_left[0]
            cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 6)
            
            # Add label
            mid_x, mid_y = (x1 + x2) // 2, (y1 + y2) // 2
            cv2.rectangle(image, (mid_x-40, mid_y-15), (mid_x+40, mid_y+5), (0, 0, 0), -1)
            cv2.putText(image, f"LEFT {detection_type}", (mid_x-35, mid_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        # Draw right lanes (red)
        if right_lines:
            best_right = max(right_lines, key=lambda x: x[1])  # Longest line
            x1, y1, x2, y2 = best_right[0]
            cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 6)
            
            # Add label
            mid_x, mid_y = (x1 + x2) // 2, (y1 + y2) // 2
            cv2.rectangle(image, (mid_x-40, mid_y-15), (mid_x+40, mid_y+5), (0, 0, 0), -1)
            cv2.putText(image, f"RIGHT {detection_type}", (mid_x-35, mid_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        
        # Draw center line if both lanes detected
        if left_lines and right_lines:
            left_x = self.get_line_x_at_bottom(best_left[0], image.shape[0])
            right_x = self.get_line_x_at_bottom(best_right[0], image.shape[0])
            
            if left_x is not None and right_x is not None:
                center_x = (left_x + right_x) // 2
                image_center = image.shape[1] // 2
                
                # Draw calculated center (cyan)
                cv2.line(image, (center_x, image.shape[0]), 
                        (center_x, image.shape[0] - 150), (255, 255, 0), 4)
                
                # Draw ideal center (magenta)
                cv2.line(image, (image_center, image.shape[0]), 
                        (image_center, image.shape[0] - 150), (255, 0, 255), 2)
        
        return image
    
    def draw_color_coded_lanes(self, image, lines, colors):
        """Draw lanes with color coding (white/yellow)"""
        for i, line in enumerate(lines):
            x1, y1, x2, y2 = line[0]
            color_name = colors[i]
            
            # Set drawing color
            if color_name == "WHITE":
                draw_color = (255, 255, 255)
                label_color = (0, 0, 0)  # Black text on white
            else:  # YELLOW
                draw_color = (0, 255, 255)
                label_color = (0, 0, 0)  # Black text on yellow
            
            # Draw line
            cv2.line(image, (x1, y1), (x2, y2), draw_color, 6)
            
            # Add label
            mid_x, mid_y = (x1 + x2) // 2, (y1 + y2) // 2
            
            # Determine side
            slope = (y2 - y1) / (x2 - x1) if x2 != x1 else 0
            side = "LEFT" if slope < 0 else "RIGHT"
            
            label = f"{side} {color_name}"
            
            # Background rectangle
            (text_width, text_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
            cv2.rectangle(image, 
                         (mid_x - text_width//2 - 2, mid_y - text_height - 2),
                         (mid_x + text_width//2 + 2, mid_y + 2),
                         (128, 128, 128), -1)
            
            # Text
            cv2.putText(image, label, (mid_x - text_width//2, mid_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, label_color, 1)
        
        return image
    
    def calculate_lane_deviation(self, lines, image_width):
        """Calculate lane center deviation and confidence"""
        if not lines:
            return 0.0, 0.0
        
        left_lines = []
        right_lines = []
        
        # Classify lines
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 - x1 == 0:
                continue
            slope = (y2 - y1) / (x2 - x1)
            length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            
            center_x = (x1 + x2) / 2
            
            if slope < -0.2 and center_x < image_width * 0.6:
                left_lines.append((line[0], length))
            elif slope > 0.2 and center_x > image_width * 0.4:
                right_lines.append((line[0], length))
        
        image_center = image_width // 2
        lane_center = image_center
        confidence = 0.0
        
        if left_lines and right_lines:
            # Both lanes detected - highest confidence
            best_left = max(left_lines, key=lambda x: x[1])
            best_right = max(right_lines, key=lambda x: x[1])
            
            left_x = self.get_line_x_at_bottom(best_left[0], 480)  # Assume height 480
            right_x = self.get_line_x_at_bottom(best_right[0], 480)
            
            if left_x is not None and right_x is not None:
                lane_center = (left_x + right_x) // 2
                confidence = 1.0
        elif left_lines:
            # Only left lane - medium confidence
            best_left = max(left_lines, key=lambda x: x[1])
            left_x = self.get_line_x_at_bottom(best_left[0], 480)
            if left_x is not None:
                lane_center = left_x + 100  # Estimate center
                confidence = 0.6
        elif right_lines:
            # Only right lane - medium confidence
            best_right = max(right_lines, key=lambda x: x[1])
            right_x = self.get_line_x_at_bottom(best_right[0], 480)
            if right_x is not None:
                lane_center = right_x - 100  # Estimate center
                confidence = 0.6
        
        # Calculate deviation (-1 to 1)
        deviation = (lane_center - image_center) / (image_width // 2)
        deviation = np.clip(deviation, -1.0, 1.0)
        
        return float(deviation), float(confidence)
    
    def get_line_x_at_bottom(self, coords, image_height):
        """Calculate X coordinate where line intersects bottom of image"""
        x1, y1, x2, y2 = coords
        if y2 != y1:
            x_bottom = x1 + (x2 - x1) * (image_height - y1) / (y2 - y1)
            return int(x_bottom)
        return None
    
    def add_info_text(self, image, deviation, detected, confidence):
        """Add information text to image"""
        # Background rectangle
        cv2.rectangle(image, (10, 10), (450, 140), (0, 0, 0), -1)
        
        # Text information
        status = "DETECTED" if detected else "NOT DETECTED"
        color = (0, 255, 0) if detected else (0, 0, 255)
        
        cv2.putText(image, f"Lane Status: {status}", (20, 35), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(image, f"Deviation: {deviation:.3f}", (20, 65), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(image, f"Confidence: {confidence:.3f}", (20, 95), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(image, f"Frame: {self.frame_count}", (20, 125), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
    
    def should_save_image(self, detected, confidence):
        """Determine whether to save image"""
        
        # Save all frames if enabled
        if self.save_all_frames:
            return True
        
        # Save at regular intervals
        if self.frame_count % self.save_interval == 0:
            return True
        
        # Save when detection status changes
        if self.save_detection_changes:
            current_status = "detected" if detected else "not_detected"
            if self.last_detection_status != current_status:
                self.last_detection_status = current_status
                self.log_message(f"Detection status change: {current_status} -> immediate save!")
                return True
        
        # Save when no lanes detected
        if self.save_no_detection and not detected:
            # Save more frequently when no lanes (every 5 frames)
            if self.frame_count % 5 == 0:
                return True
        
        # Save when confidence is low (for problem analysis)
        if detected and confidence < 0.5:
            if self.frame_count % 5 == 0:
                return True
        
        return False
    
    def save_detection_result(self, original, processed, deviation, detected, confidence):
        """Save detection result images"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            
            # Place original and processed images side by side
            # Handle different image sizes safely
            if original.shape != processed.shape:
                # Match sizes
                h, w = min(original.shape[0], processed.shape[0]), min(original.shape[1], processed.shape[1])
                original_resized = original[:h, :w]
                processed_resized = processed[:h, :w]
                combined = np.hstack((original_resized, processed_resized))
            else:
                combined = np.hstack((original, processed))
            
            # Update statistics
            self.total_saved += 1
            
            # Generate filename (including frame number)
            status = "detected" if detected else "not_detected"
            filename = f"{timestamp}_frame{self.frame_count:06d}_{status}_dev{deviation:.3f}_conf{confidence:.3f}.jpg"
            filepath = os.path.join(self.output_dir, filename)
            
            # Save image
            cv2.imwrite(filepath, combined)
            
            # Output save reason
            save_reason = self.get_save_reason(detected, confidence)
            self.log_message(f"Saved #{self.total_saved}: {filename} ({save_reason})")
            
            # Output overall statistics (every 50 saves)
            if self.total_saved % 50 == 0:
                total_frames = self.detected_count + self.not_detected_count
                detection_rate = (self.detected_count / total_frames * 100) if total_frames > 0 else 0
                self.log_message(f"=== Statistics ===")
                self.log_message(f"Total frames: {total_frames}, Detection rate: {detection_rate:.1f}%")
                self.log_message(f"Saved images: {self.total_saved}")
        except Exception as e:
            self.log_message(f"Image save error: {e}")
    
    def get_save_reason(self, detected, confidence):
        """Return reason for saving"""
        if not detected:
            return "No lanes"
        elif confidence < 0.5:
            return "Low confidence"
    def get_save_reason(self, detected, confidence):
        """Return reason for saving"""
        if not detected:
            return "No lanes"
        elif confidence < 0.5:
            return "Low confidence"
        elif self.frame_count % self.save_interval == 0:
            return "Regular save"
        else:
            return "Status change"
    
    def stop_server(self):
        self.socket.close()
        cv2.destroyAllWindows()

def main():
    print("=" * 60)
    print("🚗 Gray-White Lane Detection Server for Unity 🚗")
    print("=" * 60)
    print("Features:")
    print("• Gray-white lane detection optimized for Unity roads")
    print("• Multi-strategy detection (RGB, HSV, LAB, relative brightness)")
    print("• Color-coded lane visualization (white/yellow)")
    print("• Smart ROI with left/right lane separation")
    print("• Real-time Unity communication")
    print("• Automatic image saving with statistics")
    print("=" * 60)
    
    # Start lane detection server
    server = LaneDetectionServer()
    
    server.log_message("Server initialization complete")
    server.log_message("Detection methods: Gray-White → Color → Basic Edge")
    server.log_message("Terminate: Ctrl+C or 'q' key in OpenCV window")
    print("=" * 60)
    
    try:
        server.start_server()
    except KeyboardInterrupt:
        server.log_message("User terminated server")
        server.stop_server()
    except Exception as e:
        server.log_message(f"Server error occurred: {e}")
        server.stop_server()

if __name__ == "__main__":
    main()
