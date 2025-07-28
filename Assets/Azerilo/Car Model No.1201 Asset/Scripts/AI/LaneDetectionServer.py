import socket
import cv2
import numpy as np
import struct
import threading
import time
import os
from collections import defaultdict
import queue

class LaneDetector:
    def __init__(self):
        """차선 감지 클래스 초기화"""
        
        # Hough Transform 파라미터 (더 관대하게)
        self.hough_threshold = 20  # 50 → 20 (더 민감)
        self.min_line_length = 20  # 50 → 20 (더 짧은 선도 감지)
        self.max_line_gap = 20     # 10 → 20 (더 큰 간격 허용)
        
        # 디버깅용 카운터
        self.frame_count = 0
        self.save_debug_images = True  # 처음 몇 장 저장
        
    def preprocess_image(self, image):
        """이미지 전처리"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        return edges
    
    def get_roi(self, image):
        """관심 영역(ROI) 설정"""
        h, w = image.shape
        
        vertices = np.array([
            [0, h],                           # 좌하단
            [0, int(h * 0.6)],               # 좌중단
            [int(w * 0.35), int(h * 0.4)],   # 좌상단
            [int(w * 0.7), int(h * 0.4)],    # 우상단
            [w, int(h * 0.6)],               # 우중단
            [w, h]                           # 우하단
        ], dtype=np.int32)
        
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, [vertices], 255)
        
        return cv2.bitwise_and(image, mask)
    
    def detect_lane_lines(self, image):
        """차선 검출 (기존 방식)"""
        processed = self.preprocess_image(image)
        roi_image = self.get_roi(processed)
        
        lines = cv2.HoughLinesP(
            roi_image,
            rho=1,
            theta=np.pi/180,
            threshold=self.hough_threshold,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap
        )
        
        return lines
    
    def detect_yellow_lines(self, image):
        """노간 차선 감지 (HSV 기반)"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 노간색 범위 정의 (HSV) - 더 넓은 범위
        lower_yellow = np.array([10, 50, 50])   # 더 넓은 범위
        upper_yellow = np.array([40, 255, 255])  # 더 넓은 범위
        
        # 노간색 마스크 생성
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # 노이즈 제거
        kernel = np.ones((3, 3), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
        
        # 엣지 검출
        yellow_edges = cv2.Canny(yellow_mask, 50, 150)
        yellow_roi = self.get_roi(yellow_edges)
        
        # HoughLinesP로 노간 차선 검출
        yellow_lines = cv2.HoughLinesP(
            yellow_roi, 1, np.pi/180, 
            threshold=self.hough_threshold,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap
        )
        
        return yellow_lines
    
    def detect_white_lines(self, image):
        """흰 차선 감지 (밝기 기반)"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 흰색 범위 (밝은 영역) - 더 넓은 범위
        white_mask = cv2.inRange(gray, 150, 255)  # 150-255로 넓힘
        
        # 노이즈 제거
        kernel = np.ones((3, 3), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
        
        # 엣지 검출
        white_edges = cv2.Canny(white_mask, 50, 150)
        white_roi = self.get_roi(white_edges)
        
        # HoughLinesP로 흰 차선 검출
        white_lines = cv2.HoughLinesP(
            white_roi, 1, np.pi/180,
            threshold=self.hough_threshold,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap
        )
        
        return white_lines
    
    def separate_left_right_lines(self, lines, image_width):
        """좌측/우측 차선 분리"""
        left_lines = []
        right_lines = []
        
        if lines is None:
            return left_lines, right_lines
        
        center_x = image_width // 2
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            if x2 - x1 == 0:
                continue
                
            slope = (y2 - y1) / (x2 - x1)
            
            # 차선 위치와 기울기로 분류 (더 관대하게)
            line_center_x = (x1 + x2) // 2
            
            if line_center_x < center_x and slope < -0.3:  # 왼쪽 + 음의 기울기
                left_lines.append(line[0])
            elif line_center_x > center_x and slope > 0.3:  # 오른쪽 + 양의 기울기
                right_lines.append(line[0])
        
        return left_lines, right_lines
    
    def classify_line_color(self, image, line):
        """차선의 색상 분류 (노간/흰색)"""
        x1, y1, x2, y2 = line
        
        # 차선 위의 여러 점에서 색상 샘플링
        num_samples = 5
        yellow_votes = 0
        white_votes = 0
        
        for i in range(num_samples):
            t = i / (num_samples - 1)
            x = int(x1 + t * (x2 - x1))
            y = int(y1 + t * (y2 - y1))
            
            # 이미지 경계 체크
            if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
                # HSV로 변환해서 노간색 체크
                pixel_bgr = image[y, x]
                pixel_hsv = cv2.cvtColor(np.uint8([[pixel_bgr]]), cv2.COLOR_BGR2HSV)[0][0]
                
                # 노간색 범위 체크
                if 10 <= pixel_hsv[0] <= 40 and pixel_hsv[1] >= 50 and pixel_hsv[2] >= 50:
                    yellow_votes += 1
                # 흰색 범위 체크 (밝기 기반)
                elif pixel_bgr[0] >= 150 and pixel_bgr[1] >= 150 and pixel_bgr[2] >= 150:
                    white_votes += 1
        
        # 투표 결과로 색상 결정
        if yellow_votes > white_votes:
            return "yellow"
        elif white_votes > 0:
            return "white"
        else:
            return "unknown"
    
    def get_lane_center_offset_with_colors(self, image, lines):
        """색상을 고려한 차선 중앙 오프셋 계산 (노간색 선 오른쪽)"""
        if lines is None:
            return None, 0.0, False, "unknown"
        
        height, width = image.shape[:2]
        vehicle_center = width // 2
        bottom_y = height - 50
        
        # 모든 차선을 색상별로 분류
        yellow_lines = []
        white_lines = []
        
        for line in lines:
            color = self.classify_line_color(image, line[0])
            if color == "yellow":
                yellow_lines.append(line[0])
            elif color == "white":
                white_lines.append(line[0])
        
        print(f"[Lane] Classified lines - Yellow: {len(yellow_lines)}, White: {len(white_lines)}")
        
        # 노간 차선 위치 계산
        yellow_x = None
        if len(yellow_lines) > 0:
            yellow_x = self.extrapolate_line(yellow_lines, bottom_y)
        
        # 흰 차선들 분리
        white_left_lines, white_right_lines = self.separate_left_right_lines(
            [[line] for line in white_lines], width
        )
        
        left_white_x = None
        right_white_x = None
        
        if len(white_left_lines) > 0:
            left_white_x = self.extrapolate_line(white_left_lines, bottom_y)
        if len(white_right_lines) > 0:
            right_white_x = self.extrapolate_line(white_right_lines, bottom_y)
        
        print(f"[Lane] Positions - Yellow: {yellow_x}, Left White: {left_white_x}, Right White: {right_white_x}")
        
        # 방향 판단 및 중앙선 침범 체크 (노간색 선 오른쪽 기준)
        vehicle_direction = "unknown"
        centerline_violation = False
        
        if yellow_x is not None:
            if yellow_x > vehicle_center:
                # 노간 차선이 오른쪽 → 정방향 주행
                vehicle_direction = "forward"
                # 정방향에서는 노간 차선(오른쪽)을 넘어서면 침범
                if vehicle_center > yellow_x - 80:  # 80픽셀 여유 (관대하게)
                    centerline_violation = True
                    print(f"[Lane] FORWARD: Crossed yellow centerline! Vehicle: {vehicle_center}, Yellow: {yellow_x}")
            else:
                # 노간 차선이 왼쪽 → 역방향 주행  
                vehicle_direction = "backward"
                # 역방향에서는 노간 차선(왼쪽)을 넘어서면 침범
                if vehicle_center < yellow_x + 80:  # 80픽셀 여유 (관대하게)
                    centerline_violation = True
                    print(f"[Lane] BACKWARD: Crossed yellow centerline! Vehicle: {vehicle_center}, Yellow: {yellow_x}")
        
        # 차선 중앙 계산
        lane_center = None
        confidence = 0.0
        
        if vehicle_direction == "forward":
            # 정방향: 왼쪽 흰 차선 + 노간(오른쪽)
            if left_white_x is not None and yellow_x is not None:
                lane_center = (left_white_x + yellow_x) // 2
                confidence = 0.9
            elif yellow_x is not None:
                lane_center = yellow_x - 120  # 추정 (노간색에서 왼쪽으로 120픽셀)
                confidence = 0.6
            elif left_white_x is not None:
                lane_center = left_white_x + 120  # 추정 (왼쪽 흰색에서 오른쪽으로 120픽셀)
                confidence = 0.5
        elif vehicle_direction == "backward":
            # 역방향: 노간(왼쪽) + 오른쪽 흰 차선
            if yellow_x is not None and right_white_x is not None:
                lane_center = (yellow_x + right_white_x) // 2
                confidence = 0.9
            elif yellow_x is not None:
                lane_center = yellow_x + 120  # 추정 (노간색에서 오른쪽으로 120픽셀)
                confidence = 0.6
            elif right_white_x is not None:
                lane_center = right_white_x - 120  # 추정 (오른쪽 흰색에서 왼쪽으로 120픽셀)
                confidence = 0.5
        else:
            # 방향 모름: 기존 방식 사용
            return self.get_lane_center_offset(image, lines)
        
        if lane_center is None:
            # 기존 방식 폴백
            return self.get_lane_center_offset(image, lines)
        
        # 오프셋 계산
        offset = (vehicle_center - lane_center) / (width / 2)
        
        print(f"[Lane] Direction: {vehicle_direction}, Offset: {offset:.3f}, Violation: {centerline_violation}")
        
        return offset, confidence, centerline_violation, vehicle_direction
    
    def get_lane_center_offset(self, image, lines):
        """기존 차선 중앙 오프셋 계산 (폴백용)"""
        height, width = image.shape[:2]
        
        left_lines, right_lines = self.separate_left_right_lines(lines, width)
        
        if len(left_lines) == 0 and len(right_lines) == 0:
            return None, 0.0, False, "unknown"
        
        bottom_y = height - 50
        
        left_x = None
        right_x = None
        
        if len(left_lines) > 0:
            left_x = self.extrapolate_line(left_lines, bottom_y)
        
        if len(right_lines) > 0:
            right_x = self.extrapolate_line(right_lines, bottom_y)
        
        vehicle_center = width // 2
        
        if left_x is not None and right_x is not None:
            lane_center = (left_x + right_x) // 2
            confidence = 1.0
        elif left_x is not None:
            lane_center = left_x + 150
            confidence = 0.7
        elif right_x is not None:
            lane_center = right_x - 150
            confidence = 0.7
        else:
            return None, 0.0, False, "unknown"
        
        offset = (vehicle_center - lane_center) / (width / 2)
        
        return offset, confidence, False, "unknown"  # 중앙선 침범 없음, 방향 모름
    
    def extrapolate_line(self, lines, y):
        """여러 선분을 하나의 직선으로 외삽"""
        if len(lines) == 0:
            return None
        
        points = []
        for line in lines:
            x1, y1, x2, y2 = line
            points.extend([(x1, y1), (x2, y2)])
        
        if len(points) < 2:
            return None
        
        x_coords = [p[0] for p in points]
        y_coords = [p[1] for p in points]
        
        try:
            coeffs = np.polyfit(y_coords, x_coords, 1)
            x = int(coeffs[0] * y + coeffs[1])
            return x
        except:
            return None

class UDPLaneDetectionServer:
    def __init__(self, video_port=8888, response_port=8889, save_frames=True):
        self.video_port = video_port
        self.response_port = response_port
        self.lane_detector = LaneDetector()
        self.save_frames = save_frames  # 프레임 저장 여부
        
        # 저장 디렉토리 생성
        if self.save_frames:
            self.save_dir = f"captured_frames_{int(time.time())}"
            os.makedirs(self.save_dir, exist_ok=True)
            print(f"[Server] Frames will be saved to: {self.save_dir}/")
        
        # UDP 소켓 초기화
        self.video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.video_socket.bind(('0.0.0.0', video_port))
        
        self.response_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # 프래그먼트 재조립용
        self.fragment_buffer = defaultdict(dict)
        self.frame_queue = queue.Queue(maxsize=10)
        
        self.running = False
        
        print(f"[Server] UDP Lane Detection Server with Enhanced Color Recognition initialized")
        print(f"[Server] Video port: {video_port}, Response port: {response_port}")
        print(f"[Server] Save frames: {'ON' if save_frames else 'OFF'}")
        print(f"[Server] Configuration: Yellow line on RIGHT, centerline crossing detection enabled")
    
    def start(self):
        """서버 시작"""
        self.running = True
        
        # 수신 스레드 시작
        receive_thread = threading.Thread(target=self.receive_frames)
        receive_thread.daemon = True
        receive_thread.start()
        
        # 처리 스레드 시작
        process_thread = threading.Thread(target=self.process_frames)
        process_thread.daemon = True
        process_thread.start()
        
        print("[Server] Server started. Waiting for frames...")
        
        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            print("[Server] Shutting down...")
            self.stop()
    
    def receive_frames(self):
        """프레임 수신 스레드"""
        print(f"[Server] Listening on {self.video_socket.getsockname()}")
        self.video_socket.settimeout(1.0)
        
        while self.running:
            try:
                data, client_addr = self.video_socket.recvfrom(65536)
                print(f"[Server] *** RECEIVED PACKET *** {len(data)} bytes from {client_addr}")
                
                # 테스트 패킷 체크
                try:
                    test_msg = data.decode('utf-8')
                    if test_msg.startswith("TEST_PACKET"):
                        print(f"[Server] Test packet received: '{test_msg}'")
                        continue
                except:
                    pass
                
                if len(data) < 24:
                    print(f"[Server] Packet too small: {len(data)} bytes (minimum 24)")
                    print(f"[Server] Raw data: {data[:50]}")
                    continue
                
                # 헤더 파싱 (Unity Pack=1 구조체와 일치)
                try:
                    header = struct.unpack('<IIIIIf', data[:24])  # Little-endian
                    seq_num, timestamp, frame_size, width, height, speed = header
                    print(f"[Server] Parsed header: seq={seq_num}, frame_size={frame_size}, {width}x{height}")
                except struct.error as e:
                    print(f"[Server] Header parse error: {e}")
                    print(f"[Server] Raw data (first 50 bytes): {data[:50].hex()}")
                    continue
                
                # 프래그먼트 체크
                if len(data) > 32 and len(data) < frame_size + 24:
                    print(f"[Server] Processing fragment for sequence {seq_num}")
                    self.handle_fragment(data, client_addr, seq_num)
                else:
                    frame_data = data[24:]
                    print(f"[Server] Processing complete frame: {len(frame_data)} bytes")
                    self.process_complete_frame(frame_data, client_addr, header)
                    
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[Server] Receive error: {e}")
                    print(f"[Server] Error type: {type(e).__name__}")
                    
        print("[Server] Receive thread stopped")
    
    def handle_fragment(self, data, client_addr, seq_num):
        """분할된 패킷 재조립"""
        if len(data) < 32:
            return
        
        # 분할 정보 파싱 (Little-endian)
        fragment_index = struct.unpack('<I', data[24:28])[0]
        total_fragments = struct.unpack('<I', data[28:32])[0]
        fragment_data = data[32:]
        
        print(f"[Server] Fragment {fragment_index+1}/{total_fragments} for seq {seq_num}: {len(fragment_data)} bytes")
        
        # 프래그먼트 저장
        self.fragment_buffer[seq_num][fragment_index] = fragment_data
        
        # 모든 프래그먼트가 도착했는지 확인
        if len(self.fragment_buffer[seq_num]) == total_fragments:
            print(f"[Server] All fragments received for seq {seq_num}, reassembling...")
            
            # 프레임 재조립
            complete_frame = b''
            for i in range(total_fragments):
                if i in self.fragment_buffer[seq_num]:
                    complete_frame += self.fragment_buffer[seq_num][i]
            
            # 헤더 정보 (첫 번째 프래그먼트에서)
            header_data = data[:24]
            header = struct.unpack('<IIIIIf', header_data)
            
            print(f"[Server] Reassembled frame: {len(complete_frame)} bytes")
            self.process_complete_frame(complete_frame, client_addr, header)
            
            # 버퍼 정리
            del self.fragment_buffer[seq_num]
    
    def process_complete_frame(self, frame_data, client_addr, header):
        """완전한 프레임 처리"""
        try:
            # 큐가 가득 찬 경우 오래된 프레임 제거
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    pass
            
            self.frame_queue.put((frame_data, client_addr, header))
            print(f"[Server] Frame queued for processing: {len(frame_data)} bytes")
            
        except Exception as e:
            print(f"[Server] Frame processing error: {e}")
    
    def process_frames(self):
        """프레임 처리 스레드"""
        while self.running:
            try:
                frame_data, client_addr, header = self.frame_queue.get(timeout=1.0)
                
                print(f"[Server] Processing frame {header[0]}: {len(frame_data)} bytes")
                
                # 이미지 포맷 감지 및 디코딩
                nparr = np.frombuffer(frame_data, np.uint8)
                
                # PNG/JPEG 자동 감지
                if frame_data[:8] == b'\x89PNG\r\n\x1a\n':
                    print(f"[Server] PNG format detected")
                    image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                elif frame_data[:2] == b'\xff\xd8':
                    print(f"[Server] JPEG format detected")
                    image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                else:
                    print(f"[Server] Unknown format, trying generic decode...")
                    image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                
                if image is None:
                    print("[Server] Failed to decode image")
                    continue
                
                print(f"[Server] Decoded image: {image.shape}, dtype: {image.dtype}")
                
                # 프레임 저장 (선택적)
                frame_num = header[0]
                if self.save_frames:
                    if frame_num < 20:  # 처음 20프레임 저장
                        save_path = f"{self.save_dir}/frame_{frame_num:04d}.jpg"
                        cv2.imwrite(save_path, image)
                        print(f"[Server] Saved: {save_path}")
                
                # 차선 감지 수행
                start_time = time.time()
                
                # 기존 방식으로 모든 차선 감지
                lines = self.lane_detector.detect_lane_lines(image)
                
                # 색상을 고려한 오프셋 계산
                result = self.lane_detector.get_lane_center_offset_with_colors(image, lines)
                
                if len(result) == 4:
                    offset, confidence, centerline_violation, direction = result
                else:
                    # 폴백: 기존 방식
                    offset, confidence = result
                    centerline_violation = False
                    direction = "unknown"
                
                process_time = (time.time() - start_time) * 1000
                
                # 디버그 이미지 저장 (처음 10프레임)
                if self.save_frames and frame_num < 10:
                    debug_image = image.copy()
                    
                    # 모든 차선 그리기
                    if lines is not None:
                        for line in lines:
                            x1, y1, x2, y2 = line[0]
                            # 색상 분류
                            color_type = self.lane_detector.classify_line_color(image, line[0])
                            if color_type == "yellow":
                                cv2.line(debug_image, (x1, y1), (x2, y2), (0, 255, 255), 3)  # 노간색
                            elif color_type == "white":
                                cv2.line(debug_image, (x1, y1), (x2, y2), (255, 255, 255), 2)  # 흰색
                            else:
                                cv2.line(debug_image, (x1, y1), (x2, y2), (128, 128, 128), 1)  # 회색 (알 수 없음)
                    
                    # 차량 중앙선 표시 (빨간색)
                    height, width = image.shape[:2]
                    cv2.line(debug_image, (width//2, 0), (width//2, height), (0, 0, 255), 2)
                    
                    # 중앙선 침범 표시
                    if centerline_violation:
                        cv2.putText(debug_image, "CENTERLINE VIOLATION!", (10, 30), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    
                    # 방향 표시
                    cv2.putText(debug_image, f"Direction: {direction}", (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    
                    debug_path = f"{self.save_dir}/debug_enhanced_{frame_num:04d}.jpg"
                    cv2.imwrite(debug_path, debug_image)
                    print(f"[Server] Saved enhanced debug: {debug_path}")
                
                # 결과 전송
                success = offset is not None
                if not success:
                    offset = 0.0
                    confidence = 0.0
                    centerline_violation = False
                
                self.send_response(
                    client_addr, 
                    success, 
                    offset, 
                    confidence, 
                    header[0],  # sequence number
                    header[1],  # timestamp
                    centerline_violation
                )
                
                violation_msg = " 🚨 VIOLATION!" if centerline_violation else ""
                print(f"[Server] Frame {header[0]}: Direction={direction}, Offset={offset:.3f}, "
                      f"Confidence={confidence:.2f}, Process={process_time:.1f}ms{violation_msg}")
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[Server] Processing error: {e}")
                import traceback
                traceback.print_exc()
    
    def send_response(self, client_addr, success, offset, confidence, seq_num, timestamp, centerline_violation=False):
        """응답 전송 (중앙선 정보 포함)"""
        try:
            # Unity 구조체와 일치: byte, float, float, byte, uint, uint
            success_byte = 1 if success else 0
            violation_byte = 1 if centerline_violation else 0
            
            # 타임스탬프를 uint32 범위로 제한 (상대적 시간 사용)
            current_time_ms = int(time.time() * 1000)
            timestamp_uint32 = current_time_ms & 0xFFFFFFFF  # uint32로 마스킹
            
            response_data = struct.pack(
                '<BffBII',  # Little-endian: byte, float, float, byte, uint, uint
                success_byte,
                offset,
                confidence,
                violation_byte,  # 중앙선 침범 정보
                timestamp_uint32,
                seq_num
            )
            
            # 클라이언트의 응답 포트로 전송
            response_addr = (client_addr[0], self.response_port)
            self.response_socket.sendto(response_data, response_addr)
            
            if centerline_violation:
                print(f"[Server] ⚠️ CENTERLINE VIOLATION sent to {response_addr}")
            else:
                print(f"[Server] Response sent to {response_addr}: success={success}, offset={offset:.3f}")
            
        except Exception as e:
            print(f"[Server] Response send error: {e}")
            print(f"[Server] Timestamp: {current_time_ms if 'current_time_ms' in locals() else 'unknown'}")
            print(f"[Server] Seq num: {seq_num}")
            import traceback
            traceback.print_exc()
    
    def stop(self):
        """서버 중지"""
        self.running = False
        self.video_socket.close()
        self.response_socket.close()
        print("[Server] Server stopped")

def main():
    """메인 함수"""
    print("=== UDP Lane Detection Server with Enhanced Color Recognition ===")
    print("- Uses existing lane detection method")
    print("- Adds color classification (yellow/white)")
    print("- Automatic direction detection")
    print("- Centerline violation detection")
    print("- Configuration: YELLOW line on RIGHT side")
    print("Press Ctrl+C to stop the server")
    
    # 서버 시작 (프레임 저장 활성화)
    server = UDPLaneDetectionServer(
        video_port=8888, 
        response_port=8889, 
        save_frames=True  # 이미지 저장 활성화
    )
    server.start()

if __name__ == "__main__":
    main()
