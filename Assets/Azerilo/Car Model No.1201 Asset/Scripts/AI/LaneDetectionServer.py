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
        """차선 검출"""
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
    
    def separate_left_right_lines(self, lines, image_width):
        """좌측/우측 차선 분리"""
        left_lines = []
        right_lines = []
        
        if lines is None:
            return left_lines, right_lines
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            if x2 - x1 == 0:
                continue
                
            slope = (y2 - y1) / (x2 - x1)
            
            if slope < -0.5:
                left_lines.append(line[0])
            elif slope > 0.5:
                right_lines.append(line[0])
        
        return left_lines, right_lines
    
    def get_lane_center_offset(self, image, lines):
        """차선 중앙에서의 오프셋 계산"""
        height, width = image.shape[:2]
        
        left_lines, right_lines = self.separate_left_right_lines(lines, width)
        
        if len(left_lines) == 0 and len(right_lines) == 0:
            return None, 0.0
        
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
            return None, 0.0
        
        offset = (vehicle_center - lane_center) / (width / 2)
        
        return offset, confidence
    
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
        
        print(f"[Server] UDP Lane Detection Server initialized")
        print(f"[Server] Video port: {video_port}, Response port: {response_port}")
        print(f"[Server] Save frames: {'ON' if save_frames else 'OFF'}")
    
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
                    if frame_num < 50:  # 처음 50프레임 저장 (너무 많이 저장 방지)
                        save_path = f"{self.save_dir}/frame_{frame_num:04d}.jpg"
                        cv2.imwrite(save_path, image)
                        print(f"[Server] Saved: {save_path}")
                    elif frame_num % 30 == 0:  # 이후 30프레임마다 샘플링
                        save_path = f"{self.save_dir}/frame_{frame_num:04d}.jpg"
                        cv2.imwrite(save_path, image)
                        print(f"[Server] Sample saved: {save_path}")
                
                # 차선 감지 수행 (전처리 과정 저장)
                start_time = time.time()
                
                # 전처리 단계별 이미지 저장 (처음 5프레임)
                if self.save_frames and frame_num < 5:
                    # 1. 원본
                    cv2.imwrite(f"{self.save_dir}/step1_original_{frame_num:04d}.jpg", image)
                    
                    # 2. 선명화
                    kernel = np.array([[-1,-1,-1], [-1, 9,-1], [-1,-1,-1]])
                    sharpened = cv2.filter2D(image, -1, kernel)
                    cv2.imwrite(f"{self.save_dir}/step2_sharpened_{frame_num:04d}.jpg", sharpened)
                    
                    # 3. 그레이스케일
                    gray = cv2.cvtColor(sharpened, cv2.COLOR_BGR2GRAY)
                    cv2.imwrite(f"{self.save_dir}/step3_gray_{frame_num:04d}.jpg", gray)
                    
                    # 4. 대비 향상
                    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
                    enhanced = clahe.apply(gray)
                    cv2.imwrite(f"{self.save_dir}/step4_enhanced_{frame_num:04d}.jpg", enhanced)
                    
                    # 5. 엣지 검출
                    blur = cv2.GaussianBlur(enhanced, (3, 3), 0)
                    edges = cv2.Canny(blur, 30, 100)
                    cv2.imwrite(f"{self.save_dir}/step5_edges_{frame_num:04d}.jpg", edges)
                    
                    # 6. ROI 적용
                    roi_edges = self.lane_detector.get_roi(edges)
                    cv2.imwrite(f"{self.save_dir}/step6_roi_{frame_num:04d}.jpg", roi_edges)
                    
                    print(f"[Server] 🔍 Saved preprocessing steps for frame {frame_num}")
                
                lines = self.lane_detector.detect_lane_lines(image)
                offset, confidence = self.lane_detector.get_lane_center_offset(image, lines)
                process_time = (time.time() - start_time) * 1000
                
                # 차선 감지 디버깅 정보
                if lines is not None:
                    print(f"[Server] Detected {len(lines)} lines")
                    
                    # 차선이 그려진 이미지도 저장 (처음 10프레임)
                    if self.save_frames and frame_num < 10:
                        debug_image = image.copy()
                        
                        # 원본 차선 그리기 (초록색)
                        for line in lines:
                            x1, y1, x2, y2 = line[0]
                            cv2.line(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        
                        # ROI 영역 표시 (파란색)
                        height, width = image.shape[:2]
                        roi_points = np.array([
                            [int(width * 0.1), height],
                            [int(width * 0.4), int(height * self.lane_detector.roi_top_ratio)],
                            [int(width * 0.6), int(height * self.lane_detector.roi_top_ratio)],
                            [int(width * 0.9), height]
                        ], dtype=np.int32)
                        cv2.polylines(debug_image, [roi_points], True, (255, 0, 0), 2)
                        
                        # 차량 중앙선 표시 (빨간색)
                        cv2.line(debug_image, (width//2, 0), (width//2, height), (0, 0, 255), 2)
                        
                        debug_path = f"{self.save_dir}/debug_lines_{frame_num:04d}.jpg"
                        cv2.imwrite(debug_path, debug_image)
                        print(f"[Server] Saved lines debug: {debug_path}")
                else:
                    print(f"[Server] No lines detected")
                
                # 결과 전송
                success = offset is not None
                if not success:
                    offset = 0.0
                    confidence = 0.0
                
                self.send_response(
                    client_addr, 
                    success, 
                    offset, 
                    confidence, 
                    header[0],  # sequence number
                    header[1]   # timestamp
                )
                
                print(f"[Server] Frame {header[0]}: Offset={offset:.3f}, "
                      f"Confidence={confidence:.2f}, Process={process_time:.1f}ms")
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[Server] Processing error: {e}")
                import traceback
                traceback.print_exc()
    
    def send_response(self, client_addr, success, offset, confidence, seq_num, timestamp):
        """응답 전송 (Unity 구조체와 호환)"""
        try:
            # Unity 구조체와 일치: byte, float, float, uint, uint
            success_byte = 1 if success else 0
            
            # 타임스탬프를 uint32 범위로 제한 (상대적 시간 사용)
            current_time_ms = int(time.time() * 1000)
            timestamp_uint32 = current_time_ms & 0xFFFFFFFF  # uint32로 마스킹
            
            response_data = struct.pack(
                '<BffII',  # Little-endian: byte, float, float, uint, uint
                success_byte,
                offset,
                confidence,
                timestamp_uint32,  # 안전한 범위로 제한된 타임스탬프
                seq_num
            )
            
            # 클라이언트의 응답 포트로 전송
            response_addr = (client_addr[0], self.response_port)
            self.response_socket.sendto(response_data, response_addr)
            
            print(f"[Server] Response sent to {response_addr}: success={success}, offset={offset:.3f}, confidence={confidence:.2f}")
            
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
    print("=== UDP Lane Detection Server ===")
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
