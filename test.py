import cv2
import numpy as np

def detect_white_lanes_aggressive(image):
    """강력한 흰색 차선 검출"""
    height, width = image.shape[:2]
    
    # 1. 여러 색공간에서 흰색 검출
    # RGB 기반
    rgb_mask = cv2.inRange(image, np.array([170, 170, 170]), np.array([255, 255, 255]))
    
    # HSV 기반
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv_mask1 = cv2.inRange(hsv, np.array([0, 0, 150]), np.array([180, 50, 255]))
    hsv_mask2 = cv2.inRange(hsv, np.array([0, 0, 200]), np.array([180, 30, 255]))
    
    # LAB 색공간 추가 (L 채널 활용)
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    l_channel = lab[:,:,0]
    lab_mask = cv2.inRange(l_channel, 180, 255)
    
    # 모든 마스크 결합
    white_mask = cv2.bitwise_or(rgb_mask, hsv_mask1)
    white_mask = cv2.bitwise_or(white_mask, hsv_mask2)
    white_mask = cv2.bitwise_or(white_mask, lab_mask)
    
    # 2. ROI 적용
    roi_mask = np.zeros_like(white_mask)
    vertices = np.array([[(0, height), (width//4, height//2), (3*width//4, height//2), (width, height)]], np.int32)
    cv2.fillPoly(roi_mask, vertices, 255)
    white_mask_roi = cv2.bitwise_and(white_mask, roi_mask)
    
    # 3. 모폴로지 연산으로 점선 연결
    kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 3))
    white_mask_roi = cv2.morphologyEx(white_mask_roi, cv2.MORPH_CLOSE, kernel_close)
    
    kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    white_mask_roi = cv2.morphologyEx(white_mask_roi, cv2.MORPH_OPEN, kernel_open)
    
    # 4. 컨투어 검출로 연속된 흰색 영역 찾기
    contours, _ = cv2.findContours(white_mask_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    white_lanes = []
    
    for contour in contours:
        if cv2.contourArea(contour) < 100:
            continue
        
        x, y, w, h = cv2.boundingRect(contour)
        
        if h < 30 or w/h > 5:
            continue
        
        if len(contour) >= 2:
            top_point = tuple(contour[contour[:,:,1].argmin()][0])
            bottom_point = tuple(contour[contour[:,:,1].argmax()][0])
            
            if bottom_point[0] != top_point[0]:
                slope = (bottom_point[1] - top_point[1]) / (bottom_point[0] - top_point[0])
                center_x = (top_point[0] + bottom_point[0]) / 2
                
                if slope < -0.2 and center_x < width * 0.6:
                    side = "LEFT"
                elif slope > 0.2 and center_x > width * 0.4:
                    side = "RIGHT"
                else:
                    continue
                
                white_lanes.append({
                    'start': bottom_point,
                    'end': top_point,
                    'side': side,
                    'color': 'WHITE',
                    'type': 'DETECTED',
                    'confidence': min(1.0, cv2.contourArea(contour) / 500)
                })
    
    # 5. 허프 변환 백업
    edges = cv2.Canny(white_mask_roi, 50, 150)
    hough_lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=15, minLineLength=20, maxLineGap=50)
    
    if hough_lines is not None:
        for line in hough_lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1) if x2 != x1 else float('inf')
            center_x = (x1 + x2) / 2
            
            if abs(slope) > 0.3:
                if slope < 0 and center_x < width * 0.6:
                    side = "LEFT"
                elif slope > 0 and center_x > width * 0.4:
                    side = "RIGHT"
                else:
                    continue
                
                length = np.sqrt((x2-x1)**2 + (y2-y1)**2)
                white_lanes.append({
                    'start': (x1, y1),
                    'end': (x2, y2),
                    'side': side,
                    'color': 'WHITE',
                    'type': 'HOUGH',
                    'confidence': min(1.0, length / 100)
                })
    
    return white_lanes

def detect_yellow_lanes(image):
    """노란색 차선 검출"""
    height, width = image.shape[:2]
    
    # HSV 변환
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # 노란색 마스크
    yellow_lower = np.array([15, 80, 80])
    yellow_upper = np.array([35, 255, 255])
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
    
    # ROI 적용
    roi_mask = np.zeros_like(yellow_mask)
    vertices = np.array([[(0, height), (width//4, height//2), (3*width//4, height//2), (width, height)]], np.int32)
    cv2.fillPoly(roi_mask, vertices, 255)
    yellow_mask_roi = cv2.bitwise_and(yellow_mask, roi_mask)
    
    # 허프 변환
    edges = cv2.Canny(yellow_mask_roi, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=30, minLineLength=20, maxLineGap=30)
    
    yellow_lanes = []
    
    if lines is not None:
        # 선분들을 연결해서 긴 선으로 만들기
        left_lines = []
        right_lines = []
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 - x1 == 0:
                continue
            slope = (y2 - y1) / (x2 - x1)
            center_x = (x1 + x2) / 2
            
            if slope < -0.3 and center_x < width * 0.6:
                left_lines.append((x1, y1, x2, y2))
            elif slope > 0.3 and center_x > width * 0.4:
                right_lines.append((x1, y1, x2, y2))
        
        # 좌측 차선 연결
        if left_lines:
            connected_left = connect_line_segments(left_lines, height)
            if connected_left:
                yellow_lanes.append({
                    'start': connected_left[0],
                    'end': connected_left[1],
                    'side': 'LEFT',
                    'color': 'YELLOW',
                    'segment_count': len(left_lines)
                })
        
        # 우측 차선 연결
        if right_lines:
            connected_right = connect_line_segments(right_lines, height)
            if connected_right:
                yellow_lanes.append({
                    'start': connected_right[0],
                    'end': connected_right[1],
                    'side': 'RIGHT',
                    'color': 'YELLOW',
                    'segment_count': len(right_lines)
                })
    
    return yellow_lanes

def connect_line_segments(segments, image_height):
    """여러 선분을 하나의 연속된 선으로 연결"""
    if not segments:
        return None
    
    # 모든 점들을 수집
    points = []
    for x1, y1, x2, y2 in segments:
        points.extend([(x1, y1), (x2, y2)])
    
    if len(points) < 2:
        return None
    
    # 선형 회귀로 최적의 직선 찾기
    x_coords = [p[0] for p in points]
    y_coords = [p[1] for p in points]
    
    x_mean = np.mean(x_coords)
    y_mean = np.mean(y_coords)
    
    numerator = sum((x - x_mean) * (y - y_mean) for x, y in zip(x_coords, y_coords))
    denominator = sum((x - x_mean) ** 2 for x in x_coords)
    
    if denominator == 0:
        return None
    
    slope = numerator / denominator
    intercept = y_mean - slope * x_mean
    
    # 이미지 범위에서 선의 시작점과 끝점 계산
    y_start = image_height
    y_end = max(min(y_coords), image_height // 3)
    
    x_start = int((y_start - intercept) / slope) if slope != 0 else int(x_mean)
    x_end = int((y_end - intercept) / slope) if slope != 0 else int(x_mean)
    
    return [(x_start, y_start), (x_end, y_end)]

def draw_lane_detection_result(image, white_lanes, yellow_lanes):
    """Unity 스타일로 차선 검출 결과 그리기"""
    result = image.copy()
    
    # 흰색 차선 그리기
    for lane in white_lanes:
        start_point = lane['start']
        end_point = lane['end']
        side = lane['side']
        
        # 흰색으로 그리기
        cv2.line(result, start_point, end_point, (255, 255, 255), 6)
        
        # 라벨 표시
        mid_x = (start_point[0] + end_point[0]) // 2
        mid_y = (start_point[1] + end_point[1]) // 2
        
        label = f"{side} WHITE"
        
        # 배경 사각형
        (text_width, text_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(result, 
                     (mid_x - text_width//2 - 2, mid_y - text_height - 2),
                     (mid_x + text_width//2 + 2, mid_y + 2),
                     (128, 128, 128), -1)
        
        # 텍스트 (검은색)
        cv2.putText(result, label, (mid_x - text_width//2, mid_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    
    # 노란색 차선 그리기
    for lane in yellow_lanes:
        start_point = lane['start']
        end_point = lane['end']
        side = lane['side']
        segment_count = lane.get('segment_count', 0)
        
        # 노란색으로 그리기
        cv2.line(result, start_point, end_point, (0, 255, 255), 6)
        
        # 라벨 표시
        mid_x = (start_point[0] + end_point[0]) // 2
        mid_y = (start_point[1] + end_point[1]) // 2
        
        label = f"{side} YELLOW ({segment_count} segs)"
        
        # 배경 사각형
        (text_width, text_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(result, 
                     (mid_x - text_width//2 - 2, mid_y - text_height - 2),
                     (mid_x + text_width//2 + 2, mid_y + 2),
                     (128, 128, 128), -1)
        
        # 텍스트 (흰색)
        cv2.putText(result, label, (mid_x - text_width//2, mid_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # 정보 텍스트 추가 (Unity 스타일)
    detected = len(white_lanes) > 0 or len(yellow_lanes) > 0
    
    # 편차 계산
    deviation = calculate_deviation(white_lanes, yellow_lanes, image.shape[1])
    confidence = 0.7 if detected else 0.0
    
    # 배경 사각형
    cv2.rectangle(result, (10, 10), (400, 120), (0, 0, 0), -1)
    
    # 텍스트 정보
    status = "DETECTED" if detected else "NOT DETECTED"
    color = (0, 255, 0) if detected else (0, 0, 255)
    
    cv2.putText(result, f"Lane Status: {status}", (20, 35), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    cv2.putText(result, f"Deviation: {deviation:.3f}", (20, 65), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(result, f"Confidence: {confidence:.3f}", (20, 95), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    return result

def calculate_deviation(white_lanes, yellow_lanes, image_width):
    """편차 계산"""
    all_lanes = white_lanes + yellow_lanes
    left_lanes = [lane for lane in all_lanes if lane['side'] == 'LEFT']
    right_lanes = [lane for lane in all_lanes if lane['side'] == 'RIGHT']
    
    image_center = image_width // 2
    lane_center = image_center
    
    if left_lanes and right_lanes:
        left_x = left_lanes[0]['start'][0]
        right_x = right_lanes[0]['start'][0]
        lane_center = (left_x + right_x) // 2
    elif left_lanes:
        left_x = left_lanes[0]['start'][0]
        lane_center = left_x + 100
    elif right_lanes:
        right_x = right_lanes[0]['start'][0]
        lane_center = right_x - 100
    
    deviation = (lane_center - image_center) / (image_width // 2)
    return np.clip(deviation, -1.0, 1.0)

def process_lane_detection(image_path):
    """메인 차선 검출 함수"""
    # 이미지 로드
    image = cv2.imread(image_path)
    if image is None:
        print(f"이미지를 찾을 수 없습니다: {image_path}")
        return None
    
    print(f"이미지 로드 완료: {image.shape}")
    
    # 흰색 차선 검출
    white_lanes = detect_white_lanes_aggressive(image)
    print(f"흰색 차선 검출: {len(white_lanes)}개")
    
    # 노란색 차선 검출
    yellow_lanes = detect_yellow_lanes(image)
    print(f"노란색 차선 검출: {len(yellow_lanes)}개")
    
    # 결과 이미지 생성
    result_image = draw_lane_detection_result(image, white_lanes, yellow_lanes)
    
    # 결과 저장
    cv2.imwrite('lane_detection_result.jpg', result_image)
    print("결과 이미지 저장됨: lane_detection_result.jpg")
    
    # 결과 표시
    cv2.imshow('Lane Detection Result', result_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    return result_image

# 메인 실행
if __name__ == "__main__":
    # 이미지 파일 경로 (Unity 스크린샷)
    image_path = "unity_20250705_170246_150_frame000004_detected_dev0.684"  # 실제 파일명으로 변경하세요
    
    print("Unity 스타일 차선 검출 시작...")
    result = process_lane_detection(image_path)
    
    if result is not None:
        print("검출 완료!")
    else:
        print("검출 실패!")
