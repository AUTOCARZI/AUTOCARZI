import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def detect_gray_white_lanes(image):
    """Detect gray-white lanes like in Unity road images"""
    height, width = image.shape[:2]
    
    print("Analyzing image color characteristics...")
    
    # Convert to different color spaces
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # STRATEGY 1: Lower threshold white detection for gray-white
    print("Strategy 1: Lower threshold white detection...")
    
    # Relaxed RGB thresholds for gray-white
    rgb_gray_white1 = cv2.inRange(image, np.array([120, 120, 120]), np.array([255, 255, 255]))
    rgb_gray_white2 = cv2.inRange(image, np.array([140, 140, 140]), np.array([220, 220, 220]))
    rgb_gray_white3 = cv2.inRange(image, np.array([100, 100, 100]), np.array([180, 180, 180]))
    
    # STRATEGY 2: HSV for low-saturation detection
    print("Strategy 2: HSV low-saturation detection...")
    
    # Low saturation, medium to high value (brightness)
    hsv_gray_white1 = cv2.inRange(hsv, np.array([0, 0, 100]), np.array([180, 60, 255]))
    hsv_gray_white2 = cv2.inRange(hsv, np.array([0, 0, 120]), np.array([180, 40, 200]))
    hsv_gray_white3 = cv2.inRange(hsv, np.array([0, 0, 80]), np.array([180, 80, 255]))
    
    # STRATEGY 3: LAB L-channel for lightness
    print("Strategy 3: LAB lightness detection...")
    
    l_channel = lab[:,:,0]
    lab_gray_white1 = cv2.inRange(l_channel, 120, 255)
    lab_gray_white2 = cv2.inRange(l_channel, 140, 200)
    lab_gray_white3 = cv2.inRange(l_channel, 100, 180)
    
    # STRATEGY 4: Grayscale thresholds
    print("Strategy 4: Grayscale intensity detection...")
    
    gray_mask1 = cv2.inRange(gray, 120, 255)
    gray_mask2 = cv2.inRange(gray, 140, 200)
    gray_mask3 = cv2.inRange(gray, 100, 180)
    
    # STRATEGY 5: Adaptive threshold for varying lighting
    print("Strategy 5: Adaptive threshold...")
    
    # Adaptive threshold can handle varying lighting conditions
    adaptive_thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                          cv2.THRESH_BINARY, 15, -5)
    
    # STRATEGY 6: Edge-enhanced detection
    print("Strategy 6: Edge-enhanced detection...")
    
    # Enhance edges first, then apply threshold
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 30, 100)
    
    # Dilate edges to create thicker lane candidates
    kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    edges_dilated = cv2.dilate(edges, kernel_dilate, iterations=1)
    
    # Combine edges with intensity detection
    edge_intensity_combined = cv2.bitwise_and(gray_mask1, edges_dilated)
    
    # STRATEGY 7: Relative brightness detection
    print("Strategy 7: Relative brightness detection...")
    
    # Find pixels that are brighter than their local neighborhood
    kernel_avg = np.ones((15, 15), np.float32) / 225
    local_avg = cv2.filter2D(gray.astype(np.float32), -1, kernel_avg)
    relative_bright = (gray.astype(np.float32) - local_avg) > 20
    relative_bright_mask = (relative_bright * 255).astype(np.uint8)
    
    # Combine all strategies
    print("Combining all detection strategies...")
    
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
    
    # Apply smart ROI
    print("Applying ROI...")
    roi_mask = np.zeros_like(gray)
    vertices = np.array([[(0, height), (width//6, height//2), (5*width//6, height//2), (width, height)]], np.int32)
    cv2.fillPoly(roi_mask, vertices, 255)
    final_roi = cv2.bitwise_and(final_combined, roi_mask)
    
    # Morphological operations for lane-like shapes
    print("Applying morphological operations...")
    
    # Close gaps in dashed lines
    kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (20, 5))
    closed = cv2.morphologyEx(final_roi, cv2.MORPH_CLOSE, kernel_close)
    
    # Remove small noise
    kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    final_result = cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel_open)
    
    return (final_result, combined1, combined2, combined3, edge_intensity_combined, 
            relative_bright_mask, adaptive_thresh, gray_mask1, hsv_gray_white1, rgb_gray_white1)

def visualize_gray_white_detection(image, results):
    """Visualize all detection strategies for gray-white lanes"""
    print("Creating comprehensive visualization...")
    
    (final_result, combined1, combined2, combined3, edge_intensity_combined, 
     relative_bright_mask, adaptive_thresh, gray_mask1, hsv_gray_white1, rgb_gray_white1) = results
    
    plt.figure(figsize=(20, 16))
    
    # Row 1: Original and color analysis
    plt.subplot(4, 5, 1)
    plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    plt.title('1. Original Image\n(Gray-White Lanes)')
    plt.axis('off')
    
    plt.subplot(4, 5, 2)
    plt.imshow(rgb_gray_white1, cmap='gray')
    plt.title('2. RGB Gray-White\n[120-255]')
    plt.axis('off')
    
    plt.subplot(4, 5, 3)
    plt.imshow(hsv_gray_white1, cmap='gray')
    plt.title('3. HSV Low-Saturation\n[0,0,100]-[180,60,255]')
    plt.axis('off')
    
    plt.subplot(4, 5, 4)
    plt.imshow(gray_mask1, cmap='gray')
    plt.title('4. Grayscale Threshold\n[120-255]')
    plt.axis('off')
    
    plt.subplot(4, 5, 5)
    plt.imshow(adaptive_thresh, cmap='gray')
    plt.title('5. Adaptive Threshold\n(lighting adaptive)')
    plt.axis('off')
    
    # Row 2: Advanced strategies
    plt.subplot(4, 5, 6)
    plt.imshow(edge_intensity_combined, cmap='gray')
    plt.title('6. Edge + Intensity\n(structural detection)')
    plt.axis('off')
    
    plt.subplot(4, 5, 7)
    plt.imshow(relative_bright_mask, cmap='gray')
    plt.title('7. Relative Brightness\n(local contrast)')
    plt.axis('off')
    
    plt.subplot(4, 5, 8)
    plt.imshow(combined1, cmap='gray')
    plt.title('8. Combined Strategy 1\n(primary thresholds)')
    plt.axis('off')
    
    plt.subplot(4, 5, 9)
    plt.imshow(combined2, cmap='gray')
    plt.title('9. Combined Strategy 2\n(medium thresholds)')
    plt.axis('off')
    
    plt.subplot(4, 5, 10)
    plt.imshow(combined3, cmap='gray')
    plt.title('10. Combined Strategy 3\n(relaxed thresholds)')
    plt.axis('off')
    
    # Row 3: Final processing steps
    plt.subplot(4, 5, 11)
    plt.imshow(final_result, cmap='gray')
    plt.title('11. Final Result\n(all strategies combined)')
    plt.axis('off')
    
    # Color analysis
    plt.subplot(4, 5, 12)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
    plt.plot(hist)
    plt.title('12. Intensity Histogram\n(shows gray-white peaks)')
    plt.xlabel('Pixel Intensity')
    plt.ylabel('Frequency')
    
    # HSV analysis
    plt.subplot(4, 5, 13)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    sat_hist = cv2.calcHist([s], [0], None, [256], [0, 256])
    plt.plot(sat_hist, color='orange')
    plt.title('13. Saturation Histogram\n(low saturation = gray)')
    plt.xlabel('Saturation')
    plt.ylabel('Frequency')
    
    # RGB channel analysis
    plt.subplot(4, 5, 14)
    b, g, r = cv2.split(image)
    plt.plot(cv2.calcHist([r], [0], None, [256], [0, 256]), color='red', alpha=0.7, label='Red')
    plt.plot(cv2.calcHist([g], [0], None, [256], [0, 256]), color='green', alpha=0.7, label='Green')
    plt.plot(cv2.calcHist([b], [0], None, [256], [0, 256]), color='blue', alpha=0.7, label='Blue')
    plt.title('14. RGB Channel Analysis\n(equal RGB = gray)')
    plt.xlabel('Intensity')
    plt.ylabel('Frequency')
    plt.legend()
    
    # Final overlay
    plt.subplot(4, 5, 15)
    overlay = cv2.cvtColor(image, cv2.COLOR_BGR2RGB).copy()
    overlay[final_result > 0] = [255, 255, 0]  # Yellow highlight
    plt.imshow(overlay)
    plt.title('15. Detection Overlay\n(yellow = detected lanes)')
    plt.axis('off')
    
    # Row 4: Statistical analysis and recommendations
    plt.subplot(4, 5, 16)
    plt.axis('off')
    
    # Calculate statistics
    total_pixels = image.shape[0] * image.shape[1]
    detected_pixels = np.sum(final_result > 0)
    coverage = (detected_pixels / total_pixels) * 100
    
    # Analyze image characteristics
    mean_brightness = np.mean(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
    mean_saturation = np.mean(cv2.split(cv2.cvtColor(image, cv2.COLOR_BGR2HSV))[1])
    
    stats_text = f"""Detection Statistics:

Total pixels: {total_pixels:,}
Detected pixels: {detected_pixels:,}
Coverage: {coverage:.2f}%

Image Characteristics:
Mean brightness: {mean_brightness:.1f}/255
Mean saturation: {mean_saturation:.1f}/255

This image has:
• {'Low' if mean_saturation < 50 else 'High'} saturation
• {'Dark' if mean_brightness < 100 else 'Bright' if mean_brightness > 180 else 'Medium'} brightness
• Gray-white lane markings

Best strategies for this image:
1. Low-saturation HSV detection
2. Relative brightness detection  
3. Adaptive thresholding"""
    
    plt.text(0.05, 0.5, stats_text, fontsize=10, verticalalignment='center',
             bbox=dict(boxstyle="round,pad=0.3", facecolor="lightcyan"))
    plt.title('16. Analysis & Statistics')
    
    # Additional overlays
    plt.subplot(4, 5, 17)
    # Strategy comparison
    comparison = cv2.cvtColor(image, cv2.COLOR_BGR2RGB).copy()
    comparison[combined1 > 0] = [255, 0, 0]      # Red: Strategy 1
    comparison[combined2 > 0] = [0, 255, 0]      # Green: Strategy 2  
    comparison[combined3 > 0] = [0, 0, 255]      # Blue: Strategy 3
    comparison[final_result > 0] = [255, 255, 0] # Yellow: Final
    plt.imshow(comparison)
    plt.title('17. Strategy Comparison\n(R=S1, G=S2, B=S3, Y=Final)')
    plt.axis('off')
    
    plt.subplot(4, 5, 18)
    # Show which strategy worked best
    strategy_scores = [
        np.sum(combined1 > 0),
        np.sum(combined2 > 0), 
        np.sum(combined3 > 0),
        np.sum(edge_intensity_combined > 0),
        np.sum(relative_bright_mask > 0)
    ]
    strategy_names = ['Combined 1', 'Combined 2', 'Combined 3', 'Edge+Intensity', 'Relative Bright']
    
    plt.bar(range(len(strategy_scores)), strategy_scores, color=['red', 'green', 'blue', 'orange', 'purple'])
    plt.title('18. Strategy Effectiveness\n(pixels detected)')
    plt.xlabel('Strategy')
    plt.ylabel('Pixels Detected')
    plt.xticks(range(len(strategy_names)), strategy_names, rotation=45, ha='right')
    
    plt.subplot(4, 5, 19)
    # Show final result with lines detected
    edges = cv2.Canny(final_result, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=10, minLineLength=15, maxLineGap=30)
    
    line_result = cv2.cvtColor(image, cv2.COLOR_BGR2RGB).copy()
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_result, (x1, y1), (x2, y2), (0, 255, 255), 3)
    
    plt.imshow(line_result)
    plt.title(f'19. Final Lines\n({len(lines) if lines is not None else 0} lines detected)')
    plt.axis('off')
    
    plt.subplot(4, 5, 20)
    plt.axis('off')
    
    # Recommendations
    recommendations = f"""Recommendations for
Gray-White Lane Detection:

✓ Use multiple color spaces
✓ Lower threshold values
✓ Combine RGB + HSV + LAB
✓ Use adaptive methods
✓ Focus on low saturation
✓ Consider relative brightness

For your Unity road:
• Threshold range: 100-180
• HSV saturation: < 60
• Use edge enhancement
• Apply morphology for gaps

Best performing strategy:
{strategy_names[np.argmax(strategy_scores)]}
({max(strategy_scores):,} pixels)"""
    
    plt.text(0.05, 0.5, recommendations, fontsize=9, verticalalignment='center',
             bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen"))
    plt.title('20. Recommendations')
    
    plt.tight_layout()
    plt.savefig('gray_white_lane_analysis.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("Gray-white lane analysis saved!")

def detect_gray_white_lanes_main(image_path):
    """Main function for gray-white lane detection"""
    # Load image
    image = cv2.imread(image_path)
    if image is None:
        print(f"Image not found: {image_path}")
        return
    
    print(f"Processing gray-white lane image: {image_path}")
    print(f"Image size: {image.shape}")
    
    # Detect gray-white lanes
    results = detect_gray_white_lanes(image)
    final_result = results[0]
    
    # Visualize the comprehensive process
    visualize_gray_white_detection(image, results)
    
    # Detect final lines with relaxed parameters
    print("Detecting final lines with optimized parameters...")
    edges = cv2.Canny(final_result, 30, 100)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=8, minLineLength=15, maxLineGap=40)
    
    # Draw results
    result_image = image.copy()
    if lines is not None:
        print(f"Detected gray-white line segments: {len(lines)}")
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(result_image, (x1, y1), (x2, y2), (0, 255, 255), 4)
    else:
        print("No gray-white line segments found.")
    
    # Save final result
    plt.figure(figsize=(12, 6))
    plt.subplot(1, 2, 1)
    plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    plt.title('Original Gray-White Road')
    plt.axis('off')
    
    plt.subplot(1, 2, 2)
    plt.imshow(cv2.cvtColor(result_image, cv2.COLOR_BGR2RGB))
    plt.title('Gray-White Lane Detection')
    plt.axis('off')
    
    plt.savefig('gray_white_lanes_final.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("Final gray-white lane result saved!")
    
    return final_result, lines

# Test the function
if __name__ == "__main__":
    image_path = "unity_20250705_170246_150_frame000004_detected_dev0.684.jpg"
    
    print("=" * 60)
    print("Gray-White Lane Detection for Unity Roads")
    print("=" * 60)
    
    mask, lines = detect_gray_white_lanes_main(image_path)
    
    print("\nProcess completed! Check these files:")
    print("- gray_white_lane_analysis.png (comprehensive 20-panel analysis)")
    print("- gray_white_lanes_final.png (final detection result)")
