# cv_tracker.py
import cv2
import numpy as np
from datasender import datasender

CAM_INDEX = 0

# Color threshold for robot detection (adjust based on your robot color)
LOWER_HSV = np.array([90,  80,  80]) 
UPPER_HSV = np.array([140, 255, 255])

# White paper detection thresholds
WHITE_LOWER = 200  # Grayscale threshold for white detection
MIN_PAPER_AREA = 5000  # Minimum area for paper detection

# Robot’s IP + port
ROBOT_IP   = "10.120.64.104"
ROBOT_PORT = 5555


def order_points(pts):
    """
    Order 4 points in order: top-left, top-right, bottom-right, bottom-left
    """
    rect = np.zeros((4, 2), dtype="float32")
    
    # Sum and difference to find corners
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)
    
    # Top-left: smallest sum
    rect[0] = pts[np.argmin(s)]
    # Bottom-right: largest sum
    rect[2] = pts[np.argmax(s)]
    # Top-right: smallest difference
    rect[1] = pts[np.argmin(diff)]
    # Bottom-left: largest difference
    rect[3] = pts[np.argmax(diff)]
    
    return rect


def detect_paper_corners(gray):
    """
    Detect the 4 corners of the white paper rectangle.
    Returns: corners (4x2 array) or None if not found
    """
    # Threshold to find white regions
    _, thresh = cv2.threshold(gray, WHITE_LOWER, 255, cv2.THRESH_BINARY)
    
    # Morphological operations to clean up
    kernel = np.ones((5, 5), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    
    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None
    
    # Find largest contour (should be the paper)
    largest_contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest_contour)
    
    if area < MIN_PAPER_AREA:
        return None
    
    # Approximate contour to polygon
    peri = cv2.arcLength(largest_contour, True)
    approx = cv2.approxPolyDP(largest_contour, 0.02 * peri, True)
    
    # If we have 4 corners, use them
    if len(approx) == 4:
        corners = approx.reshape(4, 2).astype(np.float32)
        return order_points(corners)
    
    # If not exactly 4, use bounding rectangle corners
    x, y, w, h = cv2.boundingRect(largest_contour)
    corners = np.array([
        [x, y],           # top-left
        [x + w, y],       # top-right
        [x + w, y + h],   # bottom-right
        [x, y + h]        # bottom-left
    ], dtype=np.float32)
    
    return corners


def detect_robot_corners(hsv, undist):
    """
    Detect the 4 corners of the robot rectangle.
    Returns: corners (4x2 array) or None if not found
    """
    # Create mask for robot color
    mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
    
    # Morphological operations
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None
    
    # Find largest contour (should be the robot)
    largest_contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest_contour)
    
    if area < 100:  # Minimum area threshold
        return None
    
    # Approximate contour to polygon
    peri = cv2.arcLength(largest_contour, True)
    approx = cv2.approxPolyDP(largest_contour, 0.02 * peri, True)
    
    # If we have 4 corners, use them
    if len(approx) >= 4:
        # Take first 4 points if more than 4
        if len(approx) > 4:
            # Use bounding rectangle if too many corners
            x, y, w, h = cv2.boundingRect(largest_contour)
            corners = np.array([
                [x, y],           # top-left
                [x + w, y],       # top-right
                [x + w, y + h],   # bottom-right
                [x, y + h]        # bottom-left
            ], dtype=np.float32)
        else:
            corners = approx.reshape(4, 2).astype(np.float32)
            corners = order_points(corners)
        return corners
    
    # Fallback: use bounding rectangle
    x, y, w, h = cv2.boundingRect(largest_contour)
    corners = np.array([
        [x, y],           # top-left
        [x + w, y],       # top-right
        [x + w, y + h],   # bottom-right
        [x, y + h]        # bottom-left
    ], dtype=np.float32)
    
    return corners


def format_corners(corners):
    """
    Format corners array as string for display/debugging.
    """
    if corners is None:
        return "None"
    return " | ".join([f"({int(c[0])},{int(c[1])})" for c in corners])


def main():
    # Create sender instance
    sender = datasender(ROBOT_IP, ROBOT_PORT)

    # Load calibration
    try:
        mtx  = np.load("camera_mtx.npy")
        dist = np.load("camera_dist.npy")
        H    = np.load("H_board.npy")
        print("Loaded camera calibration and homography")
    except FileNotFoundError as e:
        print(f"Error: {e}")
        print("Please run calibrate_camera.py and define_homography.py first")
        return

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera.")

    print("Press 'q' to quit.\nTracking paper and robot...")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Undistort frame
        undist = cv2.undistort(frame, mtx, dist)
        
        # Convert to grayscale for paper detection
        gray = cv2.cvtColor(undist, cv2.COLOR_BGR2GRAY)
        
        # Convert to HSV for robot detection
        hsv = cv2.cvtColor(undist, cv2.COLOR_BGR2HSV)

        # Detect paper corners
        paper_corners = detect_paper_corners(gray)
        
        # Detect robot corners
        robot_corners = detect_robot_corners(hsv, undist)

        # Draw paper corners
        if paper_corners is not None:
            for i, corner in enumerate(paper_corners):
                pt = tuple(corner.astype(int))
                cv2.circle(undist, pt, 8, (0, 255, 0), -1)  # Green for paper
                cv2.putText(undist, f"P{i+1}", (pt[0] + 10, pt[1] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # Draw paper outline
            pts = paper_corners.astype(int).reshape((-1, 1, 2))
            cv2.polylines(undist, [pts], True, (0, 255, 0), 2)
        
        # Draw robot corners
        if robot_corners is not None:
            for i, corner in enumerate(robot_corners):
                pt = tuple(corner.astype(int))
                cv2.circle(undist, pt, 8, (0, 0, 255), -1)  # Red for robot
                cv2.putText(undist, f"R{i+1}", (pt[0] + 10, pt[1] + 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            # Draw robot outline
            pts = robot_corners.astype(int).reshape((-1, 1, 2))
            cv2.polylines(undist, [pts], True, (0, 0, 255), 2)
            
            # Calculate robot center
            center = robot_corners.mean(axis=0).astype(int)
            cv2.circle(undist, tuple(center), 5, (255, 0, 0), -1)
            
            # Transform center to board coordinates
            pt_img = np.array([center[0], center[1], 1.0])
            pt_board = H @ pt_img
            x_board = pt_board[0] / pt_board[2]
            y_board = pt_board[1] / pt_board[2]
            
            cv2.putText(undist, f"({x_board:.1f}, {y_board:.1f})",
                       (center[0] + 10, center[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Print debug info
        print(f"Paper corners: {format_corners(paper_corners)}")
        print(f"Robot corners: {format_corners(robot_corners)}")

        # Send data (you can modify this format as needed)
        if robot_corners is not None:
            # Format: "robot_x1,y1;x2,y2;x3,y3;x4,y4"
            robot_str = ";".join([f"{int(c[0])},{int(c[1])}" for c in robot_corners])
            message = f"ROBOT:{robot_str}"
        else:
            message = "ROBOT:None"
        
        if paper_corners is not None:
            paper_str = ";".join([f"{int(c[0])},{int(c[1])}" for c in paper_corners])
            message += f"|PAPER:{paper_str}"
        else:
            message += "|PAPER:None"
        
        sender.forward(message)

        # Display windows
        cv2.imshow("Tracking", undist)
        
        # Show thresholded images for debugging
        _, paper_thresh = cv2.threshold(gray, WHITE_LOWER, 255, cv2.THRESH_BINARY)
        robot_mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
        cv2.imshow("Paper Detection", paper_thresh)
        cv2.imshow("Robot Detection", robot_mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    sender.close()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
