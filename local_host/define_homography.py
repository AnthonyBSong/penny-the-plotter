# define_board_homography.py
import cv2
import numpy as np
import glob
import os

CAM_INDEX = 0
CHECKERBOARD = (8, 6)  # (cols, rows) of INNER corners - must match calibrate_camera.py
CALIB_DIR = "calib"

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


def get_checkerboard_outer_corners(corners, pattern_size):
    """
    Extract the 4 outer corners from checkerboard corner detection.
    
    Args:
        corners: Detected checkerboard corners (Nx1x2 array)
        pattern_size: (cols, rows) of inner corners
    
    Returns:
        4x2 array of outer corners: [top-left, top-right, bottom-right, bottom-left]
    """
    cols, rows = pattern_size
    corners_2d = corners.reshape(-1, 2)
    
    # The corners are detected in a specific order by OpenCV
    # For an (8, 6) pattern, we have 8*6 = 48 corners
    # They are arranged in a grid: first row (0-7), second row (8-15), etc.
    
    # Top-left: first corner (index 0)
    top_left = corners_2d[0]
    
    # Top-right: last corner of first row (index cols-1 = 7)
    top_right = corners_2d[cols - 1]
    
    # Bottom-right: last corner overall (index rows*cols-1 = 47)
    bottom_right = corners_2d[rows * cols - 1]
    
    # Bottom-left: first corner of last row (index (rows-1)*cols = 40)
    bottom_left = corners_2d[(rows - 1) * cols]
    
    outer_corners = np.array([top_left, top_right, bottom_right, bottom_left], dtype=np.float32)
    
    return order_points(outer_corners)


def find_checkerboard_in_images():
    """
    Search through calibration images to find one with a checkerboard.
    Returns the image and detected corners, or None if not found.
    """
    image_pattern = os.path.join(CALIB_DIR, "*.jpg")
    images = glob.glob(image_pattern)
    
    if len(images) == 0:
        print(f"No images found in {CALIB_DIR}/")
        return None, None
    
    print(f"Searching through {len(images)} calibration images...")
    
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    for fname in sorted(images):
        img = cv2.imread(fname)
        if img is None:
            continue
            
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
        if ret:
            # Refine corner positions
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            print(f"Found checkerboard in {fname}")
            return img, corners2
    
    print("No checkerboard found in any calibration images.")
    return None, None


def main():
    # Load camera intrinsics
    try:
        mtx = np.load("camera_mtx.npy")
        dist = np.load("camera_dist.npy")
    except FileNotFoundError:
        print("Error: camera_mtx.npy or camera_dist.npy not found.")
        print("Please run calibrate_camera.py first.")
        return

    # Find checkerboard in calibration images
    img, corners = find_checkerboard_in_images()
    
    if img is None or corners is None:
        print("\nCould not find checkerboard in calibration images.")
        print("Please ensure you have calibration images with checkerboards in the calib/ folder.")
        return
    
    # Undistort the image
    undist = cv2.undistort(img, mtx, dist)
    gray = cv2.cvtColor(undist, cv2.COLOR_BGR2GRAY)
    
    # Re-detect corners in undistorted image for accuracy
    ret, corners_undist = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    if not ret:
        print("Warning: Could not re-detect checkerboard in undistorted image.")
        print("Using corners from original image.")
        corners_undist = corners
    else:
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_undist = cv2.cornerSubPix(gray, corners_undist, (11, 11), (-1, -1), criteria)
    
    # Extract outer corners of checkerboard
    image_points = get_checkerboard_outer_corners(corners_undist, CHECKERBOARD)
    
    # Visualize the detected corners
    display_img = undist.copy()
    cv2.drawChessboardCorners(display_img, CHECKERBOARD, corners_undist, True)
    
    # Draw the 4 outer corners
    for i, pt in enumerate(image_points):
        pt_int = tuple(pt.astype(int))
        cv2.circle(display_img, pt_int, 10, (0, 255, 0), -1)
        cv2.putText(display_img, f"{i+1}", (pt_int[0] + 15, pt_int[1] - 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # Draw lines connecting outer corners
    pts = image_points.astype(int).reshape((-1, 1, 2))
    cv2.polylines(display_img, [pts], True, (0, 255, 0), 2)
    
    cv2.imshow("Detected Board Corners (press any key to continue)", display_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    print("\nDetected board corners (from checkerboard outer corners):")
    print("1) Top-left:   ", image_points[0])
    print("2) Top-right:  ", image_points[1])
    print("3) Bottom-right:", image_points[2])
    print("4) Bottom-left: ", image_points[3])
    
    # Get board physical dimensions
    print("\nEnter board dimensions in centimeters.")
    print("(This should be the physical size of the area covered by the checkerboard)")
    
    # Option 1: Enter dimensions directly
    W = float(input("Board width  (x direction, cm): "))
    H = float(input("Board height (y direction, cm): "))
    
    # Define corresponding real-world points in board coordinates
    board_points = np.array([
        [0.0, 0.0],   # top-left
        [W,   0.0],   # top-right
        [W,   H],     # bottom-right
        [0.0, H],     # bottom-left
    ], dtype=np.float32)
    
    # Compute homography from image -> board
    H_mat, mask = cv2.findHomography(image_points, board_points)
    print("\nHomography matrix:\n", H_mat)
    
    # Verify homography by transforming the corners back
    print("\nVerifying homography...")
    for i, img_pt in enumerate(image_points):
        pt_homogeneous = np.array([img_pt[0], img_pt[1], 1.0])
        pt_board = H_mat @ pt_homogeneous
        pt_board = pt_board / pt_board[2]
        expected = board_points[i]
        print(f"Corner {i+1}: image {img_pt} -> board {pt_board[:2]} (expected {expected})")
    
    np.save("H_board.npy", H_mat)
    print("\nSaved H_board.npy")
    print("\nHomography setup complete!")


if __name__ == "__main__":
    main()
