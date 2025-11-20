# define_board_homography.py
import cv2
import numpy as np

CAM_INDEX = 0

clicked_points = []
frozen_image = None

def mouse_callback(event, x, y, flags, param):
    global clicked_points, frozen_image
    if event == cv2.EVENT_LBUTTONDOWN and frozen_image is not None:
        if len(clicked_points) < 4:
            clicked_points.append((x, y))
            print(f"Clicked point {len(clicked_points)}: ({x}, {y})")
            # Draw a small circle for feedback
            cv2.circle(frozen_image, (x, y), 5, (0, 0, 255), -1)
            cv2.imshow("Click 4 corners", frozen_image)

def main():
    global frozen_image, clicked_points

    # Load camera intrinsics
    mtx = np.load("camera_mtx.npy")
    dist = np.load("camera_dist.npy")

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera")

    print("Press 'c' to capture a frame for board corner selection.")
    print("Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        undist = cv2.undistort(frame, mtx, dist)
        cv2.imshow("Live (press 'c' to capture)", undist)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            frozen_image = undist.copy()
            break
        elif key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            return

    cap.release()
    cv2.destroyAllWindows()

    if frozen_image is None:
        print("No frame captured.")
        return

    clicked_points = []
    cv2.namedWindow("Click 4 corners")
    cv2.setMouseCallback("Click 4 corners", mouse_callback)
    cv2.imshow("Click 4 corners", frozen_image)

    print("Click 4 board corners in this order:")
    print("1) Top-left, 2) Top-right, 3) Bottom-right, 4) Bottom-left")

    # Wait until 4 clicks are done
    while True:
        cv2.imshow("Click 4 corners", frozen_image)
        key = cv2.waitKey(10) & 0xFF
        if len(clicked_points) == 4:
            break
        if key == ord('q'):
            print("Aborted.")
            cv2.destroyAllWindows()
            return

    cv2.destroyAllWindows()

    image_points = np.array(clicked_points, dtype=np.float32)

    # Get board physical dimensions
    print("\nEnter board dimensions in centimeters.")
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
    H_mat, _ = cv2.findHomography(image_points, board_points)
    print("Homography matrix:\n", H_mat)

    np.save("H_board.npy", H_mat)
    print("Saved H_board.npy")

if __name__ == "__main__":
    main()
