# cv_tracker.py
import cv2
import numpy as np
from datasender import datasender
CAM_INDEX = 0

# Color threshold for robot detection
LOWER_HSV = np.array([90,  80,  80]) 
UPPER_HSV = np.array([140, 255, 255])

# Robot’s IP + port
ROBOT_IP   = "10.120.64.104"
ROBOT_PORT = 5555

def main():

    # Create sender instance
    sender = datasender(ROBOT_IP, ROBOT_PORT)

    # Load calibration
    mtx  = np.load("camera_mtx.npy")
    dist = np.load("camera_dist.npy")
    H    = np.load("H_board.npy")

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera.")

    print("Press 'q' to quit.\nTracking running...")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        undist = cv2.undistort(frame, mtx, dist)

        hsv = cv2.cvtColor(undist, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        robot_found = False
        x_board = y_board = None

        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > 50:  # small threshold, ignore noise
                M = cv2.moments(c)
                if M["m00"] != 0:
                    u = int(M["m10"] / M["m00"])
                    v = int(M["m01"] / M["m00"])
                    robot_found = True

                    # Draw detection
                    cv2.circle(undist, (u, v), 5, (0, 0, 255), -1)
                    cv2.putText(undist, f"({u},{v})", (u + 5, v - 5),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 0, 255), 1)

                    # board coords -> homography
                    pt_img = np.array([u, v, 1.0])
                    pt_board = H @ pt_img
                    x_board = pt_board[0] / pt_board[2]
                    y_board = pt_board[1] / pt_board[2]

                    cv2.putText(undist, f"x={x_board:.1f}  y={y_board:.1f}",
                                (u + 5, v + 15),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (255, 0, 0), 1)

        # UDP
        if robot_found:
            message = f"X:{x_board:.2f},Y:{y_board:.2f}"
        else:
            message = "X:NaN,Y:NaN"

        sender.forward(message)

        cv2.imshow("undistorted", undist)
        cv2.imshow("mask", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    sender.close()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
