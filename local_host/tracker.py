import cv2
import numpy as np

def nothing(x):
    pass

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    cv2.namedWindow("Controls", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Controls", 400, 300)

    # How colorful an object must be (S threshold)
    cv2.createTrackbar("S_thresh", "Controls", 70, 255, nothing)
    # How dark an object can be to be considered "dark" (V upper bound)
    cv2.createTrackbar("V_dark", "Controls", 100, 255, nothing)
    # Minimum and maximum area for a blob to be kept
    cv2.createTrackbar("MinArea", "Controls", 500, 50000, nothing)
    cv2.createTrackbar("MaxArea", "Controls", 10000, 200000, nothing)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        s_thresh = cv2.getTrackbarPos("S_thresh", "Controls")
        v_dark   = cv2.getTrackbarPos("V_dark", "Controls")
        min_area = cv2.getTrackbarPos("MinArea", "Controls")
        max_area = cv2.getTrackbarPos("MaxArea", "Controls")

        # 1) colorful stuff: S >= s_thresh (any hue, any brightness)
        lower_colorful = np.array([0, s_thresh, 0], dtype=np.uint8)
        upper_colorful = np.array([179, 255, 255], dtype=np.uint8)
        mask_colorful = cv2.inRange(hsv, lower_colorful, upper_colorful)

        # 2) dark stuff: V <= v_dark (any hue, any saturation)
        lower_dark = np.array([0, 0, 0], dtype=np.uint8)
        upper_dark = np.array([179, 255, v_dark], dtype=np.uint8)
        mask_dark = cv2.inRange(hsv, lower_dark, upper_dark)

        # Combine both: anything colorful OR dark
        mask = cv2.bitwise_or(mask_colorful, mask_dark)

        # Clean up the mask a bit
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        idx = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area or area > max_area:
                continue  # too small or too big

            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h),
                          (0, 255, 0), 2)

            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
                cv2.putText(frame, f"ID {idx} ({cx},{cy})",
                            (x, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 1)
            idx += 1

        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
