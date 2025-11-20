# capture_calib_images.py
import cv2
import os

SAVE_DIR = "calib"       # folder to save images
CAM_INDEX = 0            # 0 = default camera

def main():
    os.makedirs(SAVE_DIR, exist_ok=True)

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera")

    print("Press SPACE to save an image, 'q' to quit.")

    img_id = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        cv2.imshow("Calibration Capture (SPACE=save, q=quit)", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(' '):  # SPACE
            filename = os.path.join(SAVE_DIR, f"calib_{img_id:03d}.jpg")
            cv2.imwrite(filename, frame)
            print(f"Saved {filename}")
            img_id += 1

        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
