# calibrate_camera.py
import cv2
import numpy as np
import glob

CHECKERBOARD = (9, 6)  # (cols, rows) of INNER corners

IMAGE_DIR = "calib/*.jpg"

def main():
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                30, 0.001)

    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0],
                           0:CHECKERBOARD[1]].T.reshape(-1, 2)

    objpoints = []  # 3D points
    imgpoints = []  # 2D points

    images = glob.glob(IMAGE_DIR)
    print(f"Found {len(images)} images")

    if len(images) == 0:
        print("No calibration images found. Check IMAGE_DIR.")
        return

    gray_shape = None

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_shape = gray.shape[::-1]

        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
        if ret:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                        criteria)
            imgpoints.append(corners2)

            cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            cv2.imshow('Corners', img)
            cv2.waitKey(200)
        else:
            print(f"Checkerboard not found in {fname}")

    cv2.destroyAllWindows()

    # Perform calibration
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray_shape, None, None
    )

    print("Calibration RMS error:", ret)
    print("Camera matrix:\n", mtx)
    print("Distortion coefficients:\n", dist)

    np.save("camera_mtx.npy", mtx)
    np.save("camera_dist.npy", dist)

    print("Saved camera_mtx.npy and camera_dist.npy")

if __name__ == "__main__":
    main()
