import cv2
import numpy as np
import glob

CHECKERBOARD = (9,6)  # (columns, rows) inner corners
square_size = 25  # mm

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1],3), np.float32)
objp[:,:2] = np.mgrid[0:CHECKERBOARD[0],0:CHECKERBOARD[1]].T.reshape(-1,2)
objp = objp * square_size

objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Pi Cam could not be opened")
    exit()

print("Show the checkerboard to the camera from different angles. Press SPACE to capture an image.")
print("Press ESC when done.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow("Calibration", frame)

    key = cv2.waitKey(10) & 0xFF
    if key == 27:  # ESC
        break
    elif key == 32:  # SPACE
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
            cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2, ret)
            cv2.imshow("Calibration", frame)
            cv2.waitKey(500)
            print(f"Captured image {len(objpoints)}")
        else:
            print("Checkerboard not found — try again")

cap.release()
cv2.destroyAllWindows()

if len(objpoints) < 10:
    print("Not enough captures — at least 10 recommended!")
    exit()

ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

np.savetxt("cameraMatrix.txt", cameraMatrix, delimiter=',')
np.savetxt("cameraDistortion.txt", distCoeffs, delimiter=',')

print("Calibration done. Files saved: cameraMatrix.txt, cameraDistortion.txt")
