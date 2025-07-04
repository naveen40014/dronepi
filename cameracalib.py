import cv2
import numpy as np
import glob

# Charuco board parameters
CHARUCO_ROWS = 5
CHARUCO_COLS = 7
SQUARE_LENGTH = 0.025  # meters (25 mm)
MARKER_LENGTH = 0.02   # meters (20 mm)

# Create Charuco board
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(CHARUCO_COLS, CHARUCO_ROWS, SQUARE_LENGTH, MARKER_LENGTH, aruco_dict)

# Lists to store corners and ids from all images
all_corners = []
all_ids = []
img_size = None

# Load calibration images
images = glob.glob('calib_images/*.png')

print(f"Found {len(images)} calibration images.")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict)

    if len(corners) > 0:
        # Refine detection
        _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)

        if charuco_corners is not None and charuco_ids is not None:
            if len(charuco_corners) > 3:
                all_corners.append(charuco_corners)
                all_ids.append(charuco_ids)
                img_size = gray.shape[::-1]
                print(f"✅ {fname}: {len(charuco_corners)} charuco corners detected.")
                # Optionally visualize
                cv2.aruco.drawDetectedCornersCharuco(img, charuco_corners, charuco_ids)
                cv2.imshow('Charuco Detection', img)
                cv2.waitKey(100)
            else:
                print(f"⚠ {fname}: Not enough corners ({len(charuco_corners)} detected). Skipping.")
        else:
            print(f"⚠ {fname}: Charuco detection failed. Skipping.")
    else:
        print(f"⚠ {fname}: No markers detected. Skipping.")

cv2.destroyAllWindows()

# Check if enough valid frames
if len(all_corners) < 5:
    print("❌ Not enough valid calibration images. Try capturing more with better angles/lighting.")
    exit()

# Perform calibration
ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
    all_corners, all_ids, board, img_size, None, None)

print("\n🎯 Calibration successful!")
print("Camera Matrix:\n", cameraMatrix)
print("Distortion Coefficients:\n", distCoeffs)

# Save to files
np.savetxt("cameraMatrix.txt", cameraMatrix, delimiter=',')
np.savetxt("cameraDistortion.txt", distCoeffs, delimiter=',')

print("\n✅ Saved cameraMatrix.txt and cameraDistortion.txt")
