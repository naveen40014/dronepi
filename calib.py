#!/usr/bin/env python3
"""
Unified camera calibration & distance-measurement script.

Usage:
  # Calibrate only:
  python cameracalib.py ./images --mode charuco --squares_x 5 --squares_y 7 \
      --square_length 0.04 --marker_length 0.02

  # Calibrate + measure distance on a test image:
  python cameracalib.py ./images --mode charuco --squares_x 5 --squares_y 7 \
      --square_length 0.04 --marker_length 0.02 --measure test.jpg
"""

import os
import sys
import glob
import json
import argparse

import numpy as np
import cv2

def parse_args():
    p = argparse.ArgumentParser(description="Camera calibration & optional distance measurement")
    p.add_argument("folder", help="Folder with calibration images")
    p.add_argument("--mode", choices=["chessboard", "charuco"], required=True)
    p.add_argument("--ext", nargs="+", default=["jpg","png","bmp"],
                   help="Image extensions to search")
    # chessboard params
    p.add_argument("--rows",    type=int,   default=9)
    p.add_argument("--cols",    type=int,   default=6)
    p.add_argument("--square_size", type=float, default=0.016,
                   help="Chessboard square size (m)")
    # charuco params
    p.add_argument("--squares_x",    type=int,   default=5)
    p.add_argument("--squares_y",    type=int,   default=7)
    p.add_argument("--square_length", type=float, default=0.04,
                   help="ChArUco square length (m)")
    p.add_argument("--marker_length", type=float, default=0.02,
                   help="ChArUco marker length (m)")
    # measurement
    p.add_argument("--measure", help="If set, path to one image to measure camera→board distance")
    return p.parse_args()

def find_images(folder, exts):
    imgs = []
    for e in exts:
        imgs += glob.glob(os.path.join(folder, f"*.{e}"))
    return sorted(imgs)

def calibrate_charuco(images, sx, sy, sl, ml):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
    board = cv2.aruco.CharucoBoard_create(
        squaresX=sx, squaresY=sy,
        squareLength=sl, markerLength=ml,
        dictionary=aruco_dict
    )
    params = cv2.aruco.DetectorParameters_create()
    all_corners, all_ids = [], []
    img_size = None

    for fn in images:
        img = cv2.imread(fn)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_size = gray.shape[::-1]

        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
        if ids is None or len(ids)<1:
            continue
        n, c_corners, c_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
        if n<4:
            continue

        all_corners.append(c_corners)
        all_ids.append(c_ids)
        print(f"[+] {n} ChArUco corners in {fn}")

    if len(all_corners)<3:
        sys.exit("Need at least 3 valid ChArUco images.")

    rms, cam_mtx, dist, _, _ = cv2.aruco.calibrateCameraCharuco(
        charucoCorners=all_corners,
        charucoIds=all_ids,
        board=board,
        imageSize=img_size,
        cameraMatrix=None,
        distCoeffs=None
    )
    print(f"Calibration RMS error: {rms:.4f}")
    return board, cam_mtx, dist, img_size

def measure_distance(img_path, board, cam_mtx, dist):
    params = cv2.aruco.DetectorParameters_create()
    aruco_dict = board.dictionary

    img = cv2.imread(img_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
    if ids is None or len(ids)<1:
        print("No markers found in measurement image.")
        return

    n, c_corners, c_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    if n<4:
        print("Too few Charuco corners for pose estimation.")
        return

    ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
        c_corners, c_ids, board, cam_mtx, dist, None, None
    )
    if not ok:
        print("Pose estimation failed.")
        return

    # distance = norm of translation vector
    dist_m = np.linalg.norm(tvec)
    print(f"Estimated camera→board distance: {dist_m:.3f} m")

    # draw axes (0.5×marker length)
    axis_len = board.markerLength / 2
    cv2.aruco.drawAxis(img, cam_mtx, dist, rvec, tvec, axis_len)
    out = os.path.splitext(img_path)[0] + "_measured.png"
    cv2.imwrite(out, img)
    print(f"Annotated measurement image saved to {out}")

def save_calibration(folder, cam_mtx, dist):
    os.makedirs(folder, exist_ok=True)
    np.savetxt(os.path.join(folder, "cameraMatrix.txt"), cam_mtx, delimiter=',')
    np.savetxt(os.path.join(folder, "distCoeffs.txt"), dist, delimiter=',')
    with open(os.path.join(folder, "calibration.json"), 'w') as f:
        json.dump({'camera_matrix': cam_mtx.tolist(),
                   'dist_coeffs': dist.tolist()}, f, indent=2)
    print(f"Calibration data saved to {folder}/cameraMatrix.txt, distCoeffs.txt, calibration.json")

def main():
    args = parse_args()
    imgs = find_images(args.folder, args.ext)
    if not imgs:
        sys.exit(f"No images in {args.folder} with extensions {args.ext}")

    if args.mode == "charuco":
        board, cam_mtx, dist, _ = calibrate_charuco(
            imgs, args.squares_x, args.squares_y,
            args.square_length, args.marker_length
        )
    else:
        sys.exit("Chessboard mode not shown here, see earlier version.")

    save_calibration(args.folder, cam_mtx, dist)

    if args.measure:
        if not os.path.isfile(args.measure):
            sys.exit(f"Measurement image not found: {args.measure}")
        measure_distance(args.measure, board, cam_mtx, dist)

if __name__ == "__main__":
    main()
