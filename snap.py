#!/usr/bin/env python3
"""
Snapshot capture script using Picamera2 on Raspberry Pi.

Usage:
  python3 snap_picamera2.py [--folder FOLDER] [--name NAME] [--width WIDTH] [--height HEIGHT]

Arguments:
  --folder    Folder to save snapshots (default: current directory)
  --name      Base filename (default: 'snapshot')
  --width     Desired frame width in pixels (default: native)
  --height    Desired frame height in pixels (default: native)

Controls:
  SPACE       Save current frame as JPEG
  q           Quit the application
"""
import os
import sys
import argparse
import cv2

try:
    from picamera2 import Picamera2
except ImportError:
    sys.exit("ERROR: Picamera2 module not found. Install with: sudo apt install python3-picamera2")


def parse_args():
    parser = argparse.ArgumentParser(description="Save snapshots with Picamera2")
    parser.add_argument("--folder", default=".",
                        help="Output folder for snapshots")
    parser.add_argument("--name", default="snapshot",
                        help="Base filename for saved images")
    parser.add_argument("--width", type=int, default=0,
                        help="Frame width in pixels (0 = native)")
    parser.add_argument("--height", type=int, default=0,
                        help="Frame height in pixels (0 = native)")
    return parser.parse_args()


def main():
    args = parse_args()
    os.makedirs(args.folder, exist_ok=True)

    picam2 = Picamera2()
    # configure preview with optional resolution override
    if args.width > 0 and args.height > 0:
        config = picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (args.width, args.height)}
        )
    else:
        config = picam2.create_preview_configuration(main={"format": "RGB888"})

    picam2.configure(config)
    picam2.start()

    # retrieve actual resolution
    res_w, res_h = config.main["size"]
    count = 0
    window_name = "camera"
    cv2.namedWindow(window_name)

    try:
        while True:
            frame = picam2.capture_array()
            # convert RGB (Picamera2) to BGR for OpenCV
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            cv2.imshow(window_name, frame_bgr)
            key = cv2.waitKey(1) & 0xFF

            if key == ord(' '):  # SPACE: save snapshot
                filename = os.path.join(
                    args.folder,
                    f"{args.name}_{res_w}_{res_h}_{count}.jpg"
                )
                cv2.imwrite(filename, frame_bgr)
                print(f"[+] Saved {filename}")
                count += 1

            elif key == ord('q'):  # q: quit
                break

    finally:
        picam2.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
