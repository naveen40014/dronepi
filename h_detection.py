import cv2
import numpy as np

# === CONFIGURATION ===
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CALIB_PATH = "/home/pi/drone/opencv/campi/"

# === LOAD CALIBRATION DATA ===
camera_matrix = np.loadtxt(CALIB_PATH + 'cameraMatrix.txt', delimiter=',')
camera_distortion = np.loadtxt(CALIB_PATH + 'cameraDistortion.txt', delimiter=',')

# === OPEN CAMERA USING GStreamer ===
print("🎥 Opening Pi Camera via GStreamer...")
cap = cv2.VideoCapture(
    "libcamerasrc ! video/x-raw,width=640,height=480,framerate=30/1,format=BGR ! videoconvert ! appsink",
    cv2.CAP_GSTREAMER
)

if not cap.isOpened():
    print("❌ Failed to open camera.")
    exit()

print("✅ Camera is ready. Press 'q' to exit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Failed to grab frame")
        break

    # === UNDISTORT FRAME ===
    frame_undistorted = cv2.undistort(frame, camera_matrix, camera_distortion)

    # === CONVERT TO GRAYSCALE AND THRESHOLD ===
    gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY_INV)

    # === FIND CONTOURS ===
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 1500:
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        aspect_ratio = w / float(h)
        if 0.5 < aspect_ratio < 1.5:
            roi = thresh[y:y + h, x:x + w]
            roi_resized = cv2.resize(roi, (100, 100))

            # Analyze pixel intensity bands
            vertical_band = np.sum(roi_resized[:, 40:60])
            horizontal_band = np.sum(roi_resized[45:55, :])

            if vertical_band > 10000 and horizontal_band > 10000:
                cv2.rectangle(frame_undistorted, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame_undistorted, "H", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    cv2.imshow("H Detection", frame_undistorted)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
