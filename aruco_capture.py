import cv2
import os

# Create output folder
save_dir = "aruco_images"
os.makedirs(save_dir, exist_ok=True)

# Start camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("❌ Cannot open camera.")
    exit()

count = 0
print("📸 Press SPACE to save image, q to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Failed to grab frame.")
        break

    cv2.imshow("Aruco Capture", frame)
    key = cv2.waitKey(1)

    if key % 256 == 32:  # SPACE key
        filename = f"{save_dir}/aruco_{count}.png"
        cv2.imwrite(filename, frame)
        print(f"✅ Saved {filename}")
        count += 1
    elif key % 256 == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print(f"📝 {count} images saved in {save_dir}/")
