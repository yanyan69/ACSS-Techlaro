import cv2

for i in range(3):  # try 0,1,2
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"Camera index {i} opened successfully")
        ret, frame = cap.read()
        if ret:
            print(f"Frame captured from camera {i}")
            cv2.imshow(f'Camera {i}', frame)
            cv2.waitKey(1000)
        cap.release()
    else:
        print(f"Camera index {i} failed to open")
cv2.destroyAllWindows()
