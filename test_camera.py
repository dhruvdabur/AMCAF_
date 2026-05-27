import cv2

cap = cv2.VideoCapture(4)

if not cap.isOpened():
    print("Camera not opened")
    exit()

while True:
    ret, frame = cap.read()

    if not ret:
        print("Frame not received")
        break

    cv2.imshow("Lenovo Camera Test", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()