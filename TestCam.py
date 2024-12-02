import cv2
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
if ret:
    # cv2.imshow('Test Frame', frame)
    print("success")
    cv2.waitKey(0)
else:
    print("Error capturing frame")
cap.release()
cv2.destroyAllWindows()
