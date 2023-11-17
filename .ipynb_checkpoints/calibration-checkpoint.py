import cv2
import numpy as np
from cv2 import calibrateCamera, findChessboardCorners

if __name__ == "__main__":
    # define coordinates of points
    # Capture images of checkerboard
    # create video capture obj
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise TimeoutError("Camera not open")

    while True:
        i = 0
        ret, frame = cap.read()
        if not ret:
            raise TimeoutError("Can't recieve frame")
        command = cv2.waitkey(1)
        if command == ord('q'):
            break
        else:
            cv2.imshow("Image", frame)
            if cv2.waitkey(1) == ord(' '):
                cv2.imwrite()
            cv2.destroyWindow("Image")
            
    # findChessboardCorners
    # calibrateCamera
    # define coordinates of points
