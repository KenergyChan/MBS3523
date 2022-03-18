import cv2
import numpy as np

cam = cv2.VideoCapture(0)

def nil(x):
    pass

cv2.namedWindow('MBS3523')

cv2.createTrackbar('X_POS:','MBS3523',320,640,nil)
cv2.createTrackbar('Y_POS:','MBS3523',240,480,nil)

while True:
    _, img=cam.read()
    cv2.putText(img, "MBS3523 Assignment 1d - Q5 Name: Chan Tsz Kin", (90, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0, 0, 255), 2)
    xPos = cv2.getTrackbarPos('X_POS:', 'MBS3523')
    yPos = cv2.getTrackbarPos('Y_POS:', 'MBS3523')
    cv2.line(img,(xPos,0),(xPos,480),(255,0,0))
    cv2.line(img, (0, yPos), (640, yPos), (255, 0, 0))
    cv2.imshow('MBS3523',img)

    if cv2.waitKey(20) & 0xff == 27:
        break

capture.release()
cv2.destroyAllWindows()