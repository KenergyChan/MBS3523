import cv2
import numpy as np

cam = cv2.VideoCapture(0)

EVT=0
start_point=(0,0)
RDown=0
LDown=0

def draw_rectangle(event,x,y,flags,param):
    global EVT
    global PNT
    if event == cv2.EVENT_LBUTTONDOWN:
        EVT = cv2.EVENT_LBUTTONDOWN
        PNT = (x, y)
    if event == cv2.EVENT_LBUTTONUP:
        EVT = cv2.EVENT_LBUTTONUP
        PNT = (x, y)
    if event == cv2.EVENT_RBUTTONDOWN:
        EVT = cv2.EVENT_RBUTTONDOWN
    if event == cv2.EVENT_RBUTTONUP:
        EVT = cv2.EVENT_RBUTTONUP
        cv2.destroyWindow('image_ROX')

while True:
    _, img=cam.read()
    imgOriginal = img
    cv2.imshow('MBS3523',img)
    cv2.setMouseCallback('MBS3523', draw_rectangle)

    if EVT == cv2.EVENT_LBUTTONDOWN:
        start_point=PNT

    if EVT == cv2.EVENT_LBUTTONUP:
        cv2.rectangle(img, start_point, PNT, (255, 0, 0), 3)
        imgBound = img[start_point[1]:PNT[1], start_point[0]:PNT[0]]
        cv2.imshow('image_ROI', imgBound)
        LDown=1

    if EVT == cv2.EVENT_RBUTTONDOWN:
        RDown=1

    if EVT == cv2.EVENT_RBUTTONUP & RDown==1:
        if LDown==1:
            LDown=0
        img = imgOriginal
        RDown=0

    cv2.imshow('MBS3523', img)

    if cv2.waitKey(20) & 0xff == 27:
            break

capture.release()
cv2.destroyAllWindows()
