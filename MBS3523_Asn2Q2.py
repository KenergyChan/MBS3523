import cv2
import serial
import time
import numpy as np

faceCascade = cv2.CascadeClassifier('Resources/haarcascade_frontalface_default.xml')

ser = serial.Serial('COM3',baudrate=115200,timeout=1)
time.sleep(0.5)
pos = 90
# print(type(pos))

cv2.namedWindow('MBS3523')
def NIL(x):
    pass

cam= cv2.VideoCapture(0)
cv2.createTrackbar('HL', 'MBS3523', 30, 179, NIL)
cv2.createTrackbar('HH', 'MBS3523', 55, 179, NIL)
cv2.createTrackbar('SL', 'MBS3523', 110, 255, NIL)
cv2.createTrackbar('SH', 'MBS3523', 255, 255, NIL)
cv2.createTrackbar('VL', 'MBS3523', 70, 255, NIL)
cv2.createTrackbar('VH', 'MBS3523', 255, 255, NIL)

while True:
    ret, img = cam.read()
    #imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #faces = faceCascade.detectMultiScale(imgGray, 1.05, 3)
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hueLow = cv2.getTrackbarPos('HL', 'MBS3523')
    hueHigh = cv2.getTrackbarPos('HH', 'MBS3523')
    satLow = cv2.getTrackbarPos('SL', 'MBS3523')
    satHigh = cv2.getTrackbarPos('SH', 'MBS3523')
    valueLow = cv2.getTrackbarPos('VL', 'MBS3523')
    valueHigh = cv2.getTrackbarPos('VH', 'MBS3523')
    mask1 = cv2.inRange(imgHSV, (hueLow, satLow, valueLow), (hueHigh, satHigh, valueHigh))
    cv2.imshow('Mask1', mask1)
    Contours, no_use = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for count in Contours:
        area = cv2.contourArea(count)
        (x, y, w, h) = cv2.boundingRect(count)
        if area > 100:
            # cv2.drawContours(img, [count],-1,(0,0,255),3)
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 1)
        errorPan = (x + w/2) - 640/2
        print('errorPan', errorPan)
        # print(type(errorPan))
        if abs(errorPan) > 20:
            pos = pos - errorPan/30
            print(type(pos))
        if pos > 160:
            pos = 160
            print("Out of range")
        if pos < 0:
            pos = 0
            print("out of range")
        servoPos = str(pos) + '\r'
        ser.write(servoPos.encode())
        print('servoPos = ', servoPos)
        # print(type(pos))
        time.sleep(0.1)
    cv2.imshow('MBS3523 Webcam', img)

    if cv2.waitKey(5) & 0xff == 27:
        break

ser.close()
cam.release()
cv2.destroyAllWindows()