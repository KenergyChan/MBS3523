from djitellopy import tello
from time import sleep
import cv2

me= tello.Tello()
me.connect()
print(me.get_battery())

cv2.namedWindow('MBS3523')
def NIL(x):
    pass

cam= cv2.VideoCapture(0)
cv2.createTrackbar('HL', 'MBS3523', 84, 255, NIL)
cv2.createTrackbar('HH', 'MBS3523', 140, 255, NIL)
cv2.createTrackbar('SL', 'MBS3523', 0, 255, NIL)
cv2.createTrackbar('SH', 'MBS3523', 46, 255, NIL)
cv2.createTrackbar('VL', 'MBS3523', 128, 255, NIL)
cv2.createTrackbar('VH', 'MBS3523', 201, 255, NIL)

me.streamon()

while True:
    img = me.get_frame_read().frame
    img = cv2.resize(img, (360,240))
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hueLow = cv2.getTrackbarPos('HL', 'MBS3523')
    hueHigh = cv2.getTrackbarPos('HH', 'MBS3523')
    satLow = cv2.getTrackbarPos('SL', 'MBS3523')
    satHigh = cv2.getTrackbarPos('SH', 'MBS3523')
    valueLow = cv2.getTrackbarPos('VL', 'MBS3523')
    valueHigh = cv2.getTrackbarPos('VH', 'MBS3523')
    mask1 = cv2.inRange(imgHSV, (hueLow, satLow, valueLow), (hueHigh, satHigh, valueHigh))

    cv2.imshow("Image", img)
    cv2.waitKey(1)



