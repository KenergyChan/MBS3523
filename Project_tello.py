import time
from djitellopy import tello
import cv2

me= tello.Tello()
me.connect()
print(me.get_battery())
count=0
floor=1
height= type(float(0))

me.streamon()

while True:
    me.takeoff()
    img = me.get_frame_read().frame
    img = cv2.resize(img, (360,240))

    cv2.imshow("Image", img)

    name = './images/' + floor + '/' + 'Front' + '/' + str(count) + '.jpg'
    cv2.imwrite(name, img)
    time.sleep(0.3)
    print("Creating Images........." + name)

    me.rotate_clockwise(90)
    name = './images/' + floor + '/' + 'Right' + '/' + str(count) + '.jpg'
    cv2.imwrite(name, img)
    time.sleep(0.3)
    print("Creating Images........." + name)

    me.rotate_counter_clockwise(180)
    name = './images/' + floor + '/' + 'Left' + '/' + str(count) + '.jpg'
    cv2.imwrite(name, img)
    time.sleep(0.3)
    print("Creating Images........." + name)

    me.move_up(100)
    count = 0
    height += 100
    floor += height/300

    if me.get_battery() < 20 | floor > 38:
        me.land()

    cv2.waitKey(0.5)
