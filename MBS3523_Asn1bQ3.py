import cv2
#import numpy as np
print(cv2.__version__)

capture = cv2.VideoCapture(0)
starting_x = 200
starting_y = 200
ending_x = 280
ending_y = 280
upward = True
right = True
while True:
    success, img = capture.read()
    img = cv2.resize(img, (600, 600))
    cv2.rectangle(img, (starting_x,starting_y),(ending_x,ending_y),(0,0,255), 10)
    cv2.putText(img,"MBS3523 Assignment 1b  Q3 Name: Chan Tsz Kin", (90,20), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0), 2)
    cv2.imshow('Frame', img)

#Since the inclined degree is 60, but the position of rectangle must be in integer,
#the ratio of x and y in 60 degree is about 7:12

    if upward == True:
        if starting_y >  0:
            starting_y -= 12
            ending_y -= 12
        else:
            upward = False
    else:
        if ending_y < 600 :
            starting_y += 12
            ending_y += 12
        else:
            upward = True

    if right == True:
        if ending_x < 600:
            starting_x += 7
            ending_x += 7
        else:
            right = False
    else:
        if starting_x > 0 :
            starting_x -= 7
            ending_x -= 7
        else:
            right = True

    if cv2.waitKey(20) & 0xff == ord('q'):
            break


capture.release()
cv2.destroyAllWindows()