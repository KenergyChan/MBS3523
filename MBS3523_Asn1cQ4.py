import cv2
print(cv2.__version__)

faceCascade = cv2.CascadeClassifier('Resources/haarcascade_frontalface_default.xml')
cam = cv2.VideoCapture(0)

while True:

    _, img = cam.read()
    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    imgGB = cv2.GaussianBlur(imgGray, (9, 1), 0)
    faces = faceCascade.detectMultiScale(imgGB, 1.1, 3)
    imgGray = cv2.cvtColor(imgGray, cv2.COLOR_GRAY2BGR)
    cv2.putText(imgGray, "MBS3523 Assignment 1c - Q4 Name: Chan Tsz Kin", (90, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0, 0, 0), 2)

    for (x, y, w, h) in faces:
        cv2.rectangle(imgGray, (x, y), (x + w, y + h), (255, 0, 0), 2)
        imgGray[y:y+h+1, x:x+w+1] = img[y:y+h+1, x:x+w+1]

    cv2.imshow("Gray", imgGray)

    if cv2.waitKey(20) & 0xff == ord('q'):
            break

capture.release()
cv2.destroyAllWindows()