#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

facedetect = cv2.CascadeClassifier('/home/kenergy/ChanTszKin_asn_ws/src/cv_detect/src/haarcascade_frontalface_default.xml')
         
if __name__ == '__main__':
  try:
    rospy.init_node('webcam_pub')
    pub = rospy.Publisher('video_frames', Image, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    cap = cv2.VideoCapture(0)
  # Used to convert between ROS and OpenCV images
    br = CvBridge()
 
    while not rospy.is_shutdown():
      ret, frame = cap.read()
      imgGray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
      #imgGB = cv2.GaussianBlur(imgGray, (5, 5), 0)
      faces = facedetect.detectMultiScale(imgGray, 1.1, 3)
      for x, y, w, h in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
          
      rospy.loginfo('publishing video frame')
      pub.publish(br.cv2_to_imgmsg(frame))
      rate.sleep()

  except rospy.ROSInterruptException:
    pass
