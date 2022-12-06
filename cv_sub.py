#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
 
def callback(data):
  br = CvBridge()
  rospy.loginfo("receiving video frame")
   
  # Convert ROS Image message to OpenCV image
  current_frame = br.imgmsg_to_cv2(data)
   
  # Display image
  cv2.imshow("webcam", current_frame)
  cv2.waitKey(1)
      
def receive_message():
  rospy.init_node('webcam_sub')
  rospy.Subscriber('video_frames', Image, callback)

  rospy.spin()
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  receive_message()
