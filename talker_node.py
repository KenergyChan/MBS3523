#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16

def talker():
	a=10
	rospy.init_node('talker_node', anonymous=True)
	pub = rospy.Publisher('servo', UInt16, queue_size=10)
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		pubMessage = a
		rospy.loginfo('Data sent: {}'.format(a))
		pub.publish(pubMessage)
		rate.sleep()
		if a < 170:
			a+=20

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
