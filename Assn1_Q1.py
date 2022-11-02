#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

def pose_callback(tw):

	target_x = 1
	target_ang = 0.2
	turn = 0
	first_route = True
	tw = Twist()
	msg = Pose()
	rate = rospy.Rate(10)

	while first_route == True:
		
		tw.angular.z = target_ang
		tw.linear.x = target_x
		turn += 1

		if turn == 10:
			target_ang = 0

		if msg.x > 45:
			target_ang = 3.1
			turn = 0
			msg.x = 0
			first_route = False

		pub.publish(tw)
		msg.x += tw.linear.x
		rate.sleep()

	while first_route == False:
		
		tw.angular.z = target_ang
		tw.linear.x = target_x
		turn += 1

		if turn == 10:
			target_ang = 0

		if msg.x > 100:
			target_ang = 3.1
			turn = 0
			msg.x = 0

		pub.publish(tw)
		msg.x += tw.linear.x
		rate.sleep()




if __name__ == '__main__':
	rospy.init_node('turtle_auto_control_node')
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	sub = rospy.Subscriber('/turtle1/pose', Pose, callback=pose_callback)

	rospy.loginfo('Node started!')

	rospy.spin()
