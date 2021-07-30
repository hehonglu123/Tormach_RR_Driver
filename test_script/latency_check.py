#!/usr/bin/env python
import rospy, time, copy, sys
import numpy as np
from sensor_msgs.msg import JointState
from robot_jog_msgs.msg import *




def callback(data):
	print(rospy.get_rostime()-data.header.stamp)
	


def main():
	global joint_positions

	rospy.init_node('jogging_node', anonymous=True)

	rospy.Subscriber("/joint_states", JointState, callback)
	rospy.spin()

	
	

if __name__ == '__main__':
	main()

	
	