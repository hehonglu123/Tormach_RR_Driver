#!/usr/bin/env python
import rospy, time, copy, traceback
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal

start_joint=[]
def callback(data):
	global start_joint
	start_joint=list(data.position)
	# print('joints: ',data.position)
	
def listener():
	rospy.Subscriber("/joint_states", JointState, callback)

	# rospy.spinonce()
	time.sleep(0.5)


def main():
	global start_joint
	rospy.init_node('jogging_node', anonymous=True)
	pub = rospy.Publisher('/position_trajectory_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=1)


	listener()
	joint_names=['joint_6']
	
	Tj=JointTrajectory()	
	Tj.joint_names=joint_names

	length=500
	for i in range(length):
		Tjp=JointTrajectoryPoint()
		Tjp.positions=[start_joint[-1]+np.sin(i/length)]
		print(Tjp.positions)
		Tjp.time_from_start= rospy.Duration(i/length)
		Tj.points.append(Tjp)

	FJTG=FollowJointTrajectoryGoal()
	FJTG.trajectory=Tj
	FJTAG=FollowJointTrajectoryActionGoal()
	FJTAG.goal=FJTG


	
	pub.publish(FJTAG)


if __name__ == '__main__':
	main()

	
	