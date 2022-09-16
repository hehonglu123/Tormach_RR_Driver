#!/usr/bin/env python
import rospy, actionlib, time, copy, sys
import numpy as np
from sensor_msgs.msg import JointState
from robot_jog_msgs.srv import *
from robot_jog_msgs.msg import *
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import matplotlib.pyplot as plt


joint_positions=[]
joint_positions_history=[]
def callback(data):
	global joint_positions
	joint_positions=list(data.position)
	joint_positions_history.append(joint_positions[-1])
	


def main():
	global joint_positions

	rospy.init_node('jogging_node', anonymous=True)

	rospy.Subscriber("/joint_states", JointState, callback)
	pub_t = rospy.Publisher(
			'/position_trajectory_controller/command',
			JointTrajectory,
			queue_size=1,
		)
	#publisher for jogexecute
	JE=JogExecute()
	JE.execute=True


	# joint_names=['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6',]
	joint_names=['joint_6']
	#jogging to initial position
	# joint_positions[-1]=0.
	#wait the service to be advertised, otherwise the service use will fail
	rospy.wait_for_service('/jog/absolute/set_joints')
	 
	#setup a local proxy for the service
	srv=rospy.ServiceProxy('/jog/absolute/set_joints',SetJoints)
	 
	
	#wait for first jp arrives
	time.sleep(1)
	pub = rospy.Publisher('/jog/execute', JogExecute, queue_size=1)
	rate = rospy.Rate(10) # 10hz
	#jog to 0
	while np.linalg.norm(joint_positions[-1])>0.001:
		pub.publish(JE)
		rtval=srv(joint_names,[0.])
		rate.sleep()


	#traj sin
	Tj = JointTrajectory()
	Tj.joint_names = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6',]
	rate = rospy.Rate(500)  # 10hz
	now = time.time()
	for i in range(10000):
		t=float(i/1000.)
		Tj.header.stamp = rospy.Time()
		Tjp = JointTrajectoryPoint()
		Tjp.positions = [np.sin(t)/4.,np.sin(t)/4.,np.sin(t)/4.,np.sin(t)/4.,np.sin(t)/4.,np.sin(t)/4.]
		vel = (Tjp.positions[5] - last_pos) / 1000.

		Tjp.time_from_start = rospy.Duration()
		Tjp.time_from_start.secs=int(t)
		Tjp.time_from_start.nsecs = int((t % 1)*1e9)
		Tj.points = [Tjp]

	pub_t.publish(Tj)


	

if __name__ == '__main__':
	main()

	
	