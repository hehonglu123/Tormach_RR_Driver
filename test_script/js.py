#!/usr/bin/env python
import rospy, time, copy
import numpy as np
from sensor_msgs.msg import JointState
from robot_jog_msgs.srv import *
from robot_jog_msgs.msg import *

start_joint=[]
def callback(data):
	global start_joint
	start_joint=list(data.position)
	# print('joints: ',data.position)
	
def listener():
	rospy.Subscriber("/joint_states", JointState, callback)

	# rospy.spinonce()
	time.sleep(0.1)


def main():
	global start_joint

	rospy.init_node('jogging_node', anonymous=True)

	listener()
	#publisher for jogexecute
	JE=JogExecute()
	JE.execute=True

	



	# joint_names=['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6',]
	joint_names=['joint_6']
	#jogging to initial position
	# start_joint[-1]=0.
	#wait the service to be advertised, otherwise the service use will fail
	rospy.wait_for_service('/jog/absolute/set_joints')
	 
	#setup a local proxy for the service
	srv=rospy.ServiceProxy('/jog/absolute/set_joints',SetJoints)
	 
	


	pub = rospy.Publisher('/jog/execute', JogExecute, queue_size=1)
	rate = rospy.Rate(10) # 10hz
	now=time.time()
	while time.time()-now<2.:
		pub.publish(JE)
		rtval=srv(joint_names,[np.sin(time.time()-now)])
		print(rtval)
		rate.sleep()


	

if __name__ == '__main__':
	main()

	
	