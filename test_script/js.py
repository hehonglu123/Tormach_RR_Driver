#!/usr/bin/env python
import rospy, time, copy, sys
import numpy as np
from sensor_msgs.msg import JointState
from robot_jog_msgs.srv import *
from robot_jog_msgs.msg import *
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
	time.sleep(2)
	pub = rospy.Publisher('/jog/execute', JogExecute, queue_size=1)
	rate = rospy.Rate(10) # 10hz
	#jog to 0
	while np.linalg.norm(joint_positions[-1])>0.001:
		pub.publish(JE)
		rtval=srv(joint_names,[0.])
		rate.sleep()

	#jog sin
	now=time.time()
	while time.time()-now<20.:
		pub.publish(JE)
		rtval=srv(joint_names,[np.sin(time.time()-now)])
		rate.sleep()

	plt.plot(joint_positions_history)
	plt.show()
	sys.exit()
	rospy.spin()

	
	

if __name__ == '__main__':
	main()

	
	