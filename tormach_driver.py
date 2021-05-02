#!/usr/bin/env python
import rospy, time, copy, sys
import numpy as np

#ROS libs
from sensor_msgs.msg import JointState
from robot_jog_msgs.srv import *
from robot_jog_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

#RR libs
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
from RobotRaconteurCompanion.Util.InfoFileLoader import InfoFileLoader
from RobotRaconteurCompanion.Util.DateTimeUtil import DateTimeUtil
from RobotRaconteurCompanion.Util.AttributesUtil import AttributesUtil

class Tormach(object):
	def __init__(self):
		#initialize robot parameters
		self.joint_names=['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
		#initialize ROS node
		rospy.init_node('Tormach_RR_Service', anonymous=True)
		#initialize ROS Sub for joint callback
		rospy.Subscriber("/joint_states", JointState, self._joint_callback)


		#publisher for jogexecute
		self.JE=JogExecute()
		self.JE.execute=True

		#initialize jog service
		rospy.wait_for_service('/jog/absolute/set_joints')
		self.jog_srv=rospy.ServiceProxy('/jog/absolute/set_joints',SetJoints)
		#initialize jog publisher
		time.sleep(2)
		self.jog_pub = rospy.Publisher('/jog/execute', JogExecute, queue_size=1)
		self.jog_rate = rospy.Rate(10) # 10hz

		#position command param
		#ROS
		self.traj_pub = rospy.Publisher(
			'/position_trajectory_controller/command',
			JointTrajectory,
			queue_size=1,
		)
		self.Tj = JointTrajectory()
		Tj.joint_names=self.joint_names
		self.position_rate = rospy.Rate(500)
		#RR
		self._lock=threading.Lock()
		self._running=False
		self.robot_state_struct=RRN.NewStructure("com.robotraconteur.robotics.robot.RobotState")
		self.robot_command_mode_struct=RRN.NewStructure("com.robotraconteur.robotics.robot.RobotCommandMode")
		self.command_mode=None	
		self.command_seqno=0	

	def _joint_callback(self,data):
		self.joint_positions=list(data.position)
		self.robot_state_struct.joint_position=self.joint_positions
		self.robot_state.OutValue=self.robot_state_struct

	def jog_freespace(self,joint_position,max_velocity,wait):
		##TODO enum, writeonly wire invalue
		while np.linalg.norm(self.joint_position-joint_positions)>0.001 and self.command_mode.InValue==self.robot_command_mode_struct['jog']:
			self.jog_pub.publish(JE)
			self.jog_srv(self.joint_names,joint_position)
			self.jog_rate.sleep()

	def _position_command_thread(self):
		##TODO enum, writeonly wire invalue
		while self._running:
			with self._lock:
				##read wire value
				if self.command_mode.InValue==self.robot_command_mode_struct['position_command'] and self.position_command.InValue.seqno>self.command_seqno:
					#update command_seqno
					self.command_seqno=self.position_command.InValue.seqno

					while np.linalg.norm(self.position_command.InValue.command-self.joint_position)>0.001:
						#break if new value comes in
						if self.position_command.InValue.seqno>self.command_seqno:
							break

						self.Tj.header.stamp = rospy.Time()
						Tjp = JointTrajectoryPoint()
						Tjp.positions = self.position_command.InValue.command
						vel = (Tjp.positions - self.joint_position) * rate.sleep_dur.to_sec()
						Tjp.velocities = vel
						Tjp.time_from_start = rospy.Duration()
						Tjp.time_from_start.nsecs = 1
						self.Tj.points = [Tjp]
						self.traj_pub.publish(self.Tj)
						self.position_rate.sleep()

	def start(self):
		self._running=True
		self._pos_command = threading.Thread(target=self._position_command_thread)
		self._pos_command.daemon = True
		self._pos_command.start()

	def close(self):
		self._running = False
		self._pos_command.join()



with RR.ServerNodeSetup("Tormach_Service", 11111) as node_setup:

	RRC. RegisterStdRobDefServiceTypes(RRN)
	RRN.RegisterServiceTypeFromFile('com.robotraconteur.robotics.robot.robdef')

	tormach_inst=Tormach()
	tormach_inst.start()

	RRN.RegisterService("tormach", "com.robotraconteur.robotics.robot.Robot", tormach_inst)

	print("press ctrl+c to quit")
	rospy.spin()
	signal.sigwait([signal.SIGTERM,signal.SIGINT])
	
	tormach_inst.close()
