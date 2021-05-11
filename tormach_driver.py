#!/usr/bin/env python
import rospy, time, copy, sys, threading, signal, traceback, argparse
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
	def __init__(self,robot_info):
		try:
			#TODO: 
			##turn on robot /hal
			#initialize robot parameters
			self.joint_names=['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
			#initialize ROS node
			rospy.init_node('Tormach_RR_Service', anonymous=True)
			


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
			self.Tj.joint_names=self.joint_names
			self.position_rate_num=100
			self.position_rate = rospy.Rate(self.position_rate_num)
			#RR
			self._lock=threading.Lock()
			self._running=False
			self.robot_state_struct=RRN.NewStructure("com.robotraconteur.robotics.robot.RobotState")
			self.trajectory_status_struct=RRN.NewStructure("com.robotraconteur.robotics.trajectory.TrajectoryStatus")
			self._robot_info=robot_info
			# self.position_command.InValueLifespan=0.5
			self.command_seqno=0	
			self.robot_consts = RRN.GetConstants( "com.robotraconteur.robotics.robot")
			#required?
			self.tool_changed=RR.EventHook()
			self.payload_changed=RR.EventHook()
			self.param_changed=RR.EventHook()
			# self._date_time_util = DateTimeUtil(RRN)
			# self._date_time_utc_type = RRN.GetPodDType('com.robotraconteur.datetime.DateTimeUTC')
			self._date_time_util=RRN.GetNamedArrayDType("com.robotraconteur.datetime.TimeSpec3")

			#initialize ROS Sub for joint callback
			rospy.Subscriber("/joint_states", JointState, self._joint_callback)

			self.jog_joint_ts=time.time()
		except:
			traceback.print_exc()

	@property
	def device_info(self):
		return self._robot_info.device_info

	@property
	def robot_info(self):
		return self._robot_info
	

	def _joint_callback(self,data):
		self.joint_position=list(data.position)
		self.robot_state_struct.joint_position=self.joint_position
		self.robot_state_struct.ts=np.zeros((1,),dtype=self._date_time_util)
		self.robot_state.OutValue=self.robot_state_struct

	def jog_freespace(self,joint_position,max_velocity,wait):
		while np.linalg.norm(self.joint_position-joint_position)>0.001 and self.command_mode==self.robot_consts['RobotCommandMode']['jog']:
			self.jog_pub.publish(self.JE)
			self.jog_srv(self.joint_names,joint_position)
			self.jog_rate.sleep()

	def jog_joint(self,joint_velocity, timeout, wait):
		# now=time.time()
		if self.command_mode==self.robot_consts['RobotCommandMode']['jog']:
			with self._lock:
				self.Tj.header.stamp = rospy.Time()
				Tjp = JointTrajectoryPoint()
				Tjp.positions = list(self.joint_position+joint_velocity*(1/self.position_rate_num))
				Tjp.velocities = list(joint_velocity)
				Tjp.time_from_start = rospy.Duration()
				Tjp.time_from_start.nsecs = int(1e9/self.position_rate_num)
				self.Tj.points = [Tjp]
				self.traj_pub.publish(self.Tj)
				self.position_rate.sleep()

				# if time.time()+timeout>now:
				# 	return


	def _position_command_thread(self):
		##TODO enum, writeonly wire invalue
		while self._running:
			with self._lock:
				##read wire value
				position_command_wire_packet=self.position_command.TryGetInValue()
				if (not position_command_wire_packet[0]):
					#raise exception
					continue
				if self.command_mode==self.robot_consts['RobotCommandMode']['position_command'] and position_command_wire_packet[1].seqno!=self.command_seqno:
					#update command_seqno
					self.command_seqno=position_command_wire_packet[1].seqno
					while np.linalg.norm(position_command_wire_packet[1].command-self.joint_position)>0.001:
						#break if new value comes in
						#position command wire InValue tuple?
						if self.position_command.InValue[0].seqno!=self.command_seqno:
							break
						self.Tj.header.stamp = rospy.Time()
						Tjp = JointTrajectoryPoint()
						Tjp.positions = position_command_wire_packet[1].command
						vel = (Tjp.positions - self.joint_position) * self.position_rate.sleep_dur.to_sec()
						Tjp.velocities = vel
						Tjp.time_from_start = rospy.Duration()
						Tjp.time_from_start.nsecs = 1
						self.Tj.points = [Tjp]
						self.traj_pub.publish(self.Tj)
						self.position_rate.sleep()

	def execute_trajectory(self,trajectory):
		#clear previous waypoints
		self.Tj.points=[]
		self.Tj.header.stamp = rospy.Time()
		wp_prev=trajectory.waypoints[0].joint_position
		time_diff=trajectory.waypoints[0].time_from_start
		for i in range(len(trajectory.waypoints)):
			try:
				Tjp = JointTrajectoryPoint()
				Tjp.positions = trajectory.waypoints[i].joint_position
				Tjp.velocities = np.array(Tjp.positions - wp_prev) / time_diff
				Tjp.time_from_start = rospy.Duration()
				Tjp.time_from_start.secs = int(trajectory.waypoints[i].time_from_start)
				Tjp.time_from_start.nsecs = int((trajectory.waypoints[i].time_from_start % 1)*1e9)
				self.Tj.points.append(Tjp)
				wp_prev=trajectory.waypoints[i].joint_position
			except:
				traceback.print_exc()
			try:
				time_diff=trajectory.waypoints[i+1].time_from_start-trajectory.waypoints[i].time_from_start

			except:
				pass
		self.traj_pub.publish(self.Tj)

		yield self.trajectory_status_struct


	def start(self):
		self._running=True
		self._pos_command = threading.Thread(target=self._position_command_thread)
		self._pos_command.daemon = True
		self._pos_command.start()

	def close(self):
		self._running = False
		self._pos_command.join()


def main():

	parser = argparse.ArgumentParser(description="Robot Raconteur driver service for Tormach")
	parser.add_argument("--robot-info-file", type=argparse.FileType('r'),default='tormach_robot_default_config.yml',required=True,help="Robot info file (required)")
	args, _ = parser.parse_known_args()
	RRC.RegisterStdRobDefServiceTypes(RRN)

	with args.robot_info_file:
		robot_info_text = args.robot_info_file.read()

	info_loader = InfoFileLoader(RRN)
	robot_info, robot_ident_fd = info_loader.LoadInfoFileFromString(robot_info_text, "com.robotraconteur.robotics.robot.RobotInfo", "robot")

	attributes_util = AttributesUtil(RRN)
	robot_attributes = attributes_util.GetDefaultServiceAttributesFromDeviceInfo(robot_info.device_info)

	tormach_inst=Tormach(robot_info)
	with RR.ServerNodeSetup("Tormach_Service", 11111) as node_setup:

		service_ctx = RRN.RegisterService("tormach_robot","com.robotraconteur.robotics.robot.Robot",tormach_inst)
		service_ctx.SetServiceAttributes(robot_attributes)

		
		print('registered')
		tormach_inst.start()
		print("press ctrl+c to quit")
		rospy.spin()
		# signal.sigwait([signal.SIGTERM,signal.SIGINT])
		
		tormach_inst.close()


if __name__ == "__main__":
	main()