#!/usr/bin/env python
import rospy, time, copy, sys, threading, signal, traceback, argparse, sh
import numpy as np

try:
	import RobotRaconteur as RR
except:
	print('installing dependencies')
	sh.sudo("apt-get","update")
	sh.sudo("apt-get","install","software-properties-common")
	sh.sudo("apt-add-repository", "ppa:robotraconteur/ppa")
	sh.sudo("apt-get","update")
	sh.sudo("apt-get","install","-y","python3-robotraconteur")
	import RobotRaconteur as RR

try:
	from general_robotics_toolbox import *
except:
	sh.sudo.pip3.install("general-robotics-toolbox")
	from general_robotics_toolbox import *

try:
	import RobotRaconteurCompanion as RRC
except:	
	sh.sudo.pip3.install("RobotRaconteurCompanion")
	import RobotRaconteurCompanion as RRC



#ROS libs
from sensor_msgs.msg import JointState
from robot_jog_msgs.srv import *
from robot_jog_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState, JointJog
from std_msgs.msg import Bool

##RR libs
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.RobotRaconteurPythonError import StopIterationException
from RobotRaconteurCompanion.Util.InfoFileLoader import InfoFileLoader
from RobotRaconteurCompanion.Util.DateTimeUtil import DateTimeUtil
from RobotRaconteurCompanion.Util.AttributesUtil import AttributesUtil



class traj_gen(object):
	def __init__(self,pub,traj):
		self._aborted=False
		self.pub=pub
		self.traj=traj
		rospy.Subscriber("/position_trajectory_controller/state", JointTrajectoryControllerState, self._callback)
		rospy.Subscriber("/joint_states", JointState, self._joint_callback)
		self._exe=False
	
	def _callback(self,data):
		self.pos_error=data.error.positions
		self.vel_error=data.error.velocities

	def _joint_callback(self,data):
		self.joint_position=data.position


	def Next(self):
		if not self._exe:
			self.pub.publish(self.traj)
			self._exe=True
			time.sleep(0.5)
		
		# if np.linalg.norm(self.pos_error)<=0.000001:
		if np.linalg.norm(self.joint_position-self.traj.points[-1].positions)<=0.000001:
			raise StopIterationException()
		time.sleep(0.1)
		
	def Abort(self):
		self._aborted=True
		
	def Close(self):
		raise StopIterationException()
	



class Tormach(object):
	def __init__(self,robot_info):

		try:
			#TODO: 
			##turn on robot /hal
			#initialize robot parameters
			self.joint_names=['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
			#initialize ROS node
			rospy.init_node('Tormach_RR_Service', anonymous=True)
			

			self._jog_joint_command_set=False
			self._jog_joint_command_set_timestamp=time.time()
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
			self.pose_dtype=RRN.GetNamedArrayDType("com.robotraconteur.geometry.Pose")
			self.point_dtype=RRN.GetNamedArrayDType("com.robotraconteur.geometry.Point")
			self.quaternion_dtype=RRN.GetNamedArrayDType("com.robotraconteur.geometry.Quaternion")
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
			self.command_mode=self.robot_consts['RobotCommandMode']['halt']

			#initialize ROS Sub for joint callback
			rospy.Subscriber("/joint_states", JointState, self._joint_callback)
			self.jog_joint_ts=time.time()

			#initialize kinematics para
			self.num_joints=len(robot_info.chains[0].joint_numbers)
			self.H=np.transpose(np.array(robot_info.chains[0].H.tolist()))
			self.P=np.array(robot_info.chains[0].P.tolist())
			
			self.robot_def=Robot(self.H,np.transpose(self.P),np.zeros(self.num_joints))
			self.pos_cmd_prev=np.zeros(6)

			###torque reading
			torque_sub=RRN.SubscribeService('rr+tcp://pathpilot:12121/?service=Torque')
			self.torque_state_w = torque_sub.SubscribeWire("torque")

			


		except:
			traceback.print_exc()

	@property
	def device_info(self):
		return self._robot_info.device_info

	@property
	def robot_info(self):
		return self._robot_info
	

	def _joint_callback(self,data):
		###latency check
		# print(rospy.get_rostime()-data.header.stamp)
		self.joint_position=list(data.position)
		self.robot_state_struct.joint_position=self.joint_position
		self.robot_state_struct.ts=np.zeros((1,),dtype=self._date_time_util)
		#fwdkin calculation
		self.robot_state_struct.kin_chain_tcp=np.zeros((1,),dtype=self.pose_dtype)
		transform=fwdkin(self.robot_def,self.joint_position)
		self.robot_state_struct.kin_chain_tcp[0]['position']['x']=transform.p[0]
		self.robot_state_struct.kin_chain_tcp[0]['position']['y']=transform.p[1]
		self.robot_state_struct.kin_chain_tcp[0]['position']['z']=transform.p[2]
		quat=R2q(transform.R)
		self.robot_state_struct.kin_chain_tcp[0]['orientation']['w']=quat[0]
		self.robot_state_struct.kin_chain_tcp[0]['orientation']['x']=quat[1]
		self.robot_state_struct.kin_chain_tcp[0]['orientation']['y']=quat[2]
		self.robot_state_struct.kin_chain_tcp[0]['orientation']['z']=quat[3]
		self.robot_state_struct.command_mode=self.command_mode

		#self.robot_state_struct.joint_effort=self.torque_state_w.InValue

		self.robot_state.OutValue=self.robot_state_struct

	def jog_freespace(self,joint_position,max_velocity,wait):
		while np.linalg.norm(self.joint_position-joint_position)>0.001 and self.command_mode==self.robot_consts['RobotCommandMode']['jog']:
			self.jog_pub.publish(self.JE)
			self.jog_srv(self.joint_names,joint_position)
			self.jog_rate.sleep()

	def jog_joint(self,joint_velocity, timeout, wait):
		self._jog_joint_command_set=True
		self._jog_joint_command_set_timestamp=time.time()
		self._jog_joint_velocity=joint_velocity

	def _jog_joint_thread(self):
		while self._running:
			with self._lock:
				if not self._jog_joint_command_set or self.command_mode!=self.robot_consts['RobotCommandMode']['jog']:
					continue
				if time.time()-self._jog_joint_command_set_timestamp>0.1:
					self._jog_joint_command_set=False
					continue
			
				self.Tj.header.stamp = rospy.Time()
				Tjp = JointTrajectoryPoint()
				###use current joint position here
				Tjp.positions = self.joint_position+self._jog_joint_velocity*0.1
				Tjp.velocities = self._jog_joint_velocity
				Tjp.time_from_start = rospy.Duration()

				Tjp.time_from_start.nsecs = 10#int((time.time()-self.last_call_time)*1e9)
				self.Tj.points = [Tjp]
				self.traj_pub.publish(self.Tj)

			time.sleep(0.01)



	def _position_command_thread(self):
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
					# while np.linalg.norm(position_command_wire_packet[1].command-self.joint_position)>0.001:
					# 	#break if new value comes in
					# 	if self.position_command.InValue[0].seqno!=self.command_seqno:
					# 		break
					self.Tj.header.stamp = rospy.Time()
					Tjp = JointTrajectoryPoint()
					Tjp.positions = position_command_wire_packet[1].command
					vel = (Tjp.positions - self.pos_cmd_prev) / self.position_rate.sleep_dur.to_sec()
					Tjp.velocities = vel
					Tjp.time_from_start = rospy.Duration()
					Tjp.time_from_start.nsecs = 1
					self.Tj.points = [Tjp]
					self.traj_pub.publish(self.Tj)
					self.pos_cmd_prev=Tjp.positions
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
		# self.traj_pub.publish(self.Tj)

		return traj_gen(self.traj_pub,self.Tj)

	def setf_signal(self,signal_name, value):
		#signal_name=<n>, value=1 (True) or 0 (False)
		pub = rospy.Publisher('/hal_io/digital_out_'+signal_name, Bool, queue_size=1)
		temp=Bool()
		temp.data=int(value[0])
		pub.publish(temp)

	def getf_signal(self,signal_name):
		#signal_name=<n>
		data_in = rospy.wait_for_message('/hal_io/digital_in_'+signal_name, Bool)
		return np.float64(int(data_in.data))

	def start(self):
		self._running=True
		self._pos_command = threading.Thread(target=self._position_command_thread)
		self._pos_command.daemon = True
		self._pos_command.start()

		
		self._jog_command = threading.Thread(target=self._jog_joint_thread)
		self._jog_command.daemon = True
		self._jog_command.start()

	def close(self):
		self._running = False
		self._pos_command.join()
		self._jog_command.join()

class create_gripper(object):
	def __init__(self, tool_info):
		self.device_info = tool_info.device_info
		self.tool_info = tool_info
		self.pub = rospy.Publisher('/hal_io/digital_out_1', Bool, queue_size=1)
	def open(self):
		temp=Bool()
		temp.data=False
		self.pub.publish(temp)
	def close(self):
		temp=Bool()
		temp.data=True
		self.pub.publish(temp)

def main():

	parser = argparse.ArgumentParser(description="Robot Raconteur driver service for Tormach")
	parser.add_argument("--robot-info-file", type=argparse.FileType('r'),default='config/tormach_za06_robot_default_config.yml',required=False,help="Robot info file (required)")
	parser.add_argument("--tool-info-file", type=argparse.FileType('r'),default="config/tormach_gripper_default_config.yml",required=False,help="Tool info file (required)")

	args, _ = parser.parse_known_args()
	RRC.RegisterStdRobDefServiceTypes(RRN)

	with args.robot_info_file:
		robot_info_text = args.robot_info_file.read()

	info_loader = InfoFileLoader(RRN)
	robot_info, robot_ident_fd = info_loader.LoadInfoFileFromString(robot_info_text, "com.robotraconteur.robotics.robot.RobotInfo", "robot")

	attributes_util = AttributesUtil(RRN)
	robot_attributes = attributes_util.GetDefaultServiceAttributesFromDeviceInfo(robot_info.device_info)

	######temp info_loader modification
	uuid_dtype=RRN.GetNamedArrayDType('com.robotraconteur.uuid.UUID')
	robot_info.chains[0].tcp_max_velocity = np.zeros((1,),dtype=robot_info.chains[0].tcp_max_velocity.dtype)
	robot_info.chains[0].tcp_reduced_max_velocity = np.zeros((1,),dtype=robot_info.chains[0].tcp_reduced_max_velocity.dtype)
	robot_info.chains[0].tcp_max_acceleration = np.zeros((1,),dtype=robot_info.chains[0].tcp_max_acceleration.dtype)
	robot_info.chains[0].tcp_reduced_max_acceleration = np.zeros((1,),dtype=robot_info.chains[0].tcp_reduced_max_acceleration.dtype)

	robot_info.chains[0].kin_chain_identifier.uuid = np.zeros((1,),dtype=uuid_dtype)
	robot_info.chains[0].flange_identifier.uuid = np.zeros((1,),dtype=uuid_dtype)

	for j in robot_info.joint_info:
		j.passive=False
		j.joint_identifier.uuid = np.zeros((1,),dtype=uuid_dtype)
	#########

	with args.tool_info_file:
		tool_info_text = args.tool_info_file.read()
	info_loader = InfoFileLoader(RRN)
	tool_info, camera_ident_fd = info_loader.LoadInfoFileFromString(tool_info_text, "com.robotraconteur.robotics.tool.ToolInfo", "tool")
	
	attributes_util = AttributesUtil(RRN)
	tool_attributes = attributes_util.GetDefaultServiceAttributesFromDeviceInfo(tool_info.device_info)
	gripper_inst=create_gripper(tool_info)

	with RR.ServerNodeSetup("tormach_service", 11111) as node_setup:
		print('Initilizing')
		tormach_inst=Tormach(robot_info)
		robot_service_ctx = RRN.RegisterService("tormach_robot","com.robotraconteur.robotics.robot.Robot",tormach_inst)
		robot_service_ctx.SetServiceAttributes(robot_attributes)
		
		gripper_service_ctx = RRN.RegisterService("tormach_gripper","com.robotraconteur.robotics.tool.Tool",gripper_inst)
		gripper_service_ctx.SetServiceAttributes(tool_attributes)
		
		print('registered')
		tormach_inst.start()
		print("press ctrl+c to quit")
		rospy.spin()
		# signal.sigwait([signal.SIGTERM,signal.SIGINT])
		
		tormach_inst.close()


	

	

   
	


if __name__ == "__main__":
	main()
