import numpy as np
import sh, os, time, copy, sys, threading, signal, traceback, argparse
from machinekit import hal

try:
	sh.dpkg_query("-W","python3-robotraconteur")
	from general_robotics_toolbox import *
	import RobotRaconteurCompanion as RRC

except:
	sh.sudo("apt-get","update")
	sh.sudo("apt-get","install","software-properties-common")
	sh.sudo("apt-add-repository", "ppa:robotraconteur/ppa")
	sh.sudo("apt-get","update")
	sh.sudo("apt-get","install","-y","python3-robotraconteur")
	sh.sudo.pip3.install("general-robotics-toolbox")
	sh.sudo.pip3.install("RobotRaconteurCompanion")

from general_robotics_toolbox import *
#RR libs
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
from RobotRaconteurCompanion.Util.InfoFileLoader import InfoFileLoader
from RobotRaconteurCompanion.Util.DateTimeUtil import DateTimeUtil
from RobotRaconteurCompanion.Util.AttributesUtil import AttributesUtil
from RobotRaconteur.RobotRaconteurPythonError import StopIterationException



class Tormach(object):
	def __init__(self,robot_info):
		try:
			#initialize robot parameters
			self.joint_names=['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
			#initialize ROS node			

			self._jog_joint_command_set=False
			self._jog_joint_command_set_timestamp=time.time()


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

			#initialize kinematics para
			self.num_joints=len(robot_info.chains[0].joint_numbers)
			self.H=np.transpose(np.array(robot_info.chains[0].H.tolist()))
			self.P=np.array(robot_info.chains[0].P.tolist())
			
			self.robot_def=Robot(self.H,np.transpose(self.P),np.zeros(self.num_joints))

			###HAL signal connects
			# first we need to unlink the ros_control interface pins
			self.pos_cmd_pins=[]
			self.vel_cmd_pins=[]
			self.vel_calc_pins=[]
			self.pos_cmd=[]
			self.vel_cmd=[]
			self.pos_fb=[]
			self.vel_fb=[]
			for i in range(self.num_joints):
				self.pos_cmd_pins.append(hal.Pin('hal_hw_interface.joint_'+str(i+1)+'.pos-cmd'))
				if self.pos_cmd_pins[-1].linked:
					self.pos_cmd_pins[-1].unlink()

				self.vel_cmd_pins.append(hal.Pin('hal_hw_interface.joint_'+str(i+1)+'.vel-cmd'))
				if self.vel_cmd_pins[-1].linked:
					self.vel_cmd_pins[-1].unlink()

				# unlink this pin only if you provide velocity command values

				self.vel_calc_pins.append(hal.Pin('joint'+str(i+1)+'_ros_vel_cmd.out'))
				if self.vel_calc_pins[-1].linked:
					self.vel_calc_pins[-1].unlink()

				# next create references to the HAL signals
				self.pos_cmd.append(hal.Signal('joint'+str(i+1)+'_ros_pos_cmd'))
				self.vel_cmd.append(hal.Signal('joint'+str(i+1)+'_ros_vel_cmd'))
				self.pos_fb.append(hal.Signal('joint'+str(i+1)+'_ros_pos_fb'))
				self.vel_fb.append(hal.Signal('joint'+str(i+1)+'_ros_vel_fb'))
		except:
			traceback.print_exc()

	@property
	def device_info(self):
		return self._robot_info.device_info

	@property
	def robot_info(self):
		return self._robot_info
	

	def _robot_state_update(self):
		while self._running:
			with self._lock:
				try:
					self.joint_position=[]
					self.joint_velocity=[]
					for i in range(self.num_joints):
						self.joint_position.append(self.pos_fb[i].get())
						self.joint_velocity.append(self.vel_fb[i].get())

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
					self.robot_state.OutValue=self.robot_state_struct
					time.sleep(0.001)
				except:
					traceback.print_exc()


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
				
				for i in range(self.num_joints):
					self.pos_cmd[i].set(self.joint_position[i]+self._jog_joint_velocity[i]*0.1)
					self.vel_cmd[i].set(self._jog_joint_velocity[i])

				time.sleep(0.1)



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
					while np.linalg.norm(position_command_wire_packet[1].command-self.joint_position)>0.001:
						#break if new value comes in
						#position command wire InValue tuple?
						if self.position_command.InValue[0].seqno!=self.command_seqno:
							break

						for i in range(self.num_joints):
							self.pos_cmd[i].set(position_command_wire_packet[1].command[i])
							self.vel_cmd[i].set(position_command_wire_packet[1].command[i]/0.001)

						time.sleep(0.001)


	def setf_signal(signal_name, value):
		return

	def getf_signal(signal_name):
		return

	def start(self):
		self._running=True
		self._pos_command = threading.Thread(target=self._position_command_thread)
		self._pos_command.daemon = True
		self._pos_command.start()

		
		self._jog_command = threading.Thread(target=self._jog_joint_thread)
		self._jog_command.daemon = True
		self._jog_command.start()

		self._state_update = threading.Thread(target=self._robot_state_update)
		self._state_update.daemon = True
		self._state_update.start()

	def close(self):
		self._running = False
		self._pos_command.join()
		self._jog_command.join()
		self._state_update.join()


def main():

	parser = argparse.ArgumentParser(description="Robot Raconteur driver service for Tormach")
	parser.add_argument("--robot-info-file", type=argparse.FileType('r'),default='../tormach_za06_robot_default_config.yml',required=False,help="Robot info file (required)")

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
	tormach_inst=Tormach(robot_info)


	with RR.ServerNodeSetup("tormach_service", 11111) as node_setup:

		robot_service_ctx = RRN.RegisterService("tormach_robot","com.robotraconteur.robotics.robot.Robot",tormach_inst)
		robot_service_ctx.SetServiceAttributes(robot_attributes)
		

		print('registered')
		tormach_inst.start()
		print("press ctrl+c to quit")
		signal.sigwait([signal.SIGTERM,signal.SIGINT])
		
		tormach_inst.close()
		os._exit(1)




if __name__ == "__main__":
	main()