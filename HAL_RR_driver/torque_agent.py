import numpy as np
import sh, os, time, copy, sys, threading, signal, traceback, argparse
from machinekit import hal

try:
	from general_robotics_toolbox import *
except:
	sh.sudo.pip3.install("general-robotics-toolbox")
	sh.sudo.pip3.install("RobotRaconteurCompanion")

from general_robotics_toolbox import *
#RR libs
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s


minimal_create_interface="""
service experimental.torque_reading
object torque_obj
	wire double torque [readonly]
end object
"""

class Torque(object):
	def __init__(self):
		try:
			#initialize kinematics para
			self.num_joints=6
			self._lock=threading.Lock()

			###HAL signal connects
			self.torque_fb=[]
			for i in range(self.num_joints):

				self.torque_fb.append(hal.Pin('lcec.0.'+str(i)+'.torque-actual-value'))
				self.I2tau_conversion=np.array([0.3585,0.239,0.1016,0.03328,0.02592,-0.016])

		except:
			traceback.print_exc()

	def _robot_state_update(self):
		while self._running:
			with self._lock:
				try:
					self.joint_torque=np.zeros(self.num_joints)
					for i in range(self.num_joints):
						self.joint_torque[i]=self.torque_fb[i].get()*self.I2tau_conversion[i]

					self.torque.OutValue=self.joint_torque

				except:
					traceback.print_exc()

			time.sleep(0.001)




	def start(self):
		self._running=True

		self._state_update = threading.Thread(target=self._robot_state_update)
		self._state_update.daemon = True
		self._state_update.start()

	def close(self):
		self._running = False
		self._pos_command.join()
		self._jog_command.join()
		self._state_update.join()


def main():


	with RR.ServerNodeSetup("torque_service", 12121) as node_setup:

		RRN.RegisterServiceType(minimal_create_interface)

		torque_inst=Torque()
		print('registered')
		torque_inst.start()
		RRN.RegisterService("Torque","experimental.torque_reading.torque_obj",torque_inst)
		print("press ctrl+c to quit")
		signal.sigwait([signal.SIGTERM,signal.SIGINT])




if __name__ == "__main__":
	main()