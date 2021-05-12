import RobotRaconteur as RR
from RobotRaconteur.RobotRaconteurPythonUtil import PackMessageElement
RRN=RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
from RobotRaconteurCompanion.Util.InfoFileLoader import InfoFileLoader
from RobotRaconteurCompanion.Util.DateTimeUtil import DateTimeUtil
from RobotRaconteurCompanion.Util.AttributesUtil import AttributesUtil

import numpy as np

RRC.RegisterStdRobDefServiceTypes(RRN)

uuid_dtype=RRN.GetNamedArrayDType('com.robotraconteur.uuid.UUID')

with open("tormach_za06_robot_default_config.yml") as f:
    robot_info_text = f.read()

info_loader = InfoFileLoader(RRN)
robot_info, robot_ident_fd = info_loader.LoadInfoFileFromString(robot_info_text, "com.robotraconteur.robotics.robot.RobotInfo", "robot")

robot_info.chains[0].tcp_max_velocity = np.zeros((1,),dtype=robot_info.chains[0].tcp_max_velocity.dtype)
robot_info.chains[0].tcp_reduced_max_velocity = np.zeros((1,),dtype=robot_info.chains[0].tcp_reduced_max_velocity.dtype)
robot_info.chains[0].tcp_max_acceleration = np.zeros((1,),dtype=robot_info.chains[0].tcp_max_acceleration.dtype)
robot_info.chains[0].tcp_reduced_max_acceleration = np.zeros((1,),dtype=robot_info.chains[0].tcp_reduced_max_acceleration.dtype)

robot_info.chains[0].kin_chain_identifier.uuid = np.zeros((1,),dtype=uuid_dtype)
robot_info.chains[0].flange_identifier.uuid = np.zeros((1,),dtype=uuid_dtype)

for j in robot_info.joint_info:
    j.passive=False
    j.joint_identifier.uuid = np.zeros((1,),dtype=uuid_dtype)

print(robot_info)