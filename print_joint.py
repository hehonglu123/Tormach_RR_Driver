#Simple example Robot Raconteur Robot Print joint client
from RobotRaconteur.Client import *
import numpy as np
import time,traceback, sys, yaml
from importlib import import_module


####################Start Service and robot setup

robot_sub=RRN.SubscribeService('rr+tcp://localhost:11111?service=tormach_robot')
robot=robot_sub.GetDefaultClientWait(1)

state_w = robot_sub.SubscribeWire("robot_state")

print(robot.robot_info.joint_info[0].passive)
print(robot.robot_info.device_info.device.name)


time.sleep(0.5)
robot_state_wire=state_w.TryGetInValue()
print("wire value set: ",robot_state_wire[0])
robot_state = robot_state_wire[1]
print("kin_chain_tcp: ", robot_state.kin_chain_tcp)
print("robot_joints: ", robot_state.joint_position)

