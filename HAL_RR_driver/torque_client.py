
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import time, threading
import numpy as np

temp=RRN.ConnectService('rr+tcp://pathpilot:12121/?service=Torque')
