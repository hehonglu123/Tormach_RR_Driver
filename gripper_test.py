
from RobotRaconteur.Client import *

url_gripper='rr+local:///?nodeid=a888cd86-107e-4789-9918-8043c08432f7&service=tormach_gripper'
tool=RRN.ConnectService(url_gripper)
tool.open()
time.sleep(2)
tool.close()