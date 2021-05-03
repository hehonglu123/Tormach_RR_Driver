from RobotRaconteur.Client import *
import time
import numpy as np
import matplotlib.pyplot as plt

c = RRN.ConnectService('rr+tcp://localhost:11111?service=tormach_robot')

cmd_w = c.position_command.Connect()
state_w = c.robot_state.Connect()
state_w.WaitInValueValid()

robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", c)
halt_mode = robot_const["RobotCommandMode"]["halt"]
position_mode = robot_const["RobotCommandMode"]["position_command"]
jog_mode = robot_const["RobotCommandMode"]["jog"]

RobotJointCommand = RRN.GetStructureType("com.robotraconteur.robotics.robot.RobotJointCommand",c)

c.command_mode = halt_mode
time.sleep(0.1)
c.command_mode = jog_mode

c.jog_freespace(np.zeros(6),np.ones(6), True)
print('jog complete')



c.command_mode = halt_mode
time.sleep(0.1)
c.command_mode = position_mode




command_seqno = 1

joint_positions_history=[]
now=time.time()
while time.time()-now<10:
    t = time.time()

    robot_state = state_w.InValue

    command_seqno += 1
    joint_cmd1 = RobotJointCommand()
    joint_cmd1.seqno = command_seqno
    joint_cmd1.state_seqno = robot_state.seqno
    cmd = np.array([0,0,0,0,0,1])*np.sin(t)
    joint_cmd1.command = cmd
    cmd_w.OutValue = joint_cmd1
    joint_positions_history.append(state_w.InValue.joint_position[-1])    

plt.plot(joint_positions_history)
plt.title('sin_traj')
plt.show()