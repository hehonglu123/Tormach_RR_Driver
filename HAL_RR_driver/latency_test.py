from math import sin, pi
import numpy as np
import time
from machinekit import hal
from multiprocessing import Process
import threading
import matplotlib.pyplot as plt

_FINISH=False
def joint_cb():
	global _FINISH, joint_position, joint_position_ts, pos_fb
	while True:
		if _FINISH:
			break
		joint_position.append(pos_fb.get())
		joint_position_ts.append(time.time())
		time.sleep(0.001)

def main():
	global _FINISH, joint_position, joint_position_ts, pos_fb

	# first we need to unlink the ros_control interface pins
	pos_cmd_pin = hal.Pin('hal_hw_interface.joint_6.pos-cmd')
	if pos_cmd_pin.linked:
		pos_cmd_pin.unlink()
	vel_cmd_pin = hal.Pin('hal_hw_interface.joint_6.vel-cmd')
	if vel_cmd_pin.linked:
		vel_cmd_pin.unlink()
	# unlink this pin only if you provide velocity command values
	vel_calc_pin = hal.Pin('joint6_ros_vel_cmd.out')
	if vel_calc_pin.linked:
		vel_calc_pin.unlink()

	# next create references to the HAL signals
	pos_cmd = hal.Signal('joint6_ros_pos_cmd')
	# vel_ff = hal.Signal('joint6_vel_feedforward')
	vel_cmd = hal.Signal('joint6_ros_vel_cmd')
	pos_fb = hal.Signal('joint6_ros_pos_fb')
	vel_fb = hal.Signal('joint6_ros_vel_fb')

	start_pos = pos_fb.get()
	last_pos = start_pos
	period = 0.01
	start_time = time.time()

	joint_position=[]
	joint_position_ts=[]
	joint_position_cmd=[]
	joint_position_cmd_ts=[]
	t = threading.Thread(target=joint_cb)
	t.start()

	now=time.time()
	while time.time()-now<5.:
		current_time = time.time()
		since_start = current_time - start_time
		new_pos = start_pos + sin(since_start * 2 * pi) * 0.05
		vel = (new_pos - last_pos) / period
		last_pos = new_pos
		# in the loop, we set the new cmd values
		vel_cmd.set(vel)
		pos_cmd.set(new_pos)

		joint_position_cmd.append(new_pos)
		joint_position_cmd_ts.append(time.time())
		# and read back the feedback values
		time.sleep(max(period - (time.time() - current_time), 0.001))

	
	_FINISH=True

	t.join()
	plt.plot(np.array(joint_position_ts)-joint_position_ts[0],joint_position)
	plt.plot(np.array(joint_position_cmd_ts)-joint_position_ts[0],joint_position_cmd)
	plt.legend(["joint_pos", "pos_cmd"], loc ="lower right")
	plt.show()

	np.save('latency_j6_pos.npy',np.vstack((np.array(joint_position_ts)-joint_position_ts[0],joint_position)))
	np.save('latency_j6_pos_cmd.npy',np.vstack((np.array(joint_position_cmd_ts)-joint_position_ts[0],joint_position_cmd)))


if __name__ == '__main__':
	main()
