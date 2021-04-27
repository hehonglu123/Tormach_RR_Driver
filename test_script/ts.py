#!/usr/bin/env python3
import rospy
import numpy as np
import time, sys
import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointPublisher:
	def __init__(self):
		rospy.init_node('jogging_node', anonymous=True)
		self._joint_state = JointState()
		self._listener()
		self._publisher()
		self._create_trajectory()
	def _listener(self):
		self._joint_state = rospy.wait_for_message('/joint_states', JointState)

	def _publisher(self):
		self._pub = rospy.Publisher(
			'/position_trajectory_controller/command',
			JointTrajectory,
			queue_size=1,
		)
	def _create_trajectory(self):
		Tj = JointTrajectory()
		Tj.joint_names = self._joint_state.name
		rate = rospy.Rate(20)  # 10hz
		now = time.time()
		last_pos = self._joint_state.position[5]
		while time.time() - now < 5.0:
			Tj.header.stamp = rospy.Time()
			Tjp = JointTrajectoryPoint()
			Tjp.positions = list(self._joint_state.position)
			Tjp.positions[5] = self._joint_state.position[5] + np.sin(
				time.time()
			)
			vel = (Tjp.positions[5] - last_pos) * rate.sleep_dur.to_sec()
			last_pos = Tjp.positions[5]
			Tjp.velocities = [0.0] * 6
			Tjp.velocities[5] = vel
			print(Tjp.positions)
			Tjp.time_from_start = rospy.Duration()
			Tjp.time_from_start.nsecs = 1
			Tj.points = [Tjp]
			self._pub.publish(Tj)
			rate.sleep()


def main():
	_ = JointPublisher()


if __name__ == '__main__':
	main()