#!/usr/bin/env python3
import rospy
import actionlib
import numpy as np
import time
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import (
	FollowJointTrajectoryGoal,
	FollowJointTrajectoryAction,
)

start_joint = []

class JointPublisher:
	def __init__(self):
		rospy.init_node('jogging_node', anonymous=True)
		self._joint_state = JointState()
		self._listener()
		self._action_client()
		self._publisher()
		self._create_action_trajectory()
	def _listener(self):
		self._joint_state = rospy.wait_for_message('/joint_states', JointState)
	def _action_client(self):
		self._client = actionlib.SimpleActionClient(
			'/position_trajectory_controller/follow_joint_trajectory',
			FollowJointTrajectoryAction,
		)
		self._client.wait_for_server()
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
		while time.time() - now < 20.0:
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
			FJTG = FollowJointTrajectoryGoal()
			FJTG.trajectory = Tj
			self._pub.publish(Tj)
			rate.sleep()

	def _create_action_trajectory(self):
		Tj = JointTrajectory()
		Tj.joint_names = self._joint_state.name
		length = 500
		for i in range(length):
			Tjp = JointTrajectoryPoint()
			Tjp.positions = list(self._joint_state.position)
			Tjp.positions[5] = self._joint_state.position[5] + np.sin(
				-i / length
			)
			Tjp.velocities = [0.0] * 6
			Tjp.velocities[5] = 2.0
			print(Tjp.positions)
			Tjp.time_from_start = rospy.Duration(i / length)
			Tj.points.append(Tjp)

		FJTG = FollowJointTrajectoryGoal()
		FJTG.trajectory = Tj
		self._client.send_goal(FJTG)
		ret=self._client.wait_for_result(rospy.Duration.from_sec(2.0))
		print(ret)

def main():
	_ = JointPublisher()


if __name__ == '__main__':
	main()