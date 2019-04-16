#! /usr/bin/env python

import fetch_api
import rospy

import math 

def wait_for_time():
	"""Wait for simulated time to begin.
	"""
	while rospy.Time().now().to_sec() == 0:
		pass

if __name__ == "__main__":
	rospy.init_node('motion_demo')
	wait_for_time()	

	# move forwards
	base = fetch_api.Base()
	base.go_forward(1.5)

	# hand pan and tilt
	MIN_PAN = -math.pi / 2
	MAX_PAN = math.pi / 2
	MIN_TILT = -math.pi / 2
	MAX_TILT = math.pi / 4

	head = fetch_api.Head()
	head.pan_tilt(0, 0)
	head.pan_tilt(0, MAX_TILT)
	head.pan_tilt(0, MIN_TILT)
	head.pan_tilt(0, 0)
	head.pan_tilt(MAX_PAN, 0)
	head.pan_tilt(MIN_PAN, 0)
	head.pan_tilt(0, 0)

	# torso
	MIN_HEIGHT = 0.0
	MAX_HEIGHT = 0.4
	torso = fetch_api.Torso()
	torso.set_height(MAX_HEIGHT)
	# torso.set_height(MIN_HEIGHT)

	# arm
	arm = fetch_api.Arm()
	# arm.move_to_joints(fetch_api.ArmJoints.from_list([1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]))
	arm.move_to_joints(fetch_api.ArmJoints.from_list([0, 0, 0, 0, 0, 0, 0]))

	# gripper
	gripper = fetch_api.Gripper()
	gripper.close()
	gripper.open()






