#!/usr/bin/env python


import copy
import actionlib
import rospy

from math import sin, cos
from moveit_python import (MoveGroupInterface,
						   PlanningSceneInterface,
						   PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from gazebo_msgs.srv import GetModelState

from geometry_msgs.msg import Pose, Quaternion, Point
import tf.transformations as tft
import ipdb
import graspit_commander
from math import pi

# Tools for grasping
class GraspingClient(object):

	def __init__(self):
		self.scene = PlanningSceneInterface("base_link")
		self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)
		self.move_group = MoveGroupInterface("arm", "base_link")

		find_topic = "basic_grasping_perception/find_objects"
		rospy.loginfo("Waiting for %s..." % find_topic)
		self.find_client = actionlib.SimpleActionClient(find_topic, FindGraspableObjectsAction)
		self.find_client.wait_for_server()

	def updateScene(self):
		# find objects
		goal = FindGraspableObjectsGoal()
		goal.plan_grasps = True
		import pdb; pdb.set_trace()
		self.find_client.send_goal_and_wait(goal)
		# self.find_client.send_goal(goal)
		# self.find_client.wait_for_result(rospy.Duration(5.0))
		find_result = self.find_client.get_result()


		# remove previous objects
		for name in self.scene.getKnownCollisionObjects():
			self.scene.removeCollisionObject(name, False)
		for name in self.scene.getKnownAttachedObjects():
			self.scene.removeAttachedObject(name, False)
		self.scene.waitForSync()

		# insert objects to scene
		idx = -1
		for obj in find_result.objects:
			idx += 1
			obj.object.name = "object%d"%idx
			self.scene.addSolidPrimitive(obj.object.name,
										 obj.object.primitives[0],
										 obj.object.primitive_poses[0],
										 wait = False)

		for obj in find_result.support_surfaces:
			# extend surface to floor, and make wider since we have narrow field of view
			height = obj.primitive_poses[0].position.z
			obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0],
											1.5,  # wider
											obj.primitives[0].dimensions[2] + height]
			obj.primitive_poses[0].position.z += -height/2.0

			# add to scene
			self.scene.addSolidPrimitive(obj.name,
										 obj.primitives[0],
										 obj.primitive_poses[0],
										 wait = False)

		self.scene.waitForSync()

		# store for grasping
		self.objects = find_result.objects
		self.surfaces = find_result.support_surfaces

	def getGraspableCube(self):
		graspable = None
		for obj in self.objects:
			# need grasps
			if len(obj.grasps) < 1:
				continue
			# check size
			if obj.object.primitives[0].dimensions[0] < 0.05 or \
			   obj.object.primitives[0].dimensions[0] > 0.07 or \
			   obj.object.primitives[0].dimensions[0] < 0.05 or \
			   obj.object.primitives[0].dimensions[0] > 0.07 or \
			   obj.object.primitives[0].dimensions[0] < 0.05 or \
			   obj.object.primitives[0].dimensions[0] > 0.07:
				continue
			# has to be on table
			if obj.object.primitive_poses[0].position.z < 0.5:
				continue
			return obj.object, obj.grasps
		# nothing detected
		return None, None

	def getSupportSurface(self, name):
		for surface in self.support_surfaces:
			if surface.name == name:
				return surface
		return None

	def getPlaceLocation(self):
		pass

	def pick(self, block, grasps):
		success, pick_result = self.pickplace.pick_with_retry(block.name,
															  grasps,
															  support_name=block.support_surface,
															  scene=self.scene)
		self.pick_result = pick_result
		return success

	def place(self, block, pose_stamped):
		places = list()
		l = PlaceLocation()
		l.place_pose.pose = pose_stamped.pose
		l.place_pose.header.frame_id = pose_stamped.header.frame_id

		# copy the posture, approach and retreat from the grasp used
		l.post_place_posture = self.pick_result.grasp.pre_grasp_posture
		l.pre_place_approach = self.pick_result.grasp.pre_grasp_approach
		l.post_place_retreat = self.pick_result.grasp.post_grasp_retreat
		places.append(copy.deepcopy(l))
		# create another several places, rotate each by 360/m degrees in yaw direction
		m = 16 # number of possible place poses
		pi = 3.141592653589
		for i in range(0, m-1):
			l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 2 * pi / m)
			places.append(copy.deepcopy(l))

		success, place_result = self.pickplace.place_with_retry(block.name,
																places,
																scene=self.scene)
		return success

	def tuck(self):
		joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
				  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
		pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
		while not rospy.is_shutdown():
			result = self.move_group.moveToJointPosition(joints, pose, 0.02)
			if result.error_code.val == MoveItErrorCodes.SUCCESS:
				return


def plan(joint_values=None, eef_pose=None, position=None, orientation=None):
	plan = None
	if joint_values:
		pass
	elif eef_pose:
		if type(eef_pose) is Pose:
			plan = self.arm_commander_group.plan(joints=eef_pose)
		else:
			pass
	elif position is not None and orientation is not None:
		## end-effector:
		pose_goal = Pose()
		pose_goal.orientation.w = orientation[0]
		pose_goal.orientation.x = orientation[0]
		pose_goal.orientation.y = orientation[0]
		pose_goal.orientation.z = orientation[0]
		pose_goal.position.x = position[0]
		pose_goal.position.y = position[1]
		pose_goal.position.z = position[2]
		plan = self.arm_commander_group.plan(joints=pose_goal)
	else:
		raise TypeError("Parameter types not supported!")
	return plan

def display_grasp_pose_in_rviz(end_effector_poses, reference_frame):
	import tf_manager
	my_tf_manager = tf_manager.TFManager()
	tf_list = []
	for i, p in enumerate(end_effector_poses):
		ps = PoseStamped()
		ps.pose = p
		ps.header.frame_id = reference_frame
		my_tf_manager.add_tf('G_{}'.format(i), ps)
		my_tf_manager.broadcast_tfs()

def get_ik(position, orientation, planner_time_limit=0.5):

	gripper_pose_stamped = PoseStamped()
	gripper_pose_stamped.header.frame_id = 'map'
	gripper_pose_stamped.header.stamp = rospy.Time.now()
	gripper_pose_stamped.pose = Pose(Point(*position), Quaternion(*orientation))

	service_request = PositionIKRequest()
	service_request.group_name = 'arm'
	service_request.ik_link_name = 'wrist_roll_joint'
	service_request.pose_stamped = gripper_pose_stamped
	service_request.timeout.secs = planner_time_limit
	service_request.avoid_collisions = True

	try:
		resp = self.compute_ik(ik_request=service_request)
		return resp
	except rospy.ServiceException, e:
		print("Service call failed: %s" % e)

if __name__ == "__main__":
	# Create a node
	rospy.init_node("demo")

	# Make sure sim time is working
	while not rospy.Time.now():
		pass

	# Setup clients
	grasping_client = GraspingClient()


	####################################################################


	import graspit_commander
	import tf_conversions
	import numpy as np
	from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
	import tf
	import moveit_commander as mc

	arm_commander_group = mc.MoveGroupCommander('arm')
	robot = mc.RobotCommander()
	scene = mc.PlanningSceneInterface()

	from fetch_api import Base 
	from fetch_api import Torso
	from fetch_api import Gripper

	base = Base()
	torso = Torso()
	gripper = Gripper()

	### remove all objects
	for obj_name in scene.get_known_object_names():
		scene.remove_world_object(obj_name)

	arm_commander_group.set_planner_id('RRTConnectkConfigDefault')

	# raise torso
	torso.set_height(0.4)
	# tuck arm
	grasping_client.tuck()
	# move to pregrasp preparation pose, this reduces hardness in future grasping
	import ipdb; ipdb.set_trace()
	# Manually find joint values that is reachable and above the table
	# plan = arm_commander_group.plan(joints=[0.6203397682450991, -0.5799071015665476, -1.3685228906548907, 1.587422300097166, 1.358637106693556, 1.3876913608232933, 1.6836856745017554])
	plan = arm_commander_group.plan(joints=[-0.0397012090527733, -0.1612715621777019, 0.23767634578330998, 2.1503041586123572, 3.139554873489182, 2.0222407199230616, -0.25277644099574204])
	arm_commander_group.execute(plan)

	# open gripper
	gripper.open()
	# move forward
	base.go_forward(0.78)

	get_model_state_service_name = '/gazebo/get_model_state'
	rospy.wait_for_service(get_model_state_service_name)
	get_model_state_service_proxy = rospy.ServiceProxy(get_model_state_service_name, GetModelState)

	cube_state = get_model_state_service_proxy("demo_cube", "base_link")
	cube_pose_in_base_link = cube_state.pose

	table_state = get_model_state_service_proxy("table1", "base_link")
	table_pose_in_base_link = table_state.pose

	load = True

	if load:
		import pickle
		grasps = pickle.load(open("/home/jxu/.ros/grasps_long.pk", "rb"))
	else:
		gc = graspit_commander.GraspitCommander()
		gc.clearWorld()
		q = tft.quaternion_from_euler(0, 0.5*pi, 0)
		gc.importRobot('fetch_gripper')
		gc.importGraspableBody("longBox", Pose(Point(0, 0, 0), Quaternion(*q)))
		gc.importObstacle("floor", Pose(Point(-1, -1, -0.09), Quaternion(0, 0, 0, 1)))
		grasps = gc.planGrasps()
		grasps = grasps.grasps


	# convert grasp into base_link
	grasps_in_base_link = []
	for g in grasps:
		grasp_in_cube = tf_conversions.toMatrix(tf_conversions.fromMsg(g.pose)) # matrix
		g_new = tf_conversions.toMatrix(tf_conversions.fromMsg(cube_pose_in_base_link)).dot(grasp_in_cube)
		grasps_in_base_link.append(g_new)

	# convert grasp matrix to a pose
	g_poses = []
	for g in grasps_in_base_link:
		g_poses.append(tf_conversions.toMsg(tf_conversions.fromMatrix(g)))

	box_pose = PoseStamped()
	box_pose.header.frame_id = "base_link"
	box_pose.pose = cube_pose_in_base_link
	box_name = 'box'
	scene.add_box(box_name, box_pose, size=(0.05, 0.05, 0.18))

	table_pose = PoseStamped()
	table_pose.header.frame_id = "base_link"
	table_pose.pose = table_pose_in_base_link
	table_name = 'cafe_table'
	scene.add_mesh(table_name, table_pose, '/home/jxu/.gazebo/models/cafe_table/meshes/cafe_table.ply', size=(0.026, 0.026, 0.025))

	# now check all grasp poses to see which one is reachable
	g_final = None
	for i, g_pose in enumerate(g_poses):
		g_pose.position.x -= 0.01
		plan = arm_commander_group.plan(g_pose)
		if len(plan.joint_trajectory.points) != 0:
			g_final = g_pose
			print("the {}-th grasp pose is reachable!".format(i))
			break

	if g_final is None:
		print("no grasp pose is reachable")
		ipdb.set_trace()
	else:
		### move to grasp pose
		ipdb.set_trace()
		arm_commander_group.execute(plan)

	### close gripper
	ipdb.set_trace()
	gripper.close()


	### remove object from the scene
	ipdb.set_trace()
	# Manually find joint values that is reachable and above the table
	scene.remove_world_object("box")
	### plan to move up
	ipdb.set_trace()
	plan = arm_commander_group.plan(joints=[-0.024687696845566265, -0.17734369399050465, 0.16421007378484553, 1.8713637471944633, 3.1388332971203, 2, -0.12539539477555284])
	
	### move up and go back
	ipdb.set_trace()
	arm_commander_group.execute(plan)
	base.go_forward(-0.80)