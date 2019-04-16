from geometry_msgs.msg import Pose, Quaternion, Point
import tf.transformations as tft
import ipdb
import graspit_commander
from math import pi


ipdb.set_trace()

gc = graspit_commander.GraspitCommander()
gc.clearWorld()
q = tft.quaternion_from_euler(0, 0.5*pi, 0)
gc.importRobot('fetch_gripper')
gc.importGraspableBody("longBox", Pose(Point(0, 0, 0), Quaternion(*q)))
gc.importObstacle("floor", Pose(Point(-1, -1, -0.09), Quaternion(0, 0, 0, 1)))

ipdb.set_trace()
grasps = gc.planGrasps()
grasps = grasps.grasps

ipdb.set_trace()
