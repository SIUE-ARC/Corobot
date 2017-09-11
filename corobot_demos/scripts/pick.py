#!/usr/bin/env python
import sys
import rospy
import rosparam
import random
import math

from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Vector3Stamped, Vector3
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

def calculate_grasp(header, target_position):
    print header
    print target_position
    grasp = Grasp()
    grasp.id = "grasp"

    # Grasped and not grasped information
    pre_grasp = JointTrajectory()
    pre_grasp.joint_names = ["gripper_joint"]
    pre_grasp.points = [JointTrajectoryPoint(positions=[0.015],effort=[5.0])]
    grasp.pre_grasp_posture = pre_grasp

    post_grasp = JointTrajectory()
    post_grasp.joint_names = ["gripper_joint"]
    post_grasp.points = [JointTrajectoryPoint(positions=[0.01],effort=[5.0])]
    grasp.grasp_posture = post_grasp

    # Approach and retreat paths
    forward_vector = Vector3Stamped()
    forward_vector.header.frame_id = robot.arm.get_end_effector_link()
    forward_vector.vector = Vector3(1, 0, 0)

    up_vector = Vector3Stamped()
    up_vector.header.frame_id = robot.arm.get_end_effector_link()
    up_vector.vector = Vector3(-1, 0, 0)

    approach_translation = GripperTranslation()
    approach_translation.direction = forward_vector
    approach_translation.desired_distance = 0.03
    approach_translation.min_distance = 0.01

    retreat_translation = GripperTranslation()
    retreat_translation.direction = up_vector
    retreat_translation.desired_distance = 0.03
    retreat_translation.min_distance = 0.01

    grasp.pre_grasp_approach = approach_translation
    grasp.post_grasp_retreat = retreat_translation
    grasp.post_place_retreat = retreat_translation

    # TODO Calcuate possible grasp orientations
    grasp.grasp_pose.header = header
    grasp.grasp_pose.pose.position = target_position
    grasp.grasp_pose.pose.orientation.x = 0
    grasp.grasp_pose.pose.orientation.y = 0
    grasp.grasp_pose.pose.orientation.z = 0
    grasp.grasp_pose.pose.orientation.w = 1
    grasp.grasp_quality = 1.0
    grasp.allowed_touch_objects.append("box")

    return grasp


roscpp_initialize(sys.argv)
rospy.init_node('pick_demo')

scene = PlanningSceneInterface()
robot = RobotCommander()
robot.arm.set_goal_tolerance(0.03)

rospy.sleep(1.0)

p = PoseStamped()
p.header.frame_id = 'arm_base_link'
p.pose.position.y = 0
p.pose.position.x = 0.27
p.pose.position.z = 0.1

p.pose.orientation.x = 0
p.pose.orientation.y = 0
p.pose.orientation.z = 0
p.pose.orientation.w = 1

scene.remove_world_object("box")
scene.add_box("box", p, (0.02, 0.02, 0.05))

rospy.sleep(1.0)

robot.gripper.set_named_target("open")
robot.gripper.go()

robot.arm.set_named_target("ready")
robot.arm.go()

grasp = calculate_grasp(p.header, p.pose.position)
l = PlaceLocation();
l.id="place"
l.post_place_posture = grasp.pre_grasp_posture
l.pre_place_approach = grasp.pre_grasp_approach
l.post_place_retreat = grasp.post_grasp_retreat
l.allowed_touch_objects = grasp.allowed_touch_objects

# Pick
while not rospy.is_shutdown():
    # Pick
    grasp = calculate_grasp(p.header, p.pose.position)
    robot.arm.pick("box", grasp)

    # Return to ready position
    # robot.arm.set_named_target("ready")
    # robot.arm.go()

    # Place
    p.pose.position.z = 0.15
    l.place_pose = p
    robot.arm.place("box", l)

    # Ready
    # robot.arm.set_named_target("ready")
    # robot.arm.go()

    # Pick
    grasp = calculate_grasp(p.header, p.pose.position)
    robot.arm.pick("box", grasp)

    # Ready
    # robot.arm.set_named_target("ready")
    # robot.arm.go()

    # Place
    p.pose.position.z = 0.1
    l.place_pose = p
    robot.arm.place("box", l)

roscpp_shutdown()

