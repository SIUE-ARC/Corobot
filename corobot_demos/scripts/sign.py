#!/usr/bin/env python
import sys
import rospy
import rosparam
import random
import math

from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped

roscpp_initialize(sys.argv)
rospy.init_node('sign_demo')

rospy.sleep(15)
scene = PlanningSceneInterface()
robot = RobotCommander()
robot.arm.set_goal_orientation_tolerance(0.01)

p = PoseStamped()
p.header.frame_id = robot.get_planning_frame()

poses = rosparam.get_param("poses")

current_pose = 0

# Generate movement plan
while not rospy.is_shutdown():
    pose = poses[current_pose % len(poses)]["pose"]
    current_pose += 1

    p.pose.position.x = pose["position"]["x"]
    p.pose.position.y = pose["position"]["y"]
    p.pose.position.z = pose["position"]["z"]

    p.pose.orientation.x = pose["orientation"]["x"]
    p.pose.orientation.y = pose["orientation"]["y"]
    p.pose.orientation.z = pose["orientation"]["z"]
    p.pose.orientation.w = pose["orientation"]["w"]

    robot.arm.set_pose_target(p);
    plan = robot.arm.plan()
    robot.arm.go(wait=True)

    if(plan.joint_trajectory.points):
        rospy.sleep(3.0)

roscpp_shutdown()

