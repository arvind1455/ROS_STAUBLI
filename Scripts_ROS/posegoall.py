#!/usr/bin/env python
import sys
import time
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
import actionlib
from math import pi
from std_srvs.srv import Empty
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose, Point, PointStamped
from pilz_robot_programming import *
import rospy
from tf.transformations import quaternion_from_euler
from itertools import combinations
rospy.init_node('robot_program_node')
arm_group_name = "arm"
robot = moveit_commander.RobotCommander("robot_description")
group = moveit_commander.MoveGroupCommander(arm_group_name)
group.set_planning_pipeline_id('pilz_industrial_motion_planner')
group.set_planner_id('PTP')
import numpy as np
from trajectory_msgs.msg import *
from control_msgs.msg import *
print(group.get_current_joint_values())
group.set_max_velocity_scaling_factor(1)

actual_pose = group.get_current_pose()
pose_goal = geometry_msgs.msg.PoseStamped()

waypoints = []
pose_goal.header = actual_pose.header
pose_goal.header.stamp.secs = 1652486010
pose_goal.header.stamp.nsecs = 927461624
pose_goal.header.frame_id = "base_link"
pose_goal.pose.orientation.w = 0.9
pose_goal.pose.orientation.x = 0.155
pose_goal.pose.orientation.y = 0.127
pose_goal.pose.orientation.z = 0.05
pose_goal.pose.position.x = -0.1
pose_goal.pose.position.y = -0.518
pose_goal.pose.position.z = 1.655

waypoints.append(copy.deepcopy(pose_goal))
(plan, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        0.002,  # eef_step
        0)  # jump_threshold
group.execute(plan, wait=True)