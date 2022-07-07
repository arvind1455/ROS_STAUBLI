#!/usr/bin/env python
import copy
import moveit_commander
import geometry_msgs.msg
from pilz_robot_programming import *
import rospy
import numpy as np
rospy.init_node('robot_program_node')
arm_group_name = "arm"
robot = moveit_commander.RobotCommander("robot_description")
group = moveit_commander.MoveGroupCommander(arm_group_name)
from quat_wrapper import euler_2_quat as e2q
from geometry_msgs.msg import Point
from trajectory_msgs.msg import *
from control_msgs.msg import *

group.set_max_velocity_scaling_factor(0.2)

group.set_planning_pipeline_id('pilz_industrial_motion_planner')
group.set_planner_id('LIN')

print(group.get_current_pose().pose)
def home():
    group.set_planner_id('PTP')
    joint_goal = [0.0, 0.6283179521560669, 2.0943961143493652, 0.0, -1.2217304706573486, 2.1816608905792236]
    plan = group.go(joint_goal, wait=True)
    group.set_planner_id('LIN')

def start_point():
    group.clear_path_constraints()
    waypoints = []
    wpose = group.get_current_pose().pose #accepted angles [-45,-45,-45],
    values = [0, 90, 0]
    if values[1] == 0:
        values[1] = 90
    else:
        values[1] = values[1] + 90
    quat = e2q(values[2], values[1], values[0],wpose.position.x,wpose.position.y,wpose.position.z)
    wpose.position.x = 0.4148152395083223
    wpose.position.y = 0.014012825263769908
    wpose.position.z = wpose.position.z
    wpose.orientation.x = quat[1]
    wpose.orientation.y = quat[2]
    wpose.orientation.z = quat[3]
    wpose.orientation.w = quat[0]
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        1,  # eef_step
        5)  # jump_threshold
    group.execute(plan, wait=True)

def go_dust():
    waypoints = []
    wpose = group.get_current_pose().pose
    values = [0, 90, 0]
    if values[1] == 0:
        values[1] = 90
    else:
        values[1] = values[1] + 90

    wpose.position.x = wpose.position.x
    wpose.position.y = wpose.position.y
    wpose.position.z = 0.17059262644311077
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        1,  # eef_step
        2)  # jump_threshold
    group.execute(plan, wait=True)
    # waypoints = []
    # wpose = group.get_current_pose().pose
    # wpose.position.x -= 0.020
    # waypoints.append(copy.deepcopy(wpose))
    # (plan, fraction) = group.compute_cartesian_path(
    #     waypoints,  # waypoints to follow
    #     1,  # eef_step
    #     5)  # jump_threshold
    # group.execute(plan, wait=True)
    # waypoints = []
    # wpose = group.get_current_pose().pose
    # wpose.position.x = wpose.position.x
    # wpose.position.y = wpose.position.y
    # wpose.position.z = 0.3312997540443548
    # print(wpose.position)
    # waypoints.append(copy.deepcopy(wpose))
    # (plan, fraction) = group.compute_cartesian_path(
    #     waypoints,  # waypoints to follow
    #     1,  # eef_step
    #     5)  # jump_threshold
    # group.execute(plan, wait=True)

def safe_position(i, j):
    wpose = group.get_current_pose().pose
    g = geometry_msgs.msg.TransformStamped()
    br = tf2_ros.TransformBroadcaster()
    g.header.frame_id = "base_link"
    g.child_frame_id = "link_6"
    waypoints = []
    wpose.position.y = j
    wpose.position.x = i
    wpose.position.z = 0.3312997540443548
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        1,  # eef_step
        2)  # jump_threshold
    group.execute(plan, wait=True)

def pick_and_place_spoof(i,j):
    wpose = group.get_current_pose().pose
    g = geometry_msgs.msg.TransformStamped()
    br = tf2_ros.TransformBroadcaster()
    g.header.frame_id = "base_link"
    g.child_frame_id = "link_6"
    waypoints = []
    wpose.position.y = j
    wpose.position.x = i
    wpose.position.z = wpose.position.z - 0.05
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        1,  # eef_step
        2)  # jump_threshold
    group.execute(plan, wait=True)
    safe_position(i,j)

def left_eye_capture():
    wpose = group.get_current_pose().pose
    g = geometry_msgs.msg.TransformStamped()
    br = tf2_ros.TransformBroadcaster()
    g.header.frame_id = "base_link"
    g.child_frame_id = "link_6"
    waypoints = []
    wpose.position.y = wpose.position.y
    wpose.position.x = wpose.position.x
    wpose.position.z = wpose.position.z - 0.250
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        1,  # eef_step
        2)  # jump_threshold
    group.execute(plan, wait=True)

def left_eye_safe(leye):
    safe_position(leye[0], leye[1])
    wpose = group.get_current_pose().pose
    g = geometry_msgs.msg.TransformStamped()
    br = tf2_ros.TransformBroadcaster()
    g.header.frame_id = "base_link"
    g.child_frame_id = "link_6"
    waypoints = []
    wpose.position.y = leye[1]
    wpose.position.x = leye[0]
    wpose.position.z = 0.3312997540443548
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        1,  # eef_step
        2)  # jump_threshold
    group.execute(plan, wait=True)

def saftey_level():
    wpose = group.get_current_pose().pose
    g = geometry_msgs.msg.TransformStamped()
    br = tf2_ros.TransformBroadcaster()
    g.header.frame_id = "base_link"
    g.child_frame_id = "link_6"
    waypoints = []
    wpose.position.y = wpose.position.y
    wpose.position.x = wpose.position.x
    wpose.position.z = 0.3312997540443548
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        1,  # eef_step
        2)  # jump_threshold
    group.execute(plan, wait=True)

def place_spoof(i,j):
    safe_position(i,j)
    pick_and_place_spoof(i,j)
    safe_position(i,j)

def spoof_collector(leye):
    l = np.arange(0,0.266,0.05)
    b = np.arange(0.2200,0.400, 0.05)
    for i in l:
        for j in b:
            safe_position(i, j)
            pick_and_place_spoof(i, j)
            left_eye_safe(leye)
            left_eye_capture()
            import robot_shift_binoc
            saftey_level()
            place_spoof(i,j)

home()
start_point()
go_dust()
leye = [0.4148152395083223,0.014012825263769908,0.17059262644311077]
spoof_collector(leye)


