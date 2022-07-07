#!/usr/bin/env python
import copy
import moveit_commander
import geometry_msgs.msg
from pilz_robot_programming import *
import rospy
import sender
rospy.init_node('robot_program_node')
arm_group_name = "arm"
robot = moveit_commander.RobotCommander("robot_description")
group = moveit_commander.MoveGroupCommander(arm_group_name)
from quat_wrapper import euler_2_quat as e2q
from trajectory_msgs.msg import *
from control_msgs.msg import *
print(group.get_current_joint_values())
group.set_max_velocity_scaling_factor(1)
group.set_max_velocity_scaling_factor(1)
group.set_planning_pipeline_id('pilz_industrial_motion_planner')
group.set_planner_id('LIN')
def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

def getnums(s, e,i):
   return list(range(s, e,i))

def home():
    group.set_planner_id('PTP')
    joint_goal = [-0.18998800218105316, 0.009319736622273922, 2.656315565109253, 2.9285404682159424, 1.104100227355957, -3.0445778369903564]
    plan = group.go(joint_goal, wait=True)
    group.set_planner_id('LIN')
def pos1():
    waypoints = []
    wpose = group.get_current_pose().pose  # accepted angles [-45,-45,-45],
    wpose.position.x = 0.25
    wpose.position.z = 0.25
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        0.05,  # eef_step
        5)  # jump_threshold
    group.execute(plan, wait=True)


def pos2():
    waypoints = []
    wpose = group.get_current_pose().pose #accepted angles [-45,-45,-45],
    wpose.position.x = 0.820
    wpose.position.y = 0.020
    wpose.position.z = 0.200
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                    waypoints,  # waypoints to follow
                    0.002,  # eef_step
                    0)  # jump_threshold
    group.execute(plan, wait=True)

def start_point():
    waypoints = []
    wpose = group.get_current_pose().pose #accepted angles [-45,-45,-45],
    values = [0, 0, 0]
    if values[1] == 0:
        values[1] = 90
    else:
        values[1] = values[1] + 90
    wpose.position.x = 0.720
    wpose.position.y = 0.020
    wpose.position.z = 0.800
    waypoints.append(copy.deepcopy(wpose))
    quat = e2q(values[2], values[1], values[0],wpose.position.x,wpose.position.y,wpose.position.z)
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

def mask_setup():
    print("Setup you MASK")
    start_point()
    input("Press Enter to CONTINUE")
def mot_exe(roll,yaw,pitch):
    if yaw == 0:
        yaw = 90
    else:
        yaw = yaw + 90
    waypoints = []
    wpose = group.get_current_pose().pose
    g = geometry_msgs.msg.TransformStamped()
    br = tf2_ros.TransformBroadcaster()
    g.header.frame_id = "base_link"
    g.child_frame_id = "link_6"

    quat = e2q(roll, yaw, pitch,wpose.position.x,wpose.position.y,wpose.position.z)
    wpose.orientation.x = quat[1]
    wpose.orientation.y = quat[2]
    wpose.orientation.z = quat[3]
    wpose.orientation.w = quat[0]
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        0.05,  # eef_step
        0)  # jump_threshold
    group.execute(plan, wait=True)
    print(roll, yaw, pitch)


def start_position():
    dist = 200
    waypoints = []
    wpose = group.get_current_pose().pose
    g = geometry_msgs.msg.TransformStamped()
    br = tf2_ros.TransformBroadcaster()
    g.header.frame_id = "base_link"
    g.child_frame_id = "link_6"
    q0 = wpose.orientation.w
    q1 = wpose.orientation.x
    q2 = wpose.orientation.y
    q3 = wpose.orientation.z
    x = 2 * (q1 * q3 + q0 * q2)
    y = 2 * (q2 * q3 - q0 * q1)
    z = 1 - 2 * (q1 * q1 + q2 * q2)
    wpose.position.x = wpose.position.x
    wpose.position.y = wpose.position.y - (dist / 1000 * y)
    wpose.position.z = wpose.position.z - (dist / 1000 * z)
    waypoints.append(copy.deepcopy(wpose))
    # (plan, fraction) = group.compute_cartesian_path(
    #     waypoints,  # waypoints to follow
    #     1,  # eef_step
    #     3)  # jump_threshold
    # group.execute(plan, wait=True)

    waypoints = []
    wpose.position.x = wpose.position.x
    wpose.position.y = wpose.position.y - (120 / 1000 * y)
    wpose.position.z = wpose.position.z - (120 / 1000 * z)
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        1,  # eef_step
        10)  # jump_threshold
    group.execute(plan, wait=True)

def offset(dist):
    waypoints = []
    wpose = group.get_current_pose().pose
    g = geometry_msgs.msg.TransformStamped()
    br = tf2_ros.TransformBroadcaster()
    g.header.frame_id = "base_link"
    g.child_frame_id = "link_6"
    q0 = wpose.orientation.w
    q1 = wpose.orientation.x
    q2 = wpose.orientation.y
    q3 = wpose.orientation.z
    x = 2 * (q1 * q3 + q0 * q2)
    y = 2 * (q2 * q3 - q0 * q1)
    z = 1 - 2 * (q1 * q1 + q2 * q2)
    wpose.position.x = wpose.position.x - (dist/1000*x)
    wpose.position.y = wpose.position.y - (dist / 1000 * y)
    wpose.position.z = wpose.position.z - (dist / 1000 * z)
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        1,  # eef_step
        10)  # jump_threshold
    group.execute(plan, wait=True)
    print("Offset Done at : ",dist)

start, end, intval = 0,100,50
distance = getnums(start, end,intval)
distance.append(end)
distance = sorted(distance)

start, end, interval = -45,45,15
pitch = getnums(start, end,interval)
pitch.append(end)

if 0 not in pitch:
   pitch.append(0)
   pitch = sorted(pitch)

start, end, interval = -30,30,15
yaw = getnums(start,end,interval)
yaw.append(end)

if 0 not in yaw:
   yaw.append(0)
   yaw = sorted(yaw)

start, end, interval = -180,150,30
roll = getnums(start,end,interval)
roll.append(end)

if 0 not in roll:
   roll.append(0)
   roll = sorted(roll)

import numpy as np
poles = np.arange(-2*12, 2*12, 8).tolist()
poles_spacing = []
for i in poles:
    poles_spacing.append(i/12)
poles_spacing.append(2)
poles_spacing = sorted(poles_spacing)
pose_map = []
home()
mask_setup()
start_point()
#
# # for p in poles_spacing:
# #     joint_goal = [p, 0.009319736622273922, 2.656315565109253, 2.9285404682159424, 1.104100227355957,
# #                   -3.0445778369903564]
# #     plan = group.go(joint_goal, wait=True)
for i in yaw:
        for j in pitch:
            mot_exe(0,i,j)
            start_position()
            for k in distance:
                offset(k)
                sender.capture()
                # for l in roll:
                #     mot_exe(l, i, j)
            start_point()

home()

