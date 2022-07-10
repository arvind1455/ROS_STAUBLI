#!/usr/bin/env python
import copy
import moveit_commander
import geometry_msgs.msg

from pilz_robot_programming import *
import rospy
rospy.init_node('robot_program_node')
arm_group_name = "arm"
robot = moveit_commander.RobotCommander("robot_description")
group = moveit_commander.MoveGroupCommander(arm_group_name)
from quat_wrapper import euler_2_quat as e2q
from geometry_msgs.msg import Point
from trajectory_msgs.msg import *
from control_msgs.msg import *
print(group.get_current_joint_values())
group.set_max_velocity_scaling_factor(1)

from std_msgs.msg import Int32
msg = Int32()
pub = rospy.Publisher("/gripper", Int32, queue_size=10)
counter = 0
msg.data = counter
r = rospy.Rate(10)
while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()