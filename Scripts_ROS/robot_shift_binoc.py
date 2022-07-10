# import numpy as np
#
# start, end, interval = 0,12,6
# dist = np.arange(start,end,interval).tolist()
# dist.append(end)
# print(dist)
#
# start, end, interval = -10,10,10
# pitch = np.arange(start,end,interval).tolist()
# pitch.append(end)
# print(pitch)
#
# start, end, interval = -10,10,10
# yaw = np.arange(start,end,interval).tolist()
# yaw.append(end)
# print(yaw)
#
# rolls = [0]
#
# start, end, interval = -5,5,5
# vertical = np.arange(start,end,interval).tolist()
# vertical.append(end)
# print(vertical)
#
# start, end, interval = -1,1,1
# horizontal = np.arange(start,end,interval).tolist()
# horizontal.append(end)
# print(horizontal)
#
# angles = []
# posemap = []
# for i in yaw:
#         for j in pitch:
#             for k in dist:
#                 for r in rolls:
#                     for v in vertical:
#                         for h in horizontal:
#                             posemap.append([[r,i,j],v,h,k])
#
#
# print(len(posemap))
# print(posemap)
# np.save("robot_shift_binoc.npy", posemap)

import numpy as np
pm = np.load("robot_shift_binoc.npy",allow_pickle = True)
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
from moveit_msgs import *
group.set_max_velocity_scaling_factor(0.02)
group.set_planning_pipeline_id('pilz_industrial_motion_planner')
group.set_planner_id('LIN')

def pivot_point(values,horiz,vert,dist):
    constraint = JointConstraint()
    constraint.tolerance_above = 0.00000
    constraint.tolerance_below = 0.0050000
    constraint.weight = 1
    constraint.joint_name = "joint_1"
    const = Constraints()
    const.name = "Stable_Base"
    const.joint_constraints.append(constraint)
    group.set_path_constraints(const)

    waypoints = []
    wpose = group.get_current_pose().pose #accepted angles [-45,-45,-45],

    if values[1] == 0 or values[1] == -10:
        values[1] = 180

    else:
        values[1] = values[1] + 180

    wpose.position.x = 0.41481416927832443 + (vert/1000)
    wpose.position.y = 0.014018964903929582 + (horiz/1000)
    wpose.position.z = 0.0813027594183729 - (dist/1000)
    waypoints.append(copy.deepcopy(wpose))
    quat = e2q(values[0], values[1], values[2],wpose.position.x,wpose.position.y,wpose.position.z)
    wpose.orientation.x = quat[1]
    wpose.orientation.y = quat[2]
    wpose.orientation.z = quat[3]
    wpose.orientation.w = quat[0]
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        1,  # eef_step
        2)  # jump_threshold
    group.execute(plan, wait=True)
    time.sleep(1)


group.clear_path_constraints()
home = [-0.014422610402107239, 1.370949625968933, 1.3367022275924683, 9.024688552017324e-08, 0.43392157554626465, -0.014399592764675617]
plan = group.go(home, wait=True)
group.set_planner_id('LIN')


for i in pm:
    print(i[0],i[1],i[2],i[3])
    pivot_point(i[0], i[1], i[2],i[3])


# pivot_point(None)
group.clear_path_constraints()
print(group.get_current_pose().pose)