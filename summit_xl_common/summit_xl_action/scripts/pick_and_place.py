#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

#create an instance of moveit_commander and initialize it
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()

#In order to move arm, I use the groups I created: panda_arm, panda_gripper
#I put the arm start position ( I configured this position during the moveit configuration)  
arm_group = moveit_commander.MoveGroupCommander("panda_arm")
#arm_group.set_named_target("home")
#plan1 = arm_group.go()

#In order to move arm, I use the groups I created: panda_arm, panda_gripper
#I put the gripper open position ( I configured this position during the moveit configuration)
gripper_group = moveit_commander.MoveGroupCommander("panda_gripper")
gripper_group.set_named_target("gripper_open")
plan2 = gripper_group.go()

joint_goal = arm_group.get_current_joint_values()
print('Current joint states (radians):', joint_goal)

planning_frame = arm_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

eef_link = arm_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get the joint values from the group and adjust PRE-GRASP VALUES:
#joint_goal = arm_group.get_current_joint_values()
#joint_goal[0] = 0.1033
#joint_goal[1] = -0.6809
#joint_goal[2] = -1.6513
#joint_goal[3] = -1.9068
#joint_goal[4] = -2.8610
#joint_goal[5] = 2.6502
#joint_goal[6] = 2.1147
# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
#arm_group.go(joint_goal, wait=True)
# Calling ``stop()`` ensures that there is no residual movement
#arm_group.stop()

#rospy.sleep(2)

# We can get the joint values from the group and adjust GRASP VALUES:
#joint_goal2 = arm_group.get_current_joint_values()
#joint_goal2[0] = 0.2757
#joint_goal2[1] = -0.7488
#joint_goal2[2] = -1.7823
#joint_goal2[3] = -1.7289
#joint_goal2[4] = -2.8557
#joint_goal2[5] = 2.7011
#joint_goal2[6] = 2.0536
# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
#arm_group.go(joint_goal2, wait=True)
# Calling ``stop()`` ensures that there is no residual movement
#arm_group.stop()

#rospy.sleep(5)


#joint_goal3 = arm_group.get_current_joint_values()
#joint_goal3[0] = 0.081693
#joint_goal3[1] = -0.544802
#joint_goal3[2] = 0.045998
#joint_goal3[3] = -2.308211
#joint_goal3[4] = -0.149422
#joint_goal3[5] = 3.311626
#joint_goal3[6] = 0.073769
# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
#arm_group.go(joint_goal3, wait=True)
# Calling ``stop()`` ensures that there is no residual movement
#arm_group.stop()




#arm_group.set_pose_target()
#plan1 = arm_group.go()

#I send the arm to grasp position
#pose_target.position.z = 0.65
#arm_group.set_pose_reference_frame("robot_map")
#arm_group.set_pose_target(pose_target)
#plan1 = arm_group.go()

#I put the gripper closed position (I configured this position during the moveit configuration)
gripper_group.set_named_target("gripper_closed")
plan2 = gripper_group.go()

#I lift the object (I configured this position during the moveit configuration)
#arm_group.set_pose_target(pose_target)
#plan1 = arm_group.go()

#I provide to the moveit_commander the exact coordinates we want it to put the end effector based on the /world frame.
#I send the arm to pre-grasp position
#pose_target  =geometry_msgs.msg.Pose()
#pose_target.orientation.w = 1.0
#pose_target.position.x = -0.35
#pose_target.position.y = -4.30 
#pose_target.position.z = 0.80
#arm_group.set_pose_target(pose_target)
#plan1 =arm_group.go()

#I need to shutdown the commander in order to close properly the code
rospy.sleep(5)
moveit_commander.roscpp_shutdown()
