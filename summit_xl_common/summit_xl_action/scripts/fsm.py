#!/usr/bin/env python

import roslib
import rospy
import sys
import copy
import smach
import smach_ros
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from move_base_msgs.msg import *

class GoToPickPos(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['reached'])
	def execute(self, userdata):
		rospy.loginfo('Executing state GoToPickPose')
		goal = MoveBaseGoal()
		goal.target_pose.pose.position.x = float(-0.270)
		goal.target_pose.pose.position.y = float(-3.65)
		goal.target_pose.pose.orientation.w = float(0.1)
		goal.target_pose.header.frame_id = 'robot_map'
		goal.target_pose.header.stamp = rospy.Time.now()
		navigation_action_server.send_goal(goal)
		navigation_action_server.wait_for_result()
		rospy.sleep(1)
		return 'reached'

class PickObjPrePos(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['reached'])
	def execute(self, userdata):
		rospy.loginfo('Executing state PickObjPrePos')
		gripper_group.set_named_target("gripper_open")
		plan2 = gripper_group.go()
		joint_goal = arm_group.get_current_joint_values()
		joint_goal[0] = 0.1033
		joint_goal[1] = -0.6809
		joint_goal[2] = -1.6513
		joint_goal[3] = -1.9068
		joint_goal[4] = -2.8610
		joint_goal[5] = 2.6502
		joint_goal[6] = 2.1147
		arm_group.go(joint_goal, wait=True)
		arm_group.stop()
		rospy.sleep(1)
		return 'reached'

class PickObjGraspPos(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['reached'])
	def execute(self, userdata):
		rospy.loginfo('Executing state PickObjGraspPos')
		joint_goal2 = arm_group.get_current_joint_values()
		joint_goal2[0] = 0.2757
		joint_goal2[1] = -0.7488
		joint_goal2[2] = -1.7823
		joint_goal2[3] = -1.7289
		joint_goal2[4] = -2.8557
		joint_goal2[5] = 2.7011
		joint_goal2[6] = 2.0536
		arm_group.go(joint_goal2, wait=True)
		arm_group.stop()
		gripper_group.set_named_target("gripper_closed")
		plan2 = gripper_group.go()
		rospy.sleep(1)
		
		return 'reached'

class AttachObj(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['reached'])
	def execute(self, userdata):
		rospy.loginfo('Executing state AttachObj')
		req = AttachRequest()
		req.model_name_1 = "robot"
		req.link_name_1 = "robot_arm_leftfinger"
		req.model_name_2 = "can_coke"
		req.link_name_2 = "link_0"
		attach_srv.call(req)
		req = AttachRequest()
		req.model_name_1 = "robot"
		req.link_name_1 = "robot_arm_rightfinger"
		req.model_name_2 = "can_coke"
		req.link_name_2 = "link_1"
		attach_srv.call(req)
		rospy.sleep(1)
		return 'reached'

class ArmSafePos(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['reached'])
	def execute(self, userdata):
		rospy.loginfo('Executing state ArmSafePos')
		joint_goal3 = arm_group.get_current_joint_values()
		joint_goal3[0] = 0.081693
		joint_goal3[1] = -0.544802
		joint_goal3[2] = 0.045998
		joint_goal3[3] = -2.308211
		joint_goal3[4] = -0.149422
		joint_goal3[5] = 3.311626
		joint_goal3[6] = 0.073769
		arm_group.go(joint_goal3, wait=True)
		arm_group.stop()
		rospy.sleep(1)
		return 'reached'

class GoToPlacePos(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['reached'])
	def execute(self, userdata):
		rospy.loginfo('Executing state GoToPickPose')
		goal = MoveBaseGoal()
		goal.target_pose.pose.position.x = float(7.572)
		goal.target_pose.pose.position.y = float(4.821)
		goal.target_pose.pose.orientation.w = float(-0.05)
                goal.target_pose.pose.orientation.z = float(1.58)
		goal.target_pose.header.frame_id = 'robot_map'
		goal.target_pose.header.stamp = rospy.Time.now()
		navigation_action_server.send_goal(goal)
		navigation_action_server.wait_for_result()
		return 'reached'
class PlaceObjPrePos(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['reached'])
	def execute(self, userdata):
		rospy.loginfo('Executing state ArmSafePos')
		joint_goal4 = arm_group.get_current_joint_values()
		joint_goal4[0] = -2.734515
		joint_goal4[1] = -1.082399
		joint_goal4[2] = 1.402905
		joint_goal4[3] = -1.89966
		joint_goal4[4] = -0.424151
		joint_goal4[5] = 3.16946
		joint_goal4[6] = 0.532665
		arm_group.go(joint_goal4, wait=True)
		arm_group.stop()
		return 'reached'
class PlaceObjPlacePos(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['reached'])
	def execute(self, userdata):
		rospy.loginfo('Executing state ArmSafePos')
		joint_goal5 = arm_group.get_current_joint_values()
		joint_goal5[0] = -2.761755
		joint_goal5[1] = -1.301945
		joint_goal5[2] = 1.621518
		joint_goal5[3] = -2.115458
		joint_goal5[4] = 0.565085
		joint_goal5[5] = 3.415692
		joint_goal5[6] = 0.682128
		arm_group.go(joint_goal5, wait=True)
		arm_group.stop()
		gripper_group.set_named_target("gripper_open")
		plan2 = gripper_group.go()
		rospy.sleep(1)
		return 'reached'

class DetachObj(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['reached'])
	def execute(self, userdata):
		rospy.loginfo('Executing state DetachObj')
		req = AttachRequest()
		req.model_name_1 = "robot"
		req.link_name_1 = "robot_arm_leftfinger"
		req.model_name_2 = "can_coke"
		req.link_name_2 = "link_0"
		detach_srv.call(req)
		req = AttachRequest()
		req.model_name_1 = "robot"
		req.link_name_1 = "robot_arm_rightfinger"
		req.model_name_2 = "can_coke"
		req.link_name_2 = "link_1"
		detach_srv.call(req)
		return 'reached'

class ArmHome(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['motion_finished'])
	def execute(self, userdata):
		arm_group.set_named_target("home")
		plan1 = arm_group.go()
		return 'motion_finished'

def main():
	global navigation_action_server, attach_srv, detach_srv, robot, arm_group, gripper_group
	
	#Initialzie the state machnine	
	rospy.init_node('state_machine')
	
	#Initialize the navigation server
	navigation_action_server = actionlib.SimpleActionClient('/move_base', MoveBaseAction )
	navigation_action_server.wait_for_server()

	#Initialize the link attacher server
        attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
	attach_srv.wait_for_service()
	#Initialize the link detacher server
	detach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                    Attach)
	detach_srv.wait_for_service()
	
	#create an instance of moveit_commander and initialize it
	moveit_commander.roscpp_initialize(sys.argv)
	robot = moveit_commander.RobotCommander()
	
	#In order to move arm, I use the groups I created: panda_arm, panda_gripper
	#I put the arm start position ( I configured this position during the moveit configuration)
	arm_group = moveit_commander.MoveGroupCommander("panda_arm")

	#I put the gripper open position ( I configured this position during the moveit configuration)
	gripper_group = moveit_commander.MoveGroupCommander("panda_gripper")

	#Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['motion_finished'])

	with sm:
		#Adding states to the container
		smach.StateMachine.add('GoToPickPos', GoToPickPos(), transitions={'reached':'PickObjPrePos'})
		smach.StateMachine.add('PickObjPrePos', PickObjPrePos(), transitions={'reached':'PickObjGraspPos'})
		smach.StateMachine.add('PickObjGraspPos', PickObjGraspPos(), transitions={'reached':'AttachObj'})
		smach.StateMachine.add('AttachObj', AttachObj(), transitions={'reached':'ArmSafePos'})
		smach.StateMachine.add('ArmSafePos', ArmSafePos(), transitions={'reached':'GoToPlacePos'})
		smach.StateMachine.add('GoToPlacePos', GoToPlacePos(), transitions={'reached':'PlaceObjPrePos'})
		smach.StateMachine.add('PlaceObjPrePos', PlaceObjPrePos(), transitions={'reached':'PlaceObjPlacePos'})
		smach.StateMachine.add('PlaceObjPlacePos', PlaceObjPlacePos(), transitions={'reached':'DetachObj'})
		smach.StateMachine.add('DetachObj', DetachObj(), transitions={'reached':'ArmHome'})
		smach.StateMachine.add('ArmHome', ArmHome(), transitions={'motion_finished':'motion_finished'})

	# Execute the state machine
	outcome = sm.execute()

	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()
if __name__ == '__main__':
    main()

