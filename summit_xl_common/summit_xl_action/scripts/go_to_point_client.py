#!/usr/bin/env python

import rospy
import actionlib
import time
from summit_xl_action.msg import TargetAction, TargetGoal

	 	
if __name__ == '__main__':
	rospy.init_node('go_to_point_client')
	print("client running")
	client = actionlib.SimpleActionClient('robot/go_to_point_client', TargetAction)
	client.wait_for_server()
	goal = TargetGoal()
	goal.x = 2.0
	goal.y = -1.0
	goal.theta = 0.0
	client.send_goal(goal)
	client.wait_for_result()
	client.wait_for_result(rospy.Duration.from_sec(5.0))
