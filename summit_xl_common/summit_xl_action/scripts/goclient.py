#!/usr/bin/env python

import rospy
import actionlib
import time
from summit_xl_action.msg import TargetAction,TargetActionGoal

def go_to_point_client():
	
	client = actionlib.SimpleActionClient('go_to_point_client', TargetAction)
	client.wait_for_server()
	goal = TargetActionGoal()
	goal.x = 2.0
	goal.y = -1.0
	goal.theta = 0.0
	client.send_goal(goal)
	client.wait_for_result()
	result = client.get_result()
	return result

	 	
if __name__ == '__main__':
	try:
		rospy.init_node('action_client')
		result = go_to_point_client()
		print('the result is: ', result)
	except rospy.ROSInterruptException as e:
		print("program interrupted before completion", e)
		
