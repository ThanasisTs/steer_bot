#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseActionGoal

goal_time_flag, path_time_flag = False, False
goal_time, path_time = None, None


def goal_callback(msg):
	global goal_time_flag, goal_time
	goal_time_flag = True
	goal_time = rospy.Time.now().to_sec()

def path_callback(msg):
	global path_time_flag, path_time
	path_time_flag = True
	path_time = rospy.Time.now().to_sec()

rospy.init_node('plan_time')

goal_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, goal_callback)
path_sub = rospy.Subscriber('/move_base/SBPLLatticePlanner/plan', Path, path_callback)

while not rospy.is_shutdown():
	if goal_time_flag and path_time_flag:
		print(path_time, goal_time)
		rospy.loginfo("Plan time: {} secs".format(abs(round(path_time-goal_time, 3))))
		goal_time_flag, path_time_flag = False, False


