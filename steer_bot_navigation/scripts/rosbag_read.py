#!/usr/bin/env python3
import rospy
import rosbag
from move_base_msgs.msg import MoveBaseActionGoal

import sys

rospy.init_node('rosbag_play')

pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
bag = rosbag.Bag(sys.argv[1])
msg = next(bag.read_messages(topics=['/move_base/goal']))[1]

new_msg = msg
new_msg.header.seq = 0
new_msg.header.stamp = rospy.Time.now()
new_msg.goal.target_pose.header.seq = 0
new_msg.goal.target_pose.header.stamp = rospy.Time.now()

rospy.sleep(0.2)
pub.publish(new_msg)
rospy.loginfo("Published goal")


