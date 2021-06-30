#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

pub = None
count = 0

def callback(msg):
	global pub, count
	fake_odom_msg = Odometry()
	fake_odom_msg.header.seq = count
	fake_odom_msg.header.stamp = rospy.Time.now()
	fake_odom_msg.header.frame_id = "odom"
	fake_odom_msg.child_frame_id = "base_link"
	
	fake_odom_msg.pose.pose.position.x = msg.pose[1].position.x
	fake_odom_msg.pose.pose.position.y = msg.pose[1].position.y
	fake_odom_msg.pose.pose.position.z = msg.pose[1].position.z
	fake_odom_msg.pose.pose.orientation.x = msg.pose[1].orientation.x
	fake_odom_msg.pose.pose.orientation.y = msg.pose[1].orientation.y
	fake_odom_msg.pose.pose.orientation.z = msg.pose[1].orientation.z
	fake_odom_msg.pose.pose.orientation.w = msg.pose[1].orientation.w
	fake_odom_msg.pose.covariance = [0 for _ in range(36)]

	fake_odom_msg.twist.twist.linear.x = msg.twist[1].linear.x
	fake_odom_msg.twist.twist.linear.y = msg.twist[1].linear.y
	fake_odom_msg.twist.twist.linear.z = msg.twist[1].linear.z
	fake_odom_msg.twist.twist.angular.x = msg.twist[1].angular.x
	fake_odom_msg.twist.twist.angular.y = msg.twist[1].angular.y
	fake_odom_msg.twist.twist.angular.z = msg.twist[1].angular.z
	fake_odom_msg.twist.covariance = [0 for _ in range(36)]
	
	count += 1

	pub.publish(fake_odom_msg)

def main():
	global pub
	rospy.init_node('fake_odom')
	pub = rospy.Publisher('/steer_bot/fake_odom', Odometry, queue_size=10)
	sub = rospy.Subscriber('/steer_bot/gazebo/model_states', ModelStates, callback)
	rospy.spin()


if __name__ == "__main__":
	main()