#! /usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import tf

rospy.init_node('odom_pub')

odom_pub=rospy.Publisher ('/steer_bot/groundtruth/odom', Odometry, queue_size=10)

rospy.wait_for_service ('/steer_bot/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/steer_bot/gazebo/get_model_state', GetModelState)

odom=Odometry()
header = Header()
header.frame_id='odom'

model = GetModelStateRequest()
model.model_name='steer_bot'

r = rospy.Rate(100)

while not rospy.is_shutdown():
	result = get_model_srv(model)
	odom.pose.pose = result.pose
	odom.twist.twist = result.twist
	header.stamp = rospy.Time.now()
	odom.header = header
	odom_pub.publish (odom)
	br = tf.TransformBroadcaster()
	br.sendTransform((result.pose.position.x, result.pose.position.y, 0),
                     (result.pose.orientation.x, result.pose.orientation.y, result.pose.orientation.z, result.pose.orientation.w),
                     rospy.Time.now(),
                     "base_link",
                     "odom")

	r.sleep()