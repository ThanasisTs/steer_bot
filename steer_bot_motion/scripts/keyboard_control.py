#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

from pynput import keyboard
import numpy as np
import pygame as pg

if __name__ == "__main__":
	rospy.init_node("keyboard_control")
	pub = rospy.Publisher('/steer_bot/ackermann_steering_controller/cmd_vel', Twist, queue_size=10)
	linear_vel, angular_vel = 0, 0
	keys = [keyboard.Key.up, keyboard.Key.down, keyboard.Key.right, keyboard.Key.left]
	
	pg.init()
	pg.display.set_mode((50, 50))
	pg.key.set_repeat(10)

	while not rospy.is_shutdown():
		for event in pg.event.get():
			if event.type==pg.KEYDOWN:
				if event.key == pg.K_UP:
					linear_vel += 0.1
				elif event.key == pg.K_DOWN:
					linear_vel -= 0.1
				if event.key == pg.K_LEFT:
					angular_vel += 0.1
				elif event.key == pg.K_RIGHT:
					angular_vel -= 0.1
			if event.type==pg.KEYUP:
				if event.key in [pg.K_UP, pg.K_DOWN]:
					linear_vel = 0
				elif event.key in [pg.K_LEFT, pg.K_RIGHT]:
					angular_vel = 0
		print(linear_vel, angular_vel)
		linear_vel = min(linear_vel, 1) if linear_vel >= 0 else max(linear_vel, -1)
		angular_vel = min(angular_vel, 1) if angular_vel >= 0 else max(angular_vel, -1)
		cmd_vel = Twist()
		cmd_vel.linear.x = linear_vel
		cmd_vel.angular.z = angular_vel
		pub.publish(cmd_vel)
