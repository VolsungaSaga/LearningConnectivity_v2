#!/usr/bin/env python
import rospy
import roslib
import tf

import tf2_ros

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped

import geometry_msgs.msg

#For the trajectory we want to plan, we'll need to specify action goals for the planner algorithms we're using.
# The followed trajectory must be repeatable/consistent from run to run, given a start location.

class Waypoint_Broadcaster:


	def __init__(self):
		corner1 = geometry_msgs.msg.PointStamped()
		corner2 = geometry_msgs.msg.PointStamped()
		corner3 = geometry_msgs.msg.PointStamped()
		corner4 = geometry_msgs.msg.PointStamped()

		self.defaultRectangle = [corner1, corner2, corner3, corner4]






