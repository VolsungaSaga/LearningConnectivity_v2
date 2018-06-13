#!/usr/bin/env python
import rospy
import roslib
import tf


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, PoseStamped


#In order for our system to work, the following ROS nodes must be active:
#	mocap_optitrack - For the motion capture position data.
#	turtlebot_bringup minimal launch - For the robot to broadcast odometry data.
#		turtlebot_teleop - For optionally moving the robot.




def convertToOdom_callback(data):
	#Here, we'll take the data from the optitrack pose messages and slap it into
	# an odometry message. Then, we'll immediately publish it to our topic. 

	#The covariance information, from around 30 seconds of stationary measurements.
	x_var = 4.0651e-10
	y_var = 2.1121e-10
	z_var = 2.3066e-10
	roll_var = 1e-10
	pitch_var = 1e-10
	yaw_var = 1e-10

	#vx_var
	#vy_var
	#vz_var

	#vroll_var
	#vpitch_var
	#vyaw_var


	odom_msg = Odometry()
	#Copy over position and orientation data.
	odom_msg.pose.pose.position.x = data.pose.position.x
	odom_msg.pose.pose.position.y = data.pose.position.y
	odom_msg.pose.pose.position.z = data.pose.position.z

	odom_msg.pose.pose.orientation.x = data.pose.orientation.x
	odom_msg.pose.pose.orientation.y = data.pose.orientation.y
	odom_msg.pose.pose.orientation.z = data.pose.orientation.z
	odom_msg.pose.pose.orientation.w = data.pose.orientation.w

	#Child Frame ID
	odom_msg.child_frame_id = "Robot_1/base_link"

	#Header Information
	odom_msg.header.seq = data.header.seq
	odom_msg.header.stamp = data.header.stamp
	odom_msg.header.frame_id = data.header.frame_id

	#Supply the variances of each variable.
	odom_msg.pose.covariance = [x_var, 0,0,0,0,0,0, y_var, 0,0,0,0,0, 0, z_var,0,0,0,0,0,0,roll_var, 0,0,0,0,0,0,pitch_var,0, 0,0,0,0,0,yaw_var]
	convertedData_pub.publish(odom_msg)


if __name__ == '__main__':
 	#We set up a subscriber that takes data from the MoCap publisher and calls the callback function above, passing the data as an 
	# argument.
	rospy.init_node("mocapData_conversion")
	convertedData_pub = rospy.Publisher("odom_mocapSource", Odometry, queue_size = 10)
	rospy.Subscriber("/Robot_1/pose", PoseStamped, convertToOdom_callback)

	rospy.spin()