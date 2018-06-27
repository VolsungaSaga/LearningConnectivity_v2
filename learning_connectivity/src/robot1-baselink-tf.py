#!/usr/bin/env python

import rospy
import roslib
import tf
import math

import tf2_ros

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import geometry_msgs.msg

#This node handles the broadcasting of all the transforms we need to translate between the motion capture's frame and the robot's frame.
# They include:
# Transform from world to Robot_1/base_link.
# Transform from Robot_1/base_link to base_footprint


#In order for our system to work, the following ROS nodes must be active:
#   mocap_optitrack - For the motion capture position data.
#   turtlebot_bringup minimal launch - For the robot's basic movement nodes.
#       (OPTIONAL) turtlebot_teleop - For moving the robot with either a joystick or a keyboard.
class Robot1BaseLink_tf:
    def __init__(self):
        self.tf_robotToBase = geometry_msgs.msg.TransformStamped()
        self.tf_worldToRobot = geometry_msgs.msg.TransformStamped()
        self.robot1Sub1 = rospy.Subscriber("/Robot_1/pose", PoseStamped, self.robot1ToBaseFootprint_tf_callback)
        self.robot1Sub2 = rospy.Subscriber("/Robot_1/pose", PoseStamped, self.worldToRobot1_tf_callback)



    def robot1ToBaseFootprint_tf_callback(self, msg):
        #We supply a transformation from Robot1 to the base_footprint. We consider this transformation to be a 
        # rotation and translation of 0 radians / meters in all axes.
        br = tf2_ros.TransformBroadcaster()
        self.tf_robotToBase.header.stamp = rospy.Time.now()

        self.tf_robotToBase.header.frame_id = "Robot_1/base_link"
        self.tf_robotToBase.child_frame_id = "base_footprint"
        self.tf_robotToBase.transform.translation.x = 0
        self.tf_robotToBase.transform.translation.y = 0
        self.tf_robotToBase.transform.translation.z = 0

        q = quaternion_from_euler(0,0,0)

        self.tf_robotToBase.transform.rotation.x = q[0]
        self.tf_robotToBase.transform.rotation.y = q[1]
        self.tf_robotToBase.transform.rotation.z = q[2]
        self.tf_robotToBase.transform.rotation.w = q[3]

        br.sendTransform(self.tf_robotToBase)

    def worldToRobot1_tf_callback(self,msg):
        #Here, we supply a transform from the world frame to the Robot1 frame
        br = tf2_ros.TransformBroadcaster()
        self.tf_worldToRobot.header.stamp = rospy.Time.now()
        self.tf_worldToRobot.header.frame_id = "world"
        self.tf_worldToRobot.child_frame_id = "Robot_1/base_link"
        self.tf_worldToRobot.transform.translation.x = msg.pose.position.x
        self.tf_worldToRobot.transform.translation.y = msg.pose.position.y
        self.tf_worldToRobot.transform.translation.z = 0

        self.tf_worldToRobot.transform.rotation.x = msg.pose.orientation.x
        self.tf_worldToRobot.transform.rotation.y = msg.pose.orientation.y
        self.tf_worldToRobot.transform.rotation.z = msg.pose.orientation.z
        self.tf_worldToRobot.transform.rotation.w = msg.pose.orientation.w

        #I think we need to negate the yaw rotation part.
        quat = self.tf_worldToRobot.transform.rotation
        quat_list = [quat.x, quat.y,quat.z,quat.w]
        roll, pitch, yaw = euler_from_quaternion(quat_list)
        yaw = (yaw - math.pi)
        quat = quaternion_from_euler(roll, pitch, yaw)

        self.tf_worldToRobot.transform.rotation.x = quat[0]
        self.tf_worldToRobot.transform.rotation.y = quat[1]
        self.tf_worldToRobot.transform.rotation.z = quat[2]
        self.tf_worldToRobot.transform.rotation.w = quat[3]



        br.sendTransform(self.tf_worldToRobot)

    


if __name__ == '__main__':


    rospy.init_node("robot1_baselink_tf")
    Robot1BaseLink_tf = Robot1BaseLink_tf()


    rospy.spin()