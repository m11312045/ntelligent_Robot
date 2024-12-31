#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from geometry_msgs.msg import Twist

def pose_callback(pose):
    rospy.loginfo("Robot X=%f: Y=%f: Z=%f linear_vel=%f: angular_vel=%f\n", pose.x, pose.y, pose.theta, pose.linear_velocity, pose.angular_velocity)
    if(pose.x > 9 or pose.y > 9):
        turtle_callsrv()
        turtle_stop()

def turtle_stop():
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) 
    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.linear.x = 0.0
    vel_msg.linear.x = 0.0
    vel_msg.angular.x= 0.0
    vel_msg.angular.y= 0.0
    vel_msg.angular.z= 0.0
    velocity_publisher.publish(vel_msg)

def turtle_callsrv():
    rospy.wait_for_service('/turtle1/teleport_absolute')
    try:
        call1 = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        resp1 = call1(x = 5.544445, y = 5.544445, theta = 0)
        return resp1
    except rospy.ServiceException as e:
        print ("Service call failed: {}".format(e))

def turtle():
    rospy.init_node('turtlesim_info', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    
    rospy.spin()  # 阻塞直到节点关闭

if __name__ == '__main__':
    try:
        turtle()
    except rospy.ROSInterruptException:
        pass
