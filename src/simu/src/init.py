#!/usr/bin/python
#########################################################################
# File Name: init.py
# Description: 
# Author: Jialiang Zhao
# Mail: alanzjl@163.com
# Created_Time: 2018-04-11 16:20:13
# Last modified: 2018-04-11 16:20:1523488813
#########################################################################

import rospy

from gazebo_msgs.msg import ModelState

rospy.init_node('initializer', anonymous = True)
pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size = 10)
state = ModelState()
state_ = ModelState()

state.model_name = 'obstacle'
state.pose.position.x = 1.0
state.pose.position.y = 0.0
state.pose.position.z = 0.35
state.pose.orientation.x = 0.0
state.pose.orientation.y = 0.0
state.pose.orientation.z = 0.0
state.twist.linear.x = 0.0
state.twist.linear.y = 0.0
state.twist.linear.z = 0.0
state.twist.angular.x = 0.0
state.twist.angular.y = 0.0
state.twist.angular.z = 0.0

state_.model_name = 'mobile_base'
state_.pose.position.x = 0.0
state_.pose.position.y = 0.0
state_.pose.position.z = 0.0
state_.pose.orientation.x = 0.0
state_.pose.orientation.y = 0.0
state_.pose.orientation.z = 0.0
state_.twist.linear.x = 0.0
state_.twist.linear.y = 0.0
state_.twist.linear.z = 0.0
state_.twist.angular.x = 0.0
state_.twist.angular.y = 0.0
while not rospy.is_shutdown():
    pub.publish(state)
    pub.publish(state_)
