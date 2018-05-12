#!/usr/bin/python
#########################################################################
# File Name: objectPos.py
# Description: 
# Author: Jialiang Zhao
# Mail: alanzjl@163.com
# Created_Time: 2018-04-09 22:40:24
# Last modified: 2018-04-09 22:40:1523338824
#########################################################################

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist

class ObstacleTrack:
    def __init__(self):
        rospy.init_node('obstacle_tracker', anonymous = True)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        self.twist = Twist()
        self.pose = Pose()
    
    def callback(self, data):
        for i in range(len(data.name)):
            if data.name[i] == 'obstacle':
                self.twist = data.twist[i]
                self.pose = data.pose[i]
                return
        rospy.loginfo('WARNING: No obstacle pose received.')

    @property
    def twist(self):
        return self.twist

    @property
    def pose(self):
        return self.pose

def main():
    tracker = ObstacleTrack()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print "--------------------------"
        print tracker.pose
        print tracker.twist
        rate.sleep()

if __name__ == '__main__':
    main()

