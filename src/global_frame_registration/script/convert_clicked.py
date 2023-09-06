#!/usr/bin/env python  

from __future__ import print_function

import roslib
import rospy

import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs

import sys

class convertClicked:
    def __init__(self, *args):
        self.__name = rospy.get_name()
        self.clicked_point = '/clicked_point'
        self._converted_topic = '/camera_point'
        
        self.point_click_sub = rospy.Subscriber(self.clicked_point, geometry_msgs.msg.PointStamped, self.convert_function) 
        self.point_click_pub = rospy.Publisher(self._converted_topic, geometry_msgs.msg.PoseStamped, queue_size=1)

    def convert_function(self, msg):
        pose_transformed = geometry_msgs.msg.PoseStamped()
        pose_transformed.header.frame_id = 'camera_color_optical_frame'
        pose_transformed.pose.position.x = msg.point.x
        pose_transformed.pose.position.y = msg.point.y
        pose_transformed.pose.position.z = msg.point.z
        pose_transformed.pose.orientation.x = 0
        pose_transformed.pose.orientation.y = 0
        pose_transformed.pose.orientation.z = 0
        pose_transformed.pose.orientation.w = 1
        print (pose_transformed)
        self.point_click_pub.publish(pose_transformed)
        



if __name__=='__main__':
    try:
        rospy.init_node('convertClicked_py')
        
        wc = convertClicked()
        
        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        print ('[convertClicked] Interrupt.',file=sys.stderr)