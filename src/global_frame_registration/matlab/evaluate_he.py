#!/usr/bin/env python  

from __future__ import print_function

import roslib
import rospy

import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs

import sys

class worldPoseConverter:
    def __init__(self, *args):
        self.__name = rospy.get_name()
        
        self.peg_pose_sub = rospy.Subscriber('/aruco_single_realsense/pose', geometry_msgs.msg.PoseStamped, self.convert_function)
        self.peg_pose_pub = rospy.Publisher('/reprojected/pose', geometry_msgs.msg.PoseStamped, queue_size=100)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def convert_function(self, msg):

        try:
            transform = self.tfBuffer.lookup_transform('world',
                                       'camera_link', #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second
                
            pose_transformed = tf2_geometry_msgs.do_transform_pose(msg, transform)
           
            self.peg_pose_pub.publish(pose_transformed)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('error')


if __name__=='__main__':
    try:
        rospy.init_node('evaluate_he_py')
        
        wc = worldPoseConverter()
        
        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        print ('[worldPoseConverter] Interrupt.',file=sys.stderr)