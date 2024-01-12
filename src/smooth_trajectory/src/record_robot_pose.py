#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import csv
import os

class PoseSubscriberNode:
    def __init__(self):
        rospy.init_node('robot_pose_subscriber_node', anonymous=True)
        rospy.loginfo("Welcome to the record node!")

        csv_name = rospy.get_param('robot_pose_subscriber_node/csv_name')

        self.dirname = os.path.dirname(__file__)
        # Set the path to the folder where you want to save the CSV file
        self.folder_path = os.path.join(os.path.dirname(os.path.dirname(self.dirname)), 'smooth_trajectory/src/csv')
        self.file_path = os.path.join(self.folder_path, csv_name)

        # Create or open the CSV file for writing
        self.csv_file = open(self.file_path, 'w')
        self.csv_writer = csv.writer(self.csv_file)

        # Write header to CSV file
        self.csv_writer.writerow(['timestamp', 'x', 'y', 'z'])

        # Subscribe to the PoseStamped topic
        rospy.Subscriber('/robot_position', PoseStamped, self.pose_callback, queue_size=1)

    def pose_callback(self, msg):
        rospy.loginfo("Writing csv!")
        # Extract position data from the PoseStamped message
        timestamp = msg.header.stamp
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        # Write the data to the CSV file
        self.csv_writer.writerow([timestamp, x, y, z])
        self.csv_file.flush()  # Ensure data is written immediately

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        pose_subscriber = PoseSubscriberNode()
        pose_subscriber.run()
    except rospy.ROSInterruptException:
        pass
