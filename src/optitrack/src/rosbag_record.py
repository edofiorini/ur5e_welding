#!/usr/bin/env python3

import rospy
import subprocess
import os
import signal
from six.moves import input



if __name__ == '__main__':
    try: 
        rospy.init_node('Rosbag_record')
        rospy.loginfo("Welcome to Rosbag_record node!")
        dirname = os.path.dirname(__file__)
        record_script = os.path.join(dirname, 'record.sh')

        record_folder = os.path.join(os.path.dirname(os.path.dirname(dirname)), 'smooth_trajectory/src/data')

        # Start recording.
        command = "source " + record_script   
        input(
            "============ Press `Enter` to start recording ============"
        )
        p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=record_folder, executable='/bin/bash')

        # Wait for shutdown signal to close rosbag record
        rospy.spin()
        
        exit()
        print("/n")
       
        start_trajectory_script = os.path.join(dirname, 'optitrack_trajectory.sh')


        command = "source " +  start_trajectory_script  
        exit()
        input(
            "============ Press `Enter` to start Trajectory node ============"
        )
        p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=record_folder, executable='/bin/bash')
    except rospy.ROSInterruptException:
        pass
