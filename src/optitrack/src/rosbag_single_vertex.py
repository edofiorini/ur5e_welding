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
        record_script = os.path.join(dirname, 'record_single_vertex.sh %s')

        record_folder = os.path.join(os.path.dirname(os.path.dirname(dirname)), 'smooth_trajectory/src/vertices_data')

        # Start recording.
        num_vertices = 4
        for i in range(0,num_vertices):
            vertex_name = "vline_" + str(i)
            command = "source " + record_script   
            input("============ Press `Enter` to START recording vertex number " + str(i) + " ============")
            p = subprocess.Popen([command % (vertex_name)], stdin=subprocess.PIPE, shell=True, cwd=record_folder, executable='/bin/bash')
            input("============ Press `Enter` to END recording vertex number " + str(i) + " ============")
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)  
            
            # Wait for shutdown signal to close rosbag record
            rospy.spin()
        
        exit()
    except rospy.ROSInterruptException:
        pass
