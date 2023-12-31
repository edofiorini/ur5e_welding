#!/usr/bin/env python3

import rospy
import std_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
import numpy as np
import copy
import os
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from CAD_generator import*
from data_CAD_registration import*
from utils import*
import tf
from set_configuration import*
from scipy.spatial.transform import Slerp 
from matplotlib.ticker import (MultipleLocator, AutoMinorLocator)
from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from scipy.spatial.transform import Rotation as Rot

plt.style.use('default')

#plt.rcParams['text.usetex'] = True
global log_index


def vecangle(v1, v2, normal):

    #to_rad = 2*pi/360; 
    xprod = np.cross(v1,v2, axis=0)
    
    c = np.sign(np.vdot(xprod , normal)) * np.linalg.norm(xprod)
    
    angle = np.arctan2(c,np.vdot(v1, v2)) #* to_rad

    return angle


def circular_path(pi, pf, c,  Ts):#, c, P, DP, DDP,DDDP, T, flag):

    rho = np.linalg.norm(pi-c)
    r = np.cross(c - pf, c -pi, axis=0)#c-pf, c-pi)
    
    arc_angle = vecangle(c-pf, c-pi, r)
    plength = rho*arc_angle
        
    s = np.arange(0, plength, Ts)
    p_prime = np.vstack((rho*np.cos((s/rho)), rho*np.sin((s/rho)), np.zeros(len(s))))
    
    
    x_prime = (pi - c)/rho

    norm = np.linalg.norm(r)
    
    z_prime = r/norm

    y_prime = np.cross(x_prime, z_prime, axis=0)
    
    R = np.concatenate((x_prime, y_prime, z_prime), axis=1)
  
    p = c + np.dot(R,p_prime)
    dp = np.dot(R, np.vstack((-np.sin(s/rho), np.cos(s/rho), np.zeros(len(s)))))  
    ddp = np.dot(R, np.vstack((-(1/rho)*np.cos(s/rho),-(1/rho)*np.sin(s/rho),np.zeros(len(s)))))
    dddp = np.dot(R, np.vstack((np.sin(s/rho)/(np.power(rho,2)), -np.cos(s/rho)/(np.power(rho,2)), np.zeros(len(s))))) 

    return p, dp, ddp


def linear_path(pi, pf, Ts):# P, DP, DDP, DDDP, T):
    
    s = np.arange(0, np.linalg.norm(pf - pi), Ts)
    
    p = pi + s*((pf - pi)/(np.linalg.norm(pf - pi)))
    dp = np.ones((3, len(s)))*(pf - pi)/(np.linalg.norm(pf - pi))
    ddp = np.zeros((3, len(s)))
    dddp = np.zeros((3, len(s)))
    
    return p, dp, ddp

def compute_trajectory(pi, pf, path, Ts):

    p, dp, ddp = linear_path(pi, pf, Ts)
    for i in range(0,len(np.transpose(p))):
        if i == 0:
            waypoint = [p[0,i], p[1,i], p[2,i], next_angles_rotvec[0], next_angles_rotvec[1], next_angles_rotvec[2], velocity, acceleration, 0.0]
        else:
            waypoint = [p[0,i], p[1,i], p[2,i], next_angles_rotvec[0], next_angles_rotvec[1], next_angles_rotvec[2], velocity, acceleration, blend]
        path.append(waypoint)
    #return path

def compute_orientation(starting_angle, ending_angle, path, pivoting_pos):
    #starting angle: last orientation in rotvec
    #ending_Angle: next oientation incremented in euler angle
    #pivoting_pose: last position which must be keep equal

    r_start = Rot.from_rotvec([starting_angle[3], starting_angle[4], starting_angle[5]])
    r_end = Rot.from_euler('xyz', ending_angle)

    times = np.arange(0, 1, 0.002)
    key_rots = Rot.concatenate([r_start, r_end])

    key_times = [0, 1]
    slerp = Slerp(key_times, key_rots)

    interp_rots = slerp(times)
    o = interp_rots.as_rotvec()

    for i in range(0, len(o)):
        if i == 0:
                waypoint = [pivoting_pos[0], pivoting_pos[1], pivoting_pos[2], o[i, 0], o[i, 1], o[i, 2], velocity, acceleration, 0.0]
        else:
                waypoint = [pivoting_pos[0], pivoting_pos[1], pivoting_pos[2], o[i, 0], o[i, 1], o[i, 2], velocity, acceleration, blend]
        path.append(waypoint)

def go_to_first_vertex(starting_angle, ending_angle, path, pf, Ts):
     
    r_start = Rot.from_rotvec([starting_angle[3], starting_angle[4], starting_angle[5]])
    r_end = Rot.from_euler('xyz', ending_angle)

    times = np.arange(0, 1, 0.001)
    key_rots = Rot.concatenate([r_start, r_end])

    key_times = [0, 1]
    slerp = Slerp(key_times, key_rots)

    interp_rots = slerp(times)
    o = interp_rots.as_rotvec()

    pi = np.array([[starting_angle[0]], [starting_angle[1]], [starting_angle[2]]])
    p, dp, ddp = linear_path(pi, pf, Ts)
    
    
    for i in range(0, len(o)):
        if i == 0:
                waypoint = [p[0,i], p[1,i], p[2,i], o[i, 0], o[i, 1], o[i, 2], velocity, acceleration, 0.0]
        else:
                waypoint = [p[0,i], p[1,i], p[2,i], o[i, 0], o[i, 1], o[i, 2], velocity, acceleration, blend]
        path.append(waypoint)



def get_homogeneous_matrix(rot, trans):
        hom_trans = tf.transformations.quaternion_matrix(rot)
        
        hom_trans[0,3] = trans[0]
        hom_trans[1,3] = trans[1]
        hom_trans[2,3] = trans[2]
        return hom_trans

def apply_registration_to_vertices(CAD_vertices, R, t):
    #print(np.matmul(R,np.transpose(CAD_vertices)))

    registered_CAD_vertices = np.matmul(R,(np.transpose(CAD_vertices))) + translation
    return registered_CAD_vertices
        

def apply_transformation(homogeneous_matrix, vector):

    R = homogeneous_matrix[0:3,0:3]
    one = np.ones(len(np.transpose(vector)))
    vector = np.vstack((vector, one))
    
    #translation = np.array([[homogeneous_matrix[0,3]], [homogeneous_matrix[1,3]], [homogeneous_matrix[2,3]]])
    new_vector = np.matmul(homogeneous_matrix, vector)
    return new_vector[0:3,:]

def only_vertices(vertices, path):
    curr_pose = rtde_r.getActualTCPPose()
    offset = 0.003

    for i in range(0, len(np.transpose(vertices))):
        print("round", i)
        curr_angles = Rot.from_rotvec([curr_pose[3], curr_pose[4], curr_pose[5]]).as_euler('xyz')

        axis = [1,0, 0,1,0, 1]
        angles = [+np.pi/6,0, -np.pi/6, -np.pi/6, +np.pi/6, +np.pi/6]
        
        curr_angles[axis[i]] += angles[i]
        next_angles_rotvec = Rot.from_euler('xyz', curr_angles).as_rotvec()
             
        if i == 0:
                print("going first vertex", vertices[:,i])
                waypoint = [vertices[0, i] - 0.001, vertices[1, i] - 0.001, vertices[2, i], next_angles_rotvec[0], next_angles_rotvec[1], next_angles_rotvec[2], velocity, acceleration, 0.0]       
        elif i ==1:
                #print("pivoting_pose", curr_pose)
                #compute_orientation(path[-1], curr_angles, path, [vertices[0, i], vertices[1, i], vertices[2, i]])
                #r_end = Rot.from_euler('xyz', curr_angles).as_rotvec()
                #waypoint = [vertices[0, i-1], vertices[1, i-1], vertices[2, i-1], r_end[0], r_end[1], r_end[2], velocity, acceleration, 0.0]
                #path.append(waypoint)
                print("vertex", vertices[:, i])
                waypoint = [vertices[0, i] - offset, vertices[1, i] - 0.001, vertices[2, i], path[-1][3], path[-1][4], path[-1][5], velocity, acceleration, 0.0]
        elif i ==2:
                #print("pivoting_pose", curr_pose)
                #compute_orientation(path[-1], curr_angles, path, [vertices[0, i], vertices[1, i], vertices[2, i]])
                r_end = Rot.from_euler('xyz', curr_angles).as_rotvec()
                waypoint = [vertices[0, i-1] -0.007, vertices[1, i-1] + offset, vertices[2, i-1], r_end[0], r_end[1], r_end[2], velocity, acceleration, 0.0]
                path.append(waypoint)
                print("vertex", vertices[:, i])
                waypoint = [vertices[0, i] -0.006, vertices[1, i] - offset, vertices[2, i], path[-1][3], path[-1][4], path[-1][5], velocity, acceleration, 0.0]
        elif i ==3:
                #print("pivoting_pose", curr_pose)
                #compute_orientation(path[-1], curr_angles, path, [vertices[0, i], vertices[1, i], vertices[2, i]])
                r_end = Rot.from_euler('xyz', curr_angles).as_rotvec()
                waypoint = [vertices[0, i-1] - offset, vertices[1, i-1] - offset - 0.005, vertices[2, i-1], r_end[0], r_end[1], r_end[2], velocity, acceleration, 0.002]
                path.append(waypoint)
                print("vertex", vertices[:, i])
                waypoint = [vertices[0, i] + offset, vertices[1, i] - offset , vertices[2, i], path[-1][3], path[-1][4], path[-1][5], velocity, acceleration, 0.0]
        elif i ==4:
                #print("pivoting_pose", curr_pose)
                #compute_orientation(path[-1], curr_angles, path, [vertices[0, i], vertices[1, i], vertices[2, i]])
                r_end = Rot.from_euler('xyz', curr_angles).as_rotvec()
                waypoint = [vertices[0, i-1] - 0.003, vertices[1, i-1] - offset, vertices[2, i-1], r_end[0], r_end[1], r_end[2], velocity, acceleration, 0.002]
                path.append(waypoint)
                print("vertex", vertices[:, i])
                waypoint = [vertices[0, i] - 0.002, vertices[1, i], vertices[2, i], path[-1][3], path[-1][4], path[-1][5], velocity, acceleration, 0.0]
                
        path.append(waypoint)

def publish_trajectory_to_RVIZ(pub, trajectory):
    ps = PoseArray()
    ps.header.frame_id = "ur/base"
    ps.header.stamp = rospy.Time.now()
    for i in range(0, len(trajectory)):
        if (rospy.is_shutdown()):
            break

        
        pose = Pose()
        pose.position.x = trajectory[i][0]
        pose.position.y = trajectory[i][1]
        pose.position.z = trajectory[i][2]

        quat = Rot.from_rotvec([trajectory[i][3], trajectory[i][4], trajectory[i][5]]).as_quat()
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        
        ps.poses.append(pose)   
    pub.publish(ps)

def logCallback(event):
    tcpPose_msg = PoseStamped()
    trajPose_msg = PoseStamped()
    global log_index

    tcpPose = rtde_r.getActualTCPPose()
    quat = Rot.from_rotvec([tcpPose[3], tcpPose[4], tcpPose[5]]).as_quat()
    tcpPose_msg.header.frame_id = "ur/base"
    tcpPose_msg.header.stamp = rospy.Time.now()
    tcpPose_msg.pose.position.x = tcpPose[0]
    tcpPose_msg.pose.position.y = tcpPose[1]
    tcpPose_msg.pose.position.z = tcpPose[2]
    tcpPose_msg.pose.orientation.x = quat[0]
    tcpPose_msg.pose.orientation.y = quat[1]
    tcpPose_msg.pose.orientation.z = quat[2]
    tcpPose_msg.pose.orientation.w = quat[3]
    pub_current_robot_pose.publish(tcpPose_msg)

    if log_index < (len(path_all_points) - 1):
        if log_index == 0:
            #print("sleep")
            rospy.sleep(2)
        #print("in")
        
        quat = Rot.from_rotvec([path_all_points[log_index][3], path_all_points[log_index][4], path_all_points[log_index][5]]).as_quat()
        trajPose_msg.header.stamp = rospy.Time.now()
        trajPose_msg.header.frame_id = "ur/base"
        trajPose_msg.pose.position.x = path_all_points[log_index][0]
        trajPose_msg.pose.position.y = path_all_points[log_index][1]
        trajPose_msg.pose.position.z = path_all_points[log_index][2]
        trajPose_msg.pose.orientation.x = quat[0]
        trajPose_msg.pose.orientation.y = quat[1]
        trajPose_msg.pose.orientation.z = quat[2]
        trajPose_msg.pose.orientation.w = quat[3]
        pub_reference_traj.publish(trajPose_msg)
        log_index += 1
    

if __name__ == '__main__':	
    try:
        rospy.init_node('Trajectory', anonymous=True)
        transformer = tf.TransformListener()
        br = tf.TransformBroadcaster()
        pub_rviz = rospy.Publisher("smooth_trajectory_RVIZ", PoseArray, queue_size=1)
        pub_current_robot_pose = rospy.Publisher("robot_position", PoseStamped, queue_size=1)
        pub_reference_traj = rospy.Publisher("reference_trajectory", PoseStamped, queue_size=1)
        log_index = 0
    
        rospy.loginfo("Welcome to the node!")

        # Getting data from bag
        if SAVE_CSV:
            print("Export rosbag to csv")
            rosbag_to_csv(DATA_BASE_PATH)


        df = create_data_frame(DATA_BASE_PATH)

        print("Loading optitrack data...")
        data_from_optitrack = df.loc[:,["pose.position.x", "pose.position.y", "pose.position.z"]]
        data_from_optitrack.iloc[:,1] += 0.02
        #data_from_optitrack.iloc[:,2] += 0.20
        # data_from_optitrack.iloc[:,0] -= 0.00
        estimated_vertices = check_optitrack_data(data_from_optitrack)
        
        CAD_data, CAD_vertices = create_CAD_trajectory(data_from_optitrack.to_numpy()[1,:], data_from_optitrack)

        data_from_optitrack_flatten = data_from_optitrack.to_numpy()
        data_from_optitrack_flatten[:,1] = data_from_optitrack.to_numpy()[1,1]
        
        fig	 = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        #ax.scatter(CAD_data[:, 2],  CAD_data[:, 0], CAD_data[:, 1], c='green', marker='o', label='CAD-based desired trajectory')	
        ax.scatter(CAD_vertices[:, 2],  CAD_vertices[:, 0], CAD_vertices[:, 1], c='blue', marker='o', label='CAD data vertices')
        ax.scatter(data_from_optitrack_flatten[:, 2], data_from_optitrack_flatten[:, 0], data_from_optitrack_flatten[:, 1], c='r', marker='o', label='Acquired optitrack trajectory')
        #ax.scatter(data_from_optitrack_flatten[50, 2], data_from_optitrack_flatten[50, 0], data_from_optitrack_flatten[50, 1], c='orange', marker='o', label='optitrack data flatten direction', s=100)
        ax.scatter(CAD_data[50, 2],  CAD_data[50, 0], CAD_data[50, 1], c='purple', marker='o', label='CAD data direction', s=100)
        #plt.title("Acquired data from optitrack and CAD")
        plt.legend()
         
        #ax.xaxis.set_minor_locator(AutoMinorLocator())
        #ax.yaxis.set_minor_locator(AutoMinorLocator())
        #ax.zaxis.set_minor_locator(AutoMinorLocator())
        ax.zaxis.set_rotate_label(False)  # disable automatic rotation
        ax.set_xlabel('\n' + r"$Z$  [m]", fontsize=15, linespacing=3)
        ax.set_ylabel('\n' + r"$X$  [m]", fontsize=15, linespacing=3)
        ax.set_zlabel('\n' + r"$Y$  [m]", fontsize=15, linespacing=3, rotation=90)
        #plt.show()
        
        print("Applying point cloud registration...")
        CAD_data_registered, R, t, H = point_cloud_registration_ICP(np.transpose(CAD_data), np.transpose(data_from_optitrack_flatten)) 
        translation = np.array([[t[0]], [t[1]], [t[2]]])

        fig	 = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        ax.scatter(data_from_optitrack_flatten[:, 2], data_from_optitrack_flatten[:, 0], data_from_optitrack_flatten[:, 1], c='r', marker='o', label='Acquired optitrack trajectory')
        ax.scatter(data_from_optitrack_flatten[200, 2], data_from_optitrack_flatten[200, 0], data_from_optitrack_flatten[200, 1], c='black', marker='o', s= 100, label='Acquired optitrack trajectory')
        ax.scatter(data_from_optitrack_flatten[0, 2], data_from_optitrack_flatten[0, 0], data_from_optitrack_flatten[0, 1], c='black', marker='o', s= 100, label='Acquired optitrack trajectory')
        
        ax.scatter(CAD_data_registered[:, 2], CAD_data_registered[:, 0], CAD_data_registered[:, 1], c='b', marker='o', label='Acquired optitrack trajectory')
        ax.scatter(CAD_data_registered[200, 2], CAD_data_registered[200, 0], CAD_data_registered[200, 1], c='green', marker='o', s= 100, label='Acquired optitrack trajectory')
        ax.scatter(CAD_data_registered[0, 2], CAD_data_registered[0, 0], CAD_data_registered[0, 1], c='green', marker='o', s= 100, label='Acquired optitrack trajectory')
        
        plt.legend()   
        #ax.xaxis.set_minor_locator(AutoMinorLocator())
        #ax.yaxis.set_minor_locator(AutoMinorLocator())
        #ax.zaxis.set_minor_locator(AutoMinorLocator())
        ax.zaxis.set_rotate_label(False)  # disable automatic rotation
        ax.set_xlabel('\n' + r"$Z$  [m]", fontsize=15, linespacing=3)
        ax.set_ylabel('\n' + r"$X$  [m]", fontsize=15, linespacing=3)
        ax.set_zlabel('\n' + r"$Y$  [m]", fontsize=15, linespacing=3, rotation=90)
        #plt.show()
        
        
        
        # Apply transformation in optitrack world frame for overlapping the vertices of the cad with the one acquired 
        vertices_registered = apply_registration_to_vertices(CAD_vertices, R, t)
        print("fir", CAD_vertices)
        print("pro", vertices_registered)
        
        # Transforming the trajectory for welding the object in frame ur_base
        (trans, rot) = transformer.lookupTransform("/ur/base", "/optitrack_world", rospy.Time(0))
        optitrack_to_link0_hom_trans = get_homogeneous_matrix(rot, trans)
        vertices_ur_base = apply_transformation(optitrack_to_link0_hom_trans, vertices_registered)

        print("vertices", vertices_ur_base)
        exit()

        
        Ts = 0.002
        rtde_frequency = 500.0
        rtde_c = RTDEControl("192.168.137.130")# rtde_frequency, RTDEControl.FLAG_USE_EXT_UR_CAP)
        rtde_r = rtde_receive.RTDEReceiveInterface("192.168.137.130")

        velocity = 0.5
        acceleration = 0.5
        blend = 0.007

        # Go to "home demo" pose
        waypoint_j = [el*np.pi/180 for el in [-1.20, -87.64, -86.19, -96.23, 91.22, -0.81]]
        waypoint_j.extend([0.5, 0.5, 0.0])
        rtde_c.moveJ([waypoint_j])

        
        #get current robot status
        curr_pose = rtde_r.getActualTCPPose()

        #Define first orientation
        curr_angles = Rot.from_rotvec([curr_pose[3], curr_pose[4], curr_pose[5]]).as_euler('xyz')
        curr_angles[1] += np.pi/6
        next_angles_rotvec = Rot.from_euler('xyz', curr_angles).as_rotvec()

        path_only_vertex = []
        path_all_points = []

  

        only_vertices(vertices_ur_base, path_only_vertex)

        # 1
        #compute_orientation(curr_pose, curr_angles, path_all_points, curr_pose)

        pi = np.array([[vertices_ur_base[0,0]], [vertices_ur_base[1,0]], [vertices_ur_base[2,0]]])
        pf = np.array([[vertices_ur_base[0,1]], [vertices_ur_base[1,1]], [vertices_ur_base[2,1]]])
        #go_to_first_vertex(curr_pose, curr_angles, path_only_vertex, pi, Ts)
        compute_trajectory(pi, pf, path_all_points, Ts)

        # 2
        curr_angles = Rot.from_rotvec([curr_pose[3], curr_pose[4], curr_pose[5]]).as_euler('xyz')
        curr_angles[0] -= np.pi/6
        next_angles_rotvec = Rot.from_euler('xyz', curr_angles).as_rotvec()
        compute_orientation(path_all_points[-1], curr_angles, path_all_points, path_all_points[-1])
        pi = np.array([[vertices_ur_base[0,1]], [vertices_ur_base[1,1]], [vertices_ur_base[2,1]]])
        pf = np.array([[vertices_ur_base[0,2]], [vertices_ur_base[1,2]], [vertices_ur_base[2,2]]])
        compute_trajectory(pi, pf, path_all_points, Ts)

        # 3
        curr_angles = Rot.from_rotvec([curr_pose[3], curr_pose[4], curr_pose[5]]).as_euler('xyz')
        curr_angles[1] -= np.pi/6
        next_angles_rotvec = Rot.from_euler('xyz', curr_angles).as_rotvec()
        compute_orientation(path_all_points[-1], curr_angles, path_all_points, path_all_points[-1])
        pi = np.array([[vertices_ur_base[0,2]], [vertices_ur_base[1,2]], [vertices_ur_base[2,2]]])
        pf = np.array([[vertices_ur_base[0,3]], [vertices_ur_base[1,3]], [vertices_ur_base[2,3]]])
        compute_trajectory(pi, pf, path_all_points, Ts)

        # 4
        curr_angles = Rot.from_rotvec([curr_pose[3], curr_pose[4], curr_pose[5]]).as_euler('xyz')
        curr_angles[0] += np.pi/6
        next_angles_rotvec = Rot.from_euler('xyz', curr_angles).as_rotvec()
        compute_orientation(path_all_points[-1], curr_angles, path_all_points, path_all_points[-1])
        pi = np.array([[vertices_ur_base[0,3]], [vertices_ur_base[1,3]], [vertices_ur_base[2,3]]])
        pf = np.array([[vertices_ur_base[0,4]], [vertices_ur_base[1,4]], [vertices_ur_base[2,4]]])
        compute_trajectory(pi, pf, path_all_points, Ts)

        # # # back to the starting orientation
        # # curr_angles = Rot.from_rotvec([curr_pose[3], curr_pose[4], curr_pose[5]]).as_euler('xyz')
        # # compute_orientation(path_only_vertex[-1], curr_angles, path_only_vertex, path_only_vertex[-1])

        #print(len(path_only_vertex))
        publish_trajectory_to_RVIZ(pub_rviz, path_only_vertex)

        #print(path_only_vertex)
        plot_traj = np.asarray(path_all_points)
        print(plot_traj[:,0])
        
        fig	 = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(vertices_ur_base[0, :], vertices_ur_base[1, :], vertices_ur_base[2, :], c='r', marker='o', label='Acquired optitrack trajectory', s=100)
        ax.scatter(plot_traj[:, 0], plot_traj[:,1], plot_traj[:, 2], c='b', marker='o', label='Acquired optitrack trajectory')
        plt.legend()   
        #ax.xaxis.set_minor_locator(AutoMinorLocator())
        #ax.yaxis.set_minor_locator(AutoMinorLocator())
        #ax.zaxis.set_minor_locator(AutoMinorLocator())
        ax.zaxis.set_rotate_label(False)  # disable automatic rotation
        ax.set_xlabel('\n' + r"$Z$  [m]", fontsize=15, linespacing=3)
        ax.set_ylabel('\n' + r"$X$  [m]", fontsize=15, linespacing=3)
        ax.set_zlabel('\n' + r"$Y$  [m]", fontsize=15, linespacing=3, rotation=90)
        #plt.show()
    
        rtde_r.startFileRecording("data.csv")
        rospy.Timer(rospy.Duration(0.002), logCallback)
        rtde_c.moveL(path_only_vertex)
        rtde_c.stopScript()

    except rospy.ROSInterruptException:
        pass
