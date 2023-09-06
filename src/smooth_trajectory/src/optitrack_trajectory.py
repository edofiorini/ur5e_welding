#!/usr/bin/env python3

from sympy import centroid
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
from scipy.spatial.transform import Rotation as R

plt.style.use('default')

#plt.rcParams['text.usetex'] = True



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

    r_start = R.from_rotvec([starting_angle[3], starting_angle[4], starting_angle[5]])
    r_end = R.from_euler('xyz', ending_angle)

    times = np.arange(0, 1, 0.001)
    key_rots = R.concatenate([r_start, r_end])

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

def get_homogeneous_matrix(rot, trans):
        hom_trans = tf.transformations.quaternion_matrix(rot)
        
        hom_trans[0,3] = trans[0]
        hom_trans[1,3] = trans[1]
        hom_trans[2,3] = trans[2]
        return hom_trans

def apply_registration_to_vertices(CAD_vertices, R, t):
    print(np.matmul(R,np.transpose(CAD_vertices)))

    registered_CAD_vertices = np.matmul(R,(np.transpose(CAD_vertices))) + translation
    print(registered_CAD_vertices)
        

def apply_transformation(homogeneous_matrix, vector):

    R = homogeneous_matrix[0:3,0:3]
    one = np.ones(len(np.transpose(vector)))
    vector = np.vstack((vector, one))
    
    #translation = np.array([[homogeneous_matrix[0,3]], [homogeneous_matrix[1,3]], [homogeneous_matrix[2,3]]])
    new_vector = np.matmul(homogeneous_matrix, vector)
    return new_vector[0:3,:]

 
if __name__ == '__main__':	
    try:
        rospy.init_node('Trajectory', anonymous=True)
        transformer = tf.TransformListener()
        br = tf.TransformBroadcaster()
        r = rospy.Rate(50)
    
        rospy.loginfo("Welcome to the node!")

        # Getting data from bag
        if SAVE_CSV:
            print("Export rosbag to csv")
            rosbag_to_csv(DATA_BASE_PATH)


        df = create_data_frame(DATA_BASE_PATH)

        if SAVE_PLOT:
            print("Save plot")
            plot_optitrack_data(df, DATA_BASE_PATH, show_plot = True, save_plot = False)

        print("Loading optitrack data...")
        data_from_optitrack = df.loc[:,["pose.position.x", "pose.position.y", "pose.position.z"]]
        data_from_optitrack.iloc[:,1] += 0.30
        data_from_optitrack.iloc[:,2] += 0.15
        data_from_optitrack.iloc[:,0] -= 0.00
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
        plt.show()
        
        print("Applying point cloud registration...")
        CAD_data_registered, R, t, H = point_cloud_registration_ICP(np.transpose(CAD_data), np.transpose(data_from_optitrack_flatten)) 
        translation = np.array([[t[0]], [t[1]], [t[2]]])
        
        # Apply transformation in optitrack world frame for overlapping the vertices of the cad with the one acquired 
        vertices_registered = apply_registration_to_vertices(CAD_vertices, R, t)
        
        # Transforming the trajectory for welding the object in frame ur_base
        (trans, rot) = transformer.lookupTransform("/ur_base", "/optitrack_world", rospy.Time(0))
        optitrack_to_link0_hom_trans = get_homogeneous_matrix(rot, trans)
        vertices_ur_base = apply_transformation(optitrack_to_link0_hom_trans, vertices_registered)

        Ts = 0.001
        rtde_frequency = 500.0
        rtde_c = RTDEControl("127.0.0.1")#, rtde_frequency, RTDEControl.FLAG_USE_EXT_UR_CAP)
        rtde_r = rtde_receive.RTDEReceiveInterface("127.0.0.1")

        velocity = 0.5
        acceleration = 0.5
        blend = 0.008

        # Go to "home demo" pose
        waypoint_j = [el*np.pi/180 for el in [-91.71, -98.96, -126.22, -46.29, 91.39, -1.78]]
        waypoint_j.extend([0.5, 0.5, 0.0])
        rtde_c.moveJ([waypoint_j])

        #get current robot status
        curr_pose = rtde_r.getActualTCPPose()

        #Define first orientation
        curr_angles = R.from_rotvec([curr_pose[3], curr_pose[4], curr_pose[5]]).as_euler('xyz')
        curr_angles[1] += np.pi/6
        next_angles_rotvec = R.from_euler('xyz', curr_angles).as_rotvec()

        path_c = []

        # 1
        compute_orientation(curr_pose, curr_angles, path_c, curr_pose)
        pi = np.array([[vertices_ur_base[0,0]], [vertices_ur_base[0,0]], [vertices_ur_base[2,0]]])
        pf = np.array([[vertices_ur_base[0,1]], [vertices_ur_base[1,1]], [vertices_ur_base[2,1]]])
        compute_trajectory(pi, pf, path_c, Ts)

        # 2
        curr_angles = R.from_rotvec([curr_pose[3], curr_pose[4], curr_pose[5]]).as_euler('xyz')
        curr_angles[0] -= np.pi/6
        next_angles_rotvec = R.from_euler('xyz', curr_angles).as_rotvec()
        compute_orientation(path_c[-1], curr_angles, path_c, path_c[-1])
        pi = np.array([[vertices_ur_base[0,1]], [vertices_ur_base[1,1]], [vertices_ur_base[2,1]]])
        pf = np.array([[vertices_ur_base[0,2]], [vertices_ur_base[1,2]], [vertices_ur_base[2,2]]])
        compute_trajectory(pi, pf, path_c, Ts)

        # 3
        curr_angles = R.from_rotvec([curr_pose[3], curr_pose[4], curr_pose[5]]).as_euler('xyz')
        curr_angles[1] -= np.pi/6
        next_angles_rotvec = R.from_euler('xyz', curr_angles).as_rotvec()
        compute_orientation(path_c[-1], curr_angles, path_c, path_c[-1])
        pi = np.array([[vertices_ur_base[0,2]], [vertices_ur_base[1,2]], [vertices_ur_base[2,2]]])
        pf = np.array([[vertices_ur_base[0,3]], [vertices_ur_base[1,3]], [vertices_ur_base[2,3]]])
        compute_trajectory(pi, pf, path_c, Ts)

        # 4
        curr_angles = R.from_rotvec([curr_pose[3], curr_pose[4], curr_pose[5]]).as_euler('xyz')
        curr_angles[0] += np.pi/6
        next_angles_rotvec = R.from_euler('xyz', curr_angles).as_rotvec()
        compute_orientation(path_c[-1], curr_angles, path_c, path_c[-1])
        pi = np.array([[vertices_ur_base[0,3]], [vertices_ur_base[1,3]], [vertices_ur_base[2,3]]])
        pf = np.array([[vertices_ur_base[0,4]], [vertices_ur_base[1,4]], [vertices_ur_base[2,4]]])
        compute_trajectory(pi, pf, path_c, Ts)

        # back to the starting orientation
        curr_angles = R.from_rotvec([curr_pose[3], curr_pose[4], curr_pose[5]]).as_euler('xyz')
        compute_orientation(path_c[-1], curr_angles, path_c, path_c[-1])

        #rtde_r.startFileRecording("data.csv")
        rtde_c.moveL(path_c)
        rtde_c.moveL(path_c)
        rtde_c.moveL(path_c)
        rtde_c.stopScript()

    except rospy.ROSInterruptException:
        pass
