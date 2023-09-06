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
from move_group_interface import MoveGroupInterface
from primitives import*
from CAD_generator import*
from data_CAD_registration import*
from utils import*
import tf
from set_configuration import*
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp 
from matplotlib.ticker import (MultipleLocator, AutoMinorLocator)

plt.style.use('default')

#plt.rcParams['text.usetex'] = True

Ts = 0.001
current_pose = PoseStamped()
counter_pose = 0

def compute_centroid(CAD_trajectory):
    # CAD_trajectory is all the trajectory computed starting from CAD
    
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([0,0,0,1])
    #roll, pitch, yaw = TF.transformations.euler_from_matrix(matrix_1, axes='sxyz') #zyx | roll pitch yaw xyz | zyz 
    centroid_orientation = np.vstack((roll, pitch, yaw))
    
    
    if CAD_OBJECT_SIZE == "rectangle_doublecircle" or CAD_OBJECT_SIZE == "deep_rectangle_doublecircle":

        # y axis is the deep of the object, so you have to add that offset 
        # x and z are computed respect to the first point of the trajectory. 
        # So it depends on from where you started the trajectory
        #centroid_position = np.array([CAD_trajectory[0,0] - 0.065, CAD_trajectory[1,0] - 0.006, CAD_trajectory[2,0]-0.015])
        centroid_position = np.array([CAD_trajectory[0,0] - 0.05, CAD_trajectory[1,0] - 0.006, CAD_trajectory[2,0]-0.03])
        
    else:	

        # In this case is just the mean along the three axis
        length = len(np.transpose(CAD_trajectory))#.shape[0]
        sum_x = np.sum(CAD_trajectory[0, :])
        sum_y = np.sum(CAD_trajectory[1, :])
        sum_z = np.sum(CAD_trajectory[2, :])
        if CAD_OBJECT_SIZE == "circle":
            centroid_position = np.array([sum_x/length, sum_y/length - 0.006, sum_z/length])
        else:	
            centroid_position = np.array([sum_x/length, sum_y/length, sum_z/length])
        
    return centroid_position, centroid_orientation

def generate_smooth_trajectory(vertices, primitive_type, rotation_type, time, rot_panda_world):
    
    
    position = np.array([])
    orientation = np.array([])
    
    if CAD_OBJECT_SIZE == "rectangle_doublecircle":

        radius_1, radius_2 = 0.035, 0.02
        c_1 = np.array([[vertices[2,0]], [vertices[2,1]], [vertices[2,2] - radius_1]])
        c_2 = np.array([[vertices[5,0]], [vertices[5,1]], [vertices[5,2] + radius_2]])
        for i in range(0,len(vertices)):
            if primitive_type[i] == 0:
                if i == len(vertices)-1:
                    a = np.array([[vertices[i,0]],[vertices[i,1]],[vertices[i,2]]])
                    b = np.array([[vertices[0,0]],[vertices[0,1]],[vertices[0,2]]])
                    p, dp, ddp = linear_trajectory(a, b ,Ts, time[i][0], time[i][1])
                    o, do, ddo, _  = set_orientation(time[i][0], time[i][1], Ts, rotation_type[i], rot_panda_world, get_euler_mode=True)
                    position = np.append(position, p, axis=0)
                    orientation = np.append(orientation, o, axis=0)
                        
                else:
                    a = np.array([[vertices[i,0]],[vertices[i,1]],[vertices[i,2]]])
                    b = np.array([[vertices[i+1,0]],[vertices[i+1,1]],[vertices[i+1,2]]])
                    p, dp, ddp = linear_trajectory(a, b,Ts, time[i][0], time[i][1])
                    o, do, ddo, _ = set_orientation(time[i][0], time[i][1], Ts, rotation_type[i], rot_panda_world, get_euler_mode=True)
                    
                    if i == 0:
                        position = p
                        orientation = o
                        
                    else:
                        
                        position = np.append(position, p, axis=1)
                        orientation = np.append(orientation, o, axis=1)
                        
                    if i == 0 or i == 3:
                        
                        R_start = Rotation.from_quat([orientation[0,-1], orientation[1,-1], orientation[2,-1], orientation[3,-1]])
                        R_end = set_orientation(time[i][0], time[i][1], Ts, rotation_type[i+1], rot_panda_world, get_euler_mode=False) 

                        o, do, ddo = EE_orientation(R_start, R_end, Ts,  time[i][1], time[i][1] + 3, 1)
                        
                        
                        orientation = np.append(orientation, np.transpose(o), axis=1)
                        
                        p = np.ones((3,len(o)))
                        p[0,:] = position[:,-1][0]
                        p[1,:] = position[:,-1][1]
                        p[2,:] = position[:,-1][2]
                        position = np.append(position, p, axis=1)

            else:
                if i == len(vertices)-1:
                    a = np.array([[vertices[i,0]],[vertices[i,1]],[vertices[i,2]]])
                    b = np.array([[vertices[0,0]],[vertices[0,1]],[vertices[0,2]]])					
                    o_1, do_1, ddo_1, _ = set_orientation(time[i][0], time[i][1], Ts, rotation_type[i], rot_panda_world, get_euler_mode=True)
                    p_1, dp_1, ddp_1 = circular_trajectory(a, b, Ts, c_2, time[i][0], time[i][1])
                    position = np.append(position, p_1, axis=1)
                    orientation = np.append(orientation, np.transpose(o_1), axis=1)
                else:
                    
                    a = np.array([[vertices[i,0]],[vertices[i,1]],[vertices[i,2]]])
                    b = np.array([[vertices[i+1,0]],[vertices[i+1,1]],[vertices[i+1,2]]])
                    o_1, do_1, ddo_1, _ = set_orientation(time[i][0], time[i][1], Ts, rotation_type[i], rot_panda_world, get_euler_mode=True)
                    p_1, dp_1, ddp_1 = circular_trajectory(a, b, Ts, c_1, time[i][0], time[i][1])
                    position = np.append(position, p_1, axis=1)
                    orientation = np.append(orientation, np.transpose(o_1), axis=1)
        
    elif CAD_OBJECT_SIZE == "rectangle" or CAD_OBJECT_SIZE == "B3":
                
        for i in range(0,len(vertices)-1):
            print(i)
            if primitive_type[i] == 0:
                a = np.array([[vertices[i,0]],[vertices[i,1]],[vertices[i,2]]])
                b = np.array([[vertices[i+1,0]],[vertices[i+1,1]],[vertices[i+1,2]]])
                
                p, dp, ddp = linear_trajectory(a, b,Ts, time[i][0], time[i][1])
                o, do, ddo, _ = set_orientation(time[i][0], time[i][1], Ts, rotation_type[i], rot_panda_world, get_euler_mode=True)
                
                if i == 0:
                    position = p
                    orientation = o
                else:
                    position = np.append(position, p, axis=1)
                    orientation = np.append(orientation, o, axis=1)

                if i < len(vertices)-2:
                    R_start = Rotation.from_quat([orientation[0,-1], orientation[1,-1], orientation[2,-1], orientation[3,-1]])
                    R_end = set_orientation(time[i][0], time[i][1], Ts, rotation_type[i+1], rot_panda_world, get_euler_mode=False) 
                    #print("turn", i, R_start.as_quat(), R_end.as_quat())
                    o, do, ddo = EE_orientation(R_start, R_end, Ts,  time[i][1], time[i][1] + 3, 1)
                    #print("before adding the pointer", orientation)
                    #print("slerp", o)
                    orientation = np.append(orientation, np.transpose(o), axis=1)
                    
                    p = np.ones((3,len(o)))
                    p[0,:] = position[:,-1][0]
                    p[1,:] = position[:,-1][1]
                    p[2,:] = position[:,-1][2]
                    position = np.append(position, p, axis=1)
                    print("POSITION",p, position)
        
    elif CAD_OBJECT_SIZE == "deep_rectangle_doublecircle":
        
        radius_1, radius_2 = 0.035, 0.02
        c_1 = np.array([[vertices[2,0]], [vertices[2,1]], [vertices[2,2] - radius_1]])
        c_2 = np.array([[vertices[5,0]], [vertices[5,1]], [vertices[5,2] + radius_2]])
        
        for i in range(0,len(vertices)):
            if primitive_type[i] == 0:			
                if i == len(vertices)-1:
                    a = np.array([[vertices[0,0]],[vertices[0,1]],[vertices[0,2]]])
                    b = np.array([[vertices[i,0]],[vertices[i,1]],[vertices[i,2]]])
                    p, dp, ddp = linear_trajectory(a, b,Ts, time[i][0], time[i][1])
                    o, do, ddo, _  = set_orientation(time[i][0], time[i][1], Ts, rot_panda_world, rotation_type[i])
                    position = np.append(position, p, axis=1)
                    orientation = np.append(orientation, o, axis=1)
                        
                else:
                    a = np.array([[vertices[i,0]],[vertices[i,1]],[vertices[i,2]]])
                    b = np.array([[vertices[i+1,0]],[vertices[i+1,1]],[vertices[i+1,2]]])
                    p, dp, ddp = linear_trajectory(a, b,Ts, time[i][0], time[i][1])
                    o, do, ddo, _ = set_orientation(time[i][0], time[i][1], Ts, rot_panda_world, rotation_type[i])
                    if i == 0:
                        position = p
                        orientation = o
                    else:
                        position = np.append(position, p, axis=1)
                        orientation = np.append(orientation, o, axis=1)
                        
            else:
                if i == len(vertices)-2:
                    a = np.array([[vertices[i,0]],[vertices[i,1]],[vertices[i,2]]])
                    b = np.array([[vertices[0,0]],[vertices[0,1]],[vertices[0,2]]])
                    o_1, do_1, ddo_1, _ = set_orientation(time[i][0], time[i][1], Ts, rot_panda_world, rotation_type[i])
                    p_1, dp_1, ddp_1 = circular_trajectory(a, b, Ts, c_2, time[i][0], time[i][1])
                    position = np.append(position, p_1, axis=1)
                    orientation = np.append(orientation, o_1, axis=1)
                else:
                    a = np.array([[vertices[i,0]],[vertices[i,1]],[vertices[i,2]]])
                    b = np.array([[vertices[i+1,0]],[vertices[i+1,1]],[vertices[i+1,2]]])
                    o_1, do_1, ddo_1, _ = set_orientation(time[i][0], time[i][1], Ts, rot_panda_world, rotation_type[i])
                    p_1, dp_1, ddp_1 = circular_trajectory(a, b, Ts, c_1, time[i][0], time[i][1])
                    position = np.append(position, p_1, axis=1)
                    orientation = np.append(orientation, o_1, axis=1)

    elif CAD_OBJECT_SIZE == "circle":
        radius = 0.13/2
        c = np.array([[vertices[0,0] + radius], [vertices[0,1]], [vertices[0,2] ]])
        for i in range(0,len(vertices)-1):
            a = np.array([[vertices[i,0]],[vertices[i,1]],[vertices[i,2]]])
            b = np.array([[vertices[i+1,0]],[vertices[i+1,1]],[vertices[i+1,2]]])
            p, dp_1, ddp_1 = circular_trajectory(a, b, Ts, c, time[i][0], time[i][1])

            R_start = set_orientation(time[i][0], time[i][1], Ts, rotation_type[i], get_euler_mode=False)
            if i == len(vertices)-2:
                R_end = set_orientation(time[i][0], time[i][1], Ts, rotation_type[0], rot_panda_world, get_euler_mode=False) 
            else:
                R_end = set_orientation(time[i][0], time[i][1], Ts, rotation_type[i+1], rot_panda_world, get_euler_mode=False)
            o, do, ddo = EE_orientation(R_start, R_end, Ts,  time[i][0], time[i][1], 1)
                    
            if i == 0:
                position = p
                orientation = np.transpose(o)
            else:
                position = np.append(position, p, axis=1)
                orientation = np.append(orientation, np.transpose(o), axis=1)
                

    # fig	 = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.scatter(position[0,:], position[1,:], position[2,:], c='black', marker='o', label='Robot Trajectory position from CAD')
    # #ax.scatter(position[0,0], position[1,0], position[2,0], c='red', marker='o', label='First Robot Trajectory point from CAD', s=100)
    # ax.scatter(vertices[:,0], vertices[:,1], vertices[:,2], c='blue', marker='o', label='First Robot Trajectory point from CAD', s=100)
    # #plt.title("Robot Trajectory position")
    # #plt.legend()
    # ax.zaxis.set_rotate_label(False)  # disable automatic rotation
    # ax.set_xlabel('\n' + r"$X$  [m]", fontsize=15, linespacing=3)
    # ax.set_ylabel('\n' + r"$Y$  [m]", fontsize=15, linespacing=3)
    # ax.set_zlabel('\n' + r"$Z$  [m]", fontsize=15, linespacing=3, rotation=90)
    # #plt.title(r"$\mathbf{Mixed \quad double \quad circle}$", fontsize=10)
    # #plt.zlabel(r"$\mathbf{Z}$")
    # plt.show()
    
    
    return position, orientation

def get_current_robot_pose(msg):
    global current_pose
    current_pose = msg

def go_to_object(first_traj_point, start_orientation, time):

    # position
    if SIMULATION:
        (trans,rot) = transformer.lookupTransform("/panda_link0", "/robotic_welding_tool", rospy.Time(0))
        print(rot)
        

        current_robot_position = np.array([[trans[0]],[trans[1]],[trans[2]]])
        
        current_robot_orientation_quat = np.array([rot[0], rot[1], rot[2], rot[3]])
        
    else:
        current_robot_position = np.array([[current_pose.pose.position.x],[current_pose.pose.position.y],[current_pose.pose.position.z]])
        current_robot_orientation_quat = np.array([current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w ])
    first_vertex_position = np.array([[first_traj_point[0]],[first_traj_point[1]],[first_traj_point[2]]])
    
    # orientation
    
    
    
    
    
    first_vertex_orientation_quat = start_orientation 
    
    r_start = Rotation.from_quat([current_robot_orientation_quat[0], current_robot_orientation_quat[1], current_robot_orientation_quat[2], current_robot_orientation_quat[3] ])
    r_end = Rotation.from_quat([first_vertex_orientation_quat[0], first_vertex_orientation_quat[1], first_vertex_orientation_quat[2], first_vertex_orientation_quat[3] ])
    
    s, sd, sdd, sddd, s_tmp = fifth_polynomials(0, np.linalg.norm(first_vertex_position - current_robot_position), 0, 0, 0, 0, time[0][0], time[0][1], Ts)
    
    times = np.arange(time[0][0], time[0][1], 0.001)
    
    
    key_rots = Rotation.concatenate([r_start, r_end])
    
    key_times = [0, 20]
    slerp = Slerp(key_times, key_rots)
    
    interp_rots = slerp(times)
    o = interp_rots.as_quat()
    
    o_euler = interp_rots.as_euler('zyx', degrees=False)
    p, dp, ddp = linear_trajectory(current_robot_position, first_vertex_position, Ts, time[0][0], time[0][1])
    
    return p, o, np.transpose(o_euler)

def publish_trajectory_to_PD_controller(pub, position, orientation, rr):

    ps = PoseArray()
    ps.header.frame_id = "/panda_link0"
    ps.header.stamp = rospy.Time.now()
    for i in range(0, np.min([len(np.transpose(position)),len(orientation)])):
        if (rospy.is_shutdown()):
            break

        pose = Pose()
        pose.position.x = position[0,i]
        pose.position.y = position[1,i]
        pose.position.z = position[2,i] 
        pose.orientation.x = orientation[i, 0]
        pose.orientation.y = orientation[i, 1]
        pose.orientation.z = orientation[i, 2]
        pose.orientation.w = orientation[i, 3]

        ps.poses.append(pose)   
    pub.publish(ps)

def publish_trajectory_to_RVIZ(pub_1, position, quaternion):

    
    ps = PoseArray()
    ps.header.frame_id = "world"
    ps.header.stamp = rospy.Time.now()
    for i in range(0, len(np.transpose(position)), 1500):
        if (rospy.is_shutdown()):
            break

         # rpy
        
        pose = Pose()
        pose.position.x = position[0,i]
        pose.position.y = position[1,i]
        pose.position.z = position[2,i]
        pose.orientation.x = quaternion[0,i]
        pose.orientation.y = quaternion[1,i]
        pose.orientation.z = quaternion[2,i]
        pose.orientation.w = quaternion[3,i]

        ps.poses.append(pose)   
    pub_1.publish(ps)
    

def get_homogeneous_matrix(rot, trans):
        hom_trans = tf.transformations.quaternion_matrix(rot)
        
        hom_trans[0,3] = trans[0]
        hom_trans[1,3] = trans[1]
        hom_trans[2,3] = trans[2]
        return hom_trans

        

def apply_transformation(homogeneous_matrix, vector):

    R = homogeneous_matrix[0:3,0:3]
    one = np.ones(len(np.transpose(vector)))
    vector = np.vstack((vector, one))
    
    #translation = np.array([[homogeneous_matrix[0,3]], [homogeneous_matrix[1,3]], [homogeneous_matrix[2,3]]])
    new_vector = np.matmul(homogeneous_matrix, vector)
    return new_vector[0:3,:]

def pub_centroid(event):
    
    br.sendTransform((CAD_centroid_position[0], CAD_centroid_position[1], CAD_centroid_position[2]),(0.0, 0.0, 0.0, 1.0), rospy.Time.now(),"CAD", "optitrack_world")
    
def pub_registered_centroid(event):
    
    br.sendTransform((registered_CAD_centroid_position[0], registered_CAD_centroid_position[1], registered_CAD_centroid_position[2]),(registered_centroid_quaternion[0], registered_centroid_quaternion[1], registered_centroid_quaternion[2], registered_centroid_quaternion[3]), rospy.Time.now(),"registered_CAD", "optitrack_world")

def pub_current_pose_simulation(event):
    global counter_pose
    (trans,rot) = transformer.lookupTransform("/panda_link0", "/robotic_welding_tool", rospy.Time(0))
    current_pose = PoseStamped()
    current_pose.header.stamp = rospy.Time.now()
    current_pose.pose.position.x = trans[0]
    current_pose.pose.position.y = trans[1]
    current_pose.pose.position.z = trans[2]
    pub_sim_pose.publish(current_pose)

    if counter_pose < len(np.transpose(complete_traj_position_in_link0)):
        desired_pose = PoseStamped()
        desired_pose.header.stamp = rospy.Time.now()
        desired_pose.pose.position.x = complete_traj_position_in_link0[0,counter_pose]
        desired_pose.pose.position.y = complete_traj_position_in_link0[1,counter_pose]
        desired_pose.pose.position.z = complete_traj_position_in_link0[2,counter_pose]
        print(desired_pose, counter_pose, "of", len(np.transpose(complete_traj_position_in_link0)))
        pub_sim_desired_pose.publish(desired_pose)
        counter_pose += 1
 
if __name__ == '__main__':	
    try:
        rospy.init_node('Trajectory', anonymous=True)
        transformer = tf.TransformListener()
        br = tf.TransformBroadcaster()
        rospy.Subscriber("/panda_force_ctrl/current_pose", PoseStamped, get_current_robot_pose)
        pub_1 = rospy.Publisher("smooth_trajectory_RVIZ", PoseArray, queue_size=1)
        pub = rospy.Publisher("smooth_trajectory", PoseArray, queue_size=1)
        pub_sim_pose = rospy.Publisher("panda_simulation_pose", PoseStamped, queue_size=1)
        pub_sim_desired_pose = rospy.Publisher("panda_simulation_desired_pose", PoseStamped, queue_size=1)
        rr = rospy.Rate(50)
    
        rospy.loginfo("Welcome to the node!")
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
        
        print("here")
        fig	 = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        #ax.scatter(CAD_data[:, 2],  CAD_data[:, 0], CAD_data[:, 1], c='green', marker='o', label='CAD-based desired trajectory')	
        #ax.scatter(CAD_vertices[:, 2],  CAD_vertices[:, 0], CAD_vertices[:, 1], c='blue', marker='o', label='CAD data vertices')
        ax.scatter(data_from_optitrack_flatten[:, 2], data_from_optitrack_flatten[:, 0], data_from_optitrack_flatten[:, 1], c='r', marker='o', label='Acquired optitrack trajectory')
        #ax.scatter(data_from_optitrack_flatten[50, 2], data_from_optitrack_flatten[50, 0], data_from_optitrack_flatten[50, 1], c='orange', marker='o', label='optitrack data flatten direction', s=100)
        #ax.scatter(CAD_data[50, 2],  CAD_data[50, 0], CAD_data[50, 1], c='purple', marker='o', label='CAD data direction', s=100)
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
        exit()
        print("Applying point cloud registration...")
        #CAD_data_registered = point_cloud_registration_TEASER(CAD_data, data_from_optitrack_flatten)
        CAD_data_registered, R, t, H = point_cloud_registration_ICP(np.transpose(CAD_data), np.transpose(data_from_optitrack_flatten)) 
        translation = np.array([[t[0]], [t[1]], [t[2]]])
        

        # Compute centroid from CAD position and orientation w.r.t optitrack_world frame
        CAD_centroid_position, CAD_centroid_orientation  = compute_centroid(np.transpose(CAD_data))
        rospy.Timer(rospy.Duration(0.001), pub_centroid)

        # Apply registration w.r.t optitrack_world frame
        registered_CAD_centroid_position = np.matmul(R,np.array([[CAD_centroid_position[0]], [CAD_centroid_position[1]], [CAD_centroid_position[2]]])) + translation
        registered_centroid_euler = tf.transformations.euler_from_matrix(R)
        registered_centroid_quaternion = tf.transformations.quaternion_from_euler(registered_centroid_euler[0], registered_centroid_euler[1], registered_centroid_euler[2] )
        rospy.Timer(rospy.Duration(0.01), pub_registered_centroid)
        
       
        (trans_panda_world, rot_panda_world) = transformer.lookupTransform("/panda_link0", "/world", rospy.Time(0))
        move_interface = MoveGroupInterface()
        
        print("Computing smooth trajectory...")
        if CAD_OBJECT_SIZE == "rectangle_doublecircle":
            # Loading scene on rviz	
            dirname = os.path.dirname(__file__)
            scene_mesh = os.path.join(dirname, 'scene_mesh/mixed_shape_flipped_2.stl')
            move_interface.add_mesh('object', registered_CAD_centroid_position, registered_centroid_quaternion, scene_mesh)

            primitive_type = np.array([0,0,1,0,0,1])
            rotation_type = ['y', 'yx', 'yxy_circ', 'yxy', 'yxyx', 'yxyx_circ']
            time = [(0,5), (8,13), (13,18), (18,23), (25,30), (30, 35) ]
        elif CAD_OBJECT_SIZE == "rectangle" or CAD_OBJECT_SIZE == "B3":
            # Loading scene on rviz
            if CAD_OBJECT_SIZE == "rectangle":
                move_interface.add_box('box', np.transpose(registered_CAD_centroid_position), registered_centroid_quaternion, size=(0.2, 0.008, 0.2))  #0.29,0.001, 0.21
            else:
                move_interface.add_box('box', np.transpose(registered_CAD_centroid_position), registered_centroid_quaternion, size=(0.055, 0.001, 0.085))  #0.29,0.001, 0.21
            primitive_type = np.array([0,0,0,0])
            rotation_type = ['y', 'yx', 'yxy', 'yxyx' ]
            time = [(0,7), (10,17), (20,27), (30,37) ]
        elif CAD_OBJECT_SIZE == "deep_rectangle_doublecircle":
            # Loading scene on rviz
            dirname = os.path.dirname(__file__)
            scene_mesh = os.path.join(dirname, 'scene_mesh/mixed_shape_flipped.stl')
            move_interface.add_mesh('object', registered_CAD_centroid_position, registered_centroid_quaternion, scene_mesh)

            primitive_type = np.array([0,0,1,0,0,1,0])
            rotation_type = ['y', 'yx', 'yxy_circ', 'yxy', 'yxyx', 'yxyx_circ', 'y' ]
            time = [(0,5), (5,10), (10,15), (15,20), (20,25), (25, 30), (30,35) ]
        elif CAD_OBJECT_SIZE == "circle":
            # Loading scene on rviz
            dirname = os.path.dirname(__file__)
            scene_mesh = os.path.join(dirname, 'scene_mesh/circle_shape_flipped.stl')
            move_interface.add_mesh('object', registered_CAD_centroid_position, registered_centroid_quaternion, scene_mesh)

            primitive_type = np.array([1,1,1,1])
            rotation_type = ['y', 'yx', 'yxy', 'yxyx' ]
            time = [(0,5), (5,10), (10,15), (15,20) ]	
        
        
        
        #Computing trajectory in optitrack world setting time and kind of orientation
        #orientation is already in panda_link0 frame
        traj_position, orientation_link0_quat = generate_smooth_trajectory((CAD_vertices), primitive_type, rotation_type, time, rot_panda_world)
        print("The size of trajectory is: POSITION", len(np.transpose(traj_position)),"ORIENTATION", len(np.transpose(orientation_link0_quat)) )
        
        
        # Apply transformation in optitrack world frame for overlapping data acquired and trajectory generated from CAD 
        registered_traj_position = np.matmul(R,(traj_position)) + translation
        
        # Time = np.arange(0, 37, Ts) 
        # fig	 = plt.figure()
        # ax_1 = fig.add_subplot(311)
        # ax_1.scatter(Time,  registered_traj_position[2,:], c='green', marker='o', label='Registrated CAD-based desired trajectory',s=5)
        # ax_1.scatter(Time,  traj_position[2,:], c='blue', marker='o', label='Registrated CAD-based desired trajectory', s=5)
        # ax_2 = fig.add_subplot(312)
        # ax_2.scatter(Time,  registered_traj_position[1,:], c='green', marker='o', label='Registrated CAD-based desired trajectory', s=5)
        # ax_2.scatter(Time,  traj_position[1,:], c='blue', marker='o', label='Registrated CAD-based desired trajectory', s=5)
        # ax_3 = fig.add_subplot(313)
        # ax_3.scatter(Time,  registered_traj_position[2,:], c='green', marker='o', label='Registrated CAD-based desired trajectory', s=5)
        # ax_3.scatter(Time,  traj_position[2,:], c='blue', marker='o', label='Registrated CAD-based desired trajectory', s=5)
        # plt.show()
        #plt.show()
        # Plotting result
        # fig	 = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # ax.scatter(registered_traj_position[2,:], registered_traj_position[0,:], registered_traj_position[1,:], c='green', marker='o', label='Registrated CAD-based desired trajectory')
        # ax.scatter(data_from_optitrack_flatten[:, 2], data_from_optitrack_flatten[:, 0], data_from_optitrack_flatten[:, 1], c='red', marker='o', label='Acquired optitrack trajectory')
        # #ax.scatter(registered_CAD_centroid_position[2], registered_CAD_centroid_position[0], registered_CAD_centroid_position[1],  c='green', marker='o', label='centroid')
        # #ax.scatter(registered_traj_position[2,0], registered_traj_position[0,0], registered_traj_position[1,0], c='red', marker='o', label='First registrated robot trajectory point', s=100)
        # #ax.scatter(registered_traj_position[2,100], registered_traj_position[0,100], registered_traj_position[1,100], c='purple', marker='o', label='Registrated robot trajectory direction', s=100)
        # #ax.scatter(data_from_optitrack_flatten[0, 2], data_from_optitrack_flatten[0, 0], data_from_optitrack_flatten[0, 1], c='red', marker='o', label='First acquired optitrack data point', s=100)
        # #ax.scatter(data_from_optitrack_flatten[50, 2], data_from_optitrack_flatten[50, 0], data_from_optitrack_flatten[50, 1], c='orange', marker='o', label='acquired optitrack data direction', s=100)
        # #plt.title("Robot Trajectory after registration and acquired data from optitrack")
        # plt.legend()
        # ax.zaxis.set_rotate_label(False)  # disable automatic rotation
        # ax.set_xlabel('\n' + r"$Z$  [m]", fontsize=15, linespacing=3)
        # ax.set_ylabel('\n' + r"$X$  [m]", fontsize=15, linespacing=3)
        # ax.set_zlabel('\n' + r"$Y$  [m]", fontsize=15, linespacing=3, rotation=90)
        # plt.show() 
        
        
        
        # Transforming the trajectory for welding the object in frame panda_link0
        (trans, rot) = transformer.lookupTransform("/panda_link0", "/optitrack_world", rospy.Time(0))
        optitrack_to_link0_hom_trans = get_homogeneous_matrix(rot, trans)
        position_in_link0 = apply_transformation(optitrack_to_link0_hom_trans, registered_traj_position)
        
        
        # Time = np.arange(0, 37, Ts) 
        # fig	 = plt.figure()
        # ax_1 = fig.add_subplot(311)
        # ax_1.scatter(Time,  position_in_link0[0,:], c='green', marker='o', label='Registrated CAD-based desired trajectory',s=5)
        # ax_2 = fig.add_subplot(312)
        # ax_2.scatter(Time,  position_in_link0[1,:], c='green', marker='o', label='Registrated CAD-based desired trajectory', s=5)
        # ax_3 = fig.add_subplot(313)
        # ax_3.scatter(Time,  position_in_link0[2,:], c='green', marker='o', label='Registrated CAD-based desired trajectory', s=5)
        # plt.show()
        
        
        if SIMULATION:
            print("Planning...")
            constraints = move_interface.impose_path_constraints()
            cartesian_plan, fraction = move_interface.plan_cartesian_path(position_in_link0, orientation_link0_quat, pub_sim_desired_pose, constraints)
            print(type(cartesian_plan))
            
            # alpha= 150 rospy.Timer(rospy.Duration(0.03355705), pub_current_pose_simulation)
            #rospy.Timer(rospy.Duration(0.03448278), pub_current_pose_simulation)
            rospy.Timer(rospy.Duration(0.1), pub_current_pose_simulation)
            #rospy.Timer(rospy.Duration(0.03333), pub_current_pose_simulation)
            #rospy.Timer(rospy.Duration(0.03703704), pub_current_pose_simulation)
            # rospy.Timer(rospy.Duration(0.0060423), pub_current_pose_simulation)
            
            print("Executing...")
            move_interface.execute_plan(cartesian_plan)
            #rospy.sleep(30)
        else:
            # Going from actual robot pose to the object to weld already in panda_link0
            home_to_obj_pos, home_to_obj_orie, home_to_obj_orie_euler  = go_to_object(position_in_link0[:,0], orientation_link0_quat[:,0], [(0,20)])
            
            print("#####", home_to_obj_orie, "##", orientation_link0_quat)
            
            # orientation_link0_euler = ([])
            # for i in range(0, len(np.transpose(orientation_link0_quat))):
            #     angle = tf.transformations.euler_from_quaternion([orientation_link0_quat[0,i], orientation_link0_quat[1,i], orientation_link0_quat[2,i], orientation_link0_quat[3,i] ], axes='szyx')
            #     orientation_link0_euler = np.append(orientation_link0_euler, angle, axis=0)                
            # orientation_link0_euler = np.reshape(orientation_link0_euler, (int(len(orientation_link0_euler)/3),3))
            
            # Merging the two different trajectories in panda_link0
            complete_traj_position_in_link0 = np.append(home_to_obj_pos, position_in_link0, axis=1)
            
            # Keeping orientation in euler angle
            #complete_traj_orientation_in_link0_euler = np.append(home_to_obj_orie_euler, np.transpose(orientation_link0_euler), axis=1)
            
            # Keeping orientation in quaternion
            complete_traj_orientation_in_link0_quat = np.append(home_to_obj_orie, np.transpose(orientation_link0_quat), axis=0)
           
            fig	 = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.scatter(complete_traj_position_in_link0[2,:], complete_traj_position_in_link0[0,:], complete_traj_position_in_link0[1,:], c='black', marker='o', label='Registrated robot trajectory')
            plt.title("Complete Robot Trajectory")
            plt.legend()
            plt.show() 	

            (trans, rot) = transformer.lookupTransform("/world", "/panda_link0", rospy.Time(0))
            link0_to_world_hom_trans = get_homogeneous_matrix(rot, trans)

            position_rviz = np.zeros((3,len(np.transpose(complete_traj_position_in_link0))))
            orientation_rviz = np.zeros((4,len(np.transpose(complete_traj_position_in_link0))) )
            for i in range(0, len(np.transpose(complete_traj_position_in_link0))):
               tmp = get_homogeneous_matrix(complete_traj_orientation_in_link0_quat[i,:], complete_traj_position_in_link0[:,i])
               res = np.dot(link0_to_world_hom_trans, tmp)
               rot = res[0:3,0:3]
               tras = res[0:3,3]
               quat = Rotation.from_matrix(rot).as_quat()   
               position_rviz[:,i] = tras
               orientation_rviz[:,i] = quat
            
            #for i in range(0, len(np.transpose(complete_traj_orientation_in_link0_euler))):
                #quaternion = tf.transformations.quaternion_from_euler(complete_traj_orientation_in_link0_euler[0,i], complete_traj_orientation_in_link0_euler[1,i], complete_traj_orientation_in_link0_euler[2,i], axes='szyx') # rpy
                #print(quaternion, "\n")
        
            
            publish_trajectory_to_RVIZ(pub_1, position_rviz, orientation_rviz)
            #print("####", complete_traj_orientation_in_link0_quat)
            print("Sending command to the controller...")
            publish_trajectory_to_PD_controller(pub, complete_traj_position_in_link0, complete_traj_orientation_in_link0_quat, rr)

    except rospy.ROSInterruptException:
        pass
