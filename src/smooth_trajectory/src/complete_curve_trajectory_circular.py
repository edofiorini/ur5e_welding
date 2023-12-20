#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
import numpy as np
import os
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from utils_only_vertices import*
import tf
from scipy.spatial.transform import Slerp 
from matplotlib.ticker import (MultipleLocator, AutoMinorLocator)
from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from scipy.spatial.transform import Rotation as Rot
from sklearn import datasets, linear_model
import csv
from scipy.optimize import minimize

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
    
    one = np.ones(len(np.transpose(vector)))
    vector = np.vstack((vector, one))

    #translation = np.array([[homogeneous_matrix[0,3]], [homogeneous_matrix[1,3]], [homogeneous_matrix[2,3]]])
    new_vector = np.matmul(homogeneous_matrix, vector)
    return new_vector[0:3,:]

def circular_vertices(vertices, path):
    curr_pose = rtde_r.getActualTCPPose()
    #curr_pose = [0.115, -0.132, 0.216, 2.253, -2.295, 0.077]
    k = 0
    for i in range(0, len(np.transpose(vertices)) - 3):
       
        print("Generating trajectory for vertex number ", i)
        curr_angles = Rot.from_rotvec([curr_pose[3], curr_pose[4], curr_pose[5]]).as_euler('xyz')

        axis = [1, 0,1,0, 1, 1]
        angles = [+np.pi/6, -np.pi/6, -np.pi/6, +np.pi/6, +np.pi/6, +np.pi/6]
        
        curr_angles[axis[i]] += angles[i]
        next_angles_rotvec = Rot.from_euler('xyz', curr_angles).as_rotvec()
             
        if i == 0:
                
            waypoint = [vertices[0, i], vertices[1, i], vertices[2, i], next_angles_rotvec[0], next_angles_rotvec[1], next_angles_rotvec[2], velocity, acceleration, 0.0]       
            path.append(waypoint)

        else:
                

            pi = np.array([[vertices[0, k]], [vertices[1, k]], [vertices[2, k]]])
            pf = np.array([[vertices[0, k+1]], [vertices[1, k+1]], [vertices[2, k+1]]])
            c = np.array([[vertices[0, 0]], [vertices[1, 0] + radius], [vertices[2, 0]]]) #robot + radius y
            k = k + 2
            circ_path = circular_path(pi, pf, c,  Ts)
            print("Pi:", pi)
            print("Pf:", pf)
            print("Path:", circ_path)
            
            r_start = Rot.from_rotvec([path[-1][3], path[-1][4], path[-1][5]])
            r_end = Rot.from_euler('xyz', curr_angles)
            times = np.linspace(0, 1, len(np.transpose(circ_path)))
            key_rots = Rot.concatenate([r_start, r_end])
            key_times = [0, 1]
            slerp = Slerp(key_times, key_rots)
            interp_rots = slerp(times)
            o = interp_rots.as_rotvec()


            # from tuple to array
            circ_path = np.array(circ_path)
            
            for j in range(0, len(o)):
                if i == 0:
                    waypoint = [circ_path[0][0, j], circ_path[0][1, j], circ_path[0][2, j], o[j, 0], o[j, 1], o[j, 2], velocity, acceleration, 0.0]
                else:
                    waypoint = [circ_path[0][0, j], circ_path[0][1, j], circ_path[0][2, j], o[j, 0], o[j, 1], o[j, 2], velocity, acceleration, 0.0]
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
    
def write_ref_to_csv(path):
        print("Writing csv!")
        # Extract position data from the PoseStamped message
        for i in range(0, len((path))):
            x = path[i, 0]
            y = path[i, 1]
            z = path[i, 2]

            # Write the data to the CSV file
            csv_writer.writerow([x, y, z])
            csv_file.flush()  # Ensure data is written immediately


def write_vertices_to_csv(vertices):
        print("Writing csv!")

        for i in range(0, len(np.transpose(vertices))):
            vertex_x = vertices[0, i]
            vertex_y = vertices[1, i]
            vertex_z = vertices[2, i]

            # Write the data to the CSV file
            csv_writer_vert.writerow([vertex_x, vertex_y, vertex_z])
            csv_file_vert.flush()  # Ensure data is written immediately

def quarter_circle(params, theta, x, y):
    radius, center_x, center_y = params
    x_model = radius * np.cos(theta) + center_x
    y_model = radius * np.sin(theta) + center_y
    residuals = np.concatenate((x_model - x, y_model - y))
    return np.sum(residuals**2)

radius = 0.095
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

        data_name = rospy.get_param('Trajectory/folder_name')
        single_vertex_plot = rospy.get_param('Trajectory/show_single_vertex_plot')
        all_vertices_plot = rospy.get_param('Trajectory/show_all_vertices_plot')
        csv_ref_name = rospy.get_param('Trajectory/csv_ref_name')
        VERTICES_NUM = rospy.get_param('Trajectory/vertices_num')
        DATA_BASE_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'vertices_data')

        # creating vertices container
        overall_data = np.array([])
        #all_waypoints = np.empty([3, VERTICES_NUM*10])
        all_waypoints = np.empty([3, VERTICES_NUM*2])

        for i in range(0,VERTICES_NUM):
            print("Generating vertex number ", i)
            
            #Getting data from bag
            FOLDER = data_name + "_" + str(i) + ".bag"
            TRAJECTORY = data_name + "_" + str(i)
            rosbag_to_csv(DATA_BASE_PATH, FOLDER)


            df = create_data_frame(DATA_BASE_PATH, TRAJECTORY)

            data_from_optitrack = df.loc[:,["pose.position.x", "pose.position.y", "pose.position.z"]]  
            
            # y-axis in optitrack is pointing up
            data_from_optitrack_flatten = data_from_optitrack.to_numpy()
            data_from_optitrack_flatten[:,1] = data_from_optitrack.to_numpy()[1,1]

            overall_data = np.append(overall_data, data_from_optitrack_flatten)

            if i == 0:
                all_waypoints[0, 0] = data_from_optitrack_flatten[2, 0]
                all_waypoints[0, 1] = data_from_optitrack_flatten[-1, 0]

                all_waypoints[1, 0] = data_from_optitrack_flatten[1,1]
                all_waypoints[1, 1] = data_from_optitrack_flatten[1,1]

                all_waypoints[2, 0] = data_from_optitrack_flatten[2, 2]
                all_waypoints[2, 1] = data_from_optitrack_flatten[-1, 2]

            else:
                all_waypoints[0, i*2] = data_from_optitrack_flatten[2, 0]
                all_waypoints[0, i*2 +1] = data_from_optitrack_flatten[-1, 0]

                all_waypoints[1, i] = data_from_optitrack_flatten[1,1]
                all_waypoints[1, i+1] = data_from_optitrack_flatten[1,1]

                all_waypoints[2, i*2] = data_from_optitrack_flatten[2, 2]
                all_waypoints[2, i*2 +1] = data_from_optitrack_flatten[-1, 2]

            # data_from_optitrack_flatten[:, 0] = data_from_optitrack_flatten[:, 0]*1000
            # data_from_optitrack_flatten[:, 1] = data_from_optitrack_flatten[:, 1]*1000
            # data_from_optitrack_flatten[:, 2] = data_from_optitrack_flatten[:, 2]*1000 

            # initial_params = [0.095,  0.38, -0.2823712]  # Provide a reasonable initial guess
            # x = data_from_optitrack_flatten[:, 0]
            # y =  data_from_optitrack_flatten[:, 2]
            # theta = np.linspace(0, -np.pi / 2, len(x))

            # # Minimize the objective function
            # result = minimize(quarter_circle, initial_params, args=(theta, x, y))

            # radius, center_x, center_y = result.x

            # # Plot the results
            # fitted_points_x = radius * np.cos(theta) + center_x
            # fitted_points_y = radius * np.sin(theta) + center_y
            # plt.scatter(x, y, color='blue', label='Synthetic Data with Noise')
            # plt.plot(fitted_points_x, fitted_points_y, color='red', label='Fitted Quarter Circle')
            # plt.xlabel('X')
            # plt.ylabel('Y')
            # plt.legend()
            # plt.show()

            # print(result.x)
                
       

            if single_vertex_plot:
                lw = 2
                # plt.scatter(data_from_optitrack_flatten[:, 0], data_from_optitrack_flatten[:, 2], c='r', marker='o', label=TRAJECTORY)
                # plt.legend(loc="lower right")
                # plt.xlabel("Input")
                # plt.ylabel("Response")
                # plt.show()
        y_mean = np.mean(all_waypoints[1,:], axis=0)
        all_waypoints[1,:] = y_mean
        
        
        overall_data = np.reshape(overall_data,(int(len(overall_data)/3),3))

        # uniform y-axes for each vertex
        # y_mean = np.mean(all_waypoints[1,:], axis=0)
        # all_waypoints[1,:] = y_mean

 
        
        if all_vertices_plot:
            fig	 = plt.figure()
            ax = fig.add_subplot(111, projection="3d")
            ax.scatter(all_waypoints[0, :], all_waypoints[1, :], all_waypoints[2, :], c='r', marker='o', label=TRAJECTORY)
            plt.show()


        print("Generated vertices w.r.t. optitrack frame:\n", all_waypoints)
        
        
        # Transforming the trajectory for welding the object in frame ur_base
        (trans, rot) = transformer.lookupTransform("/ur/base", "/optitrack_world", rospy.Time(0))
        optitrack_to_link0_hom_trans = get_homogeneous_matrix(rot, trans)
        all_waypoints_ur_base = apply_transformation(optitrack_to_link0_hom_trans, all_waypoints)

        (trans, rot) = transformer.lookupTransform("/ur/base", "/optitrack_world", rospy.Time(0))
        optitrack_to_link0_hom_trans = get_homogeneous_matrix(rot, trans)
        overall_data_ur_base = apply_transformation(optitrack_to_link0_hom_trans, np.transpose(overall_data))


        # offset in y for safety reasons
        all_waypoints_ur_base[2,:] += 0.02
        print("ur_vertices", all_waypoints_ur_base)
        
        
        
        Ts = 0.002
        rtde_frequency = 500.0
        rtde_c = RTDEControl("192.168.137.214")# rtde_frequency, RTDEControl.FLAG_USE_EXT_UR_CAP)
        rtde_r = rtde_receive.RTDEReceiveInterface("192.168.137.214")

        velocity = 0.5
        acceleration = 0.5
        blend = 0.007

        # Go to "home demo" pose
        waypoint_j = [el*np.pi/180 for el in [-1.20, -87.64, -86.19, -96.23, 91.22, -0.81]]
        waypoint_j.extend([0.5, 0.5, 0.0])
        rtde_c.moveJ([waypoint_j])

        
        #get current robot status
        curr_pose = rtde_r.getActualTCPPose()

        path_only_vertex = []


        #Generiting trajectory passing only vertices
        #only_vertices(all_waypoints_ur_base, path_only_vertex)
        circular_vertices(all_waypoints_ur_base, path_only_vertex)
        
        dirname = os.path.dirname(__file__)
        # Set the path to the folder where you want to save the CSV file
        folder_path = os.path.join(os.path.dirname(os.path.dirname(dirname)), 'smooth_trajectory/src/csv')
        file_path = os.path.join(folder_path, csv_ref_name)
        file_path_vertices = os.path.join(folder_path, 'vertices_complete_circular.csv')
        
        # Create or open the CSV file for writing
        csv_file = open(file_path, 'w')
        csv_writer = csv.writer(csv_file)

        csv_file_vert = open(file_path_vertices, 'w')
        csv_writer_vert = csv.writer(csv_file_vert)

        # Write header to CSV file
        csv_writer.writerow(['x', 'y', 'z', 'vertex_x', 'vertex_y', 'vertex_z'])
        csv_writer_vert.writerow(['vertex_x', 'vertex_y', 'vertex_z'])

        plot_path_array = np.asarray(path_only_vertex)
        
        fig	 = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.scatter(plot_path_array[:, 0], plot_path_array[:, 1], c='red', marker='o', label='Acquired vertices', s=0.3)
        plt.show()
        
        write_ref_to_csv(plot_path_array)
        write_vertices_to_csv(all_waypoints_ur_base)
        
        publish_trajectory_to_RVIZ(pub_rviz, path_only_vertex)
 
        
        plot_traj = np.asarray(path_only_vertex)
        
        fig	 = plt.figure()
        ax = fig.add_subplot(111)
        ax.scatter(all_waypoints_ur_base[0, :]*1000, all_waypoints_ur_base[1, :]*1000, c='green', marker='o', label='Detected waypoints', s=5)
        ax.scatter(overall_data_ur_base[0, :]*1000, overall_data_ur_base[1, :]*1000, c='red', marker='o', label='Acquired waypoints', s=0.2)
        #ax.scatter(data_from_robot[:, 0]*1000, data_from_robot[:, 1]*1000, c='blue', marker='o', label='Robot pose', s=1)


        #ax.scatter(plot_traj[:, 0], plot_traj[:,1], plot_traj[:, 2], c='b', marker='o', label='Acquired optitrack trajectory')
        plt.legend()   
        #ax.xaxis.set_minor_locator(AutoMinorLocator())
        #ax.yaxis.set_minor_locator(AutoMinorLocator())
        #ax.zaxis.set_minor_locator(AutoMinorLocator())
        #ax.zaxis.set_rotate_label(False)  # disable automatic rotation
        #ax.set_xlabel('\n' + r"$Z$  [m]", fontsize=15, linespacing=3)
        #ax.set_ylabel('\n' + r"$X$  [m]", fontsize=15, linespacing=3)
        #ax.set_zlabel('\n' + r"$Y$  [m]", fontsize=15, linespacing=3, rotation=90)
        plt.show()

        
        
        #rtde_r.startFileRecording("data.csv")
        rospy.Timer(rospy.Duration(0.002), logCallback)
        rtde_c.moveL(path_only_vertex)
        rtde_c.stopScript()

    except rospy.ROSInterruptException:
        pass