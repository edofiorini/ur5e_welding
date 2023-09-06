from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp 

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

Ts = 0.001
rtde_frequency = 500.0
rtde_c = RTDEControl("10.42.0.2")#, rtde_frequency, RTDEControl.FLAG_USE_EXT_UR_CAP)
rtde_r = rtde_receive.RTDEReceiveInterface("10.42.0.2")

base = 0.1 # [m]
hight = 0.1 # [m]
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
pi = np.array([[curr_pose[0]], [curr_pose[1]], [curr_pose[2]]])
pf = np.array([[curr_pose[0]], [curr_pose[1] - hight], [curr_pose[2]]])
compute_trajectory(pi, pf, path_c, Ts)

# 2
curr_angles = R.from_rotvec([curr_pose[3], curr_pose[4], curr_pose[5]]).as_euler('xyz')
curr_angles[0] -= np.pi/6
next_angles_rotvec = R.from_euler('xyz', curr_angles).as_rotvec()
compute_orientation(path_c[-1], curr_angles, path_c, path_c[-1])
pi = np.array([[curr_pose[0]], [curr_pose[1] - hight], [curr_pose[2]]])
pf = np.array([[curr_pose[0] + base], [curr_pose[1] - hight], [curr_pose[2]]])
compute_trajectory(pi, pf, path_c, Ts)

# 3
curr_angles = R.from_rotvec([curr_pose[3], curr_pose[4], curr_pose[5]]).as_euler('xyz')
curr_angles[1] -= np.pi/6
next_angles_rotvec = R.from_euler('xyz', curr_angles).as_rotvec()
compute_orientation(path_c[-1], curr_angles, path_c, path_c[-1])
pi = np.array([[curr_pose[0] + base], [curr_pose[1] - hight], [curr_pose[2]]])
pf = np.array([[curr_pose[0] + base], [curr_pose[1] + hight], [curr_pose[2]]])
compute_trajectory(pi, pf, path_c, Ts)

# 4
curr_angles = R.from_rotvec([curr_pose[3], curr_pose[4], curr_pose[5]]).as_euler('xyz')
curr_angles[0] += np.pi/6
next_angles_rotvec = R.from_euler('xyz', curr_angles).as_rotvec()
compute_orientation(path_c[-1], curr_angles, path_c, path_c[-1])
pi = np.array([[curr_pose[0] + base], [curr_pose[1] + hight], [curr_pose[2]]])
pf = np.array([[curr_pose[0]], [curr_pose[1]], [curr_pose[2]]])
compute_trajectory(pi, pf, path_c, Ts)

# back to the starting orientation
curr_angles = R.from_rotvec([curr_pose[3], curr_pose[4], curr_pose[5]]).as_euler('xyz')
compute_orientation(path_c[-1], curr_angles, path_c, path_c[-1])

rtde_r.startFileRecording("data.csv")
rtde_c.moveL(path_c)
rtde_c.moveL(path_c)
rtde_c.moveL(path_c)
rtde_c.stopScript()

