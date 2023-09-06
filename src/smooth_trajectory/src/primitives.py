#!/usr/bin/env python3

import numpy as np
import math
import tf as TF
from timing_law import*
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp 

def linear_path(pi, pf, Ts):# P, DP, DDP, DDDP, T):
    
    s = np.arange(0, np.linalg.norm(pf - pi), Ts)
    
    p = pi + s*((pf - pi)/(np.linalg.norm(pf - pi)))
    dp = np.ones((3, len(s)))*(pf - pi)/(np.linalg.norm(pf - pi))
    ddp = np.zeros((3, len(s)))
    dddp = np.zeros((3, len(s)))
    
    return p, dp, ddp

def linear_trajectory(pi, pf, Ts, ti, tf):

    # Timing law for s(t)
    s, sd, sdd, sddd, s_tmp = fifth_polynomials(0, np.linalg.norm(pf - pi), 0.0, 0.0, 0, 0, ti, tf, Ts)
    print("s", s)
    # computhe the ptilde with timing law
    p_tilde = pi + s *((pf - pi)/(np.linalg.norm(pf - pi)))
    dp_tilde = sd *((pf - pi)/(np.linalg.norm(pf - pi)))
    ddp_tilde = sdd *((pf - pi)/(np.linalg.norm(pf - pi)))

    
    return p_tilde, dp_tilde, ddp_tilde

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
    z_prime = r/np.linalg.norm(r)
    y_prime = np.cross(x_prime, z_prime, axis=0)
    
    R = np.concatenate((x_prime, y_prime, z_prime), axis=1)
  
    p = c + np.dot(R,p_prime)
    dp = np.dot(R, np.vstack((-np.sin(s/rho), np.cos(s/rho), np.zeros(len(s)))))  
    ddp = np.dot(R, np.vstack((-(1/rho)*np.cos(s/rho),-(1/rho)*np.sin(s/rho),np.zeros(len(s)))))
    dddp = np.dot(R, np.vstack((np.sin(s/rho)/(np.power(rho,2)), -np.cos(s/rho)/(np.power(rho,2)), np.zeros(len(s))))) 

    return p, dp, ddp

def circular_trajectory(pi, pf, Ts, c, ti, tf):
            
    
    rho = np.linalg.norm(pi-c)
    r = np.cross(c-pf, c-pi, axis=0) 
    arc_angle = vecangle(c-pf, c-pi, r) 
    plength = rho*arc_angle
        
    # Timing law for s(t)
    s, sd, sdd, sddd, s_tmp = fifth_polynomials(0, plength, 0, 0, 0, 0, ti, tf, Ts)
       
    print("s", s)
    p_prime = np.vstack((rho*np.cos((s/rho)), rho*np.sin((s/rho)), np.zeros(len(s))))
    
    # R matrix to find the versors
    x_prime = (pi - c)/rho
    z_prime = r/np.linalg.norm(r)
    y_prime = np.cross(x_prime, z_prime, axis=0)
    R = np.concatenate((x_prime, y_prime, z_prime), axis=1)
    
    # computhe the ptilde with timing law
    p_tilde = c + np.dot(R,p_prime)
    dp_tilde = np.dot(R, np.vstack(( -sd*np.sin(s/rho), sd*np.cos(s/rho), np.zeros(len(s)))))  
    ddp_tilde = np.dot(R, np.vstack((   np.power(-sd,2)*(1/rho)*np.cos(s/rho) - sdd*np.sin(s/rho), np.power(-sd,2)*(1/rho)*np.sin(s/rho) + sdd*np.sin(s/rho), np.zeros(len(s)) )))
            
    
    return p_tilde, dp_tilde, ddp_tilde

def EE_orientation(r_start, r_end, Ts, ti, tf, circular_type):

    # Timing law for s(t)
    PHI_i = r_start.as_euler('zyx', degrees=False)
    PHI_f = r_end.as_euler('zyx', degrees=False)
    
    s, sd, sdd, sddd, s_tmp = fifth_polynomials(0, np.linalg.norm(PHI_f - PHI_i), 0.0, 0.0, 0.0, 0.0, ti, tf, Ts)
    
    
    if circular_type == 0:
        r_start = r_start.as_quat()
        o = np.ones((4,len(s)))

        o[0,:] = r_start[0]
        o[1,:] = r_start[1]
        o[2,:] = r_start[2]
        o[3,:] = r_start[3]
        
    else:
        
        key_rots = Rotation.concatenate([r_start, r_end]) 
        key_times = [ti, tf]
        slerp = Slerp(key_times, key_rots)
        # Timing law for s(t)
        times = np.linspace(ti,tf,len(s))
        
        interp_rots = slerp(times.tolist())
        o = interp_rots.as_quat()

    # computhe the ptilde with timing law
    
    do = 1# sd*((PHI_f - PHI_i)/(np.linalg.norm(PHI_f - PHI_i)))
    ddo = 1# sdd*((PHI_f - PHI_i)/(np.linalg.norm(PHI_f - PHI_i)))
    
    return o, do, ddo

def set_orientation(ti, tf, Ts, rotation_type, rot_panda_world, get_euler_mode=False):
    
    #switch to one if you want to have an incremental orientation
    circular_type = 0

    # base orientation get from rviz, already in panda_link0 frame
    #orientation = TF.transformations.quaternion_matrix([0.518338, 0.854537, 0.0283424, 0.0169715] )
    orientation = TF.transformations.quaternion_matrix([0,0,0,1] )
    base_orientation = orientation[0:3,0:3]

    # theta_1 = 50#270#y
    # theta_2 = 50#270 #x
    
    # theta_3 = -50#-270#y
    # theta_4 = -50#-25#x

    theta_1 = 50#270#y
    theta_2 = -50#270 #x
    
    theta_3 = -50#-270#y
    theta_4 = +50#-25#x
    theta_z = 180

    matrix_1 = np.array([[ np.cos(theta_1), 0, np.sin(theta_1)],
                         [ 0           ,   1,   0           ],
                         [-np.sin(theta_1), 0, np.cos(theta_1)]])

    matrix_2 = np.array([[ 1, 0           , 0           ],
                         [ 0, np.cos(theta_2),-np.sin(theta_2)],
                         [ 0, np.sin(theta_2), np.cos(theta_2)]])

    matrix_z_flip = np.array([[ 1, 0           , 0           ],
                         [ 0, np.cos(theta_z),-np.sin(theta_z)],
                         [ 0, np.sin(theta_z), np.cos(theta_z)]])
    
    matrix_3 = np.array([[ np.cos(theta_3), 0, np.sin(theta_3)],
                         [ 0           ,    1,    0           ],
                         [-np.sin(theta_3), 0, np.cos(theta_3)]])

    matrix_4 = np.array([[ 1, 0           , 0                ],
                         [ 0, np.cos(theta_4),-np.sin(theta_4)],
                         [ 0, np.sin(theta_4), np.cos(theta_4)]])


    if rotation_type == 'y':
        R_start = np.dot(base_orientation, matrix_1)
        R_end = np.dot(base_orientation, matrix_1)

    elif rotation_type == 'yx':
        R_start = np.dot(base_orientation,matrix_2)
        R_end = np.dot(base_orientation,matrix_2)

    elif rotation_type == 'yxy_circ':
        circular_type = 1
        R_start = np.dot(base_orientation,matrix_2)
        R_end = np.dot(base_orientation,matrix_3)

    elif rotation_type == 'yxy':
        R_start = np.dot(base_orientation,matrix_3)
        R_end = np.dot(base_orientation,matrix_3)
        
    elif rotation_type == 'yxyx':
        R_start = np.dot(base_orientation, matrix_4)
        R_end = np.dot(base_orientation, matrix_4)
    elif rotation_type == 'yxyx_circ':
        circular_type = 1
        R_end = np.dot(base_orientation, matrix_1)
        R_start = np.dot(base_orientation, matrix_4)
        
    #print(rot_panda_world)
    rot_panda_world_matrix = TF.transformations.quaternion_matrix(rot_panda_world)
    rot_panda_world_matrix = rot_panda_world_matrix[0:3,0:3]
    R_start = np.dot(matrix_z_flip, R_start)
    R_end = np.dot(matrix_z_flip, R_end)
    R_start = np.dot(rot_panda_world_matrix, R_start)
    R_end = np.dot(rot_panda_world_matrix, R_end)
    R_start = Rotation.from_matrix(R_start)
    R_end = Rotation.from_matrix(R_end)
    
    
    if get_euler_mode:
        
        
        o, do, ddo = EE_orientation(R_start, R_end, Ts,  ti, tf, circular_type)
        return o, do, ddo, R_start
        

    return R_start