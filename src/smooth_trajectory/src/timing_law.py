#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from set_configuration import*

def fifth_polynomials(qi, qf, dqi, dqf, ddqi, ddqf, ti, tf, Ts):

    deltaT = tf - ti 
    H = np.array([[ 1, 0, np.power(0,2), np.power(0,3), np.power(0,4), np.power(0,5)],
                  [0,  1,  2*0, 3*(np.power(0,2)),  4*(np.power(0,3)),   5*(np.power(0,4))],
                  [0,  0,  2,    6*0,    12*(np.power(0,2)),  20*(np.power(0,3))],
                  [1,  deltaT, np.power(deltaT,2), np.power(deltaT,3), np.power(deltaT,4),  np.power(deltaT,5)],
                  [0,  1,  2*deltaT, 3*(np.power(deltaT,2)),  4*(np.power(deltaT,3)),   5*(np.power(deltaT,4))],
                  [0,  0,  2,    6*deltaT,    12*(np.power(deltaT,2)),  20*(np.power(deltaT,3))]])
    
    Q = np.array([[qi], [dqi], [ddqi], [qf], [dqf], [ddqf]])

    
    A = np.dot(np.linalg.inv(H),Q)

    a0 = A[0]
    a1 = A[1]
    a2 = A[2]
    a3 = A[3]
    a4 = A[4]
    a5 = A[5]

    T = np.arange(ti, tf, Ts) 

    q = a5*(np.power(T - ti,5)) + a4*(np.power(T - ti,4)) + a3*(np.power(T - ti,3)) + a2*(np.power(T - ti,2)) + a1*(T-ti) + a0
    qd = 5*a5*(np.power(T-ti,4)) + 4*a4*(np.power(T - ti, 3)) + 3*a3*(np.power(T - ti, 2)) + 2*a2*(T - ti) + a1
    qdd = 20*a5*(np.power(T-ti, 3)) + 12*a4*(np.power(T-ti, 2)) + 6*a3*(T-ti) + 2*a2
    qddd = 60*a5*(np.power(T-ti, 2)) + 24*a4*(T-ti) + 6*a3

    if plot_timing_law:
        fig	 = plt.figure()
        ax = fig.add_subplot(111)

        ax.scatter(T, q, c='blue', marker='o', label='First Robot Trajectory point from CAD', s=20)
#   
 #       ax.zaxis.set_rotate_label(False)  # disable automatic rotation
        #ax.set_xlabel('\n' + r"$Position$  [cm]", fontsize=15, linespacing=2)
        #ax.set_ylabel('\n' + r"$Time$  [sec]", fontsize=15, linespacing=2)
        #ax.set_zlabel('\n' + r"$Z$  [cm]", fontsize=15, linespacing=3, rotation=90)
   #     plt.title(r"$\mathbf{Mixed \quad double \quad circle}$", fontsize=10)
        #plt.zlabel(r"$\mathbf{Z}$")
        plt.show()

    return q, qd, qdd, qddd, T