#!/usr/bin/env python3

import numpy as np
import open3d as o3d
import copy
from set_configuration import*



def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def point_cloud_registration_ICP(src, dst):

    source = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(np.transpose(src))
    #o3d.visualization.draw_geometries([source])

    target = o3d.geometry.PointCloud()
    target.points = o3d.utility.Vector3dVector(np.transpose(dst))
    #o3d.visualization.draw_geometries([target])


    #o3d.visualization.draw_geometries([source, target])

    threshold = ICP_THRESHOLD ##rectangle_circle   ##1.5 #rectangle_1         ## rectangle  1.45      ## Pointer 0 1.67              # Pointer 3 1.475
    trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
                             [-0.139, 0.967, -0.215, 0.7],
                             [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])
    
    #print("Initial alignment")
    evaluation = o3d.pipelines.registration.evaluate_registration(source, target,
                                                        threshold, trans_init)
    

    #print("Apply point-to-point ICP")
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
    
    print("Transformation is:")
    print(reg_p2p.transformation)
    #result = draw_registration_result(source, target, reg_p2p.transformation)
    #print(np.asarray(result.points))
    H = reg_p2p.transformation

    if CAD_OBJECT_SIZE == 'rectangle_doublecircle_':
        H = np.array([[8.73658594e-01, -3.81139122e-04, -4.86735392e-01, -1.22697696e-01],
                      [3.33066907e-16,  1.00011706e+00, -1.40165657e-15,  1.91633246e-05],
                      [ 4.86060348e-01, -8.49718626e-04,  8.73835029e-01, -2.09292821e-01],
                      [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

    elif CAD_OBJECT_SIZE == 'rectangle_doublecircle__':
        H = np.array([[8.86149113e-01, -4.03371810e-04, -4.63567294e-01, -1.39420770e-01],
                      [-1.95567544e-16,  1.00011706e+00,  2.83040972e-16, -4.43675927e-06],
                      [ 4.62897127e-01, -8.39392611e-04,  8.86343254e-01, -1.44615936e-01],
                      [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

    elif CAD_OBJECT_SIZE == 'rectangle_':
        H = np.array([[ 8.80966830e-01, -3.94048389e-04, -4.73357631e-01, -1.20544644e-01],
 [-1.38777878e-16,  1.00011706e+00, -2.49800181e-16,  1.47902617e-05],
 [ 4.72685355e-01, -8.43809599e-04,  8.81153536e-01, -2.77228891e-01],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
    R, t = H[0:3,0:3], H[0:3,3]

    return np.asarray(source.transform(H).points), R, t, H





