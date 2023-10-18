#!/usr/bin/env python3

import numpy as np
from shapely.geometry import LineString, Point
from set_configuration import*


def compute_CAD_rectangle_trajectory(first_point, l_1, l_2, data, start_sign):

	x_A, z_A = first_point[0], first_point[2]
	x_B, z_B =  first_point[0], first_point[2] 

	if start_sign == "z+":
		z_B +=  l_1
	elif start_sign == "z-":
		z_B -=  l_1
	if start_sign == "x+":
		x_B +=  l_1
	elif start_sign == "x-":
		x_B -=  l_1
	
	
	
	point_A = Point(x_A, z_A)
	point_B = Point(x_B, z_B)
		
	line1 = LineString([point_A, point_B])
	
	if CAD_ORIENTATION == 'left':
		direction = 'left'
		i = 1
	else:
		direction = 'right'
		i = 0
  
	first_parallel = line1.parallel_offset(l_2 - 0.00, direction)
	point_C = first_parallel.coords[i]
	line2 = LineString([point_B, point_C])
	
	second_parallel = line2.parallel_offset(l_1, direction) #  for right take the 0 index point, left 1
	point_D = second_parallel.coords[i]
	line3 = LineString([point_C, point_D])

	line4 = LineString([point_D, point_A])

	vertices = list(line1.coords) + [list(line2.coords[1])] + [list(line3.coords[1])] + [list(line4.coords[1])]

	x, y, z = ([] for i in range(3))
	number_samples = int(len(data.to_numpy())/4) 
	for index in range(0, 4):
		if index == 3:
			x.extend(np.linspace(vertices[index][0],vertices[0][0],number_samples).tolist())
			y.extend(np.linspace(first_point[1], first_point[1], number_samples).tolist())
			z.extend(np.linspace(vertices[index][1],vertices[0][1],number_samples).tolist())
		else:
			x.extend(np.linspace(vertices[index][0],vertices[index + 1][0],number_samples).tolist())
			y.extend(np.linspace(first_point[1],first_point[1],number_samples).tolist())
			z.extend(np.linspace(vertices[index][1],vertices[index + 1][1],number_samples).tolist())

	CAD_trajectory = np.column_stack((np.transpose(np.array(x)), np.transpose(np.array(y)), np.transpose(np.array(z))))

	complete_vertices = []
	for i in range(0, len(vertices)):
		tmp = [vertices[i][0], first_point[1], vertices[i][1]] 
		complete_vertices.extend(tmp)
	complete_vertices = np.reshape(complete_vertices,(int(len(complete_vertices)/3),3))	

	return CAD_trajectory, complete_vertices

def compute_CAD_circle_trajectory(first_point, diameter):
	# Create a circle trajectory starting from the radius and first data_point
	# Two semicircle are concatenated 
	C_x, C_y = first_point[0] + diameter/2, first_point[2]

	# radius, where the semicircle starts and ends, translation respect the origin 0.0
	r, h, k = diameter/2, C_x, first_point[2]
	x_1 = np.linspace(h -r, h + r, 10000)

	# upper and lower semicircle
	y_1 = k + np.sqrt(r**2 - (x_1 - h)**2)
	y_2 = k - np.sqrt(r**2 - (x_1 - h)**2)
	x, y, z = ([] for i in range(3))
	x.extend(x_1.tolist())
	y.extend(np.linspace(first_point[1], first_point[1], len(y_1)).tolist())
	z.extend(y_1.tolist())

	x.extend(x_1.tolist())
	y.extend(np.linspace(first_point[1], first_point[1], len(y_1)).tolist())
	z.extend(y_2.tolist())
	CAD_trajectory = np.column_stack((np.transpose(np.array(x)), np.transpose(np.array(y)), np.transpose(np.array(z))))

	
	a = [C_x - diameter/2, first_point[1], C_y]
	b = [C_x, first_point[1], C_y + diameter/2]
	c = [C_x + diameter/2, first_point[1], C_y ]
	d = [C_x , first_point[1], C_y - diameter/2]
	e = [C_x - diameter/2, first_point[1], C_y]
	
	vertices = np.vstack((a, b, c , d, e))

	return CAD_trajectory, vertices

def compute_CAD_rectangle_circle_trajectory(first_point, l_1, l_2, diameter, data):
	#Create the rectangle + circle trajectory
	#Different line and one semicircle are concatenated
	x_A, z_A = first_point[0], first_point[2]
	x_B, z_B =  first_point[0], first_point[2] + l_1
	point_A = Point(x_A, z_A)
	point_B = Point(x_B, z_B)

	line1 = LineString([point_A, point_B])
	
	if CAD_ORIENTATION == 'left':
		direction = 'left'
		i = 1
	else:
		direction = 'right'
		i = 0

	first_parallel = line1.parallel_offset(l_2, direction)
	point_C = first_parallel.coords[i]
	line2 = LineString([point_B, point_C])
	
	second_parallel = line2.parallel_offset(l_1, direction) #  for right take the 0 index point, left 1
	point_D = second_parallel.coords[i]
	line3 = LineString([point_C, point_D])

	A, D = line1.coords[0],  line3.coords[1]
	C_x, C_y = A[0] + diameter/2, A[1]
	
	vertices = list(line1.coords) + [list(line2.coords[1])] + [list(line3.coords[1])]

	x, y, z = ([] for i in range(3))

	number_samples = int(len(data.to_numpy())/4) 
	for index in range(0, 4):
		if index == 3:
			# radius, where the semicircle starts and ends, translation respect the origin 0.0
			r, h, k = diameter/2, C_x, A[1]
			x_1 = np.linspace(h -r, h + r, 10000)
			y_1 = k - np.sqrt(r**2 - (x_1 - h)**2) #change the sign for having upper or lower semicircle
			x.extend(x_1.tolist())
			y.extend(np.linspace(first_point[1], first_point[1], len(y_1)).tolist())
			z.extend(y_1.tolist())

		else:
			x.extend(np.linspace(vertices[index][0],vertices[index + 1][0],number_samples).tolist())
			y.extend(np.linspace(first_point[1],first_point[1],number_samples).tolist())
			z.extend(np.linspace(vertices[index][1],vertices[index + 1][1],number_samples).tolist())

	CAD_trajectory = np.column_stack((np.transpose(np.array(x)), np.transpose(np.array(y)), np.transpose(np.array(z))))

	complete_vertices = []
	for i in range(0, len(vertices)):
		tmp = [vertices[i][0], first_point[1], vertices[i][1]] 
		complete_vertices.extend(tmp)
	complete_vertices = np.reshape(complete_vertices,(int(len(complete_vertices)/3),3))

	return CAD_trajectory, complete_vertices

def compute_CAD_rectangle_doublecircle_trajectory(first_point, l_1, l_2, l_3, l_4, radius_1, radius_2, data):
	#Create the rectangle + doublecircle trajectory
	#Different line and two semicircle are concatenated
	x_A, z_A = first_point[0], first_point[2]
	x_B, z_B =  first_point[0], first_point[2] + l_1
	point_A = Point(x_A, z_A)
	point_B = Point(x_B, z_B)

	line1 = LineString([point_A, point_B])
	
	if CAD_ORIENTATION == 'left':
		direction = 'left'
		i = 1
	else:
		direction = 'right'
		i = 0

	first_parallel = line1.parallel_offset(l_2, direction)
	point_C = first_parallel.coords[i]
	line2 = LineString([point_B, point_C])
	

	x_D, y_D = point_C[0] + radius_1, point_C[1] - radius_1
	x_E, y_E = x_D , y_D - l_3
	point_D = Point(x_D, y_D)
	point_E = Point(x_E, y_E)
	line3 = LineString([point_D, point_E])
	
	
	second_parallel = line3.parallel_offset(l_4, direction) #  for right take the 0 index point, left 1
	point_F = second_parallel.coords[i]
	line4 = LineString([point_E, point_F])
	
	vertices = list(line1.coords) + [list(line2.coords[1])]  + list(line3.coords) + [list(line4.coords[1])]
	
	
	 

	x, y, z = ([] for i in range(3))

	number_samples = int(len(data.to_numpy())/4) 
	for index in range(3, 6):

		if index == 2 or index == 5:
			if index == 2:
				C, D = line2.coords[1],  line3.coords[0]
				Center_x, Center_y = C[0], C[1] - radius_1
				radius = radius_1
				sign = 'positive'
			else:
				F, A = line4.coords[1], line1.coords[0]
				Center_x, Center_y = F[0], F[1] + radius_2
				radius = radius_2
				sign = 'negative'
			r, h, k = radius, Center_x, Center_y
			x_1 = np.linspace(h -r, h + r, 10000)
			
			if sign == 'positive': 
				y_1 = k + np.sqrt(r**2 - (x_1 - h)**2) #change the sign for having upper or lower semicircle
			else:
				y_1 = k - np.sqrt(r**2 - (x_1 - h)**2) #change the sign for having upper or lower semicircle
			x_component = len(x_1.tolist())/2
			y_component = len(np.linspace(first_point[1], first_point[1], len(y_1)).tolist())/2
			z_component = len(y_1.tolist())/2
			
			if sign == 'positive':
				x.extend(x_1.tolist()[int(x_component)-1:-1])
				y.extend(np.linspace(first_point[1], first_point[1], len(y_1)).tolist()[int(y_component)-1:-1])
				z.extend(y_1.tolist()[int(z_component)-1:-1])
			else:
				x.extend(x_1.tolist()[0:int(x_component)])
				y.extend(np.linspace(first_point[1], first_point[1], len(y_1)).tolist()[0:int(y_component)])
				z.extend(y_1.tolist()[0:int(z_component)])
		else:
			x.extend(np.linspace(vertices[index][0],vertices[index + 1][0],number_samples).tolist())
			y.extend(np.linspace(first_point[1],first_point[1],number_samples).tolist())
			z.extend(np.linspace(vertices[index][1],vertices[index + 1][1],number_samples).tolist())

	CAD_trajectory = np.column_stack((np.transpose(np.array(x)), np.transpose(np.array(y)), np.transpose(np.array(z))))

	complete_vertices = []
	for i in range(0, len(vertices)):
		tmp = [vertices[i][0], first_point[1], vertices[i][1]] 
		complete_vertices.extend(tmp)
	complete_vertices = np.reshape(complete_vertices,(int(len(complete_vertices)/3),3))
	
	return CAD_trajectory, complete_vertices

def compute_CAD_rectangle_doublecircle_trajectory_fake(first_point, l_1, l_2, l_3, l_4, radius_1, radius_2, data):
	#Create the rectangle + doublecircle trajectory
	#Different line and two semicircle are concatenated
	x_A, z_A = first_point[0], first_point[2]
	x_B, z_B =  first_point[0], first_point[2] + l_1
	point_A = Point(x_A, z_A)
	point_B = Point(x_B, z_B)

	line1 = LineString([point_A, point_B])
	
	if CAD_ORIENTATION == 'left':
		direction = 'left'
		i = 1
	else:
		direction = 'right'
		i = 0

	first_parallel = line1.parallel_offset(l_2, direction)
	point_C = first_parallel.coords[i]
	line2 = LineString([point_B, point_C])
	

	x_D, y_D = point_C[0] + radius_1, point_C[1] - radius_1
	x_E, y_E = x_D , y_D - l_3
	point_D = Point(x_D, y_D)
	point_E = Point(x_E, y_E)
	line3 = LineString([point_D, point_E])
	
	
	second_parallel = line3.parallel_offset(l_4, direction) #  for right take the 0 index point, left 1
	point_F = second_parallel.coords[i]
	line4 = LineString([point_E, point_F])
	
	vertices = list(line1.coords) + [list(line2.coords[1])]  + list(line3.coords) + [list(line4.coords[1])]
	
	
	 

	x, y, z = ([] for i in range(3))

	number_samples = int(len(data.to_numpy())/4) 
	for index in range(0, 3):

		if index == 2 or index == 5:
			if index == 2:
				C, D = line2.coords[1],  line3.coords[0]
				Center_x, Center_y = C[0], C[1] - radius_1
				radius = radius_1
				sign = 'positive'
			else:
				F, A = line4.coords[1], line1.coords[0]
				Center_x, Center_y = F[0], F[1] + radius_2
				radius = radius_2
				sign = 'negative'
			r, h, k = radius, Center_x, Center_y
			x_1 = np.linspace(h -r, h + r, 10000)
			
			if sign == 'positive': 
				y_1 = k + np.sqrt(r**2 - (x_1 - h)**2) #change the sign for having upper or lower semicircle
			else:
				y_1 = k - np.sqrt(r**2 - (x_1 - h)**2) #change the sign for having upper or lower semicircle
			x_component = len(x_1.tolist())/2
			y_component = len(np.linspace(first_point[1], first_point[1], len(y_1)).tolist())/2
			z_component = len(y_1.tolist())/2
			
			if sign == 'positive':
				x.extend(x_1.tolist()[int(x_component)-1:-1])
				y.extend(np.linspace(first_point[1], first_point[1], len(y_1)).tolist()[int(y_component)-1:-1])
				z.extend(y_1.tolist()[int(z_component)-1:-1])
			else:
				x.extend(x_1.tolist()[0:int(x_component)])
				y.extend(np.linspace(first_point[1], first_point[1], len(y_1)).tolist()[0:int(y_component)])
				z.extend(y_1.tolist()[0:int(z_component)])
		else:
			x.extend(np.linspace(vertices[index][0],vertices[index + 1][0],number_samples).tolist())
			y.extend(np.linspace(first_point[1],first_point[1],number_samples).tolist())
			z.extend(np.linspace(vertices[index][1],vertices[index + 1][1],number_samples).tolist())

	CAD_trajectory = np.column_stack((np.transpose(np.array(x)), np.transpose(np.array(y)), np.transpose(np.array(z))))

	complete_vertices = []
	for i in range(0, len(vertices)):
		tmp = [vertices[i][0], first_point[1], vertices[i][1]] 
		complete_vertices.extend(tmp)
	complete_vertices = np.reshape(complete_vertices,(int(len(complete_vertices)/3),3))

	return CAD_trajectory, complete_vertices

def compute_CAD_deep_rectangle_doublecircle_trajectory(first_point, l_1, l_2, l_3, l_4, radius_1, radius_2, data):
	#Create the rectangle + doublecircle trajectory
	#Different line and two semicircle are concatenated
	
	CAD_trajectory_up, vertices = compute_CAD_rectangle_doublecircle_trajectory(first_point, l_1, l_2, l_3, l_4, radius_1, radius_2, data)

	x, y, z = ([] for i in range(3))
	number_samples = int(len(data.to_numpy())/4) 

	x.extend(np.linspace(vertices[0][0],vertices[0][0],number_samples).tolist())
	y.extend(np.linspace(vertices[0][1],vertices[0][1] + 0.02,number_samples).tolist())
	z.extend(np.linspace(vertices[0][2],vertices[0][2],number_samples).tolist())

	CAD_trajectory_deep = np.column_stack((np.transpose(np.array(x)), np.transpose(np.array(y)), np.transpose(np.array(z))))

	CAD_trajectory = np.append(CAD_trajectory_up, CAD_trajectory_deep, axis=0)
	#print(CAD_trajectory_deep[len(CAD_trajectory_deep)-1, :])
	#print(vertices)
	#exit()
	#vertices = np.append(vertices, CAD_trajectory_deep[len(CAD_trajectory_deep)-1, :], axis=0 )
	vertices = np.vstack((vertices, CAD_trajectory_deep[len(CAD_trajectory_deep)-1, :]))
	
	
	complete_vertices = []
	for i in range(0, len(vertices)):
		tmp = [vertices[i][0], first_point[1], vertices[i][1]] 
		complete_vertices.extend(tmp)
	complete_vertices = np.reshape(complete_vertices,(int(len(complete_vertices)/3),3))
	
	return CAD_trajectory, vertices

def create_CAD_trajectory(first_point, data):
	print("Creating CAD trajectory...")
	if CAD_OBJECT_SIZE == "rectangle":
		if CAD_OBJECT_START in ["z+", "z-"]:
			l_1, l_2 = 0.20, 0.20 #0.205
			CAD_trajectory, vertices = compute_CAD_rectangle_trajectory(first_point, l_1, l_2, data, CAD_OBJECT_START)
		else:
			l_1, l_2 = 0.29, 0.21
			CAD_trajectory, vertices = compute_CAD_rectangle_trajectory(first_point, l_1, l_2, data, CAD_OBJECT_START)
	elif CAD_OBJECT_SIZE == "B3":
		l_1, l_2 = 0.085, 0.055
		CAD_trajectory, vertices = compute_CAD_rectangle_trajectory(first_point, l_1, l_2, data)
	elif CAD_OBJECT_SIZE == "circle":
		diameter = 0.13
		CAD_trajectory, vertices = compute_CAD_circle_trajectory(first_point, diameter)
	elif CAD_OBJECT_SIZE == "rectangle_circle":
		l_1, l_2, diameter = 0.1, 0.12, 0.12
		CAD_trajectory, vertices = compute_CAD_rectangle_circle_trajectory(first_point, l_1, l_2, diameter, data)
	elif CAD_OBJECT_SIZE == "rectangle_doublecircle":
		#l_1, l_2, l_3, l_4, radius_1, radius_2 = 0.08, 0.095, 0.065, 0.11, 0.035, 0.02
		l_1, l_2, l_3, l_4, radius_1, radius_2 = 0.11, 0.065, 0.095, 0.08, 0.035, 0.02
		CAD_trajectory, vertices = compute_CAD_rectangle_doublecircle_trajectory(first_point, l_1, l_2, l_3, l_4, radius_1, radius_2, data)
	elif CAD_OBJECT_SIZE == "deep_rectangle_doublecircle":
		l_1, l_2, l_3, l_4, radius_1, radius_2 = 0.083, 0.0953, 0.0653, 0.113, 0.035, 0.02
		CAD_trajectory, vertices = compute_CAD_deep_rectangle_doublecircle_trajectory(first_point, l_1, l_2, l_3, l_4, radius_1, radius_2, data)
	return CAD_trajectory, vertices