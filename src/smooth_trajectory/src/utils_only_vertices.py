#!/usr/bin/env python3

import bagpy
from bagpy import bagreader
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import pandas as pd
import seaborn as sns
from sklearn.linear_model import LinearRegression
import set_configuration
#from set_configuration import*
#from config import*

plt.style.use('seaborn')


def rosbag_to_csv(data_base_path = None, FOLDER = None):
	BAG_PATH = os.path.join(data_base_path, FOLDER)
	if (os.path.isfile(BAG_PATH)):
		trajectory_bag = bagreader(BAG_PATH)
		for topic in trajectory_bag.topic_table['Topics']:
			print(trajectory_bag.message_by_topic(topic)) # save to csv by topic
				
def create_data_frame(data_base_path = None, TRAJECTORY=None):
	if not os.path.isdir(data_base_path):
		os.mkdir(data_base_path)

	for trajectory in os.scandir(data_base_path):
		if trajectory.name not in TRAJECTORY:
			continue

		if os.path.isdir(os.path.join(data_base_path, trajectory.name)):
			CSV_BAG_PATH = os.path.join(data_base_path, trajectory.name, 'surgeon-left-end_effector.csv')
			
			csv_bag_df = None

			if (os.path.isfile(CSV_BAG_PATH)):
				csv_bag_df = pd.read_csv(CSV_BAG_PATH, sep=",")

	return csv_bag_df

def plot_optitrack_data(csv_bag_df = None, data_base_path = None, show_plot = False, save_plot = False):
	
	for trajectory in os.scandir(data_base_path, TRAJECTORY):
		if trajectory.name not in TRAJECTORY:
			continue

		fig = plt.figure()
		ax = plt.axes(projection='3d')

		x_points = csv_bag_df.loc[:,"pose.position.x"]
		y_points = csv_bag_df.loc[:,"pose.position.y"]
		z_points = csv_bag_df.loc[:,"pose.position.z"]
		ax.plot3D(x_points, y_points, z_points, 'gray')
		ax.set_xlabel('x')
		ax.set_ylabel('y')
		ax.set_zlabel('z')
		#plt.setp(ax, xlim=(0,0.5), ylim=(0,0.5), zlim=(0,0.5))

		if not os.path.isdir(os.path.join(data_base_path, trajectory.name, 'plot')):
			os.mkdir(os.path.join(data_base_path, trajectory.name, 'plot'))

		if save_plot:
			fig.savefig(os.path.join(data_base_path, trajectory.name, 'plot', 'optitrack_trajectory' + '.png'))
		if show_plot:
			plt.show()
		plt.cla() 


def check_optitrack_data(data):
	points = data.to_numpy()
	x_all, y_all = points[:,0], points[:,2]
	window_size = 100
	counter, index = 0, 0
	estimated_vertices = []
	
	while (counter + window_size) < len(x_all):
		x = x_all[counter: counter + window_size].reshape((-1, 1))
		y = y_all[counter: counter + window_size]
		model = LinearRegression().fit(x, y)
		#print(model.score(x,y))
		if model.score(x,y) < 0.09:
			#print("in", model.score(x,y))
			index = counter + 1

			while model.score(x,y) < 0.1:
				#print("##", model.score(x,y))
				x = x_all[index: index + window_size].reshape((-1, 1))
				y = y_all[index: index + window_size]
				model = LinearRegression().fit(x,y)
				index += 1
			#print("range", counter, index + window_size)
			vertex_1, vertex_2 =  np.mean(x_all[counter:index + window_size]), np.mean(y_all[counter: index + window_size])
			#print(vertex_1, vertex_2)	
			estimated_vertices.append([vertex_1, vertex_2])
			#estimated_vertices = np.concatenate([estimated_vertices,np.mean(y_all[counter:index + window_size])])
	
			""" plt.scatter(x_all[counter:index + window_size].reshape((-1, 1)), y_all[counter: index + window_size],color='g') 
			plt.plot(x_all[counter:index + window_size].reshape((-1, 1)), model.predict(x_all[counter:index + window_size].reshape((-1, 1))),color='k')
			plt.plot(data.to_numpy()[:,0],data.to_numpy()[:,2] )
			plt.show() """
			counter = index + window_size
			index = 0
		counter += 1
	
	return estimated_vertices