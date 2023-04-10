import matplotlib.pyplot as plt
import numpy as np
import random
import csv
from sklearn import preprocessing
from sklearn.neural_network import MLPRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error
from sklearn.metrics import r2_score
import time 
import pickle

def ML(cluters):
	x,y,z = clusters[0]["position"]         #1st doughball
	print(x,y,z)
	
	single = True
	
	dis_list = []
	 

	#dis_list[0] = 1
	
	for cluster in clusters[1:]:
		single = False
		x_n,y_n,z_n = x,y,z
		x_n,y_n,z_n = cluster["position"]      #x_n = new x, y_n new y, z_n new z
		dis = np.sqrt((x-x_n)**2 + (y-y_n)**2)
		
		if dis <= 0.07: 
			theta = np.arctan((y-y_n)/(x-x_n))
			if theta < 0:
				theta += np.pi #finding distance between doughballs
			theta = (theta*180)/np.pi
			dis_list.append(x_n-x,y_n-y,theta)
	print(dis_list)
	
	
	
