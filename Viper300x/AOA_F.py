import numpy as np

#Determine AOA based on list of cluster given by scanner function
def AOA(clusters):

	Disdd = 0.07
	Disyl = .565
	Disdyx = .015
	Disdxy = .015
	
	
	pick_up = True     #Variable to determine whether pickup is possible

	single = True

	pick_up_case = 1
	above = 1           #perpendicular
	above_roll_45 = 2   #perpendicular with a 45 degree roll
	above_roll_90 = 3   #perpendicalar with a 90 degree roll
	above_roll_135 = 4   #perpendicular with a 45 degree roll
	below = 5           #parallel 

	x,y,z = clusters[0]["position"]         #1st doughball
	print(x,y,z)
	
	dis_list = [0] * (len(clusters)-1)
	print(len(clusters)-1)
	i = 0
	#dis_list[0] = 1
	print(dis_list)
	
	for cluster in clusters[1:]:
		single = False
		x_n,y_n,z_n = x,y,z
		x_n,y_n,z_n = cluster["position"]      #x_n = new x, y_n new y, z_n new z
		dis = np.sqrt((x-x_n)**2 + (y-y_n)**2)
		dis_list[i] = dis
		i += 1
	if len(dis_list) > 1:
		dis = min(dis_list)
		min_index = dis_list.index(dis) + 1
	else:
		dis = 1
		min_index = 0
		
	print(min_index)
	x_n,y_n,z_n = clusters[min_index]["position"]
	
	print(dis)
	theta = np.arctan((y-y_n)/(x-x_n))
	if theta < 0:
		theta += np.pi #finding distance between doughballs
	theta = (theta*180)/np.pi
	print(theta)
	if dis > Disdd:                         #if distance from doughballto doughball is greater than 50mm  
		if y < Disyl:                      #and doughball is less than 565mm from robot arm  
			pick_up = True                 #then pick_up from above is possible (perpendicular)
			pick_up_case = above
			print("here")
			return pick_up, pick_up_case
		else:
			pick_up = True                 #if doughball is more than 565mm from robot arm 
			pick_up_case = below           #then pickup from below (parallel)
			return pick_up, pick_up_case
	if y < Disyl:
		if (theta < 15 or theta > 345 or theta < 195 and theta > 165):            
			pick_up = True
			pick_up_case = above_roll_90
			return pick_up, pick_up_case
		if (theta < 75 and theta > 15 or theta > 195 and theta < 255):           #might need adjustments
			pick_up = True
			pick_up_case = above_roll_45
			return pick_up, pick_up_case
		if (theta < 165 and theta > 105 or theta > 285 and theta < 345):          #might need adjustments
			pick_up = True
			pick_up_case = above_roll_135
			return pick_up, pick_up_case
		else:
			pick_up = True                 #then pick_up from above is possible (perpendicular)
			pick_up_case = above
			print("there")
			return pick_up, pick_up_case
	else:
		pick_up = False
		return pick_up, pick_up_case
	if single == True:
		if y > Disyl:
			pick_up = True
			pick_up_case = 5
		
	return pick_up, pick_up_case
