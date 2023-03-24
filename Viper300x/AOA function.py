import numpy as np

#Determine AOA based on list of cluster given by scanner function
def AOA(clusters):
    pick_up = False     #Variable to determine whether pickup is possible
    
    pick_up_case = 0
    above = 1           #perpendicular
    above_roll_45 = 2   #perpendicular with a 45 degree roll
    above_roll_90 = 3   #perpendicalar with a 90 degree roll
    below = 4           #parallel 
    
    x,y,z = clusters[0]["position"]         #1st doughball
    
    dis = np.sqrt()
    
    for cluster in clusters[1:]:
        x_n,y_n,z_n = cluster["position"]      #x_n = new x, y_n new y, z_n new z
        dis = np.sqrt((x-x_n)**2 + (y-y_n)**2) #finding distance between doughballs
        if dis > 0.05:                         #if distance from doughballto doughball is greater than 50mm  
            if y < 0.565:                      #and doughball is less than 565mm from robot arm  
                pick_up = True                 #then pick_up from above is possible (perpendicular)
                pick_up_case = above
                break
            else:
                pick_up = True                 #if doughball is more than 565mm from robot arm 
                pick_up_case = below           #then pickup from below (parallel)
                break
        if (np.abs(y-y_n) < 0.015):            
            if ((np.abs(x-x_n) < 0.035)):
                pick_up = True
                pick_up_case = above_roll_90
                break
        if (np.abs(x-x_n) < 0.015):           #might need adjustments
            if ((np.abs(y-y_n) < 0.035)):
                pick_up = True
                pick_up_case = above_roll_45
                break
            
            
            