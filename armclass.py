import time
import numpy as np
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from sklearn import preprocessing
from sklearn.neural_network import MLPRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error
from sklearn.metrics import r2_score
import time 
import pickle

#Armcode class contains all our functions used for our project
class armcode:
    
    #This function initializes all of our varialbles used throughout the Armcode Class
    def __init__(self):

        self.bot = InterbotixManipulatorXS("vx300s", moving_time=2.5, accel_time=.75)
        self.pcl = InterbotixPointCloudInterface()
        self.armtag = InterbotixArmTagInterface()
        self.success, self.clusters = self.pcl.get_cluster_positions(ref_frame="vx300s/base_link", sort_axis="y", reverse=False)
        self.counter = 0
        self.counter2 = 0 
        self.angle = 0
        self.z = 0.01
        self.flag = False
        self.y_off = 0
        self.x_off = 0
        self.x,self.y = 0,0
        self.length = 0

        self.RF = pickle.load(open("Mrgoodboy", 'rb'))

#The Scanner function scans the area and ensures that there is no abnormaly
#large objects. If there is a large object in the area then the function asks
#the user to fix the large object. If the object is not to large then pass_on
#will be true and it will go into the AOA function
    def scanner(self):
        self.success, self.clusters = self.pcl.get_cluster_positions(ref_frame="vx300s/base_link", sort_axis="y", reverse=False)
       
        pass_on = True
        if len(self.clusters) >= 1:
            if self.clusters[0]["num_points"] >= 350:
                input("Fix Big Cluster. Then Press Enter.")
                print(self.clusters[0]["num_points"])
                return False
            x,y,self.z = self.clusters[0]["position"]
        else: 
            pass_on = False
            return pass_on
        
        print(self.z)
        
        if self.z < 0.06:
            pass_on = True
            print(self.z)

        else:
            pass_on = False 
            
        if self.success == False:
            pass_on = False

 
        
        return pass_on

#The Calibrate function loads our filter parameters and then reads a single
#doughball in the center in order to determine the offset for our x and y values
    def calibrate(self):
        self.pcl.load_params(filepath = "AOAerror.yaml")
        self.home()
        self.bot.arm.set_ee_pose_components(y = 0.55, z=0.6)
        time.sleep(0.5)
        self.armtag.find_ref_to_arm_base_transform(num_samples = 10)
        self.home()
        input("press button to calibrate")
        self.scanner()
        x,y,z = self.clusters[0]["position"]
        self.y_off = y - 0.5325
        self.x_off = x - 0.012
        print(self.y_off, self.x_off)

#The ML function uses a pretrained network to predict the best angle for 
#picking up one cluster within a set of clusters. The angle is then passed on
#to the AOA function.
    def ml(self):
        self.x,self.y,self.z = self.clusters[0]["position"]         #1st doughball
        
        single = True
        
        dis_list = []
        self.length = 0
        self.size = []

        #dis_list[0] = 1


        for cluster in self.clusters[1:]:
            single = False
            x_n,y_n,z_n = self.x,self.y,self.z
            x_n,y_n,z_n = cluster["position"]      #x_n = new x, y_n new y, z_n new z
            dis = np.sqrt((self.x-x_n)**2 + (self.y-y_n)**2)
            
            if dis <= 0.07: 
                self.length += 1
                theta = np.arctan((self.y-y_n)/(self.x-x_n))
                if theta < 0:
                    theta += np.pi #finding distance between doughballs
                theta = (theta*180)/np.pi
                dis_list.append(x_n-self.x)
                dis_list.append(y_n-self.y)
                dis_list.append(theta)
                self.size.append(cluster["num_points"])
        
        self.flag = False

        if len(dis_list) == 3:
            dis_list.append(0)
            dis_list.append(0)
            dis_list.append(0)
            dis_list.append(0)
            dis_list.append(0)
            dis_list.append(0)
        elif len(dis_list) == 6:
            dis_list.append(0)
            dis_list.append(0)
            dis_list.append(0)
        elif len(dis_list) == 0: 
            self.flag = True
            return

        
        array = [dis_list]
        y_pred = self.RF.predict(array)
        print(y_pred)
        self.angle = y_pred
    
 #The Compare function scans everything around the intended doughball 
 #if the dougball was not picked up then the robot reattempts to pick up the 
 #doughball. If the doughball is close to another doughball then the user must 
 # fix the issue before continuing 
    def compare(self):
        count = 0 
        size = []
        for cluster in self.clusters:
            single = False
            x_n,y_n,z_n = self.x,self.y,self.z
            x_n,y_n,z_n = cluster["position"]      #x_n = new x, y_n new y, z_n new z
            dis = np.sqrt((self.x-x_n)**2 + (self.y-y_n)**2)
            
            if dis <= 0.07: 
                count += 1
                size.append(cluster["num_points"])

        print(size)
        print(self.size)



        if self.length == count:
            sum1 = sum(size)
            sum2 = sum(self.size)
            dif = abs(sum1-sum2)
            if dif <= 15:
                
                return True
        else:
           return False

#The AOA (Angle of Approach) function determines whether the robot arm would
#pick up the doughball from a perpendicular or a parallel approach. If it is a
#perpendicular approach then the function is passed on an angle from the ML
#function which determines the best roll for the given scenario. 
    def AOA(self):
        self.bot.gripper.open()
        Disyl = .565 #blue tape distance
        r_dis = 0.015

        x,y,z = self.clusters[0]["position"]         #1st doughball
        print(x,y,z)

        x,y,z = self.clusters[0]["position"]
        #X,Y = self.error(x,y)
        
        x -= self.x_off
        y -= self.y_off
        r = np.sqrt(x**2 + y**2)
            
        if r < Disyl and r > 0.175: 
            theta = np.arctan(y/x)
            if theta < 0:
                theta += np.pi
            # shift radius
            r += r_dis
            # conversion
            x = r * np.cos(theta)
            y = r * np.sin(theta)
            roll = (self.angle)*3.1415/180
            ro112 = (self.angle)*1.5/90
            print(roll)
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=0.2, pitch=1.5)                     
            self.bot.arm.set_single_joint_position("wrist_rotate", roll)
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=0.2, pitch=1.5, roll = ro112)
            self.bot.arm.set_ee_cartesian_trajectory(z=z-0.185)	
        elif r > 0.175: 
            x -= 0.0
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=0.2)
            self.bot.arm.set_ee_cartesian_trajectory(z=z-0.18)

        self.bot.gripper.close()
        self.bot.arm.set_ee_cartesian_trajectory(z=z+0.18)

#The Place function moves the robot arm in a way so that it places the 
#dougballs in a 4x4 array 
    def place(self):
        self.bot.arm.set_ee_pose_components(x=0.3, z=0.2)

        if self.counter == 4:
            self.counter = 0
            self.counter2 += 1
            if self.counter2 == 4:
                self.counter2 == 0
        self.bot.arm.set_ee_pose_components(x=self.counter2*0.1-0.1, y=-0.7+0.055*self.counter, z=0.2)
        #bot.arm.set_ee_pose_components(y=-0.4, z=0.2)
        self.bot.arm.set_ee_cartesian_trajectory(z=self.z-0.18)
        self.bot.gripper.open() 
        self.bot.arm.set_ee_pose_components(y=-0.35, z=0.2)
        self.counter += 1
        
#The Home function will be the "safe" position where the robot arm goes before 
#and after moving so that it is not in the way of the camera
    def home(self):
        self.bot.arm.set_ee_pose_components(x=0.3, z=0.2)

#The Run function calls all of the previous functions in the order we intend
#the tasks to be completed 
    def run(self):
        self.calibrate()
        self.home()
        while(1==1):
            pass_on = self.scanner()
            print(pass_on)
            while(pass_on == True):
                self.ml()
                self.AOA()
                self.home()
                self.scanner()
                x = self.compare()
                if x == True:
                    self.place()
                pass_on = self.scanner()
