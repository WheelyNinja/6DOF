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

class armcode:
    def __init__(self):

        self.bot = InterbotixManipulatorXS("vx300s", moving_time=2.5, accel_time=.75)
        self.pcl = InterbotixPointCloudInterface()
        self.armtag = InterbotixArmTagInterface()
        self.success, self.clusters = self.pcl.get_cluster_positions(ref_frame="vx300s/base_link", sort_axis="y", reverse=False)
        self.angle = 0

        self.MLP = pickle.load(open("model_pickle", 'rb'))

    def scanner(self):
        self.success, self.clusters = self.pcl.get_cluster_positions(ref_frame="vx300s/base_link", sort_axis="y", reverse=False)
            
        pass_on = True
        x,y,z = self.clusters[0]["position"]
        X,Y = self.error(x,y)
        
        if z < 0.04:
            pass_on = True
            x += X
            y += Y
        else:
            pass_on = False 
            
        if self.success == False:
            pass_on = False
        
        return pass_on, x, y

    def ml(self):
        x,y,z = self.clusters[0]["position"]         #1st doughball
        print(x,y,z)
        
        single = True
        
        dis_list = []
        

        #dis_list[0] = 1
        
        for cluster in self.clusters[1:]:
            single = False
            x_n,y_n,z_n = x,y,z
            x_n,y_n,z_n = cluster["position"]      #x_n = new x, y_n new y, z_n new z
            dis = np.sqrt((x-x_n)**2 + (y-y_n)**2)
            
            if dis <= 0.07: 
                theta = np.arctan((y-y_n)/(x-x_n))
                if theta < 0:
                    theta += np.pi #finding distance between doughballs
                theta = (theta*180)/np.pi
                dis_list.append(x_n-x)
                dis_list.append(y_n-y)
                dis_list.append(theta)
        
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
            dis_list.append(0)
            dis_list.append(0)
            dis_list.append(0)
            dis_list.append(0)
            dis_list.append(0)
            dis_list.append(0)
            dis_list.append(0)
            dis_list.append(0)
            dis_list.append(0)

        
        array = [dis_list]
        y_pred = self.MLP.predict(array)
        print(y_pred)
        self.angle = y_pred

    def AOA(self):
        Disdd = 0.07 #distance between doughball centers
        Disyl = .565 #blue tape distance
        r_dis = 0.0375

        x,y,z = self.clusters[0]["position"]         #1st doughball
        print(x,y,z)

        pass_on = True
        x,y,z = self.clusters[0]["position"]
        X,Y = self.error(x,y)
        
        x += X
        y += Y

    
        if y < Disyl: 
            r = np.sqrt(x**2 + y**2)
            theta = np.arctan(y/x)
            if theta < 0:
                theta += np.pi
            # shift radius
            r += r_dis
            # conversion
            x = r * np.cos(theta)
            y = r * np.sin(theta)
            roll = (self.angle)*1.5/90                     
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=0.2, pitch=1.5, roll = roll)
            self.bot.arm.set_ee_cartesian_trajectory(z=z-0.18)	
        else: 
            x -= 0.0
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=0.2)
            self.bot.arm.set_ee_cartesian_trajectory(z=z-0.18)

        self.bot.gripper.close()
        self.bot.arm.set_ee_cartesian_trajectory(z=z+0.18)

    def error(self, x, y):
        if x > 0:
            if 0.1475 < y and y <= 0.1825:
                X = -0.015434
                Y = -0.00355   
            elif 0.1825 < y and y <= 0.2175:
                X = -0.016784
                Y = -0.003625
            elif 0.1275 < y and y <= 0.2525:
                X = -0.01675
                Y = -0.003642
            elif 0.2525 < y and y <= 0.2875:
                X = -0.016625
                Y = -0.004792
            elif 0.2875 < y and y <= 0.3225:
                X = -0.018467
                Y = -0.006209
            elif 0.3225 < y and y <= 0.3575:
                X = -0.019759
                Y = -0.005592
            elif 0.3575 < y and y <= 0.3925:
                X = -0.020467
                Y = -0.004059
            elif 0.3925 < y and y <= 0.4275:
                X = -0.020758
                Y = -0.003525
            elif 0.4275 < y and y <= 0.4625:
                X = -0.021758
                Y = -0.003917
            elif 0.4625 < y and y <= 0.4975:
                X = -0.023125
                Y = -0.00485
            elif 0.4975 < y and y <= 0.5325:
                X = -0.023276
                Y = -0.00495
            elif 0.5325 < y and y <= 0.5675:
                X = -0.024125
                Y = -0.004033
            elif 0.5675 < y and y <= 0.6025:
                X = -0.025484
                Y = -0.003542
            elif 0.6025 < y and y <= 0.6375:
                X = -0.026125
                Y = -0.004009
            elif 0.6375 < y and y <= 0.6725:
                X = -0.027192
                Y = -0.003175
            elif 0.6725 < y and y <= 0.7075:
                X = -0.028433
                Y = -0.003167
            #this point may include a error differnce 
            elif 0.7075 < y and y <= 0.7425:
                X = -0.028073
                Y = -0.006075

        if x == 0:
            if 0.1475 < y and y <= 0.1825:
                X = -0.00965
                Y = -0.0026
            elif 0.1825 < y and y <= 0.2175:
                X = -0.01095
                Y = -0.00265
            elif 0.1275 < y and y <= 0.2525:
                X = -0.0124
                Y = -0.00265
            elif 0.2525 < y and y <= 0.2875:
                X = -0.0136
                Y = -0.0047
            elif 0.2875 < y and y <= 0.3225:
                X = -0.0154
                Y = -0.0068
            elif 0.3225 < y and y <= 0.3575:
                X = -0.0169
                Y = -0.0054
            elif 0.3575 < y and y <= 0.3925:
                X = -0.0171
                Y = -0.0054
            elif 0.3925 < y and y <= 0.4275:
                X = -0.01765
                Y = -0.00545
            elif 0.4275 < y and y <= 0.4625:
                X = -0.01685
                Y = -0.00665
            elif 0.4625 < y and y <= 0.4975:
                X = -0.0164
                Y = -0.007
            elif 0.4975 < y and y <= 0.5325:
                X = -0.01745
                Y = -0.00735
            elif 0.5325 < y and y <= 0.5675:
                X = -0.0197
                Y = -0.0074
            elif 0.5675 < y and y <= 0.6025:
                X = -0.0223
                Y = -0.00755
            elif 0.6025 < y and y <= 0.6375:
                X = -0.02325
                Y = -0.00825
            elif 0.6375 < y and y <= 0.6725:
                X = -0.02515
                Y = -0.00755
            elif 0.6725 < y and y <= 0.7075:
                X = -0.0254
                Y = -0.00705
            elif 0.7075 < y and y <= 0.7425:
                X = -0.02555
                Y = -0.0092

        if x < 0:
            if 0.1475 < y and y <= 0.1825:
                X = -0.013042
                Y = -0.004167
            elif 0.1825 < y and y <= 0.2175:
                X = -0.012867
                Y = -0.005433
            elif 0.1275 < y and y <= 0.2525:
                X = -0.012942
                Y = -0.005017
            elif 0.2525 < y and y <= 0.2875:
                X = -0.014183
                Y = -0.006732
            elif 0.2875 < y and y <= 0.3225:
                X = -0.015558
                Y = -0.009757
            elif 0.3225 < y and y <= 0.3575:
                X = -0.0159
                Y = -0.010084
            elif 0.3575 < y and y <= 0.3925:
                X = -0.016125
                Y = -0.009209
            elif 0.3925 < y and y <= 0.4275:
                X = -0.016758
                Y = -0.008342
            elif 0.4275 < y and y <= 0.4625:
                X = -0.017358
                Y = -0.008858
            elif 0.4625 < y and y <= 0.4975:
                X = -0.018008
                Y = -0.009665
            elif 0.4975 < y and y <= 0.5325:
                X = -0.018783
                Y = -0.007875
            elif 0.5325 < y and y <= 0.5675:
                X = -0.017492
                Y = -0.0065
            #these next point for now on include an error differnce in the x ue to bad readings
            elif 0.5675 < y and y <= 0.6025:
                X = -0.01635
                Y = -0.008909
            elif 0.6025 < y and y <= 0.6375:
                X = -0.016175
                Y = -0.01105
            elif 0.6375 < y and y <= 0.6725:
                X = -0.016834
                Y = -0.009132
            elif 0.6725 < y and y <= 0.7075:
                X = -0.018142
                Y = -0.009884
            #this point has a error differnece in y due to a bad reading
            elif 0.7075 < y and y <= 0.7425:
                X = -0.016792
                Y = -0.024967
        return X,Y

    def place(self):
        self.bot.arm.set_ee_pose_components(x=0.3, z=0.2)

        self.bot.arm.set_ee_pose_components(y=-0.4, z=0.2)
        self.bot.gripper.open() 
        self.bot.arm.set_ee_pose_components(y=-0.35, z=0.2)
    
    def home(self):
        self.bot.arm.set_ee_pose_components(x=0.3, z=0.2)

    def run(self):
        self.place()
        while(1==1):
            pass_on, x, y = self.scanner()
            while(pass_on == True):
                self.ml()
                self.AOA()
                self.place()
                pass_on, x, y = self.scanner()



