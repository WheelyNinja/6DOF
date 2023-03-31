import time
import numpy as np
from AOA_F import * 
from Robot_Case import *
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface

# This script uses a color/depth camera to get the arm to find objects and pick them up.
# For this demo, the arm is placed to the left of the camera facing outward. When the
# end-effector is located at x=0, y=-0.3, z=0.2 w.r.t. the 'wx200/base_link' frame, the AR
# tag should be clearly visible to the camera. A small basket should also be placed in front of the arm.
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_perception xsarm_perception.launch robot_model:=wx200'
# Then change to this directory and type 'python pick_place.py'

def main():
# Initialize the arm module along with the pointcloud and armtag modules
	bot = InterbotixManipulatorXS("vx300s", moving_time=2.5, accel_time=.75)
	pcl = InterbotixPointCloudInterface()
	armtag = InterbotixArmTagInterface()
	#r_dis = 0.03 #for parallel AOA
	#r_dis = 0.05 #for perpendicular AOA
	#x_dis = -0.0191 #original x
	#y_dis = -0.0045 #original y
	#set initial arm and gripper pose
	bot.arm.set_ee_pose_components(x=0.3, z=0.2)
	bot.gripper.open()

	# get the ArmTag pose
	# bot.arm.set_ee_pose_components(y=-0.3, z=0.2)
	# time.sleep(0.5)
	# armtag.find_ref_to_arm_base_transform()
	#bot.arm.set_ee_pose_components(x=0.3, z=0.2)

	# get the cluster positions
	# sort them from max to min 'x' position w.r.t. the 'wx200/base_link' frame
	while(1==1):
		success, clusters = pcl.get_cluster_positions(ref_frame="vx300s/base_link", sort_axis="y", reverse=False)
		print(type(clusters))

		# pick up all the objects and drop them in a virtual basket in front of the robot
		
		while (success==1):
			pick_up, pick_up_case = AOA(clusters)
			print("pick up: ", pick_up)
			print("pick up case: ", pick_up_case)
			robot_case(bot, pick_up, pick_up_case, clusters)
			
			bot.arm.set_ee_pose_components(x=0.3, z=0.2)

			bot.arm.set_ee_pose_components(y=-0.4, z=0.2)
			#bot.arm.set_ee_pose_components(y=-0.4, z=0.2)
			#bot.arm.set_ee_cartesian_trajectory(z=0.2)
			bot.gripper.open() 
			bot.arm.set_ee_pose_components(y=-0.35, z=0.2)
			
			success, clusters = pcl.get_cluster_positions(ref_frame="vx300s/base_link", sort_axis="y", reverse=False)
			bot.arm.set_ee_pose_components(x=0.3, z=0.2)
		#bot.arm.set_ee_pose_components(x=0.3, z=0.2)
		#bot.arm.go_to_sleep_pose()
		#time.sleep(2)
		
#def robot_case(bot, pick_up, pick_up_case, clusters):
	#if pick_up == True:
		#x,y,z = clusters[0]["position"]
		#if pick_up_case == 1:
			#bot.arm.set_ee_pose_components(x=x, y=y, z=0.2, pitch=1.5)
			#bot.arm.set_ee_cartesian_trajectory(z=z-0.18)
		#elif pick_up_case == 2:
			#bot.arm.set_ee_pose_components(x=x, y=y, z=0.2, pitch=1.5, roll = 0.5) #roll and z need to be adjusted
			#bot.arm.set_ee_cartesian_trajectory(z=z-0.18)
		#elif pick_up_case == 3:
			#bot.arm.set_ee_pose_components(x=x, y=y, z=0.2, pitch=1.5, roll = 1.5) #roll and z need to be adjusted
			#bot.arm.set_ee_cartesian_trajectory(z=z-0.18)
		#elif pick_up_case == 4:
			#bot.arm.set_ee_pose_components(x=x, y=y, z=0.2)
			#bot.arm.set_ee_cartesian_trajectory(z=z-0.18)
		#bot.gripper.close()
		#bot.arm.set_ee_cartesian_trajectory(z=z+0.18)
		
if __name__=='__main__':
    main()
