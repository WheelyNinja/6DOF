import numpy as np
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface

def robot_case(bot, pick_up, pick_up_case, clusters):
	if pick_up == True:
		x,y,z = clusters[0]["position"]
		r_dis = 0.0375 #for perpendicular AOA
		if pick_up_case == 5: #below
			bot.arm.set_ee_pose_components(x=x, y=y, z=0.2)
			bot.arm.set_ee_cartesian_trajectory(z=z-0.18)
		else:
			r = np.sqrt(x**2 + y**2)
			theta = np.arctan(y/x)
			if theta < 0:
				theta += np.pi
			# shift radius
			r += r_dis
			# conversion
			x = r * np.cos(theta)
			print(x)
			y = r * np.sin(theta)
			print(y)
			if pick_up_case == 1: #above
				bot.arm.set_ee_pose_components(x=x, y=y, z=0.2, pitch=1.5)
				bot.arm.set_ee_cartesian_trajectory(z=z-0.18)			
			elif pick_up_case == 2: #above 45 roll
				bot.arm.set_ee_pose_components(x=x, y=y, z=0.2, pitch=1.5, roll = 0.75) #roll and z need to be adjusted
				bot.arm.set_ee_cartesian_trajectory(z=z-0.18)
			elif pick_up_case == 3: #above 90 roll
				bot.arm.set_ee_pose_components(x=x, y=y, z=0.2, pitch=1.5, roll = 1.5) #roll and z need to be adjusted
				bot.arm.set_ee_cartesian_trajectory(z=z-0.18)
			elif pick_up_case == 4: #above 135 roll
				bot.arm.set_ee_pose_components(x=x, y=y, z=0.2, pitch=1.5, roll = -0.75) #roll and z need to be adjusted
				bot.arm.set_ee_cartesian_trajectory(z=z-0.18)
			
		bot.gripper.close()
		bot.arm.set_ee_cartesian_trajectory(z=z+0.18)
		
