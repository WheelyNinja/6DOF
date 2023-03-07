import time
import math
import numpy as np
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

    # distance change for picking from above
    r_dis = 0.04 # change to length of gripper to center might need to account for shift in grippers
    z_dis = 0.04 # fix

	# set initial arm and gripper pose
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

		# pick up all the objects and drop them in a virtual basket in front of the robot
		
        while (success==1):
            for cluster in clusters:
                x, y, z = cluster["position"]
                
                
                
                # coversion to polar 
                r = math.sqrt(x**2 + y**2)
                theta = math.atan(y/x)
                
                # shift radius
                r += r_dis
                # conversion
                x = r * math.cos(theta)
                y = r * math.sin(theta)
                
                # shift z
                z += z_dis
                
                # go to x,y,z
                bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.2)
                
                # look down
               
                pos = bot.arm.get_joint_commands()
                pos[4] += np.pi/2.0

                #bot.arm.set_ee_pose_components(x=x, y=y, z=0.2)
                bot.arm.set_ee_cartesian_trajectory(z=z-0.18)
                bot.gripper.close()
                bot.arm.set_ee_pose_components(x=x+.02, y=y-0.03, z=z+0.1)
                bot.arm.set_ee_pose_components(x=0.3, z=0.2)
                
                bot.arm.set_ee_pose_components(y=-0.4, z=0.2)
                #bot.arm.set_ee_pose_components(y=-0.4, z=0.2)
                bot.arm.set_ee_cartesian_trajectory(z=z-0.18)
                bot.gripper.open() 
                bot.arm.set_ee_pose_components(y=-0.35, z=0.2)
				
                break
            success, clusters = pcl.get_cluster_positions(ref_frame="vx300s/base_link", sort_axis="y", reverse=False)
		#bot.arm.set_ee_pose_components(x=0.3, z=0.2)
		#bot.arm.go_to_sleep_pose()
        time.sleep(2)
if __name__=='__main__':
    main()