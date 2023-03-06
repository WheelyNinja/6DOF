import time as tm
import numpy as np
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface


# This script makes the end-effector perform pick, pour, and place tasks
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250'
# Then change to this directory and type 'python bartender.py'

def main():
	bot = InterbotixManipulatorXS("vx300s", moving_time=2.5, accel_time=.75)
	pcl = InterbotixPointCloudInterface()
	armtag = InterbotixArmTagInterface()
	#print(x,y,z)
	bot.arm.go_to_home_pose()
	bot.arm.set_ee_pose_components(x=.3, z=0.2, pitch = np.pi/4)
	#bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
	#pos = bot.arm.get_joint_commands()
	#print(pos)
	#pos[4] += np.pi/2.0
	#print(pos)
	#bot.arm.set_joint_positions(pos)
	#bot.arm.set_ee_cartesian_trajectory(z=-0.15)
	#bot.arm.set_ee_pose_components(y=0.3, z=0.2)
	#bot.arm.set_single_joint_position("waist", np.pi/2.0)
	#bot.arm.set_ee_cartesian_trajectory(yaw=np.pi/2.0)
	bot.gripper.open()
	tm.sleep(30)
	armtag.find_ref_to_arm_base_transform()
	tm.sleep(999)
	bot.gripper.close()
	bot.arm.go_to_home_pose()
	bot.arm.go_to_sleep_pose()


if __name__=='__main__':
	main()
