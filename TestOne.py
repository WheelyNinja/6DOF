from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np
import time as tm

# This script makes the end-effector perform pick, pour, and place tasks
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250'
# Then change to this directory and type 'python bartender.py'

def main():
    bot = InterbotixManipulatorXS("vx300s", "arm", "gripper")
    bot.arm.set_ee_pose_components(x=0.3, y=0.5,z=0.3)
    # bot.arm.set_single_joint_position("waist", np.pi/2.0)
    bot.gripper.open()
    tm.sleep(1)
    bot.gripper.close()
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()
 

if __name__=='__main__':
    main()
