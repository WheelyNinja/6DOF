from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np
import time as tm

# This script makes the end-effector perform pick, pour, and place tasks
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx300s'
# Then change to this directory and type 'python bartender.py'

def main():
    bot = InterbotixManipulatorXS("vx300s", "arm", "gripper")
    bot.arm.set_ee_pose_components(x=0.3, y=0.3, z=0.3)
    
    bot.gripper.open()
    
    bot.arm.set_single_joint_position("wrist_angle", np.pi/4.42)
    
    bot.arm.set_single_joint_position("elbow", np.pi/4.36)

    bot.arm.set_single_joint_position("shoulder", np.pi/39.27)
    
    bot.gripper.set_pressure(3.5)
    
    bot.gripper.close()
    
    bot.arm.set_single_joint_position("elbow", np.pi/10)
    bot.arm.set_single_joint_position("waist", np.pi/1.9)
    
    bot.gripper.open()
    tm.sleep(1)
    bot.gripper.close()
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()
 

if __name__=='__main__':
    main()
