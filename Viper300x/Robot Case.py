def robot_case(pick_up, pick_up_case, clusters):
    if pick_up = True:
        x,y,z = clusters[0]["position"]
        if pick_up_case = 1:
            bot.arm.set_ee_pose_components(x=x, y=y, z=0.2, pitch=1.5)
            ee_cartesian_trajectory(z=z-0.18)
        elif pick_up_case = 2:
            bot.arm.set_ee_pose_components(x=x, y=y, z=0.2, pitch=1.5, roll = 0.5) #roll and z need to be adjusted
            ee_cartesian_trajectory(z=z-0.18)
        elif pick_up_case = 3:
            bot.arm.set_ee_pose_components(x=x, y=y, z=0.2, pitch=1.5, roll = 1.5) #roll and z need to be adjusted
            ee_cartesian_trajectory(z=z-0.18)
        elif pick_up_case = 4:
            bot.arm.set_ee_pose_components(x=x, y=y, z=0.2)
            ee_cartesian_trajectory(z=z-0.18)
        close()
        ee_pose_components(x=x+.02, y=y-0.03, z=z+0.1)
        ee_pose_components(x=0.3, z=0.2)
    
   		bot.arm.set_ee_pose_components(x=x, y=y, z=0.2)
        ee_cartesian_trajectory(z=z-0.18)
		bot.gripper.close()
        ee_pose_components(x=x+.02, y=y-0.03, z=z+0.1)
        ee_pose_components(x=0.3, z=0.2)
    
        ee_pose_components(y=-0.4, z=0.2)
		#bot.arm.set_ee_pose_components(y=-0.4, z=0.2)
		bot.arm.set_ee_cartesian_trajectory(z=z-0.18)
		bot.gripper.open() 
		bot.arm.set_ee_pose_components(y=-0.35, z=0.2)
				
		break
        
            
            
            
    