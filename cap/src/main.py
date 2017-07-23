from robot import RobotArm

ra = RobotArm('127.0.0.1', 19997)

ra.restart_sim()

ra.enable_claw(False)
pos = ra.get_position(ra.cylinder_handle)
ra.goto_position(pos)
ra.enable_claw(True)
pos[2] *= 5 
ra.goto_position(pos)
ra.goto_position(ra.bin_position) 
ra.enable_claw(False)

ra.stop_sim()

ra.disconnect()
