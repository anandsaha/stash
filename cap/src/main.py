from robot import RobotArm
import numpy as np 
import time

ra = RobotArm('127.0.0.1', 19997)

ra.restart_sim()

ra.goto_position([-0.17, -0.09, 0.25])
time.sleep(5)

ra.enable_claw(False)
pos = ra.get_position(ra.cylinder_handle)
ra.goto_position(pos)
ra.enable_claw(True)
pos[2] *= 5 
ra.goto_position(pos)
ra.goto_position(ra.bin_position) 
ra.enable_claw(False)

i = 0
dim = ra.get_env_dimensions()

while i < 100:
    x = np.arange(dim[0][0], dim[0][1], 0.002)
    y = np.arange(dim[1][0], dim[1][1], 0.002)
    z = np.arange(dim[2][0], dim[2][1], 0.002)

    ra.goto_position([np.random.choice(x), np.random.choice(y), np.random.choice(z)])

    i += 1

ra.stop_sim()

ra.disconnect()
