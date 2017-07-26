from robot import RobotArm
import numpy as np 
import time
import vrep

def distance(pos1, pos2):
    x2 = np.square(pos1[0] - pos2[0])
    y2 = np.square(pos1[1] - pos2[1])
    z2 = np.square(pos1[2] - pos2[2])
    return np.sqrt(x2 + y2 + z2)

def status(ra):
    return
    print("\nIs object held> ", ra.is_object_held())
    print("Is object in bin> ", ra.is_object_in_bin())

ra = RobotArm('127.0.0.1', 19997)
ra.restart_sim()
pos = ra.get_position(ra.cylinder_handle)
pos[2] += 0.025
status(ra)
ra.goto_position(pos)
status(ra)
pos = ra.get_position(ra.cylinder_handle)
status(ra)
ra.enable_claw(True)
status(ra)
pos[2] *= 6 
ra.goto_position(pos)
status(ra)
pos = ra.get_position(ra.bin_handle)
status(ra)
pos[2] *= 4
ra.goto_position(pos)

status(ra)
ra.enable_claw(False)

status(ra)
"""
#ra.get_gripper_status(vrep.simx_opmode_streaming)
#ra.get_gripper_status(vrep.simx_opmode_buffer)
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
"""


ra.stop_sim()

ra.disconnect()

