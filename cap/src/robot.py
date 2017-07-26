# (C) Anand Saha <anandsaha@gmail.com>
# 23rd July 2017

import time
import vrep
import numpy as np

class RobotArm(object):

    def __init__(self, ip, port):

        vrep.simxFinish(-1)
        self.clientID=vrep.simxStart(ip,port,True,True,5000,5)
        if self.clientID != -1:
            print('Successfully connected to V-REP')
        else:
            raise RuntimeError('Could not connect to V-REP')

        self.sleep_sec=0.4
        self.sleep_sec_min=0.2
        self.main_object = 'uarm'
        self.claw_enabled = False

        err, self.cylinder_handle = vrep.simxGetObjectHandle(self.clientID, 'uarm_pickupCylinder2', 
                vrep.simx_opmode_blocking)

        err, self.bin_handle = vrep.simxGetObjectHandle(self.clientID, 'uarm_bin', 
                vrep.simx_opmode_blocking)

        err, self.claw_handle = vrep.simxGetObjectHandle(self.clientID, 'uarm_Gripper_motor1Method2', 
                vrep.simx_opmode_blocking)

        err, self.sensor_handle = vrep.simxGetObjectHandle(self.clientID, 'uarmGripper_fakeGrippingSensor', 
                vrep.simx_opmode_blocking)

        err, self.center_handle = vrep.simxGetObjectHandle(self.clientID, 'uarmGripper_motor1Method2', 
                vrep.simx_opmode_blocking)

        if err != vrep.simx_return_ok:
            print("Could not get gripper handle")

        self.cylinder_height = self.get_object_height(self.cylinder_handle)


    def __del__(self):
        print('Disconnecting from V-REP')
        self.disconnect()


    def disconnect(self):
        self.stop_sim()
        vrep.simxGetPingTime(self.clientID)
        vrep.simxFinish(self.clientID)


    def get_position(self, handle):
        time.sleep(self.sleep_sec_min)
        err, pos = vrep.simxGetObjectPosition(self.clientID, handle, -1, 
                vrep.simx_opmode_blocking)
        return pos


    def get_env_dimensions(self):
        # X min, max; Y min, max; Z min, max
        dim = [[-0.31, -0.22], [-0.11, -0.09], [0, 0.12]]    
        return dim


    def goto_position(self, pos):
        time.sleep(self.sleep_sec)

        pos_adjusted = list(pos)
        pos_adjusted.append(0.0)
        pos_adjusted.append(0.0)

        res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID, 
                self.main_object, vrep.sim_scripttype_childscript,
                'computeAnglesFromGripperPositionEx', 
                [], pos_adjusted, [], bytearray(), 
                vrep.simx_opmode_blocking)

        res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID, 
                self.main_object, vrep.sim_scripttype_childscript,
                'moveToPositionEx', 
                [], retFloats, [], bytearray(), 
                vrep.simx_opmode_blocking)


    def enable_claw(self, enable):
        time.sleep(self.sleep_sec_min)

        i_enable = 1
        if not enable:
            i_enable = -1

        res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID, 
                self.main_object, vrep.sim_scripttype_childscript,
                'enableGripperEx', 
                [i_enable], [], [], bytearray(), 
                vrep.simx_opmode_blocking)
        
        self.claw_enabled = enable



    def get_gripper_status(self, mode):
        time.sleep(self.sleep_sec_min)

        err, state, point, handle, vector = vrep.simxReadProximitySensor(self.clientID, 
                self.sensor_handle, mode);

        if err != vrep.simx_return_ok:
            print("Could not get gripper status")
            return

        print(state)

    def start_sim(self):
        time.sleep(self.sleep_sec)
        vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_oneshot)


    def stop_sim(self):
        time.sleep(self.sleep_sec)
        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_oneshot)


    def restart_sim(self):
        self.stop_sim()
        self.start_sim()


    def get_object_height(self, handle):
        time.sleep(self.sleep_sec_min)
        err, minval = vrep.simxGetObjectFloatParameter(self.clientID,
                handle,vrep.sim_objfloatparam_modelbbox_min_z, vrep.simx_opmode_blocking)

        if err != vrep.simx_return_ok:
            print("Could not get min z")

        err, maxval = vrep.simxGetObjectFloatParameter(self.clientID,
                handle,vrep.sim_objfloatparam_modelbbox_max_z, vrep.simx_opmode_blocking)

        if err != vrep.simx_return_ok:
            print("Could not get max z")

        return (maxval - minval)

    def is_object_held(self):
        pos_cylinder = self.get_position(self.cylinder_handle)
        pos_gripper = self.get_position(self.center_handle)

        x = np.abs(pos_cylinder[0] - pos_gripper[0])
        y = np.abs(pos_cylinder[1] - pos_gripper[1])
        z = np.abs(pos_cylinder[2] - pos_gripper[2])

        if x < 0.01 and y < 0.01 and z < (self.cylinder_height / 2) and self.claw_enabled:
            return True
        
        return False 

    def is_object_in_bin(self):
        pos_bin = self.get_position(self.bin_handle)
        pos_cyl = self.get_position(self.cylinder_handle)
        return self.distance(pos_bin,pos_cyl) < 0.013

    @staticmethod
    def distance(pos1, pos2):
        x2 = np.square(pos1[0] - pos2[0])
        y2 = np.square(pos1[1] - pos2[1])
        z2 = np.square(pos1[2] - pos2[2])

        return np.sqrt(x2 + y2 + z2)

