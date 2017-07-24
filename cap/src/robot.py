# (C) Anand Saha <anandsaha@gmail.com>
# 23rd July 2017

import time
import vrep

class RobotArm(object):

    def __init__(self, ip, port):

        vrep.simxFinish(-1)
        self.clientID=vrep.simxStart(ip,port,True,True,5000,5)
        if self.clientID != -1:
            print('Successfully connected to V-REP')
        else:
            raise RuntimeError('Could not connect to V-REP')

        self.sleep_sec=0.5
        self.main_object = 'uarm'

        err, self.cylinder_handle = vrep.simxGetObjectHandle(self.clientID, 'uarm_pickupCylinder', 
                vrep.simx_opmode_blocking)

        err, self.bin_handle = vrep.simxGetObjectHandle(self.clientID, 'uarm_bin', 
                vrep.simx_opmode_blocking)

        err, self.claw_handle = vrep.simxGetObjectHandle(self.clientID, 'uarm_Gripper_motor1Method2', 
                vrep.simx_opmode_blocking)


    def __del__(self):
        print('Disconnecting from V-REP')
        self.disconnect()

    def disconnect(self):
        self.stop_sim()
        vrep.simxGetPingTime(self.clientID)
        vrep.simxFinish(self.clientID)


    def get_position(self, handle):
        err, pos = vrep.simxGetObjectPosition(self.clientID, handle, -1, 
                vrep.simx_opmode_blocking)
        return pos


    def get_env_dimensions(self):
        # X min, max; Y min, max; Z min, max
        dim = [[-0.38, -0.17], [-0.13, -0.09], [0, 0.25]] 
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
        time.sleep(self.sleep_sec)

        i_enable = 1
        if not enable:
            i_enable = -1

        res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(self.clientID, 
                self.main_object, vrep.sim_scripttype_childscript,
                'enableGripperEx', 
                [i_enable], [], [], bytearray(), 
                vrep.simx_opmode_blocking)


    def start_sim(self):
        time.sleep(self.sleep_sec)
        vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_oneshot)


    def stop_sim(self):
        time.sleep(self.sleep_sec)
        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_oneshot)


    def restart_sim(self):
        self.stop_sim()
        self.start_sim()


