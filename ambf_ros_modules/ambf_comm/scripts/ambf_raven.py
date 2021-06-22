import time
from ambf_client import Client
import math
import numpy as np
import ambf_raven_def as ard

'''
author: Sean Fabrega
ambf_raven defines methods for an ambf_raven robot, including homnig, sine dance and hoping soon
cube tracing and soft body manipulation'''

class ambf_raven:
    def __init__(self):
        self._client = Client()
        self._client.connect()
        raw_input("We can see what objects the client has found. Press Enter to continue...")
        # You can print the names of objects found. We should see all the links found
        print(self._client.get_obj_names())
        self.homed = [False, False]
        #self.l_handle =  self._client.get_obj_handle('raven_2/base_link_L')
        #print(self.l_handle.get_num_joints())
        #self.r_handle = self._client.get_obj_handle('raven_2/base_link_R')
        self.arms = [self._client.get_obj_handle('raven_2/base_link_L'), self._client.get_obj_handle('raven_2/base_link_R')]
        self.start_jp = np.zeros((2,7)) #indexed at 0
        self.delta_jp = np.zeros((2,7))
        self.home_joints = ard.home_joints
        self.dance_scale_joints = ard.dance_scale_joints
        self.loop_rate = ard.raven_loop_rate
        self.raven_joints = ard.raven_joints
        self.rc = [0,0]
        self.rampup_count = np.array(self.rc)
        self.i = 0
        self.speed = 10.00/self.loop_rate
        self.rampup_speed = 0.5/self.loop_rate

    def go_home(self,first_entry, arm, count):
        '''
        first entry --> bool
        arm --> bool (0 for left, 1 for right)
        count --> int
        '''
        #if first time calling go home
        if(first_entry):
            for i in range(self.arms[arm].get_num_joints()):
                self.start_jp[arm][i] = self.arms[arm].get_joint_pos(i)
                self.delta_jp[arm][i] = self.home_joints[i] - self.arms[arm].get_joint_pos(i)
        #gradualizes movement from a to b
        scale = min(1.0*count/self.loop_rate, 1.0)
        #array containing distance to go to start point
        diff_jp = [0,0,0,0,0,0,0]

        #sets position for each joint
        for i in range(self.arms[arm].get_num_joints()):
            self.arms[arm].set_joint_pos(i, scale * self.delta_jp[arm][i] + self.start_jp[arm][i])
            diff_jp[i] = abs(self.home_joints[i] - self.arms[arm].get_joint_pos(i))
        #in progress, indicates when arm is honed


        max_value = np.max(diff_jp)

        if(max_value < 0.1):
            #for i in range(7):
            #print("joint num " + str(i) + "joint position " + str(state.get_joint_pos(i)))
            self.homed[arm] = True

        else:
            self.homed[arm] = False
        return self.homed[arm]

    def sine_dance(self,first_entry, arm, count, rampup_count):
        self.homed[arm] = False
        for i in range(self.raven_joints):
            offset = (i+arm)*math.pi/2
            #print("offset = " + str(offset))
            rampup = min(self.rampup_speed*self.rampup_count[arm], 1.0)
            #print("rampup = " + str(rampup))
            self.arms[arm].set_joint_pos(i, rampup*self.dance_scale_joints[i]*math.sin(self.speed*(count+offset))+self.home_joints[i])
            #if(i == 0):
            #   print("actual joint position = " + str(state.get_joint_pos(0)))
            #   print("commanded join position = " + str(dance_scale_joints[0]*math.sin(speed*(count+offset))+home_joints[0]))
            self.rampup_count[arm] += 1
