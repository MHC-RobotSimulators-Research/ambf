import multiprocessing
import sys
import os
import time
from ambf_client import Client
import math
import numpy as np

# index 0 --> home
# index 1 --> sine_dance
# index 2 --> Quit

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
        self.home_joints = [math.pi/3, math.pi*3/5, -0.09, math.pi*3/4, 0, math.pi/6, math.pi/6]
        self.dance_scale_joints = [0.3, 0.3, 0.06, 0.3, 1.2, math.pi/6, math.pi/6]
        self.loop_rate = 1000
        self.raven_joints = 7
        self.rc = [0,0]
        self.rampup_count = np.array(self.rc)
        self.i = 0
        self.speed = 1.00/self.loop_rate
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

def control_reset():
    new_control = [False, False, False]
    return new_control

def do(q, raven):
        control = [False, False, False]
        while not control[2]:
            if not q.empty():
                control = q.get()
            while control[0]and not any(raven.homed): #if after homing, code breaks, needs assistance
                for i in range(raven.loop_rate):
                    if not i:
                        print("staring homing")
                        raven.homed[0] = raven.go_home(1, 1, i)
                        raven.homed[1] = raven.go_home(1, 0, i)
                    else:
                        raven.homed[0] = raven.go_home(0, 1, i)
                        raven.homed[1] = raven.go_home(0, 0, i)
                    time.sleep(0.01)
                if raven.homed[0] and raven.homed[1]:
                    print("Raven is homed!")
                if not q.empty():
                    control = q.get()
            while control[1]:
                if raven.i == 0:
                    raven.sine_dance(1, 1, raven.i, raven.rampup_count)
                    raven.sine_dance(1, 0, raven.i, raven.rampup_count)
                else:
                    raven.sine_dance(0, 1, raven.i, raven.rampup_count)
                    raven.sine_dance(0, 0, raven.i, raven.rampup_count)
                    if not q.empty():
                        control = q.get()
                raven.i += 1
                time.sleep(0.01)
        print("shutting down...\n")
        os.system('kill %d' % os.getpid())
        exit(0)

def get_input(q, stdin):
    control = [False, False, False]
    sys.stdin = stdin
    print("Input Menu:\n")
    print("input 'h' for home\n")
    print("input 's' for sine dance\n")
    print("input 'q' for quit\n")
    print("Please select a control mode:")
    userinput = raw_input()
    while not control[2]:
        print("Switching control modes...\n")
        control = control_reset()
        if userinput == 'h':
            print("homing...")
            control[0] = True
            q.put(control)
            userinput = raw_input("Input key to switch control modes\n")
            continue
        elif userinput == 's':
            print("doing sine dance...")
            control[1] = True
            q.put(control)
            userinput = raw_input("Input key to switch control modes\n")
            continue
        elif userinput == 'q':
            control[2] = True
            q.put(control)



def main():
    raven = ambf_raven()
    q = multiprocessing.Queue()
    newstdin = os.fdopen(os.dup(sys.stdin.fileno()))
    p1 = multiprocessing.Process(target = get_input, args = (q, newstdin))
    p1.start()
    do(q, raven)
    p1.join()


if __name__ == '__main__':
    main()
