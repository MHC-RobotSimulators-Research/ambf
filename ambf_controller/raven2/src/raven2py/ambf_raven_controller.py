import multiprocessing
import pygame
import sys
sys.path.insert(0, 'ambf/ambf_ros_modules/ambf_client/python/ambf_client')
import os
import time
from ambf_client import Client
import math
import numpy as np
import ambf_raven as arav


'''
author: Sean
ambf_raven_controller is a Client for operating the ambf_raven simulated robot, specifically designed for
using sine_dance and recording data to be used create ml file
'''


def control_reset():
    new_control = [False, False, False, False]
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
                    start = time.time()
                    raven.sine_dance(1, 1, raven.i, raven.rampup_count)
                    raven.sine_dance(1, 0, raven.i, raven.rampup_count)
                else:
                    raven.sine_dance(0, 1, raven.i, raven.rampup_count)
                    raven.sine_dance(0, 0, raven.i, raven.rampup_count)
                    if not q.empty():
                        control = q.get()
                raven.i += 1
                print("Time = "),
                print(time.time() - start)
                print(raven.get_t_command())
                time.sleep(0.01)
        print("shutting down...\n")
        os.system('kill %d' % os.getpid())
        exit(0)

def get_input(q, stdin):
    control = [False, False, False, False]
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
    raven = arav.ambf_raven()
    q = multiprocessing.Queue()
    newstdin = os.fdopen(os.dup(sys.stdin.fileno()))
    p1 = multiprocessing.Process(target = get_input, args = (q, newstdin))
    p1.start()
    do(q, raven)
    p1.join()


if __name__ == '__main__':
    main()
