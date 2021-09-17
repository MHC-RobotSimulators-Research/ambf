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
import csv
record = False # this flag determines whether ambf raven states are recorded

'''
author: Sean
ambf_raven_controller is a Client for operating the ambf_raven simulated robot, specifically designed for
using sine_dance and recording data to be used create ml file
'''


def control_reset():
    '''
    resets all control values
    '''
    new_control = [False, False, False, False]
    return new_control

def do(q, raven, csvfloats):
    '''
    performs the main actions of the robot based on the values
    in the control array
    '''
    control = [False, False, False, False]
    '''
    control[0] = homing
    control[1] = sine dance
    control[2] = quit
    control[3] = file mode
    '''
    while not control[2]:
        if not q.empty():
            control = q.get()
        while control[0]and not any(raven.homed): #if after homing, code breaks, needs assistance
            for i in range(raven.loop_rate):
                if not i:
                    print("staring homing")
                    #moves raven incrementally towards home position, if home position is reached, returns True
                    raven.homed[0] = raven.go_home(1, 1, i)
                    raven.homed[1] = raven.go_home(1, 0, i)
                else:
                    raven.homed[0] = raven.go_home(0, 1, i)
                    raven.homed[1] = raven.go_home(0, 0, i)
                time.sleep(0.01)
            if raven.homed[0] and raven.homed[1]:
                print("Raven is homed!")
            #checks the queue afterwards to see if user provided new input
            if not q.empty():
                control = q.get()
        while control[1]:
            if raven.i == 0:
                start = time.time()
                #similar to homing, moves raven incrementally in a sine pattern
                raven.sine_dance(1, 1, raven.i, raven.rampup_count)
                raven.sine_dance(1, 0, raven.i, raven.rampup_count)
            else:
                raven.sine_dance(0, 1, raven.i, raven.rampup_count)
                raven.sine_dance(0, 0, raven.i, raven.rampup_count)
                if not q.empty():
                    control = q.get()
            raven.i += 1
            time.sleep(0.01)
        while control[3] and not raven.finished:
            '''
            moves raven along a trajectory defined by a .csv function with 7 columns for each
            joint position in the desired movement
            '''
            if record:
                with open('raven_state_record.csv', 'wb') as file:
                    writer = csv.writer(file)
                    start = time.time()

                    while not raven.finished or not q.empty():
                        curr_time = time.time() - start
                        if int(curr_time * 1000) > 110256:
                            raven.finished = True
                            print("Raven has completed set trajectory")
                            break
                        raven.set_raven_pos(csvfloats[int(curr_time * 1000)])
                        line = raven.get_raven_status()
                        if report:
                            writer.writerow(line)
            else:
                start = time.time()
                while not raven.finished or not q.empty():
                    curr_time = time.time() - start
                    if int(curr_time * 1000) > 110256:
                        raven.finished = True
                        print("Raven has completed set trajectory")
                        break
                    raven.set_raven_pos(csvfloats[int(curr_time * 1000)])
                    time.sleep(0.01)
            if not q.empty():
                control = q.get()

    print("shutting down...\n")
    os.system('kill %d' % os.getpid())
    exit(0)

def get_input(q, stdin):
    '''
    continuously loops to collect new inputs from user in order to switch
    control modes
    '''
    control = [False, False, False, False]
    sys.stdin = stdin #this is to access standard input in the thread
    print("Input Menu:\n")
    print("input 'h' for home\n")
    print("input 's' for sine dance\n")
    print("input 'f' for motion from file\n")
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
        elif userinput == 'f':
            control[3] = True
            q.put(control)
            userinput = raw_input("Input key to switch control modes\n")



def main():
    '''
    runs the controller by initializing a thread to collect user inputs and
    calling the do() method to move the robot according to what the user inputs
    '''
    raven = arav.ambf_raven()#creates raven object
    q = multiprocessing.Queue()#creates queue for sharing data between main thread and get_input thread
    newstdin = os.fdopen(os.dup(sys.stdin.fileno()))#connects standard input to thread
    p1 = multiprocessing.Process(target = get_input, args = (q, newstdin))#creates multiprocess
    p1.start()#starts multiprocess
    with open("ravenpath.csv", mode = 'r') as file:#loads .csv file
        csvFile = csv.reader(file)
        csvfloats = np.zeros((110255, 16), dtype = "float")
        i = 0
        j = 0
        for lines in csvFile:
            for cell in lines:
                csvfloats[i][j] = (float(cell.strip("\xef\xbb\xbf")))
                j += 1
            i += 1
            j = 0
    do(q, raven, csvfloats)
    p1.join()


if __name__ == '__main__':
    main()
