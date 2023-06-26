import multiprocessing as mp
import sys
import os
import time
from ambf_client import Client
import math as m
import numpy as np
import ambf_raven as arav
import csv
import ambf_raven_def as ard
import ambf_xbox_controller as axc


'''
author: Sean
ambf_raven_controller is a Client for operating the ambf_raven simulated robot, specifically designed for
using sine_dance and recording data to be used create ml file
'''

sys.path.insert(0, 'ambf/ambf_ros_modules/ambf_client/python/ambf_client')


def control_reset():
    """
    resets all control values
    """
    new_control = [False, False, False, False, False]
    return new_control


def do(q, raven, csvData, xbc):
    """
    performs the main actions of the robot based on the values
    in the control array

    Args:

        q : a multiprocessing queue
        raven : an ambf_raven instance
        csvData : an array containing data from csv
        xbc : an ambf_xbox_controller instance
    """
    control = [False, False, False, False, False]
    '''
    control[0] = homing
    control[1] = sine dance
    control[2] = quit
    control[3] = file mode
    control[4] = manual mode
    '''
    # Sets which mode will be used in manual control
    arm_control = [True, True]
    # True for p5 ik and false for standard ik
    ik_mode = True
    last_j4 = 0.0 # used for smooth rotation of joint 4

    # DH values for home position of each arm
    home_dh = np.array([[1.04719755, 1.88495559, -0.03, 2.35619449 - m.pi / 2, 0., 0., 0.52359878],
                        [1.04719755, 1.88495559, -0.03, 2.35619449 - m.pi / 2, 0., -0., 0.52359878]],
                       dtype="float")

    while not control[2]:
        if not q.empty():
            control = q.get()
        while control[0]and not any(raven.homed): #if after homing, code breaks, needs assistance
            for i in range(raven.loop_rate):
                if not i:
                    print("starting homing")
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
            if ard.RECORD_FLAG:
                with open(ard.TO_FILE, 'wb') as file:
                    writer = csv.writer(file)
                    line = raven.get_raven_status(0,True)
                    writer.writerow(line)
                    start = time.time()

                    while not raven.finished or not q.empty():
                        curr_time = time.time() - start
                        if int(curr_time * 1000) >= csvData.shape[0]:
                            raven.finished = True
                            print("Raven has completed set trajectory")
                            break
                        raven.set_raven_pos(csvData[int(curr_time * 1000)])
                        line = raven.get_raven_status(int(curr_time * 1000))
                        writer.writerow(line)
            else:
                start = time.time()
                while not raven.finished or not q.empty():
                    curr_time = time.time() - start
                    if int(curr_time * 50) >= csvData.shape[0]:
                        raven.finished = True
                        print("Raven has completed set trajectory")
                        break
                    raven.set_raven_pos(csvData[int(curr_time * 50)])
                    #time.sleep(0.01)
            if not q.empty():
                control = q.get()

        while control[4]:
            '''
            Manual control mode for the simulated raven2 using an xbox controller. There are two
            modes. The first enables simultaneous control of both arms on the xyz axes, but locks
            joints 4, 5, and 6 to their home positions. Accessed by simultaneously pressing back 
            and start buttons.
            
            Left stick: left arm x and y
            Left trigger: left arm gripper open close
            Left button: when pressed left stick up and down controls left arm z
            Right stick: right arm x and y
            Right trigger: right arm gripper open close
            Right button: when pressed right stick up and down controls right arm z
            A button: use p5 inverse kinematics
            B button: use standard inverse kinematics
            
            The second control mode only controls one arm at a time, but adds control of joints 4 and 5.
            Accessed by pressing back for the left arm and start for the right arm.
            
            Left stick: selected arm x and y
            Left trigger: selected arm gripper open close
            Left button: when pressed left stick up and down controls selected arm z
            Right stick: controls grippers angle and rotation
            '''

            div = 200   # how much the raw input values will be divided by to produce the change in x,y,z
            dead_zone = 0.1

            # Cartesian coordinates are relative to the current position
            x = [0.0, 0.0]
            y = [0.0, 0.0]
            z = [0.0, 0.0]
            # gangle is absolute
            gangle = [0.0, 0.0]

            # DH values for home position of each arm
            home_dh = np.array([[1.04719755, 1.88495559, -0.03, 2.35619449 - m.pi / 2, 0., 0., 0.52359878],
                                [1.04719755, 1.88495559, -0.03, 2.35619449 - m.pi / 2, 0., -0., 0.52359878]],
                               dtype="float")

            controller = xbc.read()

            # Set which control mode to use
            if controller[2][4] and controller[2][5]:
                arm_control[0] = True
                arm_control[1] = True
                print("Controlling both arms")
            elif controller[2][4]:
                arm_control[0] = True
                arm_control[1] = False
                print("Controlling the left arm")
            elif controller[2][5]:
                arm_control[0] = False
                arm_control[1] = True
                print("Controlling the righ arm")

            # Set kinematics mode
            if controller[2][0]:
                ik_mode = True
                print("Using p5 inverse kinematics")
            if controller[2][1]:
                ik_mode = False
                print("Using standard inverse kinematics")

            # Coarse control of both raven arms
            if arm_control[0] and arm_control[1]:
                # Update coordinates for left arm, note x and y are swapped to make controls more intuitive
                if controller[0][3] == 1 and dead_zone < abs(controller[0][1]):
                    z[0] = -controller[0][1] / div
                else:
                    if dead_zone < abs(controller[0][0]):
                        y[0] = -controller[0][0] / div
                    if dead_zone < abs(controller[0][1]):
                        x[0] = -controller[0][1] / div
                # Update coordinates for right arm
                if controller[1][3] == 1 and dead_zone < abs(controller[1][1]):
                    z[1] = -controller[1][1] / div
                else:
                    if dead_zone < abs(controller[1][0]):
                        y[1] = -controller[1][0] / div
                    if dead_zone < abs(controller[1][1]):
                        x[1] = -controller[1][1] / div
                # Set gripper angles
                gangle[0] = 1 - (controller[0][2] / 4)
                # for the right arm gangle needs to be negative, this is to fix a bug somewhere else that I can't find
                gangle[1] = -1 + (controller[1][2] / 4)

                # Plan next move based off of modifies cartesian coordinates
                raven.plan_move(0, x[0], y[0], z[0], gangle[0], ik_mode, home_dh)
                raven.plan_move(1, x[1], y[1], z[1], gangle[1], ik_mode, home_dh)

            # Fine control of one arm
            elif arm_control[0] or arm_control[1]:
                # Decide which arm to control
                arm = 0
                if arm_control[1]:
                    arm = 1

                # Cartesian control of desired arm
                if controller[0][3] == 1 and dead_zone < abs(controller[0][1]):
                    z[arm] = -controller[0][1] / div
                else:
                    if dead_zone < abs(controller[0][0]):
                        y[arm] = -controller[0][0] / div
                    if dead_zone < abs(controller[0][1]):
                        x[arm] = -controller[0][1] / div

                # Set gripper angle
                if arm_control[0]:
                    gangle[0] = 1 - (controller[1][2] / 4)
                else:
                    gangle[1] = -1 + (controller[1][2] / 4)

                # Control gripper position using home_dh values
                if dead_zone < abs(controller[1][0]) or dead_zone < abs(controller[1][1]):
                    # Position j5
                    j5 = m.sqrt(controller[1][0] ** 2 + controller[1][1] ** 2)
                    if controller[1][0] < 0:
                        home_dh[arm][4] = j5
                    else:
                        home_dh[arm][4] = -j5
                    # Find lj position
                    try:
                        lj_pos = m.atan(controller[1][1] / controller[1][0])
                    # exception to handle x = 0
                    except ZeroDivisionError:
                        if last_j4 < 0:
                            lj_pos = -m.pi / 2
                        else:
                            lj_pos = m.pi / 2
                    # calculate new j4 position
                    j4 = lj_pos
                    # Set j4 depending on which arm is being controlled
                    if arm_control[0]:
                        home_dh[arm][3] = -j4
                    else:
                        home_dh[arm][3] = j4

                # Reset last_j4 when there is no input
                else:
                    last_j4 = 0.0

                # Plan new position based off of desired cartesian changes
                raven.plan_move(0, x[0], y[0], z[0], gangle[0], True, home_dh)
                raven.plan_move(1, x[1], y[1], z[1], gangle[1], True, home_dh)

            # Incrementally move the simulated raven to the new planned position
            for i in range(raven.man_steps):
                if not i:
                    raven.moved[0] = raven.move(1, 1, i)
                    raven.moved[1] = raven.move(1, 0, i)
                else:
                    raven.moved[0] = raven.move(0, 1, i)
                    raven.moved[1] = raven.move(0, 0, i)
                time.sleep(0.01)
            # if raven.moved[0] and raven.moved[1]:
            #     print("Raven has moved!")

            # rumble the controller when raven is limited
            rumble = [0.0, 0.0]
            for i in range(2):
                if raven.limited[i]:
                    rumble[i] = 1
            if rumble[0] != 0.0 or rumble[1] != 0.0:
                xbc.rumble(rumble[0], rumble[1], 100)

            if not q.empty():
                control = q.get()

    print("shutting down...\n")
    os.system('kill %d' % os.getpid())
    exit(0)


def get_input(q, stdin, file_valid):
    """
    continuously loops to collect new inputs from user in order to switch
    control modes
    """
    control = [False, False, False, False, False]
    sys.stdin = stdin #this is to access standard input in the thread
    print("Input Menu:\n")
    print("input 'h' for home\n")
    print("input 's' for sine dance\n")
    if file_valid:
        print("input 'f' for motion from file\n")
    print("input 'm' for manual mode\n")
    print("input 'q' for quit\n")
    print("Please select a control mode:")
    userinput = input()
    while not control[2]:
        print("Switching control modes...\n")
        control = control_reset()
        if userinput == 'h':
            print("homing...")
            control[0] = True
            q.put(control)
            userinput = input("Input key to switch control modes\n")
            continue
        elif userinput == 's':
            print("doing sine dance...")
            control[1] = True
            q.put(control)
            userinput = input("Input key to switch control modes\n")
            continue
        elif userinput == 'q':
            control[2] = True
            q.put(control)
        elif userinput == 'f' and file_valid:
            control[3] = True
            q.put(control)
            userinput = input("Input key to switch control modes\n")
        elif userinput == 'm':
            control[4] = True
            q.put(control)
            userinput = input("Input key to switch control modes\n")


def file_loader():
    file_valid = True
    csvData= []
    if os.path.exists(ard.FROM_FILE):
        # specified utf 8 encoding to prevent BOM from being included in the first cell
        with open(ard.FROM_FILE, mode = 'r', encoding="utf-8-sig") as file:#loads .csv file
            csvFile = csv.reader(file)
            for lines in csvFile:
                csvLine = []
                for cell in lines:
                    csvLine.append(float(cell.strip("\xef\xbb\xbf")))
                csvData.append(csvLine)
    csvData = np.asarray(csvData, dtype = "float")

    # sanity check to see if the file follows the expected format
    if (csvData.shape[0] == 0):
        file_valid = False
        print("Raven trajectory file empty or not found.")
    elif (csvData.shape[1] != ard.COL_IN_FILE):
        file_valid = False
        print("Raven trajectory file format invalid. ("+str( ard.COL_IN_FILE)+" cols expected)")
    return file_valid, csvData


def main():
    '''
    runs the controller by initializing a thread to collect user inputs and
    calling the do() method to move the robot according to what the user inputs
    '''
    # load external raven trajectory file
    file_valid, csvData = file_loader()
    # creates raven object
    raven = arav.ambf_raven()
    # set raven man_steps
    raven.man_steps = 20
    # creates xbox controller object
    xbc = axc.XboxController()
    # creates queue for sharing data between main thread and get_input thread
    input_q = mp.Queue()
    # connects standard input to thread
    newstdin = os.fdopen(os.dup(sys.stdin.fileno()))
    # creates multiprocess
    get_inputs = mp.Process(target=get_input, args=(input_q, newstdin, file_valid))
    # starts multiprocess
    get_inputs.start()
    do(input_q, raven, csvData, xbc)
    get_inputs.join()


if __name__ == '__main__':
    main()
