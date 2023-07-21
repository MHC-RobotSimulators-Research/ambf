import threading as th
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
import ambf_raven_recorder as arr
import raven_fk as fk


'''
authors: Natalie Chalfant, Sean Fabrega
ambf_raven_controller is a Client for operating the ambf_raven simulated robot, specifically designed for
using manual move mode to control the simulated raven II using an xbox controller and interact with the
simulated environment
'''

sys.path.insert(0, 'ambf/ambf_ros_modules/ambf_client/python/ambf_client')

CONTROL = [False, False, False, False, False]
RECORD = False
RECORDING = False
RECORD_TO = ""


def control_reset():
    """
    resets all control values
    """
    new_control = [False, False, False, False, False]
    return new_control


def update_pos_two_arm(dead_zone, controller, div):
    # Coarse control of both raven arms

    pos = [[0.0, 0.0],  # x - relative
           [0.0, 0.0],  # y - relative
           [0.0, 0.0],  # z - relative
           [0.0, 0.0]]  # gangle - absolute

    # Update coordinates for left arm, note x and y are swapped to make controls more intuitive
    if controller[0][3] == 1 and dead_zone < abs(controller[0][1]):
        pos[2][0] = -controller[0][1] / div
    else:
        if dead_zone < abs(controller[0][0]):
            pos[1][0] = -controller[0][0] / div
        if dead_zone < abs(controller[0][1]):
            pos[0][0] = -controller[0][1] / div
    # Update coordinates for right arm
    if controller[1][3] == 1 and dead_zone < abs(controller[1][1]):
        pos[2][1] = -controller[1][1] / div
    else:
        if dead_zone < abs(controller[1][0]):
            pos[1][1] = -controller[1][0] / div
        if dead_zone < abs(controller[1][1]):
            pos[0][1] = -controller[1][1] / div
    # Set gripper angles
    pos[3][0] = 1 - (controller[0][2] / 4)
    # for the right arm gangle needs to be negative, this is to fix a bug somewhere else that I can't find
    pos[3][1] = -1 + (controller[1][2] / 4)

    return pos


def update_pos_one_arm(dead_zone, controller, div, arm, home_dh):

    pos = [[0.0, 0.0],  # x - relative
           [0.0, 0.0],  # y - relative
           [0.0, 0.0],  # z - relative
           [0.0, 0.0]]  # gangle - absolute

    # Cartesian control of desired arm
    if controller[0][3] == 1 and dead_zone < abs(controller[0][1]):
        pos[2][arm] = -controller[0][1] / div
    else:
        if dead_zone < abs(controller[0][0]):
            pos[1][arm] = -controller[0][0] / div
        if dead_zone < abs(controller[0][1]):
            pos[0][arm] = -controller[0][1] / div

    # Left arm
    if not arm:
        # Set left gripper angle
        pos[3][0] = 1 - (controller[1][2] / 4)

        # Set right j4
        if dead_zone < abs(controller[1][0]):
            if abs(home_dh[0][3] - controller[1][0] / 10) < m.pi:
                home_dh[0][3] += -controller[1][0] / 10
        # Position j5
        if dead_zone < abs(controller[1][1]):
            if abs(home_dh[0][4] - controller[1][1] / 10) < 2:
                home_dh[0][4] += -controller[1][1] / 10

    # Right arm
    else:
        # Set right gripper angle
        pos[3][1] = -1 + (controller[1][2] / 4)

        # Set right j4
        if dead_zone < abs(controller[1][0]):
            if abs(home_dh[1][3] + controller[1][0] / 10) < m.pi:
                home_dh[1][3] += controller[1][0] / 10
        # Position j5
        if dead_zone < abs(controller[1][1]):
            if abs(home_dh[1][4] + controller[1][1] / 10) < 2:
                home_dh[1][4] += controller[1][1] / 10

    return pos, home_dh


def rumble_if_limited(raven, xbc):
    rumble = [0.0, 0.0]
    for i in range(2):
        if raven.limited[i]:
            rumble[i] = 1
    if rumble[0] != 0.0 or rumble[1] != 0.0:
        xbc.rumble(rumble[0], rumble[1], 100)


def do(raven, csvData, xbc, recorder=None):
    """
    performs the main actions of the robot based on the values
    in the control array

    Args:
        q : a multiprocessing queue
        raven : an ambf_raven instance
        csvData : an array containing data from csv
        xbc : an ambf_xbox_controller instance
    """
    global CONTROL
    '''
    control[0] = homing
    control[1] = sine dance
    control[2] = quit
    control[3] = file mode
    control[4] = manual mode
    '''
    global RECORD
    global RECORDING
    global RECORD_TO

    # Sets which mode will be used in manual control
    arm_control = [True, True]
    # True for p5 ik and false for standard ik
    ik_mode = True
    # DH values for home position of each arm
    home_dh = np.array([[1.04719755, 1.88495559, -0.03, 2.35619449 - m.pi / 2, 0., 0., 0.52359878],
                        [1.04719755, 1.88495559, -0.03, 2.35619449 - m.pi / 2, 0., -0., 0.52359878]],
                       dtype="float")
    curr_tm = None

    while not CONTROL[2]:

        while CONTROL[0] and not any(raven.homed):  # if after homing, code breaks, needs assistance
            '''
            Homing Mode:
            '''
            raven.go_home()

        while CONTROL[1]:
            '''
            Sine Dance:
            '''
            raven.sine_dance()

        while CONTROL[3] and not raven.finished:
            '''
            File Mode:
            moves raven along a trajectory defined by a .csv function with 7 columns for each
            joint position in the desired movement
            '''
            # if ard.RECORD_FLAG:
            #     with open(ard.TO_FILE, 'wb') as file:
            #         writer = csv.writer(file)
            #         line = raven.get_raven_status(0,True)
            #         writer.writerow(line)
            #         start = time.time()
            #
            #         while not raven.finished:
            #             curr_time = time.time() - start
            #             if int(curr_time * 1000) >= csvData.shape[0]:
            #                 raven.finished = True
            #                 print("Raven has completed set trajectory")
            #                 break
            #             raven.set_raven_pos(csvData[int(curr_time * 1000)])
            #             line = raven.get_raven_status(int(curr_time * 1000))
            #             writer.writerow(line)
            # else:

            if RECORD:
                recorder.start_recording(RECORD_TO)
                recorder.write_raven_status(raven, True)

            start = time.time()
            while not raven.finished:
                if RECORD:
                    recorder.write_raven_status(raven)
                curr_time = time.time() - start
                if int(curr_time * 50) >= csvData.shape[0]:
                    raven.finished = True
                    print("Raven has completed set trajectory")
                    break
                raven.set_raven_pos(csvData[int(curr_time * 50)])
                time.sleep(0.005)
                print(curr_time)

            recorder.stop_recording()

        if CONTROL[4] and xbc is None:
            print("No xbox controller detected\n"
                  "Please connect a xbox controller and re-run the python controller if you want to use manual mode")
            CONTROL[4] = False

        while CONTROL[4] and xbc is not None:
            '''
            Manual Mode:
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
            X button: revert left arm gripper to its home position
            Y button: revert right arm gripper to its home position
            '''

            # Recorder
            if RECORD:
                if RECORDING:
                    recorder.write_raven_status(raven)
                else:
                    recorder.start_recording(RECORD_TO)
                    recorder.write_raven_status(raven, True)
                    RECORDING = True
            elif RECORDING:
                recorder.stop_recording()
                RECORDING = False

            div = 500   # how much the raw input values will be divided by to produce the change in x,y,z
            dead_zone = 0.1  # controller axes must move beyond this before they register as an input, prevents drift

            controller = xbc.read()

            if curr_tm is None:
                curr_tm = [fk.fwd_kinematics_p5(0, np.array(raven.arms[0].get_all_joint_pos(), dtype="float")),
                           fk.fwd_kinematics_p5(1, np.array(raven.arms[1].get_all_joint_pos(), dtype="float"))]

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
                print("Controlling the right arm")

            # Set kinematics mode
            if controller[2][0]:
                ik_mode = True
                print("Using p5 inverse kinematics")
            elif controller[2][1]:
                ik_mode = False
                print("Using standard inverse kinematics")

            # Home left gripper
            if controller[2][2]:
                home_dh[0] = ard.HOME_DH[0]
            # Home right gripper
            if controller[2][3]:
                home_dh[1] = ard.HOME_DH[1]

            # Control both raven arms
            if arm_control[0] and arm_control[1]:
                # modify position using controller inputs
                pos = update_pos_two_arm(dead_zone, controller, div)
                # Plan next move based off of modified cartesian coordinates
                raven.plan_move(0, pos[0][0], pos[1][0], pos[2][0], pos[3][0], ik_mode, home_dh)
                raven.plan_move(1, pos[0][1], pos[1][1], pos[2][1], pos[3][1], ik_mode, home_dh)

            # Control both raven arms with absolute cart pos
            # if arm_control[0] and arm_control[1]:
            #     # modify position using controller inputs
            #     pos = update_pos_two_arm(dead_zone, controller, div)
            #     # Plan next move based off of modified cartesian coordinates
            #
            #     curr_tm[0][0, 3] += pos[0][0]  # left x
            #     curr_tm[0][1, 3] += pos[1][0]  # left y
            #     curr_tm[0][2, 3] += pos[2][0]  # left z
            #
            #     curr_tm[1][0, 3] += pos[0][1]  # right x
            #     curr_tm[1][1, 3] += pos[1][1]  # right y
            #     curr_tm[1][2, 3] += pos[2][1]  # right z
            #
            #     raven.plan_move_abs(0, curr_tm[0], pos[3][0], ik_mode, home_dh)
            #     raven.plan_move_abs(1, curr_tm[1], pos[3][1], ik_mode, home_dh)

            # Control one raven arm
            elif arm_control[0] or arm_control[1]:
                # Decide which arm to control
                arm = 0
                if arm_control[1]:
                    arm = 1
                # modify position using controller inputs
                pos, home_dh = update_pos_one_arm(dead_zone, controller, div, arm, home_dh)
                # Plan new position based off of desired cartesian changes
                raven.plan_move(arm, pos[0][arm], pos[1][arm], pos[2][arm], pos[3][arm], True, home_dh)

            # Incrementally move the simulated raven to the new planned position
            raven.move()
            # rumble the controller when raven is limited
            rumble_if_limited(raven, xbc)

    print("shutting down...\n")
    os.system('kill %d' % os.getpid())
    exit(0)


def get_input(file_valid):
    """
    continuously loops to collect new inputs from user in order to switch
    control modes
    """
    global CONTROL
    global RECORD
    global RECORD_TO
    print("Input Menu:\n")
    print("input 'h' for home")
    print("input 's' for sine dance")
    if file_valid:
        print("input 'f' for motion from file")
        print("input 'r' for motion from file with recording")
    print("input 'm' for manual mode")
    print("input 'q' for quit\n")
    print("Please select a control mode:")
    userinput = input()
    while not CONTROL[2]:
        print("Switching control modes...\n")
        if userinput == 'h':
            CONTROL = control_reset()
            print("homing...")
            CONTROL[0] = True
            userinput = input("Input key to switch control modes\n")
            continue
        elif userinput == 's':
            CONTROL = control_reset()
            print("doing sine dance...")
            CONTROL[1] = True
            userinput = input("Input key to switch control modes\n")
            continue
        elif userinput == 'q':
            CONTROL = control_reset()
            CONTROL[2] = True
        elif userinput == 'f' and file_valid:
            CONTROL = control_reset()
            CONTROL[3] = True
            userinput = input("Input key to switch control modes\n")
        elif userinput == 'r' and file_valid:
            RECORD_TO = input("Please enter filename to record to (example.csv): ")
            CONTROL = control_reset()
            CONTROL[3] = True
            RECORD = True
            userinput = input("Input key to switch control modes\n")
        elif userinput == 'm':
            CONTROL = control_reset()
            CONTROL[4] = True
            userinput = input("Input key to switch control modes or input 'j' to begin recording\n")
        elif userinput == 'j' and CONTROL[4]:
            RECORD_TO = input("Please enter filename to record to (example.csv): ")
            RECORD = True
            userinput = input("Now recording to csv, input 'k' to stop recording\n")
        elif userinput == 'k' and CONTROL[4] and RECORDING:
            RECORD = False
            userinput = input("Recording stopped, input key to switch control modes\n")

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
    if csvData.shape[0] == 0:
        file_valid = False
        print("Raven trajectory file empty or not found.")
    elif csvData.shape[1] != ard.COL_IN_FILE:
        file_valid = False
        print("Raven trajectory file format invalid. ("+str( ard.COL_IN_FILE)+" cols expected)")
    return file_valid, csvData


def main():
    """
    runs the controller by initializing a thread to collect user inputs and
    calling the do() method to move the robot according to what the user inputs
    """
    # load external raven trajectory file
    file_valid, csvData = file_loader()
    # creates raven object
    raven = arav.ambf_raven()
    #create recorder instance
    recorder = arr.ambf_raven_recorder()
    # set raven man_steps
    raven.man_steps = 17
    # creates xbox controller object if there is a controller connected
    try:
        xbc = axc.XboxController()
    except IndexError:
        xbc = None
        print("No xbox controller detected\n"
              "Please connect a xbox controller and re-run the python controller if you want to use manual mode")
    # creates thread
    get_inputs = th.Thread(target=get_input, args=(file_valid, ))
    # starts get_inputs thread
    get_inputs.start()
    do(raven, csvData, xbc, recorder)
    get_inputs.join()


if __name__ == '__main__':
    main()
