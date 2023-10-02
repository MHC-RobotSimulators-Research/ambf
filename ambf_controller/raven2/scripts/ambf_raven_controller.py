import threading as th
import sys
import tty
import termios
import os
import time
import math as m
import numpy as np
import ambf_raven as arav
import csv
import ambf_raven_def as ard
import ambf_xbox_controller as axc
import ambf_xbox_controller_fake as axf
import ambf_raven_recorder as arr
import ambf_raven_reader as arc
import ambf_raven_grasping as arg

'''
authors: Natalie Chalfant, Sean Fabrega
ambf_raven_controller is a Client for operating the ambf_raven simulated robot, specifically designed for
using manual move mode to control the simulated raven II using an xbox controller and interact with the
simulated environment
'''

sys.path.insert(0, 'ambf/ambf_ros_modules/ambf_client/python/ambf_client')
FILE_DESCRIPTORS = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin)

CONTROL = [True, False, False, False, False, False]  # Defines control mode in do
'''
control[0] = Running
control[1] = Homing Mode
control[2] = Manual Control
control[3] = File Mode: controller inputs
control[4] = File Mode: jpos
control[5] = Sine Dance
'''
DEADZONE = 0.1  # controller axes must move beyond this before they register as an input, prevents drift
DIV = 500  # amount raw input values are divided by to produce motion
ALLOW_FAKE_CONTROLLER = True
RECORD = False
RECORDING = False
FILE_OUT = ""
FILE_IN = ""


def control_reset():
    """
    resets all control values
    """
    global CONTROL
    CONTROL = [True, False, False, False, False, False]


def update_pos_two_arm(controller):
    """
    Generates position offsets for both arms using a xbox controller as input
    Args:
        controller : an array of controller inputs
    """

    global DEADZONE
    global DIV

    delta_tm = [np.matrix([[0, 0, 0, 0],
                          [0, 0, 0, 0],
                          [0, 0, 0, 0],
                          [0, 0, 0, 0]], dtype=float),
                np.matrix([[0, 0, 0, 0],
                           [0, 0, 0, 0],
                           [0, 0, 0, 0],
                           [0, 0, 0, 0]], dtype=float)]

    gangle = [0, 0]

    # Update coordinates for both arms
    for arm in range(2):
        if controller[arm][3] == 1 and DEADZONE < abs(controller[arm][1]):
            delta_tm[arm][2, 3] = -controller[arm][1] / DIV
        else:
            # note x and y are swapped to make controls more intuitive
            if DEADZONE < abs(controller[arm][0]):
                delta_tm[arm][1, 3] = -controller[arm][0] / DIV
            if DEADZONE < abs(controller[arm][1]):
                delta_tm[arm][0, 3] = -controller[arm][1] / DIV

    # Set gripper angles
    gangle[0] = 1 - (controller[0][2] / 4)
    # for the right arm gangle needs to be negative, this is to fix a bug somewhere else that I can't find
    gangle[1] = -1 + (controller[1][2] / 4)

    return delta_tm, gangle


def update_pos_one_arm(controller, arm, curr_dh):
    """
    Generates position and grasper jpos offsets for one arm using a xbox controller as input
    Args:
        controller : an array of controller inputs
        arm : which arm is being controlled
        curr_dh : the current dh values for the raven arms
    """

    global DEADZONE
    global DIV

    delta_tm = [np.matrix([[0, 0, 0, 0],
                          [0, 0, 0, 0],
                          [0, 0, 0, 0],
                          [0, 0, 0, 0]], dtype=float),
                np.matrix([[0, 0, 0, 0],
                           [0, 0, 0, 0],
                           [0, 0, 0, 0],
                           [0, 0, 0, 0]], dtype=float)]

    gangle = [0, 0]

    # Cartesian control of desired arm
    if controller[0][3] == 1 and DEADZONE < abs(controller[0][1]):
        pos[2][arm] = -controller[0][1] / DIV
    else:
        if DEADZONE < abs(controller[0][0]):
            pos[1][arm] = -controller[0][0] / DIV
        if DEADZONE < abs(controller[0][1]):
            pos[0][arm] = -controller[0][1] / DIV

    # Left arm
    if not arm:
        # Set left gripper angle
        pos[3][0] = 1 - (controller[1][2] / 4)

        # Set right j4
        if DEADZONE < abs(controller[1][0]):
            if abs(curr_dh[0][3] - controller[1][0] / 10) < m.pi:
                curr_dh[0][3] += -controller[1][0] / 10
        # Position j5
        if DEADZONE < abs(controller[1][1]):
            if abs(curr_dh[0][4] - controller[1][1] / 10) < 2:
                curr_dh[0][4] += -controller[1][1] / 10

    # Right arm
    else:
        # Set right gripper angle
        pos[3][1] = -1 + (controller[1][2] / 4)

        # Set right j4
        if DEADZONE < abs(controller[1][0]):
            if abs(curr_dh[1][3] + controller[1][0] / 10) < m.pi:
                curr_dh[1][3] += controller[1][0] / 10
        # Position j5
        if DEADZONE < abs(controller[1][1]):
            if abs(curr_dh[1][4] + controller[1][1] / 10) < 2:
                curr_dh[1][4] += controller[1][1] / 10

    return pos, curr_dh


def rumble_if_limited(raven, xbc):
    """
    Sends rumble command to controller when the commands ask raven to go out of bounds
    (this currently does not work and causes errors after rumbling for a few seconds)
    Args:
        xbc : an ambf_xbox_controller instance
    """
    rumble = [0.0, 0.0]
    for i in range(2):
        if raven.limited[i]:
            rumble[i] = 1
    if rumble[0] != 0.0 or rumble[1] != 0.0:
        xbc.rumble(rumble[0], rumble[1], 100)


def do(raven, xbc, grasper, recorder=None, reader=None):
    """
    performs the main actions of the robot based on the values
    in the control array

    Args:
        raven : an ambf_raven instance
        xbc : an ambf_xbox_controller instance
        grasper : an ambf_raven_grasping instance
    """
    global CONTROL
    global RECORD
    global RECORDING
    global FILE_OUT

    # Sets which mode will be used in manual control
    arm_control = [True, True]
    # True for p5 ik and false for standard ik
    ik_mode = False
    # Current DH values for each arm, defaults to home position
    curr_dh = np.array([[1.04719755, 1.88495559, -0.03, 2.35619449 - m.pi / 2, 0., 0., 0.52359878],
                        [1.04719755, 1.88495559, -0.03, 2.35619449 - m.pi / 2, 0., -0., 0.52359878]],
                       dtype="float")

    while CONTROL[0]:

        while CONTROL[1]:  # if after homing, code breaks, needs assistance
            '''
            Homing Mode:
            '''
            raven.home_fast()
            control_reset()

        if CONTROL[2] and xbc is None:
            print("No xbox controller detected\n"
                  "Please connect a xbox controller and re-run the python controller if you want to use manual mode")
            CONTROL[4] = False

        while CONTROL[2] and xbc is not None:
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
            # Use recorded controller inputs or realtime controller inputs
            if CONTROL[3]:
                if reader.get_status():
                    controller = reader.read_ci()
                    # If that was the last line of the CSV
                    if not reader.get_status():
                        control_reset()
                        print("Reached the end of ", FILE_IN)
                        if RECORDING:
                            recorder.stop_recording(FILE_OUT)
                            RECORD = False
                            RECORDING = False
                        break
                else:
                    if reader.load_csv(FILE_IN, "controller"):
                        time.sleep(1)
                        controller = reader.read_ci()
                    else:
                        print(FILE_IN, " did not match expected shape")
                        control_reset()
                        break
            else:
                controller = xbc.read()

            # catch the occasional error where controller is None
            if controller is None:
                print("Something went wrong, please try again")
                control_reset()
                break

            # Record controller inputs and jpos
            if RECORD:
                if RECORDING:
                    recorder.write_raven_status(raven)
                    recorder.write_controller_inputs(controller)

                else:
                    recorder.record_raven_status()
                    recorder.record_controller_inputs()
                    recorder.write_raven_status(raven)
                    recorder.write_controller_inputs(controller)
                    RECORDING = True
            elif RECORDING:
                recorder.stop_recording(FILE_OUT)
                RECORDING = False

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
                curr_dh[0] = ard.HOME_DH[0]
            # Home right gripper
            if controller[2][3]:
                curr_dh[1] = ard.HOME_DH[1]

            # Control both raven arms
            if arm_control[0] and arm_control[1]:
                # modify position using controller inputs
                delta_tm, gangle = update_pos_two_arm(controller)
                # Plan next move based off of modified cartesian coordinates
                raven.plan_move_abs(0, delta_tm[0], gangle[0], ik_mode, curr_dh)
                raven.plan_move_abs(1, delta_tm[1], gangle[1], ik_mode, curr_dh)

                try:
                    for i in range(2):
                        grasper.set_grasp(i, controller[i][2])
                        grasper.grasp_object(i)
                except AttributeError:
                    pass

            # Control one raven arm
            elif arm_control[0] or arm_control[1]:
                # Decide which arm to control
                arm = 0
                if arm_control[1]:
                    arm = 1
                # modify position using controller inputs
                pos, curr_dh = update_pos_one_arm(controller,arm, curr_dh)
                # Plan new position based off of desired cartesian changes
                raven.plan_move(arm, pos[0][arm], pos[1][arm], pos[2][arm], pos[3][arm], True, curr_dh)

                try:
                    grasper.set_grasp(arm, controller[1][2])
                    grasper.grasp_object(arm)
                except AttributeError:
                    pass

            # Incrementally move the simulated raven to the new planned position
            raven.move()
            # rumble the controller when raven is limited
            rumble_if_limited(raven, xbc)

        while CONTROL[3] and not CONTROL[2]:
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
                recorder.start_recording(FILE_OUT)
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

        while CONTROL[4]:
            '''
            Sine Dance:
            '''
            raven.sine_dance()

    print("shutting down...\n")
    os.system('kill %d' % os.getpid())
    exit(0)


def print_menu():
    print("\nPlease select a mode:\n"
          "1. Homing Mode: return raven to its home position\n"
          "2. Manual Control: use an xbox controller to control raven\n"
          "3. File Controller Inputs: motion from a csv containing recorded controller inputs\n"
          "4. File jpos: motion from a csv containing recorded jpos\n"
          "5. Sine Dance: do a little dance :)\n"
          "0. Quit\n")


def set_file_out():

    global FILE_OUT
    global FILE_DESCRIPTORS

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, FILE_DESCRIPTORS)
    FILE_OUT = input("Please enter a filename to record to, ex: 'test'\n")
    tty.setcbreak(sys.stdin)


def set_file_in():
    global FILE_IN
    global FILE_DESCRIPTORS

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, FILE_DESCRIPTORS)
    FILE_IN = input("Please enter the file you like to load\n")
    tty.setcbreak(sys.stdin)


def check_record():
    global RECORD
    global FILE_OUT
    print("Would you like to record y/n?\n")
    while True:
        userinput = sys.stdin.read(1)[0]
        termios.tcflush(sys.stdin, termios.TCIOFLUSH)
        if userinput == 'y':
            set_file_out()
            RECORD = True
            print("Recording started, press 's' to stop")
            break
        elif userinput == 'n':
            RECORD = False
            print("Not Recording")
            break


def _get_input():
    """
    continuously loops to collect new inputs from user in order to switch
    control modes
    """
    global CONTROL
    global RECORD
    global FILE_OUT
    global FILE_IN

    print_menu()

    while CONTROL[0]:

        # get last key pressed
        userinput = sys.stdin.read(1)[0]
        termios.tcflush(sys.stdin, termios.TCIOFLUSH)

        # select mode
        if userinput == '0':
            control_reset()
            CONTROL[0] = False
            print("Quitting...")
            continue
        if userinput == '1':
            control_reset()
            CONTROL[1] = True
            print("Homing Mode selected\n"
                  "Press a key to switch modes or press 'm' to show the menu\n")
            continue
        elif userinput == '2':
            control_reset()
            CONTROL[2] = True
            print("Manual Control selected\n"
                  "Press 'r' to begin recording, press a key to switch modes, or press 'm' to show the menu\n")
            continue
        elif userinput == '3':
            control_reset()
            print("File Controller Inputs selected\n")
            set_file_in()
            check_record()
            CONTROL[2] = True
            CONTROL[3] = True
            continue
        elif userinput == '4':
            control_reset()
            print("File jpos selected\n")
            set_file_in()
            check_record()
            CONTROL[4] = True
            continue
        elif userinput == '5':
            control_reset()
            CONTROL[5] = True
            print("Sine Dance selected\n"
                  "Press a key to switch modes or press 'm' to show the menu\n")
            continue
        elif userinput == 'm':
            print_menu()
            continue

        # control recording
        elif userinput == 'r' and CONTROL[2]:
            set_file_out()
            RECORD = True
            print("Recording started, press 's' to stop")
        elif userinput == 's' and CONTROL[2]:
            RECORD = False
            print("Recording stopped\n"
                  "Press a key to switch modes or press 'm' to show the menu")


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

    global ALLOW_FAKE_CONTROLLER

    # load external raven trajectory file
    file_valid, csvData = file_loader()
    # creates raven object
    raven = arav.ambf_raven()
    # create recorder instance
    recorder = arr.ambf_raven_recorder()
    # create reader instance
    reader = arc.ambf_raven_reader()
    # create grasper instance
    grasper = arg.ambf_raven_grasping()

    # set raven man_steps
    raven.man_steps = 17

    # creates xbox controller object if there is a controller connected
    try:
        xbc = axc.XboxController()
    except IndexError:
        if ALLOW_FAKE_CONTROLLER:
            xbc = axf.ambf_xbox_controller_fake()
        else:
            xbc = None
            print("No xbox controller detected\n"
                  "Please connect a xbox controller and re-run the python controller if you want to use manual mode")

    # creates inputs thread
    get_inputs = th.Thread(target=_get_input, args=(), daemon=True)
    # starts get_inputs thread
    get_inputs.start()
    do(raven, xbc, grasper, recorder, reader)
    get_inputs.join()


if __name__ == '__main__':
    main()
