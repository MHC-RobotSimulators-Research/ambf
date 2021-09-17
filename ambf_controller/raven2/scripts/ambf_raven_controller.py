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
ambf_raven_controller is a Client for operating the ambf_raven simulated robot
'''


def control_reset():
    new_control = [False, False, False, False, False]
    return new_control

def do(q, raven):
        control = [False, False, False, False, False]
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
            while control[3] and not any(raven.moved):
                #move in x direction:
                # x = [0.0,0.0]
                # y = [0.0,0.0]
                # z = [0.0,0.0]
                # pygame.init()
                # events = pygame.event.get()
                # for event in events:
                #     if event.type == pygame.KEYDOWN:
                #         if event.key == pygame.K_Q:
                #             x[0] += 0.01
                #             raven.manual_move(0, x[0], y[0], z[0], 0)
                #         elif event.key == pygame.K_I:
                #             x[1] += 0.01
                #             raven.manual_move(1, x[1], y[1], z[1], 0)
                #         elif event.key == pygame.K_A:
                #             x[0] -= 0.01
                #             raven.manual_move(0, x[0], y[0], z[0], 0)
                #         elif event.key == pygame.K_J:
                #             x[1] -= 0.01
                #             raven.manual_move(1, x[1], y[1], z[1], 0)
                #         elif event.key == pygame.K_W:
                #             y[0] += 0.01
                #             raven.manual_move(0, x[0], y[0], z[0], 0)
                #         elif event.key == pygame.K_O:
                #             y[1] += 0.01
                #             raven.manual_move(1, x[1], y[1], z[1], 0)
                #         elif event.key == pygame.K_S:
                #             y[0] -= 0.01
                #             raven.manual_move(0, x[0], y[0], z[0], 0)
                #         elif event.key == pygame.K_K:
                #             y[1] -= 0.01
                #             raven.manual_move(1, x[1], y[1], z[1], 0)
                #         elif event.key == pygame.K_E:
                #             z[0] += 0.01
                #             raven.manual_move(0, x[0], y[0], z[0], 0)
                #         elif event.key == pygame.K_P:
                #             z[1] += 0.01
                #             raven.manual_move(1, x[1], y[1], z[1], 0)
                #         elif event.key == pygame.K_D:
                #             z[0] -= 0.01
                #             raven.manual_move(0, x[0], y[0], z[0], 0)
                #         elif event.key == pygame.K_L:
                #             z[1] -= 0.01
                #             raven.manual_move(1, x[1], y[1], z[1], 0)
                #     raven.control_move()
                x = [0.11,0.0]
                y = [0.5, 0.0]
                z = [0.0,1.0]
                raven.manual_move(0, x[0], y[0], z[0], 0)
                raven.manual_move(1, x[1], y[1], z[1], 0)
                # print(int(raven.loop_rate * np.max([x[0], x[1], y[0], y[1], z[0], z[1]])))
                for i in range(raven.loop_rate):
                    if not i:
                        raven.moved[0] = raven.move(1, 1, i)
                        raven.moved[1] = raven.move(1, 0, i)
                    else:
                        raven.moved[0] = raven.move(0, 1, i)
                        raven.moved[1] = raven.move(0, 0, i)
                    time.sleep(0.01)
                if raven.moved[0] and raven.moved[1]:
                    print("Raven has moved!")
                if not q.empty():
                    control = q.get()
        while control[4] and not raven.finished:
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
    control = [False, False, False, False]
    sys.stdin = stdin
    print("Input Menu:\n")
    print("input 'h' for home\n")
    print("input 's' for sine dance\n")
    print("input 'm' for manual mode\n")
    print("input 'f' for file mode\n")
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
        elif userinput == 'm':
            print("entering manual control mode...")
            control[3] = True
            q.put(control)
            print("Welcome to the Raven2 AMBF Manual Control Mode")
            userinput = raw_input("Input key to switch control modes\n")
            continue
        elif userinput == 'f':
            control[4] = True
            q.put(control)
            userinput = raw_input("Input key to switch control modes\n")
            continue
        elif userinput == 'q':
            control[2] = True
            q.put(control)




def main():
    raven = arav.ambf_raven()#creates simulated raven
    q = multiprocessing.Queue()#creates queue to commincate between processes
    newstdin = os.fdopen(os.dup(sys.stdin.fileno()))#for accessing standard input from the other process
    p1 = multiprocessing.Process(target = get_input, args = (q, newstdin))#starts process
    p1.start()
    do(q, raven)
    p1.join()


if __name__ == '__main__':
    main()
