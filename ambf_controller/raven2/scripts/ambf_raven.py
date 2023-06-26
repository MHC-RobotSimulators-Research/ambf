import time
from ambf_client import Client
import math
import raven_ik as ik
import raven_fk as fk
import utilities as u
import numpy as np
import ambf_raven_def as ard
import timeit

'''
author: Sean Fabrega
ambf_raven defines methods for an ambf_raven robot, including homnig, sine dance and hoping soon
cube tracing and soft body manipulation'''


class ambf_raven:
    def __init__(self):
        self._client = Client()
        self._client.connect()
        input("We can see what objects the client has found. Press Enter to continue...")
        print(self._client.get_obj_names())
        self.homed = [False, False]
        self.moved = [False, False]
        self.limited = [False, False]
        self.arms = [self._client.get_obj_handle('raven_2/base_link_L'),
                     self._client.get_obj_handle('raven_2/base_link_R')]
        self.links = []
        for i in range(2):
            link_names = self.arms[i].get_children_names()
            for j in range(self.arms[i].get_num_of_children()):
                self.links.append(self._client.get_obj_handle('raven_2/' + link_names[j]))
        self.start_jp = np.zeros((2, 7))  # indexed at 0
        self.delta_jp = np.zeros((2, 7))
        self.home_joints = ard.HOME_JOINTS
        self.next_jp = np.zeros((2, 7))
        self.dance_scale_joints = ard.DANCE_SCALE_JOINTS
        self.loop_rate = ard.LOOP_RATE
        self.raven_joints = ard.RAVEN_JOINTS
        self.rc = [0, 0]
        self.rampup_count = np.array(self.rc)
        self.i = 0
        self.speed = 10.00 / self.loop_rate
        self.rampup_speed = 0.5 / self.loop_rate
        self.finished = False
        self.man_steps = 30

    def go_home(self, first_entry, arm, count):
        '''
        first entry --> bool
        arm --> bool (0 for left, 1 for right)
        count --> int
        '''
        # if first time calling go home
        if (first_entry):
            for i in range(self.arms[arm].get_num_joints()):
                self.start_jp[arm][i] = self.arms[arm].get_joint_pos(i)
                self.delta_jp[arm][i] = self.home_joints[i] - self.arms[arm].get_joint_pos(i)
        # gradualizes movement from a to b
        scale = min(1.0 * count / self.loop_rate, 1.0)
        # array containing distance to go to start point
        diff_jp = [0, 0, 0, 0, 0, 0, 0]

        # sets position for each joint
        for i in range(self.arms[arm].get_num_joints()):
            self.arms[arm].set_joint_pos(i, scale * self.delta_jp[arm][i] + self.start_jp[arm][i])
            diff_jp[i] = abs(self.home_joints[i] - self.arms[arm].get_joint_pos(i))
        # in progress, indicates when arm is honed

        max_value = np.max(diff_jp)

        if (max_value < 0.1):
            self.homed[arm] = True

        else:
            self.homed[arm] = False
        return self.homed[arm]

    def sine_dance(self, first_entry, arm, count, rampup_count):
        self.homed[arm] = False
        for i in range(self.raven_joints):
            offset = (i + arm) * math.pi / 2
            rampup = min(self.rampup_speed * self.rampup_count[arm], 1.0)
            self.arms[arm].set_joint_pos(i,
                                         rampup * self.dance_scale_joints[i] * math.sin(self.speed * (count + offset)) +
                                         self.home_joints[i])
            self.rampup_count[arm] += 1

    def get_t_command(self):
        return (self.arms[0].get_torque_command(), self.arms[1].get_torque_command)

    def get_raven_status(self, time_now, first_entry=False):
        if first_entry:
            status = ["time"]
            for arm in range(2):
                for j in range(7):
                    status.append("jpos" + str(arm * 7 + j))
                for j in range(7):
                    status.append("jvel" + str(arm * 7 + j))
                link_names = self.arms[arm].get_children_names()
                for pre in ["_pos", "_apos", "_vel", "_avel"]:
                    for j in range(self.arms[arm].get_num_of_children()):
                        status.append(link_names[j] + pre + "x")
                        status.append(link_names[j] + pre + "y")
                        status.append(link_names[j] + pre + "z")
            return status
        else:
            status = [time_now]
            for arm in range(2):
                status.extend(self.arms[arm].get_all_joint_pos())  # 7 numbers
                status.extend(self.arms[arm].get_all_joint_vel())  # 7 numbers
                # status.extend(self.arms[arm].get_all_joint_effort()) # 7 numbers

                link_names = self.arms[arm].get_children_names()
                for j in range(self.arms[arm].get_num_of_children()):
                    curr_link = self._client.get_obj_handle('raven_2/' + link_names[j])
                    status.extend(curr_link.get_pose())  # 6 numbers
                    vel = curr_link.get_linear_vel()
                    avel = curr_link.get_angular_vel()
                    status.extend([vel.x, vel.y, vel.z, avel.x, avel.y, avel.z])  # 6 numbers
                # frc = self.arms[arm].get_force_command()
                # trq = self.arms[arm].get_torque_command()
                # status.extend([frc.x, frc.y, frc.z, trq.x, trq.y, trq.z])    # 6 numbers
            return status

    def set_raven_pos(self, pos_list):
        '''
        sets raven position based on array containing positions from the physical
        raven robot. offsets are approximate and need finalization. indexing is intuitive
        '''
        for i in range(len(pos_list)):
            if i == 0:
                self.arms[0].set_joint_pos(i, np.deg2rad(pos_list[i]) + (math.pi / 6))
            elif i == 1:
                self.arms[0].set_joint_pos(i, np.deg2rad(pos_list[i]) + (math.pi / 10))
            elif i == 2:
                self.arms[0].set_joint_pos(i, pos_list[i] / 100 - 0.26)
            elif i == 4:
                self.arms[0].set_joint_pos(i - 1, np.deg2rad(pos_list[i]) + (math.pi * 3) / 4)
            elif i == 5:
                self.arms[0].set_joint_pos(i - 1, np.deg2rad(pos_list[i]))
            elif i == 6:
                self.arms[0].set_joint_pos(i - 1, np.deg2rad(pos_list[i]) - math.pi / 12)
            elif i == 7:
                self.arms[0].set_joint_pos(i - 1, np.deg2rad(pos_list[i]) - math.pi / 12)
            elif i == 8:
                self.arms[1].set_joint_pos(i - 8, np.deg2rad(pos_list[i]) + math.pi / 6)
            elif i == 9:
                self.arms[1].set_joint_pos(i - 8, np.deg2rad(pos_list[i]) + math.pi / 10)
            elif i == 10:
                self.arms[1].set_joint_pos(i - 8, pos_list[i] / 100 - 0.26)
            elif i == 12:
                self.arms[1].set_joint_pos(i - 9, np.deg2rad(pos_list[i]) + (math.pi * 3) / 4)
            elif i == 13:
                self.arms[1].set_joint_pos(i - 9, np.deg2rad(pos_list[i]))
            elif i == 14:
                self.arms[1].set_joint_pos(i - 9, np.deg2rad(pos_list[i]) - math.pi / 12)
            elif i == 15:
                self.arms[1].set_joint_pos(i - 9, np.deg2rad(pos_list[i]) - math.pi / 12)

    def set_raven_force(self, pos_list):
        '''
        a prototype for a similar method except using force instead of joint position
        '''
        scale = 1
        for i in range(len(pos_list)):
            if i == 0:
                self.arms[0].set_joint_effort(i, (np.deg2rad(pos_list[i]) + (math.pi / 6)) / scale)
            elif i == 1:
                self.arms[0].set_joint_effort(i, (np.deg2rad(pos_list[i]) + (math.pi / 10)) / scale)
            elif i == 2:
                self.arms[0].set_joint_effort(i, (pos_list[i] / 100 - 0.26) / scale)
            elif i == 4:
                self.arms[0].set_joint_effort(i - 1, (np.deg2rad(pos_list[i]) + (math.pi * 3) / 4) / scale)
            elif i == 5:
                self.arms[0].set_joint_effort(i - 1, (np.deg2rad(pos_list[i])) / scale)
            elif i == 6:
                self.arms[0].set_joint_effort(i - 1, (np.deg2rad(pos_list[i]) - math.pi / 12) / scale)
            elif i == 7:
                self.arms[0].set_joint_effort(i - 1, (np.deg2rad(pos_list[i]) - math.pi / 12) / scale)
            elif i == 8:
                self.arms[1].set_joint_effort(i - 8, (np.deg2rad(pos_list[i]) + math.pi / 6) / scale)
            elif i == 9:
                self.arms[1].set_joint_effort(i - 8, (np.deg2rad(pos_list[i]) + math.pi / 10) / scale)
            elif i == 10:
                self.arms[1].set_joint_effort(i - 8, (pos_list[i] / 100 - 0.26) / scale)
            elif i == 12:
                self.arms[1].set_joint_effort(i - 9, (np.deg2rad(pos_list[i]) + (math.pi * 3) / 4) / scale)
            elif i == 13:
                self.arms[1].set_joint_effort(i - 9, (np.deg2rad(pos_list[i])) / scale)
            elif i == 14:
                self.arms[1].set_joint_effort(i - 9, (np.deg2rad(pos_list[i]) - math.pi / 12) / scale)
            elif i == 15:
                self.arms[1].set_joint_effort(i - 9, (np.deg2rad(pos_list[i]) - math.pi / 12) / scale)

    def plan_move(self, arm, x, y, z, gangle, p5=False, home_dh=ard.HOME_DH):
        """
        moves the desired robot arm based on inputted changes to cartesian coordinates
        Args:
            arm (int) : 0 for the left arm and 1 for the right arm
            x (float) : the desired change to the x coordinate
            y (float) : the desired change to the y coordinate
            z (float) : the desired change to the z coordinate
            gangle (float) : the gripper angle, 0 is closed
            p5 (bool) : when false uses standard kinematics, when true uses p5 kinematics
            home_dh (array) : array containing home position, or desired postion of the
                joints not set by cartesian coordinates in inv_kinematics_p5
        """
        curr_jp = np.array(self.arms[arm].get_all_joint_pos(), dtype="float")
        if p5:
            curr_tm = fk.fwd_kinematics_p5(arm, curr_jp)
        else:
            curr_tm = fk.fwd_kinematics(arm, curr_jp)
        curr_tm[0, 3] += x
        curr_tm[1, 3] += y
        curr_tm[2, 3] += z
        if p5:
            jpl = ik.inv_kinematics_p5(arm, curr_tm, gangle, home_dh)
        else:
            jpl = ik.inv_kinematics(arm, curr_tm, gangle)
        self.limited[arm] = jpl[1]
        if self.limited[arm]:
            print("Desired cartesian position is out of bounds for Raven2. Will move to max pos.")
        new_jp = jpl[0]
        self.next_jp[arm] = new_jp

        # print("fk ", timeit.timeit(lambda: fk.fwd_kinematics_p5(arm, curr_jp), setup="pass",number=1))
        # print("ik ", timeit.timeit(lambda: ik.inv_kinematics_p5(arm, curr_tm, gangle), setup="pass", number=1))


    def move(self, first_entry, arm, count):
        '''
        uses the helper function manual move to slowly increment the robot's position until
        it reaches the desired inputted position. uses a similar method structure to home
        '''

        if (first_entry):
            for i in range(self.raven_joints):
                self.start_jp[arm][i] = self.arms[arm].get_joint_pos(i)
                self.delta_jp[arm][i] = self.next_jp[arm][i] - self.arms[arm].get_joint_pos(i)
        # gradualizes movement from a to b
        scale = min(1.0 * count / self.man_steps, 1.0)
        # array containing distance to go to start point
        diff_jp = [0, 0, 0, 0, 0, 0, 0]

        # sets position for each joint
        for i in range(self.arms[arm].get_num_joints()):
            self.arms[arm].set_joint_pos(i, scale * self.delta_jp[arm][i] + self.start_jp[arm][i])
            diff_jp[i] = abs(self.next_jp[arm][i] - self.arms[arm].get_joint_pos(i))
        # in progress, indicates when arm is honed

        max_value = np.max(diff_jp)

        if (max_value < 0.1 or self.limited[arm]):
            self.moved[arm] = True

        else:
            self.moved[arm] = False
        return self.moved[arm]

    def move_now(self, arm):
        """
        Moves robot to position generated by the move function
        Args:
            arm (int): 0 for the left arm and 1 for the right arm
        """
        # array containing the difference between next_jp and current joint position
        diff_jp = [0, 0, 0, 0, 0, 0, 0]

        for i in range(self.arms[arm].get_num_joints()):
            self.arms[arm].set_joint_pos(i, self.next_jp[arm][i])
            diff_jp[i] = abs(self.next_jp[arm][i] - self.arms[arm].get_joint_pos(i))

        max_value = np.max(diff_jp)

        if (max_value < 0.1 or self.limited[arm]):
            self.moved[arm] = True

        else:
            self.moved[arm] = False