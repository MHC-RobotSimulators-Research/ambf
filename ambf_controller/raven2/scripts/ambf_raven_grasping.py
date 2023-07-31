from ambf_client import Client
import rospy
import time
import sys


class ambf_raven_grasping():

    def __init__(self):

        self.grasp = [False, False]

        self.grasper1_L = Client('grasper1L_S_A')
        self.grasper2_L = Client('grasper2L_S_A')
        self.grasper1_R = Client('grasper1R_S_A')
        self.grasper2_R = Client('grasper2R_S_A')
        self.grasper1_L.connect()
        self.grasper2_L.connect()
        self.grasper1_R.connect()
        self.grasper2_R.connect()

        # We have a sensor array (4 individual sensing elements)
        self.grasper1_L_sensor_obj = self.grasper1_L.get_obj_handle('grasper1_L_PS')
        self.grasper2_L_sensor_obj = self.grasper2_L.get_obj_handle('grasper2_L_PS')
        self.grasper1_R_sensor_obj = self.grasper1_R.get_obj_handle('grasper1_R_PS')
        self.grasper2_R_sensor_obj = self.grasper2_R.get_obj_handle('grasper2_R_PS')

        self.sensors = [[self.grasper1_L_sensor_obj, self.grasper2_L_sensor_obj],
                        [self.grasper1_R_sensor_obj, self.grasper2_R_sensor_obj]]


        # We have four corresponding constraint actuators, that we are going to actuate based on the trigger events from the
        # above sensors. Note that there is not explicit relation between a sensor and an actuator, it is we who are
        # connection the two.
        self.actuator_L0 = self.grasper1_L.get_obj_handle('grasper1_L_C0')
        self.actuator_L1 = self.grasper1_L.get_obj_handle('grasper1_L_C1')
        self.actuator_L2 = self.grasper1_L.get_obj_handle('grasper1_L_C2')
        self.actuator_L3 = self.grasper1_L.get_obj_handle('grasper1_L_C3')
        self.actuator_L4 = self.grasper2_L.get_obj_handle('grasper2_L_C0')
        self.actuator_L5 = self.grasper2_L.get_obj_handle('grasper2_L_C1')
        self.actuator_L6 = self.grasper2_L.get_obj_handle('grasper2_L_C2')
        self.actuator_L7 = self.grasper2_L.get_obj_handle('grasper2_L_C3')

        self.actuator_R0 = self.grasper1_R.get_obj_handle('grasper1_R_C0')
        self.actuator_R1 = self.grasper1_R.get_obj_handle('grasper1_R_C1')
        self.actuator_R2 = self.grasper1_R.get_obj_handle('grasper1_R_C2')
        self.actuator_R3 = self.grasper1_R.get_obj_handle('grasper1_R_C3')
        self.actuator_R4 = self.grasper2_R.get_obj_handle('grasper2_R_C0')
        self.actuator_R5 = self.grasper2_R.get_obj_handle('grasper2_R_C1')
        self.actuator_R6 = self.grasper2_R.get_obj_handle('grasper2_R_C2')
        self.actuator_R7 = self.grasper2_R.get_obj_handle('grasper2_R_C3')

        self.actuators = [[self.actuator_L0,
                           self.actuator_L1,
                           self.actuator_L2,
                           self.actuator_L3,
                           self.actuator_L4,
                           self.actuator_L5,
                           self.actuator_L6,
                           self.actuator_L7],
                          [self.actuator_R0,
                           self.actuator_R1,
                           self.actuator_R2,
                           self.actuator_R3,
                           self.actuator_R4,
                           self.actuator_R5,
                           self.actuator_R6,
                           self.actuator_R7]]

        self.actuator_state = [[False, False, False, False, False, False, False, False],
                               [False, False, False, False, False, False, False, False]]

    def set_grasp(self, arm, grasp):
        self.grasp[arm] = grasp

    # def grasp_button_cb(self):
    #     self.grasp = True
    #     print('GRASP REQUESTED')
    #
    # def release_button_cb(self):
    #     self.grasp = False
    #     print('RELEASE REQUESTED')
    #
    # def print_sensors_state(self, sensor_obj):
    #     print('Sensor Array Triggers: [', end=' ')
    #     for i in range(sensor_obj.get_count()):
    #         print(sensor_obj.is_triggered(i), end=' ')
    #
    #     print(']')
    #
    # def grasp_object(self):
    #     sensor_count = self.grasper1_L_sensor_obj.get_count()
    #
    #     if self.grasp:
    #         for i in range(sensor_count):
    #             if self.grasper1_L_sensor_obj.is_triggered(i) and self.actuators_activation_state[i] is False:
    #                 obj_name = self.grasper1_L_sensor_obj.get_sensed_object(i)
    #                 print('Grasping ', obj_name, ' via actuator ', i)
    #                 self.actuators[i].actuate(obj_name)
    #                 self.actuators_activation_state[i] = True
    #     else:
    #         for i in range(sensor_count):
    #             self.actuators[i].deactuate()
    #             if self.actuators_activation_state[i] is True:
    #                 print('Releasing object from actuator ', i)
    #             self.actuators_activation_state[i] = False

    def grasp_object_via_sensor_name(self):
        sensor_count = self.grasper1_L_sensor_obj.get_count()

        if self.grasp:
            for i in range(sensor_count):
                if self.grasper1_L_sensor_obj.is_triggered(i) and self.actuators_activation_state[i] is False:
                    self.actuators[i].actuate_from_sensor_data(self.grasper1_L_sensor_obj.get_identifier())
                    self.actuators_activation_state[i] = True
                    print('Actuating actuator ', i, 'named: ', self.actuators[i].get_name())
        else:
            for i in range(sensor_count):
                self.actuators[i].deactuate()
                if self.actuators_activation_state[i] is True:
                    print('Releasing object from actuator ', i)
                self.actuators_activation_state[i] = False

    def grasp_object(self, arm):

        if self.grasp[arm]:
            for i in range(len(self.sensors[arm])):
                for j in range(self.sensors[arm][i].get_count()):
                    if self.sensors[arm][i].is_triggered(j) and self.actuator_state[arm][(i + 1) * j] is False:
                        self.actuators[arm][(i + 1) * j].actuate_from_sensor_data(self.sensors[arm][i].get_identifier())
                        self.actuator_state[arm][(i + 1) * j] = True
                        print('Actuating actuator ', i, 'named: ', self.actuators[arm][(i + 1) * j].get_name())
        else:
            for i in range(len(self.sensors[arm])):
                for j in range(self.sensors[arm][i].get_count()):
                    self.actuators[arm][(i + 1) * j].deactuate()
                    if self.actuator_state[arm][(i + 1) * j] is True:
                        print('Releasing object from actuator ', i)
                    self.actuator_state[arm][(i + 1) * j] = False

# def main():
#     global grasper1_L_sensor_obj, grasper2_L_sensor_obj, actuators, actuators_activation_state, grasp
#     grasper1_L = Client('grasper1L-sensors-actuators')
#     grasper2_L = Client('grasper2L-sensors-actuators')
#     grasper1_L.connect()
#     grasper2_L.connect()
#
#     # We have a sensor array (4 individual sensing elements)
#     grasper1_L_sensor_obj = grasper1_L.get_obj_handle('grasper1_L_PS')
#     grasper2_L_sensor_obj = grasper1_L.get_obj_handle('grasper2_L_PS')
#
#     # We have four corresponding constraint actuators, that we are going to actuate based on the trigger events from the
#     # above sensors. Note that there is not explicit relation between a sensor and an actuator, it is we who are
#     # connection the two.
#     actuator_L0 = grasper1_L.get_obj_handle('grasper1_L_C0')
#     actuator_L1 = grasper1_L.get_obj_handle('grasper1_L_C1')
#     actuator_L2 = grasper1_L.get_obj_handle('grasper1_L_C2')
#     actuator_L3 = grasper1_L.get_obj_handle('grasper1_L_C3')
#     actuator_L4 = grasper1_L.get_obj_handle('grasper2_L_C0')
#     actuator_L5 = grasper1_L.get_obj_handle('grasper2_L_C1')
#     actuator_L6 = grasper1_L.get_obj_handle('grasper2_L_C2')
#     actuator_L7 = grasper1_L.get_obj_handle('grasper2_L_C3')
#
#     actuators = [actuator_L0, actuator_L1, actuator_L2, actuator_L3, actuator_L4, actuator_L5, actuator_L6, actuator_L7]
#     actuators_activation_state = [False, False, False, False, False, False, False, False]
#     grasp = False

    # time.sleep(1.0)

    # tk = Tk()
    # tk.title("Sensing and Grasping Example")
    # tk.geometry("250x150")
    # grasp_button = Button(
    #     tk, text="Grasp", command=grasp_button_cb, height=3, width=50, bg="red")
    # release_button = Button(
    #     tk, text="Release", command=release_button_cb, height=3, width=50, bg="green")
    # grasp_button.pack()
    # release_button.pack()

    # counter = 0
    # while not rospy.is_shutdown():
    #     try:
    #         tk.update()
    #         if not (counter % 50):
    #             print_sensors_state(sensor_obj)
    #             counter = 0
    #         # grasp_object(grasp)
    #         grasp_object_via_sensor_name(grasp)
    #         time.sleep(0.02)
    #         counter = counter + 1
    #     except KeyboardInterrupt:
    #         print('Exiting Program')
    #         break


def main():
    test = ambf_raven_grasping()
    test.grasp_object(0)

if __name__ == '__main__':
    main()
