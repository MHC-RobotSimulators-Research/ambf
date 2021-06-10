import numpy as np
import utilities as u
import kinematics as k
import ambf_raven_def as ard

def compute_FK(joint_pos, arm):
    j = [0, 0, 0, 0, 0, 0, 0, 0]
    for i in range(len(joint_pos)):
        j[i] = joint_pos[i]

    #Raven2 DH Parameters
    joint0 = k.DH(ard.raven_dh_alpha[arm][0],
                 ard.raven_dh_a[arm][0],
                 ard.raven_dh_theta[arm][0],
                 ard.raven_dh_d[arm][0],
                 joint_type = 'R')
    joint1 = k.DH(ard.raven_dh_alpha[arm][1],
                 ard.raven_dh_a[arm][1],
                 ard.raven_dh_theta[arm][1],
                 ard.raven_dh_d[arm][1],
                 joint_type = 'R')
    joint2 = k.DH(ard.raven_dh_alpha[arm][2],
                 ard.raven_dh_a[arm][2],
                 ard.raven_dh_theta[arm][2],
                 ard.raven_dh_d[arm][2],
                 joint_type = 'P')
    joint3 = k.DH(ard.raven_dh_alpha[arm][3],
                 ard.raven_dh_a[arm][3],
                 ard.raven_dh_theta[arm][3],
                 ard.raven_dh_d[arm][3],
                 joint_type = 'R')
    joint4 = k.DH(ard.raven_dh_alpha[arm][4],
                 ard.raven_dh_a[arm][4],
                 ard.raven_dh_theta[arm][4],
                 ard.raven_dh_d[arm][4],
                 joint_type = 'R')
    joint5 = k.DH(ard.raven_dh_alpha[arm][5],
                 ard.raven_dh_a[arm][5],
                 ard.raven_dh_theta[arm][5],
                 ard.raven_dh_d[arm][5],
                 joint_type = 'R')
    joint6 = k.DH(ard.raven_dh_alpha[arm][6],
                 ard.raven_dh_a[arm][6],
                 ard.raven_dh_theta[arm][6],
                 ard.raven_dh_d[arm][6],
                 joint_type = 'R')
    T_1_0 = joint0.get_trans()
    T_2_1 = joint1.get_trans()
    T_3_2 = joint2.get_trans()
    T_4_3 = joint3.get_trans()
    T_5_4 = joint4.get_trans()
    T_6_5 = joint5.get_trans()

    T_2_0 = np.matmul(T_1_0, T_2_1)
    T_3_0 = np.matmul(T_2_0, T_3_2)
    T_4_0 = np.matmul(T_3_0, T_4_3)
    T_5_0 = np.matmul(T_4_0, T_5_4)
    T_6_0 = np.matmul(T_5_0, T_6_5)


    return ard.raven_T_CB * ard.raven_T_B0[arm] * T_6_0
