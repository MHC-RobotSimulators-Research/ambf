from ambf_raven_def import *
import math as m
import numpy as np

def joint_to_dhvalue(joint, arm):
    success = False
    dhvalue = np.zeros(7, dtype = 'float')
    if arm < 0 or arm >= RAVEN_ARMS or joint.size != RAVEN_JOINTS:
        return success
    for i in range(RAVEN_JOINTS):
        if i != 2:
            if i == 5:
                if arm == 0:
                    dhvalue[i] =  (joint[i] - joint[i+1])
                else:
                    dhvalue[i] = -(joint[i] - joint[i+1])
            else:
                dhvalue[i] = joint[i]
                while dhvalue[i] > m.pi:
                    dhvalue[i] -= 2*m.pi
                while dhvalue[i] < -m.pi:
                    dhvalue[i] += 2*m.pi
        else:
            dhvalue[i] = joint[i]
    success = True

    return success, dhvalue

def fwd_trans(a, b, dh_alpha, dh_theta, dh_a, dh_d):
    if ((b <= a) or b == 0):
        ROS_ERROR("Invalid start/end indices")

    xx = float(m.cos(dh_theta[a]))
    xy = float(-m.sin(dh_theta[a]))
    xz = float(0)

    yx = float(m.sin(dh_theta[a])) * float(m.cos(dh_alpha[a]))
    yy = float(m.cos(dh_theta[a])) * float(m.cos(dh_alpha[a]))
    yz = float(-m.sin(dh_alpha[a]))

    zx = float(m.sin(dh_theta[a])) * float(m.sin(dh_alpha[a]))
    zy = float(m.cos(dh_theta[a])) * float(m.sin(dh_alpha[a]))
    zz = float(m.cos(dh_alpha[a]))

    px = float(dh_a[a])
    py = float(-m.sin(dh_alpha[a])) * float(dh_d[a])
    pz = float(m.cos(dh_alpha[a])) * float(dh_d[a])

    xf = np.matrix([[xx, xy, xz, px],
                    [yx, yy, yz, py],
                    [zx, zy, zz, pz],
                    [0, 0, 0, 1]])

    if b > a + 1:
        xf = np.matmul(xf, fwd_trans(a + 1, b, dh_alpha, dh_theta, dh_a, dh_d))

    return xf

def fwd_kinematics(arm, input_joint_pos):
    success = False

    dh_alpha = np.zeros(7, dtype = 'float')
    dh_theta = np.zeros(7, dtype = 'float')
    dh_a = np.zeros(7, dtype = 'float')
    dh_d = np.zeros(7, dtype = 'float')

    j2d = joint_to_dhvalue(input_joint_pos, arm)
    worked = j2d[0]
    jp_dh = j2d[1]

    if worked == False:
        ROS_ERROR("Something went wrong with joint to dh conversion")
        return success
    for i in range(RAVEN_JOINTS):
        if i == 2:
            dh_d[i] = jp_dh[i]
            dh_theta[i] = RAVEN_DH_THETA[arm][i]
        else:
            dh_d[i] = RAVEN_DH_D[arm][i]
            dh_theta[i] = jp_dh[i]
        dh_alpha[i] = RAVEN_DH_ALPHA[arm][i]
        dh_a[i] = RAVEN_DH_A[arm][i]

    print(dh_alpha)

    output_transformation = np.matmul(np.matmul(RAVEN_T_CB, RAVEN_T_B0[arm]), fwd_trans(0, 6, dh_alpha, dh_theta, dh_a, dh_d))


    return output_transformation

def fwd_kinematics_p5(arm, input_joint_pos):
    """
    Gets the position of joint 5 origin
    """
    success = False

    dh_alpha = np.zeros(7, dtype = 'float')
    dh_theta = np.zeros(7, dtype = 'float')
    dh_a = np.zeros(7, dtype = 'float')
    dh_d = np.zeros(7, dtype = 'float')

    j2d = joint_to_dhvalue(input_joint_pos, arm)
    worked = j2d[0]
    jp_dh = j2d[1]

    if worked == False:
        ROS_ERROR("Something went wrong with joint to dh conversion")
        return success
    for i in range(RAVEN_JOINTS):
        if i == 2:
            dh_d[i] = jp_dh[i]
            dh_theta[i] = RAVEN_DH_THETA[arm][i]
        else:
            dh_d[i] = RAVEN_DH_D[arm][i]
            dh_theta[i] = jp_dh[i]
        dh_alpha[i] = RAVEN_DH_ALPHA[arm][i]
        dh_a[i] = RAVEN_DH_A[arm][i]

    output_transformation = np.matmul(np.matmul(RAVEN_T_CB, RAVEN_T_B0[arm]), fwd_trans(0, 5, dh_alpha, dh_theta, dh_a, dh_d))

    return output_transformation
