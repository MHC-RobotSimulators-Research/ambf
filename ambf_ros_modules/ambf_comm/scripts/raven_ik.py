from PyKDL import Vector, Rotation, Frame, dot
import numpy as np
import math as m
import raven_fk as rfk
import ambf_raven_def as ard
import kinematics as k
import utilities as u
import rospy

def compute_IK(input_cp, arm):
    xf = np.matrix(np.linalg.inv(np.array(ard.raven_T_CB * ard.raven_T_B0[arm])) * input_cp)
    iksol = np.zeros((ard.raven_iksols, ard.raven_joints - 1), dtype = 'float')
    ikcheck = np.zeros(ard.raven_iksols, dtype = 'float')

    dh_alpha = np.zeros(6, dtype = 'float')
    dh_theta = np.zeros(6, dtype = 'float')
    dh_a     = np.zeros(6, dtype = 'float')
    dh_d     = np.zeros(6, dtype = 'float')

    for i in range(ard.raven_joints - 1):
        dh_alpha[i] = ard.raven_dh_alpha[arm][i]
        dh_theta[i] = ard.raven_dh_theta[arm][i]
        dh_a[i]     = ard.raven_dh_a[arm][i]
        dh_d[i]     = ard.raven_dh_d[arm][i]

    # Step 1: Compute j5
    p6rcm = np.zeros(4)
    p65 = np.zeros(4)
    T_6_0 = np.linalg.inv(xf)
    p6rcm = u.get_Origin(T_6_0) #getting the origin
    p05   = np.zeros((8,4), dtype = 'float')


    p6rcm[2] = 0 #take projection on x-y plane
    for i in range(2):
        p65 = (-1 + 2 * i) * ard.raven_ikin_param[5] * u.normalize(p6rcm)
        p05[4 * i] = p05[4 * i + 1] = p05[4 * i + 2] = p05[4 * i + 3] = (xf * p65.reshape((4,1))).reshape(4) #TODO: figure out to have this multiplication happen

    # Step 2: Compute Displacement of Prismatic Joint d3
    for i in range(ard.raven_iksols/4):
        insertion = float(0)
        insertion += len(p05[4 * i])
        iksol[4 * i + 0][2] = iksol[4 * i + 1][2] = -ard.raven_ikin_param[4] - insertion
        iksol[4 * i + 2][2] = iksol[4 * i + 3][2] = -ard.raven_ikin_param[4] + insertion

    #Step 3: Calculate Theta 2
    for i in range(ard.raven_iksols - 1): #adjusted from original to be raven_iksols - 1
        z0p5 = float(p05[i][2])
        cth2 = float(0)
        d = float(iksol[i][2] + ard.raven_ikin_param[4])
        cth2_nom = float((z0p5 / d) + ard.raven_ikin_param[1] * ard.raven_ikin_param[3])
        cth2_den = float(ard.raven_ikin_param[0] * ard.raven_ikin_param[2])

        cth2 = -cth2_nom / cth2_den

        if cth2 > 1 and cth2 < 1 + eps:
            cth2 = 1
        elif cth2 < -1 and cth2 > -1 - eps:
            cth2 = -1
        if cth2 > 1 or cth2 < -1:
            ikcheck[i] = ikcheck[i + 1] = False
        else:
            iksol[i][1] = m.acos(cth2)
            iksol[i + 1][1] = -m.acos(cth2)
        i += 1

    #Step 4: Comput theta 1
    for i in range(ard.raven_iksols):
        if ikcheck[i] == False:
            continue
        cth2     = float(m.cos(iksol[i][1]))
        sth2     = float(m.sin(iksol[i][1]))
        d        = float(iksol[i][2] + ard.raven_ikin_param[4])
        BB1      = float(sth2 * ard.raven_ikin_param[2])
        BB2      = 0
        Bmx      = np.matrix((3,3), dtype = 'float')
        xyp05    = p05[i]
        xyp05[2] = 0

        BB2      = cth2 * ard.raven_ikin_param[1] * ard.raven_ikin_param[2] - ard.raven_ikin_param[0] * ard.raven_ikin_param[3]

        if arm == 0:
            Bmx[0][0] = BB1
            Bmx[0][1] = BB2
            Bmx[0][2] = 0
            Bmx[1][0] = -BB2
            Bmx[1][1] = BB1
            Bmx[1][2] = 0
            Bmx[2][0] = 0
            Bmx[2][1] = 0
            Bmx[2][2] = 1
        else:
            Bmx[0][0] = BB1
            Bmx[0][1] = BB2
            Bmx[0][2] = 0
            Bmx[1][0] = BB2
            Bmx[1][1] = -BB1
            Bmx[1][2] = 0
            Bmx[2][0] = 0
            Bmx[2][1] = 0
            Bmx[2][2] = 1

        scth1 = np.linalg.inv(Bmx) * xyp05 * (1 /d)
        iksol[i][0] = atan2(scth1[1], scth1[0])

    #Step 5: Get Theta 4, 5, 6
    for i in range(ard.raven_iksols):
        if ikcheck[i] = False:
            continue
        dh_theta[0] = iksol[i][0]
        dh_theta[1] = iksol[i][1]
        dh_d[2]     = iksol[i][2]

        if i != 2:
            T_0_3 = k.DH(dh_alpha)

#Ending on line 466 of ambf_motion_planner.cpp



if __name__ == "__main__":
    input_cp = rfk.compute_FK(ard.home_joints, 0)
    compute_IK(input_cp, 0)
