from ambf_raven_def import *
from raven_fk import fwd_kinematics, fwd_trans, joint_to_dhvalue
import math as m
import numpy as np
import utilities as u


# alpha, theta, a, d --> 3 out of 4 are constant, for R --> theta, for P --> ?
# input_cp --> T_C_6, where the tip of the robot is with respect to the global frame, C
# T_CB --> here the B frame is with respect to C, in sim, universal frame called C in between two arms, the B frames
# T_B0 --> where the base of the robot is with respect to the B frame (middle of the robot arms)

# this method returns iksols <-- a matrix of all valid solutions, more work is needed to find the best solution

def inv_kinematics(arm, input_cp, input_gangle):

    # contextualizing the location of the tip of raven with respect to the outer plane of existence

    # xf = T_0_6, one per arm


    T_0_6 = np.matmul(np.linalg.inv(np.matmul(RAVEN_T_CB, RAVEN_T_B0[arm])), input_cp)
    iksol = np.zeros((RAVEN_IKSOLS, RAVEN_JOINTS))
    ikcheck = np.zeros(RAVEN_IKSOLS)

    dh_alpha = np.zeros(6) # known
    dh_theta = np.zeros(6) # trying to solve, except for joint 3 (index 2)
    dh_d = np.zeros(6) # known
    dh_a = np.zeros(6) # known, except for joint 3 (index 2)

    for i in range(RAVEN_JOINTS - 1):
        # in ambf_def "V" is the placeholder
        dh_alpha[i] = RAVEN_DH_ALPHA[arm][i]
        dh_theta[i] = RAVEN_DH_THETA[arm][i]
        dh_d[i]     = RAVEN_DH_D[arm][i]
        dh_a[i]     = RAVEN_DH_A[arm][i]

    for i in range(RAVEN_IKSOLS):
        iksol[i]   = np.zeros(RAVEN_JOINTS) # 6 length cannot accomodate length 7 vector??? if problem, create zero_joints length 6
        ikcheck[i] = True # flag whether particular set of joints are legal or checking eventually the closest one

    # STEP 1: Comput P5
    T_6_0 = np.linalg.inv(T_0_6) # T60 --> the 0th frame in terms of the 6th frame

    p6rcm = np.zeros((4,1), dtype = 'float')
    p6rcm[:3] = u.get_Origin(T_6_0) # x, y, z  rcm stands for remote center of motion

    p05   = np.ones((8, 4))
    p6rcm[2] = 0 # takes projection on x,y plane

    for i in range(2):
        p65 = (-1 + 2 * i) * RAVEN_IKIN_PARAM[5] * (p6rcm / np.linalg.norm(p6rcm)) # finds the position of the 5th joint with respect to the 6th joint


        p65[-1] = 1

        p05[4 * i][:3] = p05[4 * i + 1][:3] = p05[4 * i + 2][:3] = p05[4 * i + 3][:3] = np.matmul(T_0_6, p65)[:3].squeeze()
    # now we have two unique solutions

    # STEP 2: Computing the prismatic joint j3
    for i in range(int(RAVEN_IKSOLS / 4)):
        insertion = float(0)
        insertion += np.linalg.norm(p05[4 * i][:3])
        # print("insertion = ")
        # print(insertion)

        # checking the physical boundary of how much it can insert
        if insertion <= RAVEN_IKIN_PARAM[5]:
            ROS_ERROR("WARNING: Raven mechanism at RCM singularity. IK failing.")
            ikcheck[4 * i + 0] = ikcheck[4 * i + 1] = False
            ikcheck[4 * i + 3] = ikcheck[4 * i + 4] = False
            break

        # sets prismatic joint as higher or lower depending on how high or low the 5th frame is relative to the 0th frame
        iksol[4 * i + 0][2] = iksol[4 * i + 1][2] = - RAVEN_IKIN_PARAM[4] - insertion
        iksol[4 * i + 2][2] = iksol[4 * i + 3][2] = - RAVEN_IKIN_PARAM[4] + insertion
		# now we have 4 unique solutions

    # STEP 3: Evaluate Theta 2
    for i in np.arange(0, RAVEN_IKSOLS, 2): # now we have to look at 4 unique solutions
        z0p5 = float(p05[i][2]); # <-- zth position of the 5th joint with respect to the 0th joint
        d = float(iksol[i][2] + RAVEN_IKIN_PARAM[4])
        cth2_nom = float(( z0p5 / d) + RAVEN_IKIN_PARAM[1] * RAVEN_IKIN_PARAM[3])
        cth2_den = float(RAVEN_IKIN_PARAM[0] * RAVEN_IKIN_PARAM[2])
        cth2 = float(-cth2_nom / cth2_den) # cosine(theta2)


        # Smooth roundoff errors at +/- 1. <-- exceeding one or less than negative 1 by a little bit, still valid
        if cth2 > 1 and cth2 < 1 + Eps:
            cth2 = 1
        elif cth2 < -1 and cth2 > -1 - Eps:
            cth2 = -1
        if cth2 > 1 or cth2 < - 1:
            ikcheck[i] = ikcheck[i + 1] = False
        else:
            iksol[i][1] = m.acos(cth2)
            iksol[i + 1][1] = - m.acos(cth2)
        i += 1
        # now we have 8 unique solutions

    # STEP 4: Compute Theta 1
    for i in range(RAVEN_IKSOLS):
        if ikcheck[i] == False:
            continue
        cth2 = float(m.cos(iksol[i][1]))
        sth2 = float(m.sin(iksol[i][1]))
        d = float(iksol[i][2] + RAVEN_IKIN_PARAM[4])
        BB1 = sth2 * RAVEN_IKIN_PARAM[2]
        xyp05 = p05[i]
        xyp05[2] = 0
        BB2 = cth2 * RAVEN_IKIN_PARAM[1] * RAVEN_IKIN_PARAM[2] - RAVEN_IKIN_PARAM[0] * RAVEN_IKIN_PARAM[3]
        if arm == 0:
            Bmx = np.matrix([[ BB1, BB2, 0],
                             [-BB2, BB1, 0],
                             [   0,   0, 1]])
        else:
            Bmx = np.matrix([[BB1,  BB2, 0],
                             [BB2, -BB1, 0],
                             [  0,    0, 1]])
        scth1 = np.ones(4, dtype = 'float')
        scth1[:3] = np.matmul(np.linalg.inv(Bmx), xyp05[:3]) * (1 / d)
        iksol[i][0] = m.atan2(scth1[1], scth1[0])

    # STEP 5: Compute Theta 4, 5, 6

    for i in range(RAVEN_IKSOLS):
        if ikcheck[i] == False:
            continue
        # compute T_0_3
        dh_theta[0] = iksol[i][0]
        dh_theta[1] = iksol[i][1]
        dh_d[2]     = iksol[i][2]

        T_0_3 = fwd_trans(0, 3, dh_alpha, dh_theta, dh_a, dh_d)

        T_3_6 = np.matmul(np.linalg.inv(T_0_3), T_0_6)
        T_3_6_B = u.get_Basis(T_3_6)
        T_3_6_O = u.get_Origin(T_3_6)
        c5 = -float(T_3_6_B[2,2])
        s5 = float(T_3_6_O[2] - RAVEN_IKIN_PARAM[4]) / float(RAVEN_IKIN_PARAM[5])

        if m.fabs(c5) > Eps:
            c4 = float(T_3_6_O[0]) / float(RAVEN_IKIN_PARAM[5] * c5)
            s4 = float(T_3_6_O[1]) / float(RAVEN_IKIN_PARAM[5] * c5)
        else:
            c4 = T_3_6_B[0][2] / s5
            s4 = T_3_6_B[1][2] / s5
        iksol[i][3] = m.atan2(s4, c4)
        iksol[i][4] = m.atan2(s5, c5)
        if m.fabs(s5) > Eps:
            c6 = T_3_6_B[2,0] / s5
            s6 = -T_3_6_B[2,1] / s5
        else:
            dh_theta[3] = iksol[i][3]
            dh_theta[4] = iksol[i][4]
            T_0_5 = np.matmul(T_0_3, fwd_trans(3, 5, dh_alpha, dh_theta, dh_a, dh_d))
            T_5_6 = np.matmul(np.linalg.inv(T_0_5), T_0_6)
            c6 = u.get_Basis(T_5_6)[0,0]
            s6 = u.get_Origin(T_5_6)[2,0]
        iksol[i][5] = m.atan2(s6, c6)

    if not joint_to_dhvalue(HOME_JOINTS, 1):
        ROS_ERROR("Something went wrong :(")
        return False
    best_err, best_idx = find_best_solution(HOME_JOINTS, iksol, ikcheck)
    return dhvalue_to_joint(iksol[best_idx], input_gangle, arm)


def inv_kinematics_p5(arm, input_cp, input_gangle):
    """
    Calculates joint positions for joints 1, 2, and 3 using p5, then assigns joints 4,5, and 6 to their home values
    Args:
        arm (int) : which arm to perform ik for
        input_cp (array) : dh transformation for p5
        input_gangle (float) : specifies the angle of the gripper
    Returns:
        An array containing the calculated joint positions
    """
    numsols = 4
    iksol = np.zeros((numsols, RAVEN_JOINTS))
    ikcheck = np.zeros(numsols)

    dh_alpha = np.zeros(6) # known
    dh_theta = np.zeros(6) # trying to solve, except for joint 3 (index 2)
    dh_d = np.zeros(6) # known
    dh_a = np.zeros(6) # known, except for joint 3 (index 2)

    # calculate p05 from current position (p5 with respect to RCM?)
    p05_dh = np.matmul(np.linalg.inv(np.matmul(RAVEN_T_CB, RAVEN_T_B0[arm])), input_cp)
    p05 = np.array([p05_dh[0, 3], p05_dh[1, 3], p05_dh[2, 3], 1.0], dtype="float")

    for i in range(RAVEN_JOINTS - 1):
        # in ambf_def "V" is the placeholder
        dh_alpha[i] = RAVEN_DH_ALPHA[arm][i]
        dh_theta[i] = RAVEN_DH_THETA[arm][i]
        dh_d[i]     = RAVEN_DH_D[arm][i]
        dh_a[i]     = RAVEN_DH_A[arm][i]

    for i in range(numsols):
        iksol[i]   = np.zeros(RAVEN_JOINTS) # 6 length cannot accomodate length 7 vector??? if problem, create zero_joints length 6
        ikcheck[i] = True # flag whether particular set of joints are legal or checking eventually the closest one

    # STEP 2: Computing the prismatic joint j3 based on p05
    insertion = float(0)
    insertion += np.linalg.norm(p05[:3])

    # checking the physical boundary of how much it can insert
    if insertion <= RAVEN_IKIN_PARAM[5]:
        ROS_ERROR("WARNING: Raven mechanism at RCM singularity. IK failing.")
        ikcheck[0] = ikcheck[1] = False
        ikcheck[3] = ikcheck[4] = False

    # sets prismatic joint as higher or lower depending on how high or low the 5th frame is relative to the 0th frame
    iksol[0][2] = iksol[1][2] = - RAVEN_IKIN_PARAM[4] - insertion
    iksol[2][2] = iksol[3][2] = - RAVEN_IKIN_PARAM[4] + insertion
    # now we have 2 unique solutions

    # STEP 3: Evaluate Theta 2
    for i in range(int(numsols/2)): # now we have to look at 2 unique solutions
        z0p5 = float(p05[2]) # <-- z position of the 5th joint with respect to the 0th joint

        d = float(iksol[i][2] + RAVEN_IKIN_PARAM[4])
        cth2_nom = float(( z0p5 / d) + RAVEN_IKIN_PARAM[1] * RAVEN_IKIN_PARAM[3])
        cth2_den = float(RAVEN_IKIN_PARAM[0] * RAVEN_IKIN_PARAM[2])
        cth2 = float(-cth2_nom / cth2_den) # cosine(theta2)

        # Smooth roundoff errors at +/- 1. <-- exceeding one or less than negative 1 by a little bit, still valid
        if cth2 > 1 and cth2 < 1 + Eps:
            cth2 = 1
        elif cth2 < -1 and cth2 > -1 - Eps:
            cth2 = -1
        if cth2 > 1 or cth2 < - 1:
            ikcheck[i * 2] = ikcheck[i * 2 + 1] = False
        else:
            iksol[i * 2][1] = m.acos(cth2)
            iksol[i * 2 + 1][1] = - m.acos(cth2)
        # now we have 4 unique solutions

    # STEP 4: Compute Theta 1
    for i in range(numsols):
        if not ikcheck[i]:
            continue

        cth2 = float(m.cos(iksol[i][1]))
        sth2 = float(m.sin(iksol[i][1]))
        d = float(iksol[i][2] + RAVEN_IKIN_PARAM[4])
        BB1 = sth2 * RAVEN_IKIN_PARAM[2]
        xyp05 = p05
        xyp05[2] = 0
        BB2 = cth2 * RAVEN_IKIN_PARAM[1] * RAVEN_IKIN_PARAM[2] - RAVEN_IKIN_PARAM[0] * RAVEN_IKIN_PARAM[3]
        if arm == 0:
            Bmx = np.matrix([[ BB1, BB2, 0],
                             [-BB2, BB1, 0],
                             [   0,   0, 1]])
        else:
            Bmx = np.matrix([[BB1,  BB2, 0],
                             [BB2, -BB1, 0],
                             [  0,    0, 1]])
        scth1 = np.ones(4, dtype = 'float')
        scth1[:3] = np.matmul(np.linalg.inv(Bmx), xyp05[:3]) * (1 / d)
        iksol[i][0] = m.atan2(scth1[1], scth1[0])

    home_dh = np.array([[1.04719755, 1.88495559, -0.03, 2.35619449 - m.pi/2, 0., 0., 0.52359878],
                       [1.04719755, 1.88495559, -0.03, 2.35619449 - m.pi/2, 0., -0., 0.52359878]], dtype="float")

    # Assign theta 4,5, and 6 to their desired values for all iksol
    for i in range(numsols):
        for j in range(3, 6):
            iksol[i, j] = home_dh[arm, j]

    # I don't think this is needed?
    # if not joint_to_dhvalue(HOME_JOINTS, 1):
    #     ROS_ERROR("Something went wrong :(")
    #     return False

    best_err, best_idx = find_best_solution(HOME_JOINTS, iksol, ikcheck)
    return dhvalue_to_joint(iksol[best_idx], input_gangle, arm)


def dhvalue_to_joint(dhvalue, gangle, arm):
    joint = np.zeros(RAVEN_JOINTS, dtype = 'float')
    for i in range(RAVEN_JOINTS - 1):
        if i != 2:
            if i == 5:
                if arm == 0:
                    joint[i + 1] = (-dhvalue[i] + gangle) / 2
                    joint[i]     = (dhvalue[i] + gangle) / 2
                else:
                    joint[i] = (-dhvalue[i] + gangle) / 2
                    joint[i + 1] = (dhvalue[i] + gangle) / 2
            else:
                joint[i] = dhvalue[i]
            while joint[i] > m.pi:
                joint[i] -= 2 * m.pi
            while joint[i] < -m.pi:
                joint[i] += 2 * m.pi
        else:
            joint[i] = dhvalue[i]
    return apply_joint_limits(joint)

def apply_joint_limits(joint):
    limited = False
    for i in range(RAVEN_JOINTS):
        if i != 2:
            while joint[i] > m.pi:
                joint[i] -= 2 * m.pi
            while joint[i] < -m.pi:
                joint[i] += 2 * m.pi
        if joint[i] < RAVEN_JOINT_LIMITS[0][i]:
            joint[i] = RAVEN_JOINT_LIMITS[0][i]
            limted = True
        elif joint[i] > RAVEN_JOINT_LIMITS[1][i]:
            joint[i] = RAVEN_JOINT_LIMITS[1][i]
            limited = True
    return joint, limited


def find_best_solution(curr_jp, iksol, ikcheck):
    best_err = float(1E10)
    # is this supposed to be best_idx?
    best_idx = -1

    for i in range(len(iksol)):
        error = 0
        if ikcheck[i] == True:
            for j in range(RAVEN_JOINTS - 1):
                if j == 2:
                    error += 100 * (iksol[i][j] - curr_jp[j]) ** 2
                else:
                    diff = float(iksol[i][j] - curr_jp[j])
                    while diff > m.pi:
                        diff -= 2 * m.pi
                    while diff < -m.pi:
                        diff += 2 * m.pi
                    error += diff ** 2
            if error < best_err:
                best_err = error
                best_idx = i

    return best_err, best_idx
