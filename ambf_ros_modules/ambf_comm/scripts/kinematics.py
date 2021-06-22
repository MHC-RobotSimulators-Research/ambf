import numpy as np
import ambf_raven_def as ard
import math as m


class DH:
    def __init__(self, alpha, a, theta, d, joint_type, convention = 'MODIFIED'): #Raven arms use modified transmat
        self.alpha = alpha
        self.a = a
        self.theta = theta
        self.d = d
        self.joint_type = joint_type
        if convention in ['STANDARD', 'MODIFIED']:
            self.convention = convention
        else:
            raise ('ERROR, DH CONVENTION NOT UNDERSTOOD')

    def mat_from_dh(self, alpha, a, theta, d):
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        ct = np.cos(theta)
        st = np.sin(theta)

        if self.convention == 'STANDARD':
            mat = np.mat([
                [ct, -st * ca,  st * sa,  a * ct],
                [st,  ct * ca, -ct * sa,  a * st],
                [0,        sa,       ca,       d],
                [0,         0,        0,       1]
            ])
        elif self.convention == 'MODIFIED':
            mat = np.mat([
                [ct, -st, 0, a],
                [st * ca, ct * ca, -sa, -d * sa],
                [st * sa, ct * sa, ca, d * ca],
                [0, 0, 0, 1]
            ])
        else:
            raise ('ERROR, DH CONVENTION NOT UNDERSTOOD')

        return mat

    def get_trans(self):
        return self.mat_from_dh(self.alpha, self.a, self.theta, self.d)


def enforce_limits(j_raw, joint_lims):
    num_joints = len(j_raw)
    j_limited = [0.0]*num_joints

    for idx in range(num_joints):
        min_lim = joint_lims[idx][0]
        max_lim = joint_lims[idx][1]
        j_limited[idx] = max(min_lim, min(j_raw[idx], max_lim))

    return j_limited

def joint_to_dhvalue(joint, arm):
    dh_val = np.array([0, 0, 0, 0, 0, 0, 0], dtype = 'float')
    for i in range(ard.raven_joints - 1):
        if i != 2:
            if i == 5:
                if arm == 0:
                    dh_val[i] = (joint[i] - joint[i + 1])
                else:
                    dh_val[i] = -(joint[i] - joint[i + 1])
            else:
                dh_val[i] = joint[i]
            while dh_val[i] > m.pi:
                dh_val[i] -= 2 * m.pi
            while dh_val[i] < -m.pi:
                dh_val[i] += 2 * m.pi
        else:
            dh_val[i] = joint[i]
    return dh_val

def find_best_solution(curr_jp, iksol, ikcheck):
    best_err = float(1e10)
    best_idx = -1

    for i in range(ard.raven_iksols):
        error = float(0)
        if ikcheck[i] == True:
            for j in range(raven_joints - 1):
                if j == 2:
                    error += 100 * (iksol[i][j] - curr_jp[j])**2
                else:
                    diff = float(iksol[i][j] - curr_jp[j])
                    while diff > m.pi:
                        diff -= 2 * m.pi
                    while diff < -m.pi:
                        diff += 2 * m.pi
                    error += diff**2
            if error < best_err:
                best_err = error
                best_idx = i
    return best_err, best_idx

def dhvalue_to_joint(dhvalue, gangle, arm):
    joint = np.zeros(7)
    for i in range(ard.raven_joints - 1):
        if i != 2:
            if i == 5:
                if arm == 0:
                    joint[i + 1] = float(-dhvalue[i] + gangle) / 2
                    joint[i] = float(dhvalue[i] + gangle) / 2
                else:
                    joint[i] = float(-dhvalue[i] + gangle) / 2
                    joint[i + 1] = float(dhvalue[i] + gangle) / 2
            else:
                joint[i] = dhvalue[i]
            while joint[i] > m.pi:
                joint[i] -= 2 * m.pi
            while joint[i] < -m.pi:
                joint[i] += 2 * m.pi
        else:
            joint[i] = dhvalue[i]
        output_cp = apply_joint_limits(joint)
    return output_cp

def apply_joint_limits(joint):
    for i in range(ard.raven_joints):
        if i != 2:
            while joint[i] > m.pi:
                joint[i] -= 2 * m.pi
            while joint[i] < -m.pi:
                joint[i] += 2 * m.pi
        if joint[i] < ard.raven_joint_limit[0][i]:
            joint[i] = ard.raven_joint_limit[0][i]
        elif joint[i] > ard.raven_joint_limit[1][i]:
            joint[i] = ard.raven_joint_limit[1][i]
    return joint
