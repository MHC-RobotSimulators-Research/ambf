from PyKDL import Vector, Rotation, Frame, dot
import numpy as np
import math
from raven_fk import *
import rospy

def compute_IK(T_6_0):
    j1 = T_6_0.p[0]
    print(j1)

if __name__ == "__main__":
    input_cp = compute_FK(ard.home_joints, 0)
    current_cp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    print(compute_IK(input_cp))
