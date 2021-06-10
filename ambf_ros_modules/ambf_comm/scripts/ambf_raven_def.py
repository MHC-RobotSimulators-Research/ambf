
#Contains all values for Raven 2
import math as m
import numpy as np

V = -1
camera_count = 3
raven_joints = 7
raven_arms = 2
raven_iksols = 8
raven_loop_rate = 1000
safe_ori_incr = float(m.pi / (6 * raven_loop_rate))
safe_pos_incr = float(0.5 / raven_loop_rate)

zero_joints = [0, 0, 0, 0, 0, 0, 0]
max_joints = [m.pi, m.pi, 0.10, m.pi, 2, (m.pi*3)/4, (m.pi*3)/4]
min_joints = [-m.pi, -m.pi, -0.17, -m.pi, -2, 0, 0]
home_joints = [m.pi/3, (m.pi*3)/5, -0.09, (m.pi*3)/4, 0, m.pi/6, m.pi/6]
dance_scale_joints = [0.3, 0.3, 0.06, 0.3, 1.2, m.pi/6, m.pi/6]

raven_joint_limit = [[0,       m.pi/4,         -0.17,           -m.pi/2, -2,     -2,     -2],
                     [m.pi/2, (m.pi*3)/4,       0.10,            m.pi*2,  2,      2,      2]]
raven_dh_alpha    = [[0,       np.deg2rad(-75), np.deg2rad(128), 0,       m.pi/2, m.pi/2, 0],
                     [m.pi,    np.deg2rad(75),  np.deg2rad(52),  0,       m.pi/2, m.pi/2, 0]]
raven_dh_a        = [[0,       0,               0,               0,       0,      0.013,  0],
                     [0,       0,               0,               0,       0,      0.013,  0]]
raven_dh_d        = [[0,       0,               V,              -0.47,    0,      0,      0],
                     [0,       0,               V,              -0.47,    0,      0,      0]]
raven_dh_theta    = [[V,       V,               m.pi/2,          V,       V,      V,      0],
                     [V,       V,              -m.pi/2,          V,       V,      V,      0]]
raven_ikin_param  = [float(m.sin(raven_dh_alpha[0][1])),
                     float(m.cos(raven_dh_alpha[0][1])),
                     float(m.sin(raven_dh_alpha[1][2])),
                     float(m.cos(raven_dh_alpha[1][2])),
                     float(raven_dh_d[0][3]),
                     float(raven_dh_a[0][5])]
raven_T_B0        = [np.matrix([[0,         0,         1,  0.30071],
                                [0,        -1,         0,  0.061  ],
                                [1,         0,         0, -0.007  ],
                                [0,         0,         0,  1      ]],
                               dtype = 'float'),
                     np.matrix([[0,         0,        -1, -0.30071],
                                [0,         1,         0,  0.061  ],
                                [1,         0,         0, -0.007  ],
                                [0,         0,         0,  1      ]],
                               dtype = 'float')]
raven_T_CB        = np.matrix([[ 0.4231991, 0.9060367, 0,  0      ], #approx translation from Quaterion: x = 0, y = 0, z = 1, w = -math.pi/2
                               [-0.9060367, 0.4231991, 0,  0      ],
                               [ 0,         0,         1,  0      ],
                               [ 0,         0,         0,  1      ]],
                              dtype = 'float')
