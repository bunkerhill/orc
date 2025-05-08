# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np
import os
from math import sqrt

np.set_printoptions(precision=3, linewidth=200, suppress=True)
LINE_WIDTH = 60

T_SIMULATION = 5             # number of time steps simulated
dt = 0.01                      # controller time step
ndt = 10
q0 = np.array([0. , 0.0,  0.0,  0. ,  0. ,  0. , 0., 0.]).T  # initial configuration

kp = 50               # proportional gain of joint posture task
kd = 2*sqrt(kp)        # derivative gain of joint posture task

# PARAMETERS OF REFERENCE SINUSOIDAL TRAJECTORY
# amp                  = np.array([0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0])           # amplitude
# phi                  = np.array([0.0, 0.0, 0.0, 0.5*np.pi, 0.0, 0.0, 0.0, 0.0])     # phase
# freq                 = np.array([0.0, 0.0, 1.0, 0.5, 0.3, 0.0, 0.0, 0.0])           # frequency (time 2 PI)

simulate_coulomb_friction = 0
simulation_type = 'timestepping' #either 'timestepping' or 'euler'
tau_coulomb_max = 1.0*np.ones(6) # expressed as percentage of torque max

randomize_robot_model = 0
model_variation = 30.0

use_viewer = True
which_viewer = 'meshcat'
simulate_real_time = True
show_floor = False
PRINT_T = 1                   # print every PRINT_N time steps
DISPLAY_T = 0.02              # update robot configuration in viwewer every DISPLAY_N time steps
CAMERA_TRANSFORM = [2.582354784011841, 1.620774507522583, 1.0674564838409424, 0.2770655155181885, 0.5401807427406311, 0.6969326734542847, 0.3817386031150818]

lxp = 0.20                                  # foot length in positive x direction
lxn = 0.20                                  # foot length in negative x direction
lyp = 0.1                                  # foot length in positive y direction
lyn = 0.1                                  # foot length in negative y direction
lz = 0.                                     # foot sole height with respect to ankle joint
mu = 0.3                                    # friction coefficient
fMin = 5.0                                  # minimum normal force
fMax = 1000.0                               # maximum normal force
rf_frame_name = "ankle_R_Link"
lf_frame_name = "ankle_L_Link"
contactNormal = np.array([0., 0., 1.])      # direction of the normal to the contact surface

kp_contact = 10.0       # proportional gain of contact constraint
kp_foot = 10.0          # proportional gain of contact constraint
kp_com = 80.0           # proportional gain of center of mass task
kp_am = 10.0            # proportional gain of angular momentum task
kp_posture = 1.0        # proportional gain of joint posture task

w_com = 10.0             # weight of center of mass task
w_am = 1e-3             # weight of angular momentum task
w_foot = 1.0           # weight of the foot motion task
w_contact = 1000.0        # weight of foot in contact (negative means infinite weight)
w_posture = 1.0        # weight of joint posture task
w_forceRef = 1e-3       # weight of force regularization task
w_cop = 0.0
w_torque_bounds = 1.0   # weight of the torque bounds
w_joint_bounds = 0.0

gain_vector = np.array(  # gain vector for postural task
    [
        1.,
        1.,
        1.,
        1.,
        1.,
        1.,  # lleg  #low gain on axis along y and knee
        1.,
        1.
    ]  #head
)

masks_posture = np.ones(8)
tau_max_scaling = 1.45  # scaling factor of torque bounds
v_max_scaling = 1.8
N_SIMULATION = 1000  # number of time steps simulated
dt = 0.005           # controller time step
PRINT_N = 20           # print every PRINT_N time steps
DISPLAY_N = 4          # update robot configuration in viwewer every DISPLAY_N time steps
