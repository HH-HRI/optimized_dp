#!/usr/bin/python
# -*- coding: utf-8 -*-
#----------------------------------------------------------------------------
# Created By  : Haimin Hu
# Created Date: 06/13/2022
# version = '1.0'
# ---------------------------------------------------------------------------
""" Run this file to perform the HJ analysis for the Merge6D scenario."""
# ---------------------------------------------------------------------------

# Bring your packages onto the path
# import sys, os
# os.chdir("..")
# sys.path.insert(0, '')
import numpy as np
#import scipy.io as spio
# Utility functions to initialize the problem
from Grid.GridProcessing import Grid
from Shapes.ShapesFunctions import *
from SHARP.Merge6D_Unicycle_HJ import HJComp
# Plot options
from plot_options import *
# Solver core
#from solver import HJSolver
from Plots.plotting_utilities import *
import math
import pickle

''' DYNAMICS
               0      1    2     3      4      5
    states: [px_ef  px_er  py  vx_ef  vx_er  theta]
    ctrls:  [a      omega]
    dstbs:  [ax_f   ax_r]

    dot(x0) = x3
    dot(x1) = x4
    dot(x2) = v_nominal*sin(x5)
    dot(x3) = d0 - u0*cos(x5)
    dot(x4) = u0*cos(x5) - d1
    dot(x5) = u1
'''

# Defines a dictionary tracking parameters of the problem.
params = {}

# Nominal velocity of the robot
params['v_nominal'] = 4

# State, control and disturbance dimensions.
params['dim_state'] = 6
params['dim_ctrl']  = 2
params['dim_dstb']  = 2

# Look-back length and time step
params['lookback_length'] = 10.0
params['tstep'] = 0.05
params['small_number'] = 1e-5

# Number of points in each dimension.
# params['grid_points'] = [41, 41, 21, 21, 21, 7]
# params['grid_points'] = [26, 26, 13, 13, 13, 7]
params['grid_points'] = [15, 15, 9, 9, 9, 5]

# Road boundary (accounting for the car footprint).
params['rd_bd_margin'] = 1.5
params['rd_bd_min'] = -5.5 + params['rd_bd_margin']
params['rd_bd_max'] = 5.5 - params['rd_bd_margin']

# Lane center.
params['lane_center'] = 0.0
params['lane_break_margin'] = 1.5

# Grid range.
params['x0_min'] = -5.
params['x0_max'] = 20.
params['x1_min'] = -5.
params['x1_max'] = 20.
params['x2_min'] = -4.
params['x2_max'] = 4.
params['x3_min'] = -5.
params['x3_max'] = 5.
params['x4_min'] = -5.
params['x4_max'] = 5.
params['x5_min'] = -np.pi/4
params['x5_max'] = np.pi/4

# Target set specs.
params['px_r_target_lb'] = 7.0

# Control bounds.
params['aMin_R']  = -1.0
params['aMax_R']  = 1.0
params['omegaMin_R'] = -0.2
params['omegaMax_R'] = 0.2

# Disturbance bounds.
params['aMin_f'] = -0.5
params['aMax_f'] = 0.5
params['aMin_r'] = -0.5
params['aMax_r'] = 0.5

# Perform HJ analysis.
save_result = True
HJComp(params, save_result, file_name="Merge6D_Unicycle", accuracy="high")

if save_result:
  # create a binary pickle file 
  f = open("Merge6D_Unicycle_params.pkl","wb")

  # write the python object (dict) to pickle file
  pickle.dump(params, f)

  # close file
  f.close()

