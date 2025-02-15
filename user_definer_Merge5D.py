#!/usr/bin/python
# -*- coding: utf-8 -*-
#----------------------------------------------------------------------------
# Created By  : Haimin Hu
# Created Date: 06/08/2022
# version = '1.0'
# ---------------------------------------------------------------------------
""" Run this file to perform the HJ analysis for the Merge5D scenario."""
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
from SHARP.Merge5D_HJ import HJComp
# Plot options
from plot_options import *
# Solver core
#from solver import HJSolver
from Plots.plotting_utilities import *
import math
import pickle

''' DYNAMICS
               0      1    2     3      4
    states: [px_ef  px_er  py  vx_ef  vx_er]
    ctrls:  [ax     vy]
    dstbs:  [ax_f   ax_r]

    x0_dot = x3
    x1_dot = x4
    x2_dot = u1
    x3_dot = d0 - u0
    x4_dot = u0 - d1
'''

# Defines a dictionary tracking parameters of the problem.
params = {}

# State, control and disturbance dimensions.
params['dim_state'] = 5
params['dim_ctrl']  = 2
params['dim_dstb']  = 2

# Look-back length and time step
params['lookback_length'] = 10.0
params['tstep'] = 0.05
params['small_number'] = 1e-5

# Number of points in each dimension.
# params['grid_points'] = [61, 61, 31, 31, 31]
params['grid_points'] = [41, 41, 21, 21, 21]

# Road boundary (accounting for the car footprint).
params['rd_bd_min'] = -3.7
params['rd_bd_max'] = 3.7

# Lane center.
params['lane_center'] = 0.

# Grid range.
params['x0_min'] = -10.
params['x0_max'] = 25.
params['x1_min'] = -10.
params['x1_max'] = 25.
params['x2_min'] = -4.
params['x2_max'] = 4.
params['x3_min'] = -5.
params['x3_max'] = 5.
params['x4_min'] = -5.
params['x4_max'] = 5.

# Target set specs.
params['px_r_target_lb'] = 7.5

# Control bounds.
params['aMin_R']  = -2.
params['aMax_R']  = 2.
params['vyMin_R'] = -.5
params['vyMax_R'] = .5

# Disturbance bounds.
params['aMin_f'] = -0.5
params['aMax_f'] = 0.5
params['aMin_r'] = -0.5
params['aMax_r'] = 0.5

# Perform HJ analysis.
save_result = True
HJComp(params, save_result, file_name="Merge5D", accuracy="high")

if save_result:
  # create a binary pickle file 
  f = open("Merge5D_params.pkl","wb")

  # write the python object (dict) to pickle file
  pickle.dump(params, f)

  # close file
  f.close()

