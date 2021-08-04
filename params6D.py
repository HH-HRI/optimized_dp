import numpy as np
#import scipy.io as spio
# Utility functions to initialize the problem
from Grid.GridProcessing import Grid
from Shapes.ShapesFunctions import *
from hjComp import HJComp
# Specify the  file that includes dynamic systems
from dynamics.DubinsCar6D_HRI import *
# Plot options
from plot_options import *
# Solver core
from solver import HJSolver
from Plots.plotting_utilities import *
import math
import os


""" USER INTERFACES
- Define grid
- Generate initial values for grid using shape functions
- Time length for computations
- Initialize plotting option
- Call HJSolver function
"""

'''
Defining parameters for the scenario
'''
# dictionary tracking parameters of the problem
params = {}

# avoid set specs
idx = int(os.environ["SLURM_ARRAY_TASK_ID"])
# mode type: 'arc', 'rect','basic', None
modes = ['basic', 'basic', 'basic']
# if mode = None, still fill out arbitrary state
states = [[35, 1.85,10], [45, 1.85, 10], [55, 1.85, 10]]


params['avoid'] = {'lgt_lb': -5.5, 'lgt_ub': 5.5, 'lat_bd':2.0}

params['obst']  = {'state': states[idx] ,  'a_max':0.5, 'theta_max':0.05, 'v_lat_max':0.1, 'mode':modes[idx]}

# Look-back length and time step
params['lookback_length'] = 5.0
params['tstep'] = 0.1
params['small_number'] = 1e-5

# num points in each dim
params['grid_points'] = [30, 30, 12, 12, 12, 12]


# road width
params['x1_ll'] = 0
params['x2_ll'] = 0
params['rd_bd_min'] = -3.7
params['x5_ll'] = 0
params['x6_ll'] = 0
params['x1_ul'] = 95
params['x2_ul'] = 95
params['rd_bd_max'] = 3.7
params['x5_ul'] = 35
params['x6_ul'] = 35

# target set specs
params['xr_tar_overtake']  = 10
params['xr_tar_lanekeep']  = -15

# input bounds and model parameters
params['accMax_R']   = 3
params['vLatMax_R']  = 3
params['accMax_H']   = 1
params['vLatMax_H']  = 1
params['accMax_R_sh']   = 3
params['vLatMax_R_sh']  = 3
params['accMax_H_sh']   = 1
params['vLatMax_H_sh']  = 1
params['talpha']    = 0.01


HJComp(params, idx)
