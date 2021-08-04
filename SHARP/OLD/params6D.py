import numpy as np
#import scipy.io as spio
# Utility functions to initialize the problem
from Grid.GridProcessing import Grid
from Shapes.ShapesFunctions import *
from hjComp6D import HJComp
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
#idx = int(os.environ["SLURM_ARRAY_TASK_ID"])
# mode type: 'arc', 'rect','basic', None
modes = ['rect', 'rect', 'rect']


velRange = np.linspace(5, 20, 8, endpoint = True)
frontLeft = np.linspace(-2, 30, 17, endpoint = True)
frontRight = np.linspace(-2, 30, 17, endpoint = True)
behindRange = np.linspace(-15, -3, 7, endpoint = True)
print(velRange)
print(frontLeft)
print(behindRange)
state_idx = 0
states = []
for i in range(len(velRange)):
  for j in range(len(frontLeft)):
    for k in range(len(frontRight)):
      for l in range(len(behindRange)):
        states.append([[behindRange[l], 1.85, velRange[i]], [frontLeft[j], 1.85, velRange[i]], [frontRight[k], -1.85, velRange[i]]])
        state_idx = state_idx + 1
print(state_idx)
# if mode = None, still fill out arbitrary state
print(np.shape(states))
print(len(states[0])) 
idx = 0

params['avoid'] = {'lgt_lb': -5.5, 'lgt_ub': 5.5, 'lat_bd':2.0}

params['obst']  = {'state': states[idx] ,  'a_max':0.1, 'theta_max':0.05, 'v_lat_max':0.1, 'mode':modes[idx]}

# Look-back length and time step
params['lookback_length'] = 5.0
params['tstep'] = 0.1
params['small_number'] = 1e-5

# num points in each dim
params['grid_points'] = [40, 12, 22, 8, 8, 8]


# road width
params['x1_ll'] = -30
params['x2_ll'] = -30
params['rd_bd_min'] = -3.7
params['x5_ll'] = 0
params['x6_ll'] = 0
params['x1_ul'] = 75
params['x2_ul'] = 75
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