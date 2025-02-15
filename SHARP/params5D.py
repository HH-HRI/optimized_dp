# Bring your packages onto the path
import sys, os
os.chdir("..")
sys.path.insert(0, '')
import numpy as np
#import scipy.io as spio
# Utility functions to initialize the problem
from Grid.GridProcessing import Grid
from Shapes.ShapesFunctions import *
from SHARP.hjComp5D import HJComp
# Plot options
from plot_options import *
# Solver core
#from solver import HJSolver
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

velRange = [5,8,11,14,17,20]
frontLeft = [5,15,30]
frontRight = [5,15,30]
behindRange = [-15,-9,-2]
print(velRange)
print(frontLeft)
print(behindRange)
state_idx = 0

# uncomment if you want to use run in array solver 
# If you want to model rear obstacles as occupying 2 lanes, edit line 171 in hjComp5D.py
'''
# avoid set specs
idx = int(os.environ["SLURM_ARRAY_TASK_ID"]) # takes the idx from SLURM job script
states = []
for i in range(len(velRange)):
  for j in range(len(frontLeft)):
    for k in range(len(frontRight)):
      for l in range(len(behindRange)):
        states.append([[behindRange[l], 1.85, velRange[i]], [frontLeft[j], 1.85, velRange[i]], [frontRight[k], -1.85, velRange[i]]])
        state_idx = state_idx + 1
'''
##
# if  you want to specify specific obstacle initial configurations, use below. Can add arbitrary number of obstacles
idx = 0   #[x        y          v ]
states = [[[24.7013, 0.9658203, 14], [30.858-40, -1.4707,14], [11.41, -3.7002, 14]]]
##

params['avoid'] = {'lgt_lb': -5.5, 'lgt_ub': 5.5, 'lat_bd':2.0}
modes = ['rect'] # for now make all obstacles rectangle FRS
params['obst']  = {'state': states[idx] ,  'a_max':0.01, 'theta_max':0.02, 'v_lat_max':0.02, 'mode':modes[0]} 

# Look-back length and time step
params['lookback_length'] = 5.0
params['tstep'] = 0.05
params['small_number'] = 1e-5

# num points in each dim
params['grid_points'] = [24, 24, 13, 13, 15]

# road width
params['x1_ll'] = -30
params['x2_ll'] = -30
params['rd_bd_min'] = -3.7
params['x5_ll'] = 0
params['x1_ul'] = 75
params['x2_ul'] = 75
params['rd_bd_max'] = 3.7
params['x5_ul'] = 35

# target set specs
params['xr_tar_overtake']  = 10
params['xr_tar_lanekeep']  = -15

# input bounds and model parameters
params['accMax_R_sh']   = 3
params['vLatMax_R_sh']  = 4
params['thetaMax_R_sh'] = 0.6
# vlat nom and dev
params['vLgtNom_H_sh']   = 14.5
params['vLgtDev_H_sh']   = 1

params['vLatMax_H_sh']  = 1
params['talpha']    = 0.01




HJComp(params, idx)