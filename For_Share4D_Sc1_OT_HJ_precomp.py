import numpy as np
import scipy.io as spio
# Utility functions to initialize the problem
from Grid.GridProcessing import Grid
from Shapes.ShapesFunctions import *
# Specify the  file that includes dynamic systems
from dynamics.DubinsCar4D_HRI import *
# Plot options
from plot_options import *
# Solver core
from solver import HJSolver

import math

## Note: this is adapted from user_definer.py and will try to visualize the value function at the end. 
# for this scenario this visualization won't work (haven't tried to fix it yet) and will raise an error.
# however, it will still save the final value function to your computer before the error is raised

""" USER INTERFACES
- Define grid
- Generate initial values for grid using shape functions
- Time length for computations
- Initialize plotting option
- Call HJSolver function
"""


# find the implicit surface of the Robot's avoid set:
def ShapeRobotAvoid(xs,params):
  # find the implicit surface of the Robot's avoid set: 
  # -x_r + lgt_lb <= 0        (data1), and
  #  x_r - lgt_ub <= 0        (data2), and
  #  y_R - y_H - lat_bd <= 0  (data3), and
  # -y_R + y_H - lat_bd <= 0  (data4)
  # state vector = [x_r, y_R, y_H, v_r]
  #
  # NOTICE: 
  # This function assumes zero sublevel set, i.e. negative inside,
  # positive outside. Add a negative sign if using this as an avoid set.

  # set specifications
  lgt_lb = params['avoid']['lgt_lb']
  lgt_ub = params['avoid']['lgt_ub']
  lat_bd = params['avoid']['lat_bd']

  # data1: -x_r + lgt_lb <= 0
  data1 = -xs[0] + lgt_lb

  # data2: x_r - lgt_ub <= 0
  data2 = xs[0] - lgt_ub

  # data3: y_R - y_H - lat_bd <= 0
  data3 = xs[1] - xs[2] - lat_bd

  # data4: -y_R + y_H - lat_bd <= 0
  data4 = -xs[1] + xs[2] - lat_bd

  # the final data is just the intersection of the four
  data = Intersection(data1, data2)
  data = Intersection(data,  data3)
  data = Intersection(data,  data4)

  return(data)



'''
Defining parameters for the scenario
'''
# dictionary keeping track of extra arguments for the solver
extraArgs = {}
extraArgs['obstacles'] = None
# dictionary tracking parameters of the problem
params = {}

# road width
params['rd_bd_min'] = -3.7
params['rd_bd_max'] = 3.7

# relative longitudinal distance
params['rd_len_lb'] = -18
params['rd_len_ub'] = 12

# desired cruising speed
params['vDes']  = 30

# relative velocity bounds
params['v_rel_lb']  = -10
params['v_rel_ub']  = 10

# target set specs
params['xr_tar_overtake']  = 10
params['xr_tar_lanekeep']  = params['rd_len_lb'] + 3

# avoid set specs
params['avoid'] ={'lgt_lb': -5.5, 'lgt_ub': 5.5, 'lat_bd':2.0}

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


''' 
Defining the grid for the problem
'''
# states                x_r                 y_R                   y_H                  v_r
HJ_grid_min = np.array([params['rd_len_lb'], params['rd_bd_min'], params['rd_bd_min'], params['v_rel_lb']])
HJ_grid_max = np.array([params['rd_len_ub'], params['rd_bd_max'], params['rd_bd_max'], params['v_rel_ub']])

HJ_dims = 4 # number of dimensions

HJ_N = np.array([41, 15, 15, 31]) # number of grid points per dimension

HJ_pdDims = [] # periodic dimensions

# g = Grid(np.array([min, max, num_dim, pts_each_dim, pDim=[])
g = Grid(HJ_grid_min, HJ_grid_max , HJ_dims, HJ_N, HJ_pdDims)


# optimized_dp lacks the "xs" field for their grid objects, constructing here
xs = np.empty([4,HJ_N[0],HJ_N[1],HJ_N[2],HJ_N[3]])
for l in range(HJ_N[3]):
  for k in range(HJ_N[2]):
    for j in range(HJ_N[1]):
        xs[0][:,j,k,l] = g.vs[0][:,0,0,0]
    for i in range(HJ_N[0]):
        xs[1][i,:,k,l] = g.vs[1][0,:,0,0]
  for j in range(HJ_N[1]):
    for i in range(HJ_N[0]):
        xs[2][i,j,:,l] = g.vs[2][0,0,:,0]

for k in range(HJ_N[2]):
  for j in range(HJ_N[1]):
    for i in range(HJ_N[0]):
        xs[3][i,j,k,:] = g.vs[3][0,0,0,:]


'''
making target and avoid sets
'''
inf = np.inf
# going off road boundaries - Robot
rd_bd_left_R  = ShapeRectangle(g, [-inf, params['rd_bd_max']-0.5, -inf, -inf], [inf, inf, inf, inf])
rd_bd_right_R = ShapeRectangle(g, [-inf, -inf, -inf, -inf], [inf, params['rd_bd_min']+0.5, inf, inf])
D_compl_R = Union(rd_bd_left_R, rd_bd_right_R)


# going off road boundaries - Human
rd_bd_left_H  = ShapeRectangle(g, [-inf, -inf, params['rd_bd_max'], -inf], [inf, inf, inf, inf])
rd_bd_right_H = ShapeRectangle(g, [-inf, -inf, -inf, -inf], [inf, inf, params['rd_bd_min'], inf])
D_compl_H = Union(rd_bd_left_H, rd_bd_right_H)

# avoid set - Robot
HJ_avoid = ShapeRobotAvoid(xs, params)
HJ_avoid = Union(HJ_avoid, D_compl_R)


# target set - Robot
# overtake
target_ot = ShapeRectangle(g, [params['xr_tar_overtake'], 0, -inf, -inf], [inf, params['rd_bd_max'], inf, inf])
# lanekeep
target_lk = ShapeRectangle(g, [-inf, params['rd_bd_min'], -inf, -inf], [params['xr_tar_lanekeep'], params['rd_bd_max'], inf, inf])
HJ_target = Union(target_ot, target_lk)
HJ_target = Union(HJ_target, D_compl_H)



'''
compute the Reach-Avoid set
'''
uMode = "min"
dMode = "max"
HJ_minwith = "minVWithVInit"

my_car = DubinsCar4D_HRI([0,0,0,0], params['accMax_R_sh'], params['accMax_H_sh'], params['vLatMax_R_sh'], params['vLatMax_H_sh'], params['talpha'], uMode, dMode)


# Look-back length and time step
lookback_length = 15.0 #15.0
t_step = 0.05
small_number = 1e-5
tau = np.arange(start=0, stop=lookback_length + small_number, step=t_step)


po2 = PlotOptions("3d_plot", [0,1,2], [])

"""
Assign one of the following strings to `compMethod` to specify the characteristics of computation
"none" -> compute Backward Reachable Set
"minVWithV0" -> compute Backward Reachable Tube
"maxVWithVInit" -> compute max V over time
"minVWithVInit" compute min V over time
"""

extraArgs['obstacles'] = HJ_avoid

# HJSolver(dynamics object, grid, initial value function, time length, system objectives, plotting options, extra arguments)
HJSolver(my_car, g, HJ_target, tau, HJ_minwith, po2, extraArgs)


