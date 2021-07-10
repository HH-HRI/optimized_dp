import numpy as np
#import scipy.io as spio
# Utility functions to initialize the problem
from Grid.GridProcessing import Grid
from Shapes.ShapesFunctions import *
# Specify the  file that includes dynamic systems
from dynamics.DubinsCar6D_HRI import *
# Plot options
from plot_options import *
# Solver core
from solver import HJSolver
from Plots.plotting_utilities import *
import math


""" USER INTERFACES
- Define grid
- Generate initial values for grid using shape functions
- Time length for computations
- Initialize plotting option
- Call HJSolver function
"""

# find the implicit surface of the Robot's avoid set:
def ShapeRobotTarget(g,params):
  xr_tar_overtake = params['xr_tar_overtake']
  xr_tar_lanekeep = params['xr_tar_lanekeep']
  rd_bd_max = params['rd_bd_max']
  rd_bd_min = params['rd_bd_min']

  # -x_R + x_H + xr_tar_overtake <= 0
  data1 = -g.vs[0] + g.vs[1] + xr_tar_overtake
  # y_R - rd_bd_max <= 0
  data2 = g.vs[2] - rd_bd_max
  # -y_R <= 0
  data3 = -g.vs[2]

  target_ot = Intersection(data1,data2)
  target_ot = Intersection(target_ot, data3)

  # x_R - x_H - xr_tar_lanekeep <= 0
  data4 = g.vs[0] - g.vs[1] - xr_tar_lanekeep
  # y_R - rd_bd_max <= 0
  data5 = g.vs[2] - rd_bd_max
  # -y_R + rd_bd_min <= 0
  data6 = -g.vs[2] + rd_bd_min

  target_lk = Intersection(data4, data5)
  target_lk = Intersection(target_lk, data6)
  # either lanekeep OR overtake
  data = Union(target_lk, target_ot)

  return data

# find the implicit surface of the Robot's avoid set:
def ShapeRobotAvoid(g,params):
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
  data1 = -(g.vs[0] - g.vs[1]) + lgt_lb
  # data2: x_r - lgt_ub <= 0
  data2 = g.vs[0] - g.vs[1] - lgt_ub
  # data3: y_R - y_H - lat_bd <= 0
  data3 = g.vs[2] - g.vs[3] - lat_bd
  # data4: -y_R + y_H - lat_bd <= 0
  data4 = -g.vs[2] + g.vs[3] - lat_bd

  # the final data is just the intersection of the four
  data = Intersection(data1, data2)
  data = Intersection(data,  data3)
  data = Intersection(data,  data4)

  return(data)

# mode: arc, rect, None 
def ShapeMoveAvoid(grid, params, tau, static, mode = None):
  #initial state
  x0,y0,v0 = [params['obst']['x0'], params['obst']['y_H2'], params['obst']['v_H2']]

  # Collision Specifications
  lgt_lb = params['avoid']['lgt_lb'] # -5.5
  lgt_ub = params['avoid']['lgt_ub'] # 5.5
  lat_bd = params['avoid']['lat_bd'] # 2

  # reduced controls of obstacle
  a_max = params['obst']['a_max']
  theta_max = params['obst']['theta_max'] # radians, for arc dynamics model
  v_lat_max = params['obst']['v_lat_max'] # for rect dynamics model
  
  talpha = params['talpha'] # friction parameter

  # obstacle list to be reversed and stacked
  obst = []
  for timestep in range(len(tau)):
    t = tau[timestep]

    # Solved for v in the equation V = V0 + (accel - talpha*V)*t with +/- a_max
    v_t_min = (v0 - a_max*t)/(1+talpha*t)
    v_t = (v0 + a_max*t)/(1+talpha*t)
    # max and min distance traveled based on constant +/- a_max for all t
    r_Min = x0 + v0*t + .5*(-a_max - talpha*v_t_min)*(t**2)
    r_Max = x0 + v0*t + .5*(a_max - talpha*v_t)*(t**2)


    if mode =='arc':
      # corners of collision set, with center at x0, y0
      front_left = [x0+lgt_ub, x0+lgt_ub, y0+lat_bd, y0+lat_bd]
      front_right = [x0+lgt_ub, x0+lgt_ub, y0-lat_bd, y0-lat_bd]      

      back_left = [x0+lgt_lb, x0+lgt_lb, y0+lat_bd, y0+lat_bd]
      back_right = [x0+lgt_lb, x0+lgt_lb, y0-lat_bd, y0-lat_bd]

      # max distance traveled from front
      bigCyl_L1= CylinderShape(grid,[1,3,4,5], front_left, r_Max) 
      bigCyl_R1 = CylinderShape(grid,[1,3,4,5], front_right, r_Max) 
      bigCyl_L2= CylinderShape(grid,[0,2,4,5], front_left, r_Max) 
      bigCyl_R2 = CylinderShape(grid,[0,2,4,5], front_right, r_Max) 

      bigCyl_R = Intersection(bigCyl_R1, bigCyl_R2)
      bigCyl_L = Intersection(bigCyl_L1, bigCyl_L2)
      
      # min distance traveled from back
      smallCyl_L1 = CylinderShape(grid,[1,3,4,5], back_left, r_Min)
      smallCyl_R1 = CylinderShape(grid,[1,3,4,5], back_right, r_Min)
      smallCyl_L2 = CylinderShape(grid,[0,2,4,5], back_left, r_Min)
      smallCyl_R2 = CylinderShape(grid,[0,2,4,5], back_right, r_Min)

      smallCyl_L = Intersection(smallCyl_L1, smallCyl_L2)
      smallCyl_R = Intersection(smallCyl_R1, smallCyl_R2)

      # negative smallCyl marks all points not in the cylinder as obstacles,
      # intersection with bigCyl leaves hollow ring
      data_L = Intersection(bigCyl_L, -smallCyl_L)
      data_R = Intersection(bigCyl_R, -smallCyl_R)
      data = smallCyl_L
      
      # only care about the outer half of cylinder
      upperSpace1 = (y0 + lat_bd) - grid.vs[2]
      upperSpace2 = (y0 + lat_bd) - grid.vs[3]
      upperSpace = Union(upperSpace1, upperSpace2)
      data_L = Intersection(data_L, upperSpace)
      lowerSpace1 = -(y0 - lat_bd) + grid.vs[2]
      lowerSpace2 = -(y0 - lat_bd) + grid.vs[3]
      lowerSpace = Intersection(lowerSpace1, lowerSpace2)
      data_R = Intersection(data_R, lowerSpace)
      
      # outer halves of the cylinders with connecting rectangle
      data = Union(data_L, data_R)
      
      # only care about in front of vehicle
      slab_front1 = x0+lgt_lb - grid.vs[0] 
      slab_front2 = x0+lgt_lb - grid.vs[1] 
      slab_front = Union(slab_front1, slab_front2)
      data = Intersection(slab_front, data)
      
      # finding the outer points of the cylinder
      y_back_L = y0 + lat_bd + r_Min*np.sin(theta_max)
      y_front_L = y0 + lat_bd + r_Max*np.sin(theta_max)
      y_back_R = y0 - lat_bd - r_Min*np.sin(theta_max)
      y_front_R = y0 - lat_bd - r_Max*np.sin(theta_max)
      x_back = x0 + lgt_lb + r_Min*np.cos(theta_max)
      x_front = x0 + lgt_ub + r_Max*np.cos(theta_max)

      # slope calculations
      dy_L = y_front_L - y_back_L
      dx_L = x_front - x_back
      m_L = dy_L / dx_L
      m_R = -m_L # due to symmetry

      # bounds on orientation
      slope_L_1 = (grid.vs[2] - (y_back_L)) - m_L*(grid.vs[0] - x_back)
      slope_L_2 = (grid.vs[3] - (y_back_L)) - m_L*(grid.vs[1] - x_back)
      slope_L = Intersection(slope_L_1, slope_L_2)

      slope_R_1 = -(grid.vs[2] - (y_back_R)) + m_R*(grid.vs[0] - x_back)
      slope_R_2 = -(grid.vs[3] - (y_back_R)) + m_R*(grid.vs[1] - x_back)
      slope_R = Intersection(slope_R_1, slope_R_2)

      data = Intersection(data, slope_L)
      data = Intersection(data, slope_R)

      # compensating for grid inaccuracies
      buffer = 1
      print(x0+lgt_lb+r_Min)
      print(x0+lgt_ub+r_Max)
      # between cylinders is rectangle indicating theta = 0 during the time interval
      data_C_1 = ShapeRectangle(grid, [x0+lgt_lb+r_Min, -np.inf, y0-lat_bd - buffer, -np.inf, -np.inf, -np.inf], [x0+lgt_ub+r_Max, np.inf, y0+lat_bd + buffer, np.inf, np.inf, np.inf])
      data_C_2 = ShapeRectangle(grid, [-np.inf, x0+lgt_lb+r_Min, -np.inf, y0-lat_bd - buffer, -np.inf, -np.inf], [np.inf, x0+lgt_ub+r_Max, np.inf, y0+lat_bd + buffer, np.inf, np.inf])
      data_C = Union(data_C_1, data_C_2)
      data = Union(data, data_C)
    
    # Use Dynamics Model 1
    elif(mode == 'rect'):
      # strafing for dynamics model 1
      y_t = y0 + lat_bd + v_lat_max*t
      y_t_min = y0 - lat_bd  - v_lat_max*t
      x_t_min = r_Min + x0 + lgt_lb
      x_t = r_Max + x0 + lgt_ub
      data1 = ShapeRectangle(grid, [-np.inf, x_t_min, -np.inf, y_t_min, -np.inf, -np.inf], [np.inf, x_t, np.inf, y_t, np.inf, np.inf])
      data2 = ShapeRectangle(grid, [x_t_min, -np.inf, y_t_min, -np.inf, -np.inf, -np.inf], [x_t, np.inf, y_t, np.inf, np.inf, np.inf])
      data = Union(data1,data2)

    elif(mode == 'basic'): 
      v_H2 = v0
      y_H2 = y0
      x_H2 = x0 + v_H2*tau[timestep]

      # data1: -x_r + x_h2 + lgt_lb <= 0
      data1 = -grid.vs[0] + x_H2 + lgt_lb 
      # data2: x_r - xH2 - lgt_ub <= 0
      data2 = grid.vs[0] - x_H2 - lgt_ub
      # data3: y_R - y_H - lat_bd <= 0
      data3 = grid.vs[2] - y_H2 - lat_bd
      # data4: -y_R + y_H - lat_bd <= 0
      data4 = -grid.vs[2] + y_H2 - lat_bd

      # the final data is just the intersection of the four
      dataR = Intersection(data1, data2)
      dataR = Intersection(dataR,  data3)
      dataR = Intersection(dataR,  data4)

        # data1: -x_H1 + xH2 + lgt_lb <= 0
      data5 = -grid.vs[1] + x_H2 + lgt_lb 
      # data2: x_H1 - x_H2 - lgt_ub <= 0
      data6 = grid.vs[1] - x_H2 - lgt_ub
      # data3: y_H1 - y_H2 - lat_bd <= 0
      data7 = grid.vs[3] - y_H2 - lat_bd
      # data4: -y_H1 + y_H2 - lat_bd <= 0
      data8 = -grid.vs[3] + y_H2 - lat_bd

      # the final data is just the intersection of the four
      dataH = Intersection(data5, data6)
      dataH = Intersection(dataH, data7)
      dataH = Intersection(dataH, data8)
      data = Union(dataR, dataH)

    else:
      return static

    data = Union(data,static)
    obst.append(data)
  obst = np.flip(obst,0)
  avoidSet = np.stack(obst, axis = -1) 
  return(avoidSet)



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
params['avoid'] = {'lgt_lb': -5.5, 'lgt_ub': 5.5, 'lat_bd':2.0}
params['obst']  = {'x0': 35, 'v_H2':10, 'y_H2':1.85, 'a_max':0.5, 'theta_max':0.05, 'v_lat_max':0.1}

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

params['x1_ll'] = 0
params['x2_ll'] = 0
params['x5_ll'] = 0
params['x6_ll'] = 0
params['x1_ul'] = 130
params['x2_ul'] = 130
params['x5_ul'] = 35
params['x6_ul'] = 35

''' 
Defining the grid for the problem
'''
# states                x_R              x_H                y_R                   y_H                 v_R              v_H
HJ_grid_min = np.array([params['x1_ll'],  params['x2_ll'],   params['rd_bd_min'], params['rd_bd_min'], params['x5_ll'], params['x6_ll']])
HJ_grid_max = np.array([params['x1_ul'],  params['x2_ul'],   params['rd_bd_max'], params['rd_bd_max'], params['x5_ul'], params['x6_ul']])

HJ_dims = 6 # number of dimensions

HJ_N = np.array([80, 80, 25, 25, 25, 25]) # number of grid points per dimension

HJ_pdDims = [] # periodic dimensions

# g = Grid(np.array([min, max, num_dim, pts_each_dim, pDim=[])
g = Grid(HJ_grid_min, HJ_grid_max , HJ_dims, HJ_N, HJ_pdDims)


'''
making target and avoid sets
'''
# going off road boundaries - Human
rd_bd_left_H  = ShapeRectangle(g, [-np.inf, -np.inf, -np.inf, params['rd_bd_max'], -np.inf, -np.inf], [np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])
rd_bd_right_H = ShapeRectangle(g, [-np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf], [np.inf, np.inf, np.inf, params['rd_bd_min'], np.inf, np.inf])
D_compl_H = Union(rd_bd_left_H, rd_bd_right_H)

HJ_target = ShapeRobotTarget(g, params)
HJ_target = Union(HJ_target, D_compl_H)

# going off road boundaries - Robot
rd_bd_left_R  = ShapeRectangle(g, [-np.inf, -np.inf, params['rd_bd_max']-0.5, -np.inf, -np.inf, -np.inf], [np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])
rd_bd_right_R = ShapeRectangle(g, [-np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf], [np.inf, np.inf, params['rd_bd_min']+0.5, np.inf, np.inf, np.inf])
D_compl_R = Union(rd_bd_left_R, rd_bd_right_R)


# avoid set - Robot
HJ_staticAvoid = ShapeRobotAvoid(g,params)
HJ_staticAvoid = Union(HJ_staticAvoid, D_compl_R)


# Look-back length and time step
lookback_length = 5.0 #15.0
t_step = 0.1
small_number = 1e-5
tau = np.arange(start=0, stop=lookback_length + small_number, step=t_step)
HJ_avoid = ShapeMoveAvoid(g, params, tau, HJ_staticAvoid, mode = 'basic')
print("computed obstacle")


'''
compute the Reach-Avoid set
'''
uMode = "min"
dMode = "max"
HJ_minwith = "minVWithVInit"

my_car = DubinsCar6D_HRI([0,0,0,0,0,0], params['accMax_R_sh'], params['accMax_H_sh'], params['vLatMax_R_sh'], params['vLatMax_H_sh'], params['talpha'], uMode, dMode)


#po2 = PlotOptions("3d_plot", [0,1,2], [])

"""
Assign one of the following strings to `compMethod` to specify the characteristics of computation
"none" -> compute Backward Reachable Set
"minVWithV0" -> compute Backward Reachable Tube
"maxVWithVInit" -> compute max V over time
"minVWithVInit" compute min V over time
"""

extraArgs['obstacles'] = HJ_avoid

#HJSolver(dynamics object, grid, initial value function, time length, system objectives, plotting options, extra arguments)
valfun = HJSolver(my_car, g, HJ_target, tau, HJ_minwith, None, extraArgs)

'''
deriv = []
for timestep in range(len(tau)):
  spat_deriv = np.gradient(valfun[:,:,:,:,:,:,timestep], axis = [2,4])
  deriv.append(spat_deriv)

HJ_Derivs = np.stack(deriv, axis = -1) 
print(np.shape(HJ_Derivs))
np.save("6D_spat_deriv", HJ_Derivs)
'''

#po2 = PlotOptions("2d_plot", [0,2], [10,7,0,0])
#plot_isosurface(g, HJ_avoid[:,:,:,:,:,:,0], po2)
