import numpy as np
#import scipy.io as spio
# Utility functions to initialize the problem
from Grid.GridProcessing import Grid
from Shapes.ShapesFunctions import *
# Specify the  file that includes dynamic systems
from dynamics.DubinsCar5D_HRI import *
from dynamics.DubinsCar5D_2_HRI import *
# Plot options
from plot_options import *
# Solver core
from solverArray import HJSolver
from Plots.plotting_utilities import *
import math

## Shape functions for scenario

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
def ShapeMoveAvoid(grid, params, state, tau, static):
  print(state)
  #initial state
  x0,y0,v0 = state

  # Collision Specifications
  lgt_lb = params['avoid']['lgt_lb'] # -5.5
  lgt_ub = params['avoid']['lgt_ub'] # 5.5
  lat_bd = params['avoid']['lat_bd'] # 2

  # reduced controls of obstacle
  a_max = params['obst']['a_max']
  theta_max = params['obst']['theta_max'] # radians, for arc dynamics model
  v_lat_max = params['obst']['v_lat_max'] # for rect dynamics model
  mode = params['obst']['mode']
  
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

    # maybe obsolete
    if mode =='arc':
      # finding the outer points of the cylinder
      y_back_L = y0 + lat_bd + r_Min*np.sin(theta_max)
      y_front_L = y0 + lat_bd + r_Max*np.sin(theta_max)
      y_back_R = y0 - lat_bd - r_Min*np.sin(theta_max)
      y_front_R = y0 - lat_bd - r_Max*np.sin(theta_max)
      x_back = x0 + lgt_lb + r_Min*np.cos(theta_max)
      x_front = x0 + lgt_ub + r_Max*np.cos(theta_max)
     
      data_C_1 = ShapeRectangle(grid, [x_back, -np.inf, y_back_R, -np.inf, -np.inf], [x_front, np.inf, y_front_L, np.inf, np.inf])
      data_C_2 = ShapeRectangle(grid, [-np.inf, x_back, -np.inf, y_back_R, -np.inf], [np.inf, x_front, np.inf, y_front_L, np.inf])
      data_C = Union(data_C_1, data_C_2)
      data = Union(data, data_C)
    
    # Use Dynamics Model 1
    elif(mode == 'rect'):
      # how it can grow
      #print(x0)
      x_t_min = r_Min + lgt_lb
      x_t = r_Max + lgt_ub
      
      # behind ego, take up both lanes
      if x0 < -2:
        #print('back')
        y_t = params['rd_bd_max']
        y_t_min = params['rd_bd_min']
      
      # in front of ego, right lane
      elif y0 < 0:
        #print('right')
        y_t = 0
        y_t_min = params['rd_bd_min']

      # in front of ego, left lane
      else:
        #print('left')
        y_t = params['rd_bd_max']
        y_t_min = 0
      '''  
      y_t = y0 + lat_bd + v_lat_max*t
      y_t_min = y0 - lat_bd  - v_lat_max*t
      x_t_min = r_Min + x0 + lgt_lb
      x_t = r_Max + x0 + lgt_ub
      
      print(x_t)
      print(x_t_min)
      print(y_t)
      print(y_t_min)
      '''
      # data1: -x_r + x_h2 + lgt_lb <= 0
      data1 = -grid.vs[0] + x_t_min
      # data2: x_r - xH2 - lgt_ub <= 0
      data2 = grid.vs[0] - x_t
      # data3: y_R - y_H - lat_bd <= 0
      data3 = grid.vs[2] - y_t
      # data4: -y_R + y_H - lat_bd <= 0
      data4 = -grid.vs[2] + y_t_min

      # the final data is just the intersection of the four
      dataR = Intersection(data1, data2)
      dataR = Intersection(dataR,  data3)
      dataR = Intersection(dataR,  data4)

        # data1: -x_H1 + xH2 + lgt_lb <= 0
      data5 = -grid.vs[1] + x_t_min 
      # data2: x_H1 - x_H2 - lgt_ub <= 0
      data6 = grid.vs[1] - x_t
      # data3: y_H1 - y_H2 - lat_bd <= 0
      data7 = grid.vs[3] - y_t
      # data4: -y_H1 + y_H2 - lat_bd <= 0
      data8 = -grid.vs[3] + y_t_min

      # the final data is just the intersection of the four
      dataH = Intersection(data5, data6)
      dataH = Intersection(dataH, data7)
      dataH = Intersection(dataH, data8)
      data = Union(dataR, dataH)

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

    elif(mode == 'gap'):
      gapDist = x0
      direction = y0
      vel = v0

    else:
      return static

    data = Union(data,static)
    obst.append(data)
  obst = np.flip(obst,0)
  avoidSet = np.stack(obst, axis = -1) 
  return(avoidSet)




''' 
Defining the grid for the problem
'''
def HJComp(params, idx):
  # dictionary keeping track of extra arguments for the solver
  extraArgs = {}
  extraArgs['obstacles'] = None

  tau = np.arange(start=0, stop=params['lookback_length'] + params['small_number'], step=params['tstep'])
  # states                x_R              x_H                y_R                   y_H                 v_R              v_H
  HJ_grid_min = np.array([params['x1_ll'],  params['x2_ll'],   params['rd_bd_min'], params['rd_bd_min'], params['x5_ll']])
  HJ_grid_max = np.array([params['x1_ul'],  params['x2_ul'],   params['rd_bd_max'], params['rd_bd_max'], params['x5_ul']])

  HJ_dims = 5 # number of dimensions
  HJ_N = np.array(params['grid_points']) # number of grid points per dimension
  HJ_pdDims = [] # periodic dimensions

  # g = Grid(np.array([min, max, num_dim, pts_each_dim, pDim=[])
  g = Grid(HJ_grid_min, HJ_grid_max , HJ_dims, HJ_N, HJ_pdDims)

  '''
  making target and avoid sets
  '''
  # going off road boundaries - Human
  rd_bd_left_H  = ShapeRectangle(g, [-np.inf, -np.inf, -np.inf, params['rd_bd_max'], -np.inf], [np.inf, np.inf, np.inf, np.inf, np.inf])
  rd_bd_right_H = ShapeRectangle(g, [-np.inf, -np.inf, -np.inf, -np.inf, -np.inf], [np.inf, np.inf, np.inf, params['rd_bd_min'], np.inf])
  D_compl_H = Union(rd_bd_left_H, rd_bd_right_H)

  HJ_target = ShapeRobotTarget(g, params)
  HJ_target = Union(HJ_target, D_compl_H)

  # going off road boundaries - Robot
  rd_bd_left_R  = ShapeRectangle(g, [-np.inf, -np.inf, params['rd_bd_max']-0.5, -np.inf, -np.inf], [np.inf, np.inf, np.inf, np.inf, np.inf])
  rd_bd_right_R = ShapeRectangle(g, [-np.inf, -np.inf, -np.inf, -np.inf, -np.inf], [np.inf, np.inf, params['rd_bd_min']+0.5, np.inf, np.inf])
  D_compl_R = Union(rd_bd_left_R, rd_bd_right_R)


  # avoid set - Robot
  HJ_staticAvoid = ShapeRobotAvoid(g,params)
  HJ_staticAvoid = Union(HJ_staticAvoid, D_compl_R)


  HJ_avoid = ShapeMoveAvoid(g, params, params['obst']['state'][0], tau, HJ_staticAvoid)
  if len(params['obst']['state'])>1:
    for i in range(len(params['obst']['state'])-1):
        HJ_avoid = Union(ShapeMoveAvoid(g, params, params['obst']['state'][i+1], tau, HJ_staticAvoid), HJ_avoid)
  print("computed obstacle")
  extraArgs['obstacles'] = HJ_avoid

  '''
  compute the Reach-Avoid set
  '''
  uMode = "min"
  dMode = "max"
  HJ_minwith = "minVWithVInit"

  #my_car = DubinsCar5D_HRI([0,0,0,0,0], params['accMax_R_sh'], params['vLgtDev_H_sh'], params['vLgtNom_H_sh'],params['vLatMax_R_sh'], params['vLatMax_H_sh'], params['talpha'], uMode, dMode)
  my_car = DubinsCar5D_2_HRI([0,0,0,0,0], params['accMax_R_sh'], params['vLgtDev_H_sh'], params['vLgtNom_H_sh'],params['thetaMax_R_sh'], params['vLatMax_H_sh'], params['talpha'], uMode, dMode)


  """
  Assign one of the following strings to `compMethod` to specify the characteristics of computation
  "none" -> compute Backward Reachable Set
  "minVWithV0" -> compute Backward Reachable Tube
  "maxVWithVInit" -> compute max V over time
  "minVWithVInit" compute min V over time
  """
  '''
  po2 = PlotOptions("2d_plot", [0,2], [0,0,0])
  plot_isosurface(g, HJ_avoid[:,:,:,:,:,-1], po2)
  plot_isosurface(g, HJ_avoid[:,:,:,:,:,-8], po2)
  plot_isosurface(g, HJ_avoid[:,:,:,:,:,-20], po2)
  plot_isosurface(g, HJ_avoid[:,:,:,:,:,-30], po2)
  plot_isosurface(g, HJ_avoid[:,:,:,:,:,-40], po2)
  '''

  #HJSolver(dynamics object, grid, initial value function, time length, system objectives, plotting options, extra arguments)
  valfun = HJSolver(my_car, g, HJ_target, tau, HJ_minwith, None, extraArgs, idx)

