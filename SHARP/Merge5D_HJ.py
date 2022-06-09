#!/usr/bin/python
# -*- coding: utf-8 -*-
#----------------------------------------------------------------------------
# Created By  : Haimin Hu
# Created Date: 06/08/2022
# version = '1.0'
# ---------------------------------------------------------------------------
""" Formulates and solves the reachability problem for Merge5D."""
# ---------------------------------------------------------------------------

import numpy as np
#import scipy.io as spio
# Utility functions to initialize the problem
from Grid.GridProcessing import Grid
from Shapes.ShapesFunctions import *
# Specify the file that includes dynamic systems
from dynamics.Merge5D_HRI import *
# Plot options
from plot_options import *
# Solver core
from solver import HJSolver
from Plots.plotting_utilities import *
import math


def shape_robot_target(g, params):
  """
  Defines the implicit surface of the Robot's target set.
    DYNAMICS:
               0      1    2     3      4
    states: [px_ef  px_er  py  vx_ef  vx_er]
    ctrls:  [ax     vy]
    dstbs:  [ax_f   ax_r]

    x0_dot = x3
    x1_dot = x4
    x2_dot = u1
    x3_dot = d0 - u0
    x4_dot = u0 - d1
  """

  # Set specifications.
  px_r_target_lb = params['px_r_target_lb']
  lane_center = params['lane_center']
  rd_bd_min = params['rd_bd_min']
  rd_bd_max = params['rd_bd_max']

  # ---------- Region 1: Merge ----------
  # px_r_target_lb <= px_er
  # px_r_target_lb <= px_ef
  # lane_center <= py <= rd_bd_max
  target_merge = ShapeRectangle(g, [px_r_target_lb, px_r_target_lb, lane_center, -np.inf, -np.inf],
                                   [np.inf,         np.inf,         rd_bd_max,    np.inf,  np.inf])

  # ---------- Region 2: Lane Keeping ----------
  # rd_bd_min <= py <= lane_center
  target_lanekeep = ShapeRectangle(g, [-np.inf, -np.inf, rd_bd_min,  -np.inf, -np.inf],
                                      [ np.inf,  np.inf, lane_center, np.inf,  np.inf])

  # Define the overall Robot's target set.
  target = Union(target_merge, target_lanekeep)

  return target


def shape_robot_avoid(g, params):
  """
  Defines the implicit surface of the Robot's avoid set.
    DYNAMICS:
               0      1    2     3      4
    states: [px_ef  px_er  py  vx_ef  vx_er]
    ctrls:  [ax     vy]
    dstbs:  [ax_f   ax_r]

    x0_dot = x3
    x1_dot = x4
    x2_dot = u1
    x3_dot = d0 - u0
    x4_dot = u0 - d1
  """

  # Set specifications.
  px_r_target_lb = params['px_r_target_lb']
  lane_center = params['lane_center']
  rd_bd_min = params['rd_bd_min']
  rd_bd_max = params['rd_bd_max']

  # ---------- Region 1: Upper Road Bounday ----------
  # rd_bd_max <= py
  data1 = ShapeRectangle(g, [-np.inf, -np.inf, rd_bd_max, -np.inf, -np.inf],
                            [ np.inf,  np.inf, np.inf,     np.inf,  np.inf])

  # ---------- Region 2: Fail to merge in front of the rear Human car ----------
  # px_er <= px_r_target_lb
  # lane_center <= py
  data2 = ShapeRectangle(g, [-np.inf,         -np.inf, lane_center, -np.inf, -np.inf],
                            [ px_r_target_lb,  np.inf, np.inf,       np.inf,  np.inf])

  # ---------- Region 3: Fail to merge behind the Human car in front ----------
  # px_ef - px_r_target_lb <= 0
  # lane_center <= py
  data3 = ShapeRectangle(g, [-np.inf, -np.inf,         lane_center, -np.inf, -np.inf],
                            [ np.inf,  px_r_target_lb, np.inf,       np.inf,  np.inf])

  # ---------- Region 4: Lower Road Bounday ----------
  # py <= rd_bd_min
  data4 = ShapeRectangle(g, [-np.inf, -np.inf, -np.inf,    -np.inf, -np.inf],
                            [ np.inf,  np.inf,  rd_bd_min,  np.inf,  np.inf])

  # Define the overall Robot's avoid set.
  avoid = Union(data1, data2)
  avoid = Union(avoid, data3)
  avoid = Union(avoid, data4)

  return avoid


def HJComp(params, save_result=True, file_name="HJ_value"):
  ''' 
  HJ Reachability computation.
    DYNAMICS:
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

  # Define the grid.
  grid_min = np.array([params['x0_min'], params['x1_min'], params['x2_min'], params['x3_min'], params['x4_min']])
  grid_max = np.array([params['x0_max'], params['x1_max'], params['x2_max'], params['x3_max'], params['x4_max']])

  HJ_dims = 5 # number of dimensions
  HJ_N = np.array(params['grid_points']) # number of grid points per dimension
  HJ_pdDims = [] # periodic dimensions

  g = Grid(grid_min, grid_max, HJ_dims, HJ_N, HJ_pdDims)

  # Define target and avoid sets.
  target = shape_robot_target(g, params)
  avoid = shape_robot_avoid(g, params)

  # Time horizon (backward reachability).
  tau = np.arange(start=0, stop=params['lookback_length'] + params['small_number'], step=params['tstep'])

  # Define dynamics.
  uMode = "min"
  dMode = "max"
  dynamics_obj = Merge5D_HRI([0,0,0,0,0], params['aMax_R'], params['aMin_R'], params['vyMax_R'], params['vyMin_R'],
    params['aMax_f'], params['aMin_f'], params['aMax_r'], params['aMin_r'], uMode, dMode)

  """
  Assign one of the following strings to `TargetSetMode` to specify the characteristics of computation
  "TargetSetMode":
  {
  "none" -> compute Backward Reachable Set, 
  "minVWithV0" -> min V with V0 (compute Backward Reachable Tube),
  "maxVWithV0" -> max V with V0,
  "minVWithVInit" -> compute min V over time,
  "maxVWithVInit" -> compute max V over time,
  "minVWithVTarget" -> min V with target set (if target set is different from initial V0)
  "maxVWithVTarget" -> max V with target set (if target set is different from initial V0)
  }

  (optional)
  Please specify this mode if you would like to add another target set, which can be an obstacle set
  for solving a reach-avoid problem
  "ObstacleSetMode":
  {
  "minVWithObstacle" -> min with obstacle set,
  "maxVWithObstacle" -> max with obstacle set
  }
  """

  compMethod = {"TargetSetMode": "minVWithVInit",
                "ObstacleSetMode": "maxVWithObstacle"}
  # compMethod = {"TargetSetMode": "minVWithV0"} #!!!

  # Solve for the HJ value function.
  input_sets = [target, avoid]
  # input_sets = target #!!!

  # print(np.shape(input_sets[0]))
  # print(np.shape(input_sets[1]))

  plot_option = PlotOptions(do_plot=False, plot_type="3d_plot", plotDims=[0,1,2], slicesCut=[0])
  valfun = HJSolver(dynamics_obj, g, input_sets, tau, compMethod, plot_option, accuracy="high")

  # Save into file
  if save_result:
    np.save(file_name + ".npy", valfun)
