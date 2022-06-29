#!/usr/bin/python
# -*- coding: utf-8 -*-
#----------------------------------------------------------------------------
# Created By  : Haimin Hu
# Created Date: 06/09/2022
# version = '1.0'
# ---------------------------------------------------------------------------
""" Run this file to perform the HJ analysis for the Merge6D_Unicycle scenario."""
# ---------------------------------------------------------------------------

import numpy as np
import pickle

from SHARP.Merge6D_valueUtils import get_HJ_action, get_HJ_value
from Grid.GridProcessing import Grid

import sys
sys.path.append('../')
from ellReach.dyn_sys import *

import time


''' DYNAMICS
    Joint System:
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

    Robot Subsystem (linear):
             0   1   2
    states: [px  py  vx]
    ctrls:  [ax  vy]

    Robot Subsystem (nonlinear):
             0   1       2    3
    states: [px  py    theta  vx]
    ctrls:  [a   omega]

    Human Subsystem:
             0   1
    states: [px  vx]
    ctrls:  [ax]
'''

def compute_relative_state(xR, xH_f, xH_r):
  if len(xR) == 3:
    return np.array([[xH_f[0,0]-xR[0,0],
                      xR[0,0]-xH_r[0,0],
                      xR[1,0],
                      xH_f[1,0]-xR[2,0],
                      xR[2,0]-xH_r[1,0]]]).T
  elif len(xR) == 4:
    return np.array([[xH_f[0,0]-xR[0,0],
                      xR[0,0]-xH_r[0,0],
                      xR[1,0],
                      xH_f[1,0]-xR[3,0]*np.cos(xR[2,0]),
                      xR[3,0]*np.cos(xR[2,0])-xH_r[1,0],
                      xR[2,0]]]).T


def kinematic_bicycle_3D(xR, uR, ts):
#  The nonlinear kinematic bicycle model for simulation
#     State: [px py v]
#     Computed control:  [a vLat]
#     Converted control: [a theta] (theta = asin(vLat/v))
  vLat = uR[1,0]
  vx   = xR[2,0]
  theta = np.sin(np.maximum(np.minimum(vLat/vx,1),-1))

  return np.array([[xR[0,0] + ts*vx*np.cos(theta),
                    xR[1,0] + ts*vx*np.sin(theta),
                    vx + ts*uR[0,0]]]).T


def unicycle(xR, uR, ts):
#  The nonlinear unicycle model for simulation
#     State: [px, py, theta, v_body]
#     Control:  [a, \omega]
  return np.array([[xR[0,0] + ts*xR[3,0]*np.cos(xR[2,0]),
                    xR[1,0] + ts*xR[3,0]*np.sin(xR[2,0]),
                    xR[2,0] + ts*uR[1,0],
                    xR[3,0] + ts*uR[0,0]]]).T


def transform_controller(xR_4D, uR_3D, is_project=False, uR_4D_lb=None,
 uR_4D_ub=None, theta_lb=-np.pi/8, theta_ub=np.pi/8):
# Transforms the linear 3D model controller (ax, vLat) to
# the nonlinear 4D model controller (a, w), where a is the acceleration in the
# body frame
# xR_3D = [px, py, vx]
# uR_3D = [ax, vLat]
# xR_4D = [px, py, theta, v_body]
# uR_4D = [a, \omega]

  K_steering = 2.7016
  theta_target = 1/(np.sin(uR_3D[1,0]/xR_4D[3,0]))
  theta_target = np.maximum(np.minimum(theta_target, theta_ub), theta_lb)
  uR_4D = np.array([[uR_3D[0,0]/(np.cos(xR_4D[2,0])),
                     -K_steering*(xR_4D[2,0] - theta_target)]]).T
  if is_project:
    uR_4D = np.maximum(np.minimum(uR_4D, uR_4D_ub), uR_4D_lb)
  return uR_4D




# Main simulation script.
T  = 150
ts = 0.1
L  = 2.7 # wheel base of the car

# Load HJ value function and parameters.
valfun = np.load("Merge6D_Unicycle_valfun.npy", allow_pickle=True)
with open('Merge6D_Unicycle_params.pkl', 'rb') as f:
  params = pickle.load(f)

# Create grids for HJ.
with open('Merge6D_Unicycle_grid.pkl', 'rb') as f:
  g = pickle.load(f)
# grid_min = np.array([params['x0_min'], params['x1_min'], params['x2_min'],
#   params['x3_min'], params['x4_min'], params['x5_min']])
# grid_max = np.array([params['x0_max'], params['x1_max'], params['x2_max'],
#   params['x3_max'], params['x4_max'], params['x5_max']])
# HJ_dims = 6 # number of dimensions
# HJ_N = np.array(params['grid_points']) # number of grid points per dimension
# HJ_pdDims = [5] # periodic dimensions

# g = Grid(grid_min, grid_max, HJ_dims, HJ_N, HJ_pdDims)

# grid_space =\
#  [np.linspace(params['x0_min'], params['x0_max'], params['grid_points'][0]),
#   np.linspace(params['x1_min'], params['x1_max'], params['grid_points'][1]),
#   np.linspace(params['x2_min'], params['x2_max'], params['grid_points'][2]),
#   np.linspace(params['x3_min'], params['x3_max'], params['grid_points'][3]),
#   np.linspace(params['x4_min'], params['x4_max'], params['grid_points'][4])]

# Load problem parameters.
lane_center = params['lane_center']
rd_bd_min = params['rd_bd_min']
rd_bd_max = params['rd_bd_max']

# System matrices ***DT=0.1***
AH = np.array([[1, 0.1], [0, 1]])
BH = np.array([[0.005, 0.1]]).T

AR = np.array([[1, 0, 0.1], [0, 1, 0], [0, 0, 1]])
BR = np.array([[0.005, 0], [0, 0.1], [0.1, 0]])

# car_R = DynSys('DTLTI', AR, BR)
car_H = DynSys('DTLTI', AH, BH)

# Displays the systems.
# car_H1.display()
# car_H2.display()
# car_R.display()

# ---------- Trajectory simulation ----------
# Cruising speed.
vx_cruise = 2

# Robot's control bounds.
uR_lb = np.array([[params['aMin_R'], params['omegaMin_R']]]).T
uR_ub = np.array([[params['aMax_R'], params['omegaMax_R']]]).T

# Initial states.
xR_0   = np.array([[0, -2.0, 0, 1]]).T
xH_f_0 = np.array([[20, vx_cruise]]).T
xH_r_0 = np.array([[5,  vx_cruise]]).T

xR   = xR_0
xH_f = xH_f_0
xH_r = xH_r_0
xRel = compute_relative_state(xR, xH_f, xH_r)

# Target states for the robot.
px_overtake = 1.5*params['px_r_target_lb']
py_R_des = 3.7/2
vx_R_des = vx_cruise

# LQR gain for the robot.
# KR = np.array([[2.5857, 0.0000, 3.4434], [0.0000, 2.7016, 0.0000]])
KR = np.array([[0.3030, 0.0000, 0.8353], [0.0000, 2.7016, 0.0000]])
KH = np.array([[0.3030, 0.8353]])

# Main loop.
xR_traj   = xR_0
xH_f_traj = xH_f_0
xH_r_traj = xH_r_0
xRel_traj = xRel
sh_traj   = []
for k in range(T):

  # Define tracking states.
  xR_des   = np.array([[xH_r[0,0] + px_overtake, py_R_des, vx_R_des]]).T
  xH_f_des = np.array([[xR[0,0] + px_overtake, vx_R_des]]).T
  xH_r_des = np.array([[xR[0,0] - px_overtake, vx_R_des]]).T

  # Compute nominal controls.
  xR_3D = np.array([[xR[0,0], xR[1,0], xR[3,0]*np.cos(xR[2,0])]]).T
  uR_LQR = -KR @ (xR_3D - xR_des)
  if xRel[1,0] <= 0.0: # lane keeping for the early stage of simulation
    uR_LQR[1,0] = 0
  uR = transform_controller(xR, uR_LQR, True, uR_lb, uR_ub)
  # uR = np.maximum(np.minimum(uR, uR_ub), uR_lb) # projected LQR control

  human_noise_magnitude = 1.5
  uH_f_noise_bias = -0.25
  uH_r_noise_bias = 0.0
  uH_f_noise = human_noise_magnitude*(1 + uH_f_noise_bias
    - 2*np.random.rand(1).reshape(1,))
  uH_R_noise = human_noise_magnitude*(1 + uH_r_noise_bias
    - 2*np.random.rand(1).reshape(1,))

  uH_f = -KH @ (xH_f - xH_f_des) + uH_f_noise
  uH_r = -KH @ (xH_r - xH_r_des) + uH_R_noise

  # Evolve the system using nominal controls.
  xR_tmp = unicycle(xR, uR, ts)
  # xR_tmp = kinematic_bicycle_3D(xR, uR, ts)
  # xR_tmp = car_R.step(xR, uR)
  xH_f = car_H.step(xH_f, uH_f)
  xH_r = car_H.step(xH_r, uH_r)

  # Check if shielding is needed.
  xRel = compute_relative_state(xR_tmp, xH_f, xH_r)
  HJ_value = get_HJ_value(xRel, valfun, g)
  print("HJ_value:", HJ_value)
  if HJ_value >= -0.6 and (xR[1,0] >= -1.5): # and (xRel[1,0]>= -5): # shielding needed #!!!
    
    start = time.time()
    uR_sh = get_HJ_action(xRel, valfun, g, params, uMode="min")
    end = time.time()
    print("Shielding computation time:", end - start)

    #----- TO BE DELETED -----
    if xRel[1,0] >= 1:
      uR_sh[0,0] = uR[0,0]
    #-------------------------
    uR = uR_sh

    print("Shielding action:", uR.T, "\n")
    xR = unicycle(xR, uR, ts)
    sh_traj.append(True)
  else:
    xR = xR_tmp
    sh_traj.append(False)

  # xR = xR_tmp
  # sh_traj.append(False)

  # Update relative states.
  xRel = compute_relative_state(xR, xH_f, xH_r)
  xRel_traj = np.concatenate((xRel_traj, xRel), axis=1)

  # Store states and controls.
  xR_traj   = np.concatenate((xR_traj, xR), axis=1)
  xH_f_traj = np.concatenate((xH_f_traj, xH_f), axis=1)
  xH_r_traj = np.concatenate((xH_r_traj, xH_r), axis=1)

  if k == 0:
    uR_traj   = uR
    uH_f_traj = uH_f
    uH_r_traj = uH_r
  else:
    uR_traj   = np.concatenate((uR_traj, uR), axis=1)
    uH_f_traj = np.concatenate((uH_f_traj, uH_f), axis=1)
    uH_r_traj = np.concatenate((uH_r_traj, uH_r), axis=1)


# print(xRel_traj[0:3,:])
print(xR_traj)
# print('')
# print(xH_f_traj)
# print('')
# print(xH_r_traj)
print('')
print(sh_traj)

np.save("Plots/xR_traj.npy",   xR_traj)
np.save("Plots/uR_traj.npy",   uR_traj)
np.save("Plots/xH_f_traj.npy", xH_f_traj)
np.save("Plots/xH_r_traj.npy", xH_r_traj)
np.save("Plots/xRel_traj.npy", xRel_traj)
np.save("Plots/sh_traj.npy",   sh_traj)

