#!/usr/bin/python
# -*- coding: utf-8 -*-
#----------------------------------------------------------------------------
# Created By  : Haimin Hu
# Created Date: 06/13/2022
# version = '1.0'
# ---------------------------------------------------------------------------
""" Computes the optimal control and evaluates the value function."""
# ---------------------------------------------------------------------------

import numpy as np
import scipy.interpolate


def get_HJ_action(state, value, grid, params, uMode="min"):
    """
    Input:
        - state: np array specifying joint state
        - value: final value function
        - grid_space: column vector with each entry being the linspace 
                      specifying the given dimension
        - params: problem parameters including control bounds
        - uMode: "min" or "max"
    Output:
        - optimal controls
    """

    # Spatial gradients of the value function.
    spat_deriv = np.gradient(value)

    # # Define grids.
    # x0 = grid_space[0]
    # x1 = grid_space[1]
    # x2 = grid_space[2]
    # x3 = grid_space[3]
    # x4 = grid_space[4]

    # # Reshape the state vector.
    # if state.ndim == 1:
    #     state.shape = (1, params['dim_state'])
    
    # p0interp = scipy.interpolate.RegularGridInterpolator((x0, x1, x2, x3, x4), 
    #     spat_deriv[0])
    # p1interp = scipy.interpolate.RegularGridInterpolator((x0, x1, x2, x3, x4), 
    #     spat_deriv[1])
    # p2interp = scipy.interpolate.RegularGridInterpolator((x0, x1, x2, x3, x4), 
    #     spat_deriv[2])
    # p3interp = scipy.interpolate.RegularGridInterpolator((x0, x1, x2, x3, x4), 
    #     spat_deriv[3])
    # p4interp = scipy.interpolate.RegularGridInterpolator((x0, x1, x2, x3, x4), 
    #     spat_deriv[4])

    # p0 = p0interp(state)
    # p1 = p1interp(state)
    # p2 = p2interp(state)
    # p3 = p3interp(state)
    # p4 = p4interp(state)

    # p0 = grid.get_value(spat_deriv[0], state)
    # p1 = grid.get_value(spat_deriv[1], state)
    # p2 = grid.get_value(spat_deriv[2], state)
    p3 = grid.get_value(spat_deriv[3], state)
    p4 = grid.get_value(spat_deriv[4], state)
    p5 = grid.get_value(spat_deriv[5], state)
            
    # Find the optimal control.
    if uMode == "min":
        if (p4 - p3)*np.cos(state[5]) >= 0:
            u0_opt = params['aMin_R']
        else:
            u0_opt = params['aMax_R']

        if p5 >= 0:
            u1_opt = params['omegaMin_R']
        else:
            u1_opt = params['omegaMax_R']

    elif uMode == "max":
        if (p4 - p3)*np.cos(state[5]) < 0:
            u0_opt = params['aMin_R']
        else:
            u0_opt = params['aMax_R']

        if p5 < 0:
            u1_opt = params['omegaMin_R']
        else:
            u1_opt = params['omegaMax_R']

    return np.array(([[u0_opt, u1_opt]])).T


def get_HJ_value(state, value, grid):
    """
    Input:
        - state: np array specifying joint state
        - value: final value function
        - grid_space: column vector with each entry being the linspace 
                      specifying the given dimension
    Output:
        - state value
    """

    # x0 = grid_space[0]
    # x1 = grid_space[1]
    # x2 = grid_space[2]
    # x3 = grid_space[3]
    # x4 = grid_space[4]

    # # Reshape the state vector.
    # if state.ndim == 1:
    #     state.shape = (1, params['dim_state'])

    # valinterp = scipy.interpolate.RegularGridInterpolator((x0, x1, x2, x3, x4),
    #     value)

    # print(state)

    # return valinterp(state)

    return grid.get_value(value, state)