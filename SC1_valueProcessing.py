import numpy as np
import scipy.interpolate

# inputs: np array specifying joint state, final value function, column vector with each entry being the linspace specifying the given dimension
# list specifying max and min controls
# output: optimal controls
def getAction(state, value, grid_space, controls):

    spat_deriv =  np.gradient(value)

    # provides upper and lower bounds
    x1 = grid_space[0]
    x2 = grid_space[1]
    x3 = grid_space[2]
    x4 = grid_space[3]

    if len(controls) > 2: 
        accel_max = controls[0]
        accel_min = controls[1]
        vlat_max = controls[2]
        vlat_min = controls[3]
    elif len(controls) == 2:
        accel_max = controls[0]
        vlat_max = controls[1]
        accel_min = -accel_max
        vlat_min = -vlat_max
    else:
        print("invalid control dimensions")


    # state needs to be twodimensional
    if state.ndim == 1:
        state.shape = (1,np.size(state))
    
    p1interp = scipy.interpolate.RegularGridInterpolator((x1, x2, x3, x4), spat_deriv[0])
    p2interp = scipy.interpolate.RegularGridInterpolator((x1, x2, x3, x4), spat_deriv[1])
    p3interp = scipy.interpolate.RegularGridInterpolator((x1, x2, x3, x4), spat_deriv[2])
    p4interp = scipy.interpolate.RegularGridInterpolator((x1, x2, x3, x4), spat_deriv[3])


    p1 = p1interp(state)
    p2 = p2interp(state)
    p3 = p3interp(state)
    p4 = p4interp(state)
            
  

    # find optimal control for the robot
    if (p4 >= 0):
        accel = accel_min
    else:
        accel = accel_max
            
    if (p2 >= 0):
        vlat = vlat_min
    else:
        vlat = vlat_max

    return [accel, vlat]

# inputs: np array specifying joint state, final value function, column vector with each entry being the linspace specifying the given dimension
# output: state value
def getValue(state, value, grid_space):

    x1 = grid_space[0]
    x2 = grid_space[1]
    x3 = grid_space[2]
    x4 = grid_space[3]

    # state needs to be twodimensional
    if state.ndim == 1:
        state.shape = (1,np.size(state))

    valinterp = scipy.interpolate.RegularGridInterpolator((x1, x2, x3, x4), value)

    return valinterp(state)