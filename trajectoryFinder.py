import numpy as np
import scipy.io as spio

# defining parameters for scenario
lookback_length = 9.0 #15.0
t_step = 0.1
small_number = 1e-5
tau = np.arange(start=0, stop=lookback_length + small_number, step=t_step)

# grids for interpolation
HJ6d_N = [100; 100; 15; 15; 15; 15]

params = {}
params['rd_len_lb'] = -18
params['rd_len_ub'] = 12
params['rd_bd_min'] = -3.7
params['rd_bd_max'] = 3.7
params['v_rel_lb']  = -10
params['v_rel_ub']  = 10

params['x1_ll'] = 0
params['x2_ll'] = 0
params['x5_ll'] = 0
params['x6_ll'] = 0
params['x1_ul'] = 130
params['x2_ul'] = 130
params['x5_ul'] = 35
params['x6_ul'] = 35

params['talpha'] = 0.01
params['a_max'] = 3
params['v_lat_max'] = 3

params['avoid'] = {'lgt_lb': -5.5, 'lgt_ub': 5.5, 'lat_bd':2.0}
params['obst']  = {'x0': 35, 'v_H2':10, 'y_H2':1.85}

params['overtake'] = 18
params['lanekeep'] = -15

# initial state of obstacle
x0_H2 = params['obst']['x0']
y0_H2 = params['obst']['y_H2']
v_H2 = params['obst']['v_H2']

# controls
accMax_R  = 3
vLatMax_R = 3
accMax_H  = 1
vLatMax_H = 1
talpha = 0.01


# waymo data
MATLAB = spio.loadmat("XH.mat")
XH = MATLAB['XH']
XH[1,:] = XH[0,:] + 15

# value at each timestep
data_6d = np.load('new_center_final.npy')
derivs_6d = np.load("6D_spat_deriv")

x1 = np.linspace(params['x1_ll'],params['x1_ul'], np.size(data_6d,0), endpoint = True)
x2 = np.linspace(params['x2_ll'],params['x2_ul'], np.size(data_6d,1), endpoint = True)
x3 = np.linspace(params['x3_ll'],params['x3_ul'], np.size(data_6d,2), endpoint = True)
x4 = np.linspace(params['x4_ll'],params['x4_ul'], np.size(data_6d,3), endpoint = True)
x5 = np.linspace(params['x5_ll'],params['x5_ul'], np.size(data_6d,4), endpoint = True)
x6 = np.linspace(params['x6_ll'],params['x6_ul'], np.size(data_6d,5), endpoint = True)

gridspace = [x1, x2, x3, x4, x5, x6]

## Animating 6D trajectory

# initial state of human
x_H, y_H, v_H = XH[:,0]


valinterp = scipy.interpolate.RegularGridInterpolator((x1, x2, x3, x4, x5, x6), data_6d[:,:,:,:,:,:,0])

# finding initial state for robot3
while value > -0.0:
    x_R = np.random.uniform()*(x1[-1] - x1[0]) + x1[0]
    y_R= np.random.uniform()*(x3[-1] - x3[0]) + x3[0]
    v_R = np.random.uniform()*(x5[-1] - x5[0]) + x5[0]
        
    # randomly generating a state
    state = np.array([x_R, x_H, y_R, y_H, v_R, v_H])
    value = valinterp(state)


    if x_R-x_H > overtake
        value = 1
    elif x_R-px_y < lanekeep
        value = 1    
    elif x_R > x0_H2
        value = 1
    elif x_R > x_H
        value = 1 



x_R_6D = np.zeros(len(tau))
y_R_6D = np.zeros(len(tau))

x_H1_6D = np.zeros(len(tau))
y_H1_6D = np.zeros(len(tau))   

x_H2_6D = np.zeros(len(tau))
y_H2_6D = np.zeros(len(tau)) 


# calculating states and adding them to trajectory vectors
steps = 0
for k in range(len(XH1)):
    # XH timestep is 0.2 seconds, tau timestep is 0.1 seconds
    i = 2*k
    spatDeriv = derivs_6d[:,:,:,:,:,:,:,i]
    
    spatDeriv_x1 = spatDeriv[0,:,:,:,:,:,:]
    spatDeriv_x2 = spatDeriv[1,:,:,:,:,:,:]
    spatDeriv_x3 = spatDeriv[2,:,:,:,:,:,:]
    spatDeriv_x4 = spatDeriv[3,:,:,:,:,:,:]
    spatDeriv_x5 = spatDeriv[4,:,:,:,:,:,:]
    spatDeriv_x6 = spatDeriv[5,:,:,:,:,:,:]
    
    % adding state to vector
    x_R_6D[k] = x_R
    x_H1_6D[k] = x_H
    y_R_6D[k] = y_R
    y_H1_6D[k] = y_H
    
    # how human 2 moves
    x_H2_6D[k] = x0_H2 + v_H2*(tau(i))
    y_H2_6D[k] = y0_H2
    
    
    if x_R > x1_ul
        print('x_R out of bounds')
        print('step')
        print(k)
        steps = k
        break
    elif x_H > x2_ul
        print('x_H out of bounds')
        print('step')
        print(k)
        steps = k
        break
    elif v_R > x5_ul
        print('v_R out of bounds')
        print('step')
        print(k)
        steps = k
        break
    elif v_R < x5_ll
        print('v_R out of bounds')
        print('step')
        print(k)
        steps = k
        break
    elif v_H > x6_ul
        print('v_H out of bounds')
        print('step')
        print(k)
        steps = k
        break
    elif v_H < x6_ll
        print('v_H out of bounds')
        print('step')
        print(k)
        steps = k
        break      
    elif x_R-x_H > overtake
        print('overtake')
        print(k)
        steps = k
        break
    elif x_R-x_H < lanekeep
        print('lanekeep')
        print(k)
        steps = k
        break

    end
    
    print('value')
    state = np.array([x_R, x_H, y_R, y_H, v_R, v_H])
    print(valinterp(state))
    
    p3interp = scipy.interpolate.RegularGridInterpolator((x1, x2, x3, x4, x5, x6), spat_deriv[2])
    p5interp = scipy.interpolate.RegularGridInterpolator((x1, x2, x3, x4, x5, x6), spat_deriv[4])

    deriv_x3 = p3interp(state)
    deriv_x5 = p5interp(state)


    print('deriv3')
    print(deriv_x3)

    print('deriv5')
    print(deriv_x5)

    
    # find optimal control/dist
    if deriv_x5 > 0
        accOpt_R = -accMax_R
    else
        accOpt_R = accMax_R
    end 
    if deriv_x3 > 0
        vLatOpt_R = -vLatMax_R
    else
        vLatOpt_R = vLatMax_R
    end 

    '''
    % Dynamics:
    %    \dot{x}_1 = x5
    %    \dot{x}_2 = x6
    %    \dot{x}_3 = u2
    %    \dot{x}_4 = d2
    %    \dot{x}_5 = u1 - talpha * x5
    %    \dot{x}_6 = d1 - talpha * x6    
    '''

    tstep = 0.2
    x_R = x_R + tstep*v_R
    y_R = y_R + tstep*vLatOpt_R
    v_R = v_R + tstep*(accOpt_R - talpha*v_R)
    

    x_H = XH1(1,k+1)
    y_H = XH1(2,k+1)
    v_H = XH1(3,k+1)

print('found trajectory')

robot_traj = np.stack([x_R_6D,y_R_6D], -1)
H1_traj = np.stack([x_H1_6D, y_H1_6D], -1)
H2_traj = np.stack([x_H2_6D, y_H2_6D], -1)
