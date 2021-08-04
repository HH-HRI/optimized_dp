import numpy as np
import scipy.io as spio
import scipy.interpolate
import heterocl as hcl
from computeGraphs.graph_6D import *
from Grid.GridProcessing import Grid
from SC1_valueProcessing import *

valfun = np.load('new_center_final.npy')
ctrls = np.load('controls.npy')
print(np.shape(ctrls))

u1 = ctrls[0,...]
u2 = ctrls[1,...]
print(np.shape(u1))
print(np.shape(u2))

print(u1[0,0,0,0,0,0,0])
'''
# CHANGE LATER
# need spatial derivs at each timestep
spat_deriv = np.gradient(valfun)

# waymo data
MATLAB = spio.loadmat("XH.mat")
XH = MATLAB['XH']




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

x1 = np.linspace(params['x1_ll'],params['x1_ul'], np.size(valfun,0), endpoint = True)
x2 = np.linspace(params['x2_ll'],params['x2_ul'], np.size(valfun,1), endpoint = True)
x3 = np.linspace(params['x3_ll'],params['x3_ul'], np.size(valfun,2), endpoint = True)
x4 = np.linspace(params['x4_ll'],params['x4_ul'], np.size(valfun,3), endpoint = True)
x5 = np.linspace(params['x5_ll'],params['x5_ul'], np.size(valfun,4), endpoint = True)
x6 = np.linspace(params['x6_ll'],params['x6_ul'], np.size(valfun,5), endpoint = True)



gridspace = [x1, x2, x3, x4, x5, x6]



counter = 0
value = 1

for d in range(1000):
    

    # initial state of human
    px_h, py_h, v_h = XH[:,0]


    valinterp = scipy.interpolate.RegularGridInterpolator((x1, x2, x3, x4, x5, x6), valfun)

    # finding initial state for robot
    while value > -0.0:
        px_r = np.random.uniform()*(x1[-1] - x1[0]) + x1[0]
        py_r= np.random.uniform()*(x3[-1] - x3[0]) + x3[0]
        v_r = np.random.uniform()*(x5[-1] - x5[0]) + x5[0]
        
        # randomly generating a state
        state = np.array([px_r, px_h, py_r, py_h, v_r, v_h])
        value = valinterp(state)




    for k in range(np.size(XH,1)):
        # state of human at each time step
        px_h, py_h, v_h = XH[:,k]

        relx1 = px_r - px_h

        # target set
        # xr_tar_overtake = 10, 
        if(relx1 > 10 or relx1 < -15):
            print("safely")
            counter +=1
            break
        
        
        if(px_r < x1[0]):
            print("x1 out of bounds")
            break        

        if(relx2 > x2[-1] or relx2 < x2[0]):
            print("x2 out of bounds")
            break

        if(relx3 > x3[-1] or relx3 < x3[0]):
            print("x3 out of bounds")
            break
        

        # relative velocity out of bounds postive
        # but ahead of human
        if(relx4 > x4[-1] and relx1 > 0):
            print("pseudo")
            counter +=1
            break

        # relative velocity out of bounds negative
        # but behind of human
        if(relx4 < x4[0] and relx1 < 0):
            counter +=1
            break 

        state = np.array([[px_r, px_h, py_r, py_h, v_r, v_h]])

        val = valinterp(state)

        if val > 0:
            print("pseudo")
            print("left safe set")
            print("state")
            print(state)
            break


        state = np.array([relx1, relx2, relx3, relx4])
        gridspace = [x1, x2, x3,x4]
        accel, vlat = getAction(state, valfun, gridspace, [3,-3,3,-3])

        p1interp = scipy.interpolate.RegularGridInterpolator((x1, x2, x3, x4), spat_deriv[0])
        p2interp = scipy.interpolate.RegularGridInterpolator((x1, x2, x3, x4), spat_deriv[1])
        p3interp = scipy.interpolate.RegularGridInterpolator((x1, x2, x3, x4), spat_deriv[2])
        p4interp = scipy.interpolate.RegularGridInterpolator((x1, x2, x3, x4), spat_deriv[3])


        p1 = p1interp(np.array([[relx1,relx2,relx3,relx4]]))
        p2 = p2interp(np.array([[relx1,relx2,relx3,relx4]]))
        p3 = p3interp(np.array([[relx1,relx2,relx3,relx4]]))
        p4 = p4interp(np.array([[relx1,relx2,relx3,relx4]]))
            
  

        # find optimal control for the robot
        if (p4 >= 0):
            accel = -a_max
        else:
            accel = a_max
            
        if (p2 >= 0):
            vlat = -v_lat_max
        else:
            vlat = v_lat_max'''
'''
        px_r_last = px_r
        v_r_last = v_r

        px_r = px_r + 0.2*v_r
        py_r = py_r + 0.2*vlat
        v_r =  v_r + 0.2*(accel - talpha*v_r)


print(counter)'''