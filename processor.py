import numpy as np
import scipy.io as spio
import scipy.interpolate
import heterocl as hcl
from computeGraphs.graph_4D import *
from Grid.GridProcessing import Grid
from SC1_valueProcessing import *

#valfun = np.load('41-41-41-41value.npy')
valfun = np.load('new_center_final.npy')
#spio.savemat("valfun2.mat", {'valfun2':valfun})
##MATLAB = spio.loadmat("safeset.mat")
MATLAB1 = spio.loadmat("derivs1.mat")
derivs1= MATLAB1['derivs1']
MATLAB2 = spio.loadmat("derivs2.mat")
derivs2= MATLAB2['derivs2']
MATLAB3 = spio.loadmat("derivs3.mat")
derivs3= MATLAB3['derivs3']
MATLAB4 = spio.loadmat("derivs4.mat")
derivs4= MATLAB4['derivs4']
#print(np.shape(derivs1))
#safesetdata= MATLAB['safesetnoavoid']
#print(valfun.shape)
#print(safesetdata[10,10,10,10])

spat_deriv = np.gradient(valfun)

datadiff = derivs4 - spat_deriv[3]
#print(max(datadiff.min(), datadiff.max(), key=abs))



#print(spat_deriv[0])

#print(spat_deriv[1][1,1,1,1])


MATLAB = spio.loadmat("XH.mat")
XH = MATLAB['XH']

#print(np.size(XH,0))

value = 1

params = {}
params['rd_len_lb'] = -18
params['rd_len_ub'] = 12
params['rd_bd_min'] = -3.7
params['rd_bd_max'] = 3.7
params['v_rel_lb']  = -10
params['v_rel_ub']  = 10

a_max = 3
v_lat_max = 3

x1 = np.linspace(params['rd_len_lb'],params['rd_len_ub'], np.size(valfun,0), endpoint = True)
x2 = np.linspace(params['rd_bd_min'],params['rd_bd_max'], np.size(valfun,1), endpoint = True)
x3 = np.linspace(params['rd_bd_min'],params['rd_bd_max'], np.size(valfun,2), endpoint = True)
x4 = np.linspace(params['v_rel_lb'],params['v_rel_ub'], np.size(valfun,3), endpoint = True)



gridspace = [x1, x2, x3,x4]



counter = 0
for d in range(1000):
    

    # initial state of human
    px_h, py_h, v_h = XH[:,0]


    valinterp = scipy.interpolate.RegularGridInterpolator((x1, x2, x3, x4), valfun)


    while value > -0.0:
        
        '''
        i = np.random.randint(np.size(valfun, 0))
        j = np.random.randint(np.size(valfun, 1))
        # don't get to pick k
        k = np.random.randint(np.size(valfun, 2))
        l = np.random.randint(np.size(valfun, 3))
        x1rand = x1[i]
        x2rand = x2[j]
        x4rand = x4[l] 
        '''
        relx1 = np.random.uniform()*(x1[-1] - x1[0]) + x1[0]
        relx2 = np.random.uniform()*(x2[-1] - x2[0]) + x2[0]
        relx3 = py_h
        relx4 = np.random.uniform()*(x4[-1] - x4[0]) + x4[0]

        state = np.array([relx1, relx2, relx3, relx4])
        value = getValue(state, valfun, gridspace )



    # finding initial robot state
    px_r = relx1 + px_h
    py_r = relx2
    v_r = relx4 + v_h


    talpha = 0.01




    for k in range(np.size(XH,1)):
        relx1_last = relx1
        px_h_last = px_h
        v_h_last = v_h

        # state of human at each time step
        px_h, py_h, v_h = XH[:,k]
    

        # finding joint state from robot 
        relx1 = px_r - px_h
        relx2 = py_r
        relx3 = py_h
        relx4 = v_r - v_h
        
        # target set
        # xr_tar_overtake = 10, 
        if(relx1 > 10 or relx1 < -15):
            print("safely")
            counter +=1
            break

        if(relx1 < x1[0]):
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

        

        val = valinterp(np.array([[relx1, relx2, relx3, relx4]]))

        if val > 0:
            print("pseudo")
            print("left safe set")
            print("joint state")
            print(relx1)
            print(relx2)
            print(relx3)
            print(relx4)


            break


        state = np.array([relx1, relx2, relx3, relx4])
        gridspace = [x1, x2, x3,x4]
        accel, vlat = getAction(state, valfun, gridspace, [3,-3,3,-3])


        '''
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

        px_r_last = px_r
        v_r_last = v_r

        px_r = px_r + 0.2*v_r
        py_r = py_r + 0.2*vlat
        v_r =  v_r + 0.2*(accel - talpha*v_r)


print(counter)