import heterocl as hcl
import numpy as np
import time

from Plots.plotting_utilities import *
from argparse import ArgumentParser
from computeGraphs.graph_3D import *
from computeGraphs.graph_4D import *
from computeGraphs.graph_5D import *
from computeGraphs.graph_6D import *
from computeGraphs.getControls_6D import *
from TimeToReach.TimeToReach_3D import  *
from TimeToReach.TimeToReach_4D import  *
from valueIteration.value_iteration_3D import *
from valueIteration.value_iteration_4D import *
from valueIteration.value_iteration_5D import *
from valueIteration.value_iteration_6D import *

def solveValueIteration(MDP_obj):
    print("Welcome to optimized_dp \n")
    # Initialize the HCL environment
    hcl.init()
    hcl.config.init_dtype = hcl.Float()

    ########################################## INITIALIZE ##########################################

    # Convert the python array to hcl type array
    V_opt = hcl.asarray(np.zeros(MDP_obj._ptsEachDim))
    intermeds = hcl.asarray(np.ones(MDP_obj._actions.shape[0]))
    trans = hcl.asarray(MDP_obj._trans)
    gamma = hcl.asarray(MDP_obj._gamma)
    epsilon = hcl.asarray(MDP_obj._epsilon)
    count = hcl.asarray(np.zeros(1))
    maxIters = hcl.asarray(MDP_obj._maxIters)
    actions = hcl.asarray(MDP_obj._actions)
    bounds = hcl.asarray(MDP_obj._bounds)
    goal = hcl.asarray(MDP_obj._goal)
    ptsEachDim = hcl.asarray(MDP_obj._ptsEachDim)
    sVals = hcl.asarray(np.zeros([MDP_obj._bounds.shape[0]]))
    iVals = hcl.asarray(np.zeros([MDP_obj._bounds.shape[0]]))
    interpV = hcl.asarray(np.zeros([1]))
    useNN = hcl.asarray(MDP_obj._useNN)

    print(MDP_obj._bounds.shape[0])
    print(np.zeros([MDP_obj._bounds.shape[0]]))
    if MDP_obj._bounds.shape[0] == 3:
        fillVal = hcl.asarray(MDP_obj._fillVal)
        f = value_iteration_3D(MDP_obj)
    if MDP_obj._bounds.shape[0] == 4:
        f = value_iteration_4D(MDP_obj)
    if MDP_obj._bounds.shape[0] == 5:
        f = value_iteration_5D(MDP_obj)
    if MDP_obj._bounds.shape[0] == 6:
        f = value_iteration_6D(MDP_obj)

    # Build the graph and use the executable
    # Now use the executable
    t_s = time.time()
    if MDP_obj._bounds.shape[0] == 3:
        f(V_opt, actions, intermeds, trans, interpV, gamma, epsilon, iVals, sVals, bounds, goal, ptsEachDim, count,
            maxIters, useNN, fillVal)
    else:
        f(V_opt, actions, intermeds, trans, interpV, gamma, epsilon, iVals, sVals, bounds, goal, ptsEachDim, count,
          maxIters, useNN)
    t_e = time.time()

    V = V_opt.asnumpy()
    c = count.asnumpy()
    print("Finished in ", int(c[0]), " iterations")
    print("Took        ", t_e - t_s, " seconds")

    # # Write results to file
    # if (MDP_obj.dir_path):
    #     dir_path = MDP_obj.dir_path
    # else:
    #     dir_path = "./hcl_value_matrix_test/"
    #
    # if (MDP_obj.file_name):
    #     file_name = MDP_obj.file_name
    # else:
    #     file_name = "hcl_value_iteration_" + str(int(c[0])) + "_iterations_by" + (
    #         "_Interpolation" if MDP_obj._useNN[0] == 0 else "_NN")
    # MDP_obj.writeResults(V, dir_path, file_name, just_values=True)
    return V


def HJSolver(dynamics_obj, grid, init_value, tau, compMethod, plot_option, extraArgs):
    print("Welcome to optimized_dp \n")

    ################### PARSING ARGUMENTS FROM USERS #####################

    parser = ArgumentParser()
    parser.add_argument("-p", "--plot", default=False, type=bool)
    # # Print out LLVM option only
    # parser.add_argument("-l", "--llvm", default=False, type=bool)
    args = parser.parse_args()

    hcl.init()
    hcl.config.init_dtype = hcl.Float(32)

    ################# INITIALIZE DATA TO BE INPUT INTO EXECUTABLE ##########################

    print("Initializing\n")
    # if there are no obstacles, initialize g0 to array of inf
    # post integration steps won't matter
    if (isinstance(extraArgs.get('obstacles'), np.ndarray)):
        print('defining obstacles')
        g0_dim = extraArgs['obstacles'].ndim
        g0 = extraArgs['obstacles']
        if g0_dim > grid.dims:
            g0_i = g0[...,0]
        else:
            g0_i = g0
    else:
        print("no obstacles!")
        g0_dim = grid.dims
        # initialize with inf so max w/ -g0 is always returns other array
        g0_i = np.ones(tuple(grid.pts_each_dim))*np.inf

    V_0 = hcl.asarray(np.maximum(init_value, -g0_i))
    V_1 = hcl.asarray(np.zeros(tuple(grid.pts_each_dim)))
    l0  = hcl.asarray(init_value)
    probe = hcl.asarray(np.zeros(tuple(grid.pts_each_dim)))

    list_x1 = np.reshape(grid.vs[0], grid.pts_each_dim[0])
    list_x2 = np.reshape(grid.vs[1], grid.pts_each_dim[1])
    list_x3 = np.reshape(grid.vs[2], grid.pts_each_dim[2])
    if grid.dims >= 4:
        list_x4 = np.reshape(grid.vs[3], grid.pts_each_dim[3])
    if grid.dims >= 5:
        list_x5 = np.reshape(grid.vs[4], grid.pts_each_dim[4])
    if grid.dims >= 6:
        list_x6 = np.reshape(grid.vs[5], grid.pts_each_dim[5])


    # Convert to hcl array type
    list_x1 = hcl.asarray(list_x1)
    list_x2 = hcl.asarray(list_x2)
    list_x3 = hcl.asarray(list_x3)
    if grid.dims >= 4:
        list_x4 = hcl.asarray(list_x4)
    if grid.dims >= 5:
        list_x5 = hcl.asarray(list_x5)
    if grid.dims >= 6:
        list_x6 = hcl.asarray(list_x6)

    # Get executable, obstacle check initial value function
    # in principle, the inserted 2 will vary depending on dimensions/#controls to the system
    if grid.dims == 3:
        # saving optimal controls
        Ctrl = hcl.asarray(np.zeros(tuple(np.insert(grid.pts_each_dim,0,3)))) 
        solve_pde = graph_3D(dynamics_obj, grid, compMethod)
    if grid.dims == 4:
        # saving optimal controls
        Ctrl = hcl.asarray(np.zeros(tuple(np.insert(grid.pts_each_dim,0,4))))
        solve_pde = graph_4D(dynamics_obj, grid, compMethod)
    if grid.dims == 5:
        # saving optimal controls
        Ctrl = hcl.asarray(np.zeros(tuple(np.insert(grid.pts_each_dim,0,5))))
        solve_pde = graph_5D(dynamics_obj, grid, compMethod)
    if grid.dims == 6:
        # saving optimal controls
        Ctrl = hcl.asarray(np.zeros(tuple(np.insert(grid.pts_each_dim,0,2)))) # up to six, using 2 for this case
        solve_pde = graph_6D(dynamics_obj, grid, compMethod)
        init_ctrl = getControls_6D(dynamics_obj, grid)
  
    # Print out code for different backend
    #print(solve_pde)

    ################ USE THE EXECUTABLE ############
    # Variables used for timing
    execution_time = 0
    iter = 0
    tNow = tau[0]
    print("Started running\n")
    # initial value array first
    valfuns = [V_0.asnumpy()] #going to keep track of each timestep
    init_ctrl(Ctrl, V_0, list_x1, list_x2, list_x3, list_x4, list_x5, list_x6)
    ctrls = [Ctrl.asnumpy()]
    for i in range (1, len(tau)):
        #tNow = tau[i-1]
        t_minh= hcl.asarray(np.array((tNow, tau[i])))
        while tNow <= tau[i] - 1e-4:
            tmp_arr = V_0.asnumpy()
            # Start timing
            iter += 1
            start = time.time()

            if g0_dim > grid.dims:
                    g0_i = g0[...,i]

            # Run the execution and pass input into graph
            if grid.dims == 3:
                solve_pde(V_1, V_0, list_x1, list_x2, list_x3, t_minh, l0)
            if grid.dims == 4:
                solve_pde(V_1, V_0, list_x1, list_x2, list_x3, list_x4, t_minh, l0, probe)
            if grid.dims == 5:
                solve_pde(V_1, V_0, list_x1, list_x2, list_x3, list_x4, list_x5 ,t_minh, l0)
            if grid.dims == 6:
                # hard coded such that Ctrl takes only 2 inputs rather than 6
                solve_pde(V_1, V_0, list_x1, list_x2, list_x3, list_x4, list_x5, list_x6, t_minh, l0)

            tNow = np.asscalar((t_minh.asnumpy())[0])

            # Calculate computation time
            execution_time += time.time() - start

            # Some information printing
            print(t_minh)
            print("Computational time to integrate (s): {:.5f}".format(time.time() - start))
        
        # obstacle calculation after each timestep
        V_1 = hcl.asarray(np.maximum(V_1.asnumpy(),-g0_i))
        init_ctrl(Ctrl, V_1, list_x1, list_x2, list_x3, list_x4, list_x5, list_x6)

        # after each timestep adding value array to list
        valfuns.append(V_1.asnumpy())
        ctrls.append(Ctrl.asnumpy())
    
    ##FOR BACKWARDS REACHABILITY NEED TO FLIP OVER LAST AXIS
    valfuns = np.flip(valfuns,0)
    ctrls = np.flip(ctrls,0)

    # puts value arrays along new axis
    V_1 = np.stack(valfuns, axis=-1)
    ctrls = np.stack(ctrls, axis=-1)

    # Time info printing
    print("Total kernel time (s): {:.5f}".format(execution_time))
    print("Finished solving\n")

    # Save into file
    np.save("new_center_final.npy", V_1)
    #np.save("new_center_final.npy", V_1.asnumpy())
    np.save("controls.npy", ctrls)

    print(np.sum(V_1 < 0))
    #print(np.sum(V_1.asnumpy() < 0))

    ##################### PLOTTING #####################
    if args.plot:
        # plot Value table when speed is maximum
        plot_isosurface(grid, V_1, plot_option)
        #plot_isosurface(grid, V_1.asnumpy(), plot_option)
        #plot_isosurface(g, my_V, [0, 1, 3], 10)
    return V_1
    #return V_1.asnumpy()

def TTRSolver(dynamics_obj, grid, init_value, epsilon, plot_option):
    print("Welcome to optimized_dp \n")
    ################# INITIALIZE DATA TO BE INPUT INTO EXECUTABLE ##########################

    print("Initializing\n")
    hcl.init()
    hcl.config.init_dtype = hcl.Float(32)

    # Convert initial distance value function to initial time-to-reach value function
    init_value[init_value < 0] = 0
    init_value[init_value > 0] = 100
    V_0 = hcl.asarray(init_value)
    prev_val = np.zeros(init_value.shape)

    # Re-shape states vector
    list_x1 = np.reshape(grid.vs[0], grid.pts_each_dim[0])
    list_x2 = np.reshape(grid.vs[1], grid.pts_each_dim[1])
    list_x3 = np.reshape(grid.vs[2], grid.pts_each_dim[2])
    if grid.dims >= 4:
        list_x4 = np.reshape(grid.vs[3], grid.pts_each_dim[3])
    if grid.dims >= 5:
        list_x5 = np.reshape(grid.vs[4], grid.pts_each_dim[4])
    if grid.dims >= 6:
        list_x6 = np.reshape(grid.vs[5], grid.pts_each_dim[5])

    # Convert states vector to hcl array type
    list_x1 = hcl.asarray(list_x1)
    list_x2 = hcl.asarray(list_x2)
    list_x3 = hcl.asarray(list_x3)
    if grid.dims >= 4:
        list_x4 = hcl.asarray(list_x4)
    if grid.dims >= 5:
        list_x5 = hcl.asarray(list_x5)
    if grid.dims >= 6:
        list_x6 = hcl.asarray(list_x6)

    # Get executable

    if grid.dims == 3:
        solve_TTR = TTR_3D(dynamics_obj, grid)
    if grid.dims == 4:
        solve_TTR = TTR_4D(dynamics_obj, grid)
    if grid.dims == 5:
        solve_TTR = TTR_5D(my_car, g)
    if grid.dims == 6:
        solve_TTR = TTR_6D(my_car, g)
    print("Got Executable\n")

    # Print out code for different backend
    # print(solve_pde)

    ################ USE THE EXECUTABLE ############
    error = 1000
    count = 0
    while error > epsilon:
        print("Iteration: {} Error: {}".format(count, error))
        count += 1
        if grid.dims == 3:
            solve_TTR(V_0, list_x1, list_x2, list_x3)
        if grid.dims == 4:
            solve_TTR(V_0, list_x1, list_x2, list_x3, list_x4)
        if grid.dims == 5:
            solve_TTR(V_0, list_x1, list_x2, list_x3, list_x4, list_x5)
        if grid.dims == 6:
            solve_TTR(V_0, list_x1, list_x2, list_x3, list_x4, list_x5, list_x6 )

        error = np.max(np.abs(prev_val - V_0.asnumpy()))
        prev_val = V_0.asnumpy()

    print("Finished solving\n")

    ##################### PLOTTING #####################
    plot_isosurface(grid, V_0.asnumpy(), plot_option)
    return V_0.asnumpy()

