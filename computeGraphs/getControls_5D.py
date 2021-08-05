import heterocl as hcl
import time
from computeGraphs.CustomGraphFunctions import *

##############################  5D DERIVATIVE FUNCTIONS #############################
def spa_derivX5_5d(i, j, k, l, m, V, g):  # Left -> right == Outer Most -> Inner Most
    left_deriv = hcl.scalar(0, "left_deriv")
    right_deriv = hcl.scalar(0, "right_deriv")
    if 5 not in g.pDim:
        with hcl.if_(m == 0):
            left_boundary = hcl.scalar(0, "left_boundary")
            left_boundary[0] = V[i, j, k, l, m] + my_abs(V[i, j, k, l, m + 1] - V[i, j, k, l, m]) * my_sign(
                V[i, j, k, l, m])
            left_deriv[0] = (V[i, j, k, l, m] - left_boundary[0]) / g.dx[4]
            right_deriv[0] = (V[i, j, k, l, m + 1] - V[i, j, k, l, m]) / g.dx[4]
        with hcl.elif_(m == V.shape[4] - 1):
            right_boundary = hcl.scalar(0, "right_boundary")
            right_boundary[0] = V[i, j, k, l, m] + my_abs(V[i, j, k, l, m] - V[i, j, k, l, m - 1]) * my_sign(
                V[i, j, k, l, m])
            left_deriv[0] = (V[i, j, k, l, m] - V[i, j, k, l, m - 1]) / g.dx[4]
            right_deriv[0] = (right_boundary[0] - V[i, j, k, l, m]) / g.dx[4]
        with hcl.elif_(m != 0 and m != V.shape[4] - 1):
            left_deriv[0] = (V[i, j, k, l, m] - V[i, j, k, l, m - 1]) / g.dx[4]
            right_deriv[0] = (V[i, j, k, l, m + 1] - V[i, j, k, l, m]) / g.dx[4]
        return left_deriv[0], right_deriv[0]
    else:
        with hcl.if_(m == 0):
            left_boundary = hcl.scalar(0, "left_boundary")
            left_boundary[0] = V[i, j, k, l, V.shape[4] - 1]
            left_deriv[0] = (V[i, j, k, l, m] - left_boundary[0]) / g.dx[4]
            right_deriv[0] = (V[i, j, k, l, m + 1] - V[i, j, k, l, m]) / g.dx[4]
        with hcl.elif_(m == V.shape[4] - 1):
            right_boundary = hcl.scalar(0, "right_boundary")
            right_boundary[0] = V[i, j, k, l, 0]
            left_deriv[0] = (V[i, j, k, l, m] - V[i, j, k, l, m - 1]) / g.dx[4]
            right_deriv[0] = (right_boundary[0] - V[i, j, k, l, m]) / g.dx[4]
        with hcl.elif_(m != 0 and m != V.shape[4] - 1):
            left_deriv[0] = (V[i, j, k, l, m] - V[i, j, k, l, m - 1]) / g.dx[4]
            right_deriv[0] = (V[i, j, k, l, m + 1] - V[i, j, k, l, m]) / g.dx[4]
        return left_deriv[0], right_deriv[0]


def spa_derivX4_5d(i, j, k, l, m, V, g):  # Left -> right == Outer Most -> Inner Most
    left_deriv = hcl.scalar(0, "left_deriv")
    right_deriv = hcl.scalar(0, "right_deriv")
    if 4 not in g.pDim:
        with hcl.if_(l == 0):
            left_boundary = hcl.scalar(0, "left_boundary")
            left_boundary[0] = V[i, j, k, l, m] + my_abs(V[i, j, k, l + 1, m] - V[i, j, k, l, m]) * my_sign(
                V[i, j, k, l, m])
            left_deriv[0] = (V[i, j, k, l, m] - left_boundary[0]) / g.dx[3]
            right_deriv[0] = (V[i, j, k, l + 1, m] - V[i, j, k, l, m]) / g.dx[3]
        with hcl.elif_(l == V.shape[3] - 1):
            right_boundary = hcl.scalar(0, "right_boundary")
            right_boundary[0] = V[i, j, k, l, m] + my_abs(V[i, j, k, l, m] - V[i, j, k, l - 1, m]) * my_sign(
                V[i, j, k, l, m])
            left_deriv[0] = (V[i, j, k, l, m] - V[i, j, k, l - 1, m]) / g.dx[3]
            right_deriv[0] = (right_boundary[0] - V[i, j, k, l, m]) / g.dx[3]
        with hcl.elif_(l != 0 and l != V.shape[3] - 1):
            left_deriv[0] = (V[i, j, k, l, m] - V[i, j, k, l - 1, m]) / g.dx[3]
            right_deriv[0] = (V[i, j, k, l + 1, m] - V[i, j, k, l, m]) / g.dx[3]
        return left_deriv[0], right_deriv[0]
    else:
        with hcl.if_(l == 0):
            left_boundary = hcl.scalar(0, "left_boundary")
            left_boundary[0] = V[i, j, k, V.shape[3] - 1, m]
            left_deriv[0] = (V[i, j, k, l, m] - left_boundary[0]) / g.dx[3]
            right_deriv[0] = (V[i, j, k, l + 1, m] - V[i, j, k, l, m]) / g.dx[3]
        with hcl.elif_(l == V.shape[3] - 1):
            right_boundary = hcl.scalar(0, "right_boundary")
            right_boundary[0] = V[i, j, k, 0, m]
            left_deriv[0] = (V[i, j, k, l, m] - V[i, j, k, l - 1, m]) / g.dx[3]
            right_deriv[0] = (right_boundary[0] - V[i, j, k, l, m]) / g.dx[3]
        with hcl.elif_(l != 0 and l != V.shape[3] - 1):
            left_deriv[0] = (V[i, j, k, l, m] - V[i, j, k, l - 1, m]) / g.dx[3]
            right_deriv[0] = (V[i, j, k, l + 1, m] - V[i, j, k, l, m]) / g.dx[3]
        return left_deriv[0], right_deriv[0]


def spa_derivX3_5d(i, j, k, l, m, V, g):  # Left -> right == Outer Most -> Inner Most
    left_deriv = hcl.scalar(0, "left_deriv")
    right_deriv = hcl.scalar(0, "right_deriv")
    if 3 not in g.pDim:
        with hcl.if_(k == 0):
            left_boundary = hcl.scalar(0, "left_boundary")
            left_boundary[0] = V[i, j, k, l, m] + my_abs(V[i, j, k + 1, l, m] - V[i, j, k, l, m]) * my_sign(
                V[i, j, k, l, m])
            left_deriv[0] = (V[i, j, k, l, m] - left_boundary[0]) / g.dx[2]
            right_deriv[0] = (V[i, j, k + 1, l, m] - V[i, j, k, l, m]) / g.dx[2]
        with hcl.elif_(k == V.shape[2] - 1):
            right_boundary = hcl.scalar(0, "right_boundary")
            right_boundary[0] = V[i, j, k, l, m] + my_abs(V[i, j, k, l, m] - V[i, j, k - 1, l, m]) * my_sign(
                V[i, j, k, l, m])
            left_deriv[0] = (V[i, j, k, l, m] - V[i, j, k - 1, l, m]) / g.dx[2]
            right_deriv[0] = (right_boundary[0] - V[i, j, k, l, m]) / g.dx[2]
        with hcl.elif_(k != 0 and k != V.shape[2] - 1):
            left_deriv[0] = (V[i, j, k, l, m] - V[i, j, k - 1, l, m]) / g.dx[2]
            right_deriv[0] = (V[i, j, k + 1, l, m] - V[i, j, k, l, m]) / g.dx[2]
        return left_deriv[0], right_deriv[0]
    else:
        with hcl.if_(k == 0):
            left_boundary = hcl.scalar(0, "left_boundary")
            left_boundary[0] = V[i, j, V.shape[2] - 1, l, m]
            left_deriv[0] = (V[i, j, k, l, m] - left_boundary[0]) / g.dx[2]
            right_deriv[0] = (V[i, j, k + 1, l, m] - V[i, j, k, l, m]) / g.dx[2]
        with hcl.elif_(k == V.shape[2] - 1):
            right_boundary = hcl.scalar(0, "right_boundary")
            right_boundary[0] = V[i, j, 0, l, m]
            left_deriv[0] = (V[i, j, k, l, m] - V[i, j, k - 1, l, m]) / g.dx[2]
            right_deriv[0] = (right_boundary[0] - V[i, j, k, l, m]) / g.dx[2]
        with hcl.elif_(k != 0 and k != V.shape[2] - 1):
            left_deriv[0] = (V[i, j, k, l, m] - V[i, j, k - 1, l, m]) / g.dx[2]
            right_deriv[0] = (V[i, j, k + 1, l, m] - V[i, j, k, l, m]) / g.dx[2]
        return left_deriv[0], right_deriv[0]


def spa_derivX2_5d(i, j, k, l, m, V, g):  # Left -> right == Outer Most -> Inner Most
    left_deriv = hcl.scalar(0, "left_deriv")
    right_deriv = hcl.scalar(0, "right_deriv")
    if 2 not in g.pDim:
        with hcl.if_(j == 0):
            left_boundary = hcl.scalar(0, "left_boundary")
            left_boundary[0] = V[i, j, k, l, m] + my_abs(V[i, j + 1, k, l, m] - V[i, j, k, l, m]) * my_sign(
                V[i, j, k, l, m])
            left_deriv[0] = (V[i, j, k, l, m] - left_boundary[0]) / g.dx[1]
            right_deriv[0] = (V[i, j + 1, k, l, m] - V[i, j, k, l, m]) / g.dx[1]
        with hcl.elif_(j == V.shape[1] - 1):
            right_boundary = hcl.scalar(0, "right_boundary")
            right_boundary[0] = V[i, j, k, l, m] + my_abs(V[i, j, k, l, m] - V[i, j - 1, k, l, m]) * my_sign(
                V[i, j, k, l, m])
            left_deriv[0] = (V[i, j, k, l, m] - V[i, j - 1, k, l, m]) / g.dx[1]
            right_deriv[0] = (right_boundary[0] - V[i, j, k, l, m]) / g.dx[1]
        with hcl.elif_(j != 0 and j != V.shape[1] - 1):
            left_deriv[0] = (V[i, j, k, l, m] - V[i, j - 1, k, l, m]) / g.dx[1]
            right_deriv[0] = (V[i, j + 1, k, l, m] - V[i, j, k, l, m]) / g.dx[1]
        return left_deriv[0], right_deriv[0]
    else:
        with hcl.if_(j == 0):
            left_boundary = hcl.scalar(0, "left_boundary")
            left_boundary[0] = V[i, V.shape[1] - 1, k, l, m]
            left_deriv[0] = (V[i, j, k, l, m] - left_boundary[0]) / g.dx[1]
            right_deriv[0] = (V[i, j + 1, k, l, m] - V[i, j, k, l, m]) / g.dx[1]
        with hcl.elif_(j == V.shape[1] - 1):
            right_boundary = hcl.scalar(0, "right_boundary")
            right_boundary[0] = V[i, 0, k, l, m]
            left_deriv[0] = (V[i, j, k, l, m] - V[i, j - 1, k, l, m]) / g.dx[1]
            right_deriv[0] = (right_boundary[0] - V[i, j, k, l, m]) / g.dx[1]
        with hcl.elif_(j != 0 and j != V.shape[1] - 1):
            left_deriv[0] = (V[i, j, k, l, m] - V[i, j - 1, k, l, m]) / g.dx[1]
            right_deriv[0] = (V[i, j + 1, k, l, m] - V[i, j, k, l, m]) / g.dx[1]
        return left_deriv[0], right_deriv[0]


def spa_derivX1_5d(i, j, k, l, m, V, g):  # Left -> right == Outer Most -> Inner Most
    left_deriv = hcl.scalar(0, "left_deriv")
    right_deriv = hcl.scalar(0, "right_deriv")
    if 1 not in g.pDim:
        with hcl.if_(i == 0):
            left_boundary = hcl.scalar(0, "left_boundary")
            left_boundary[0] = V[i, j, k, l, m] + my_abs(V[i + 1, j, k, l, m] - V[i, j, k, l, m]) * my_sign(
                V[i, j, k, l, m])
            left_deriv[0] = (V[i, j, k, l, m] - left_boundary[0]) / g.dx[0]
            right_deriv[0] = (V[i + 1, j, k, l, m] - V[i, j, k, l, m]) / g.dx[0]
        with hcl.elif_(i == V.shape[0] - 1):
            right_boundary = hcl.scalar(0, "right_boundary")
            right_boundary[0] = V[i, j, k, l, m] + my_abs(V[i, j, k, l, m] - V[i - 1, j, k, l, m]) * my_sign(
                V[i, j, k, l, m])
            left_deriv[0] = (V[i, j, k, l, m] - V[i - 1, j, k, l, m]) / g.dx[0]
            right_deriv[0] = (right_boundary[0] - V[i, j, k, l, m]) / g.dx[0]
        with hcl.elif_(i != 0 and i != V.shape[0] - 1):
            left_deriv[0] = (V[i, j, k, l, m] - V[i - 1, j, k, l, m]) / g.dx[0]
            right_deriv[0] = (V[i + 1, j, k, l, m] - V[i, j, k, l, m]) / g.dx[0]
        return left_deriv[0], right_deriv[0]
    else:
        with hcl.if_(i == 0):
            left_boundary = hcl.scalar(0, "left_boundary")
            left_boundary[0] = V[V.shape[0] - 1, j, k, l, m]
            left_deriv[0] = (V[i, j, k, l, m] - left_boundary[0]) / g.dx[0]
            right_deriv[0] = (V[i + 1, j, k, l, m] - V[i, j, k, l, m]) / g.dx[0]
        with hcl.elif_(i == V.shape[0] - 1):
            right_boundary = hcl.scalar(0, "right_boundary")
            right_boundary[0] = V[0, j, k, l, m]
            left_deriv[0] = (V[i, j, k, l, m] - V[i - 1, j, k, l, m]) / g.dx[0]
            right_deriv[0] = (right_boundary[0] - V[i, j, k, l, m]) / g.dx[0]
        with hcl.elif_(i != 0 and i != V.shape[0] - 1):
            left_deriv[0] = (V[i, j, k, l, m] - V[i - 1, j, k, l, m]) / g.dx[0]
            right_deriv[0] = (V[i + 1, j, k, l, m] - V[i, j, k, l, m]) / g.dx[0]
        return left_deriv[0], right_deriv[0]


########################## 5D graph definition ########################

def getControls_5D(my_object, g):
    V_init = hcl.placeholder(tuple(g.pts_each_dim), name="V_init", dtype=hcl.Float())
    # robot control tables
    #Ctrl= hcl.placeholder(tuple(np.insert(g.pts_each_dim,0,6)), name="Ctrl", dtype=hcl.Float())
    Ctrl= hcl.placeholder(tuple(np.insert(g.pts_each_dim,0,2)), name="Ctrl", dtype=hcl.Float())

    # Positions vector
    x1 = hcl.placeholder((g.pts_each_dim[0],), name="x1", dtype=hcl.Float())
    x2 = hcl.placeholder((g.pts_each_dim[1],), name="x2", dtype=hcl.Float())
    x3 = hcl.placeholder((g.pts_each_dim[2],), name="x3", dtype=hcl.Float())
    x4 = hcl.placeholder((g.pts_each_dim[3],), name="x4", dtype=hcl.Float())
    x5 = hcl.placeholder((g.pts_each_dim[4],), name="x5", dtype=hcl.Float())


    #t = hcl.placeholder((2,), name="t", dtype=hcl.Float())
    t = hcl.scalar(0, "t")
    def graph_create(Ctrl, V_init, x1, x2, x3, x4, x5):
        with hcl.Stage("Hamiltonian"):
            with hcl.for_(0, V_init.shape[0], name="i") as i:
                with hcl.for_(0, V_init.shape[1], name="j") as j:
                    with hcl.for_(0, V_init.shape[2], name="k") as k:
                        with hcl.for_(0, V_init.shape[3], name="l") as l:
                            with hcl.for_(0, V_init.shape[4], name="m") as m:
                                    # Variables to calculate dV_dx
                                    dV_dx1_L = hcl.scalar(0, "dV_dx1_L")
                                    dV_dx1_R = hcl.scalar(0, "dV_dx1_R")
                                    dV_dx1 = hcl.scalar(0, "dV_dx1")
                                    dV_dx2_L = hcl.scalar(0, "dV_dx2_L")
                                    dV_dx2_R = hcl.scalar(0, "dV_dx2_R")
                                    dV_dx2 = hcl.scalar(0, "dV_dx2")
                                    dV_dx3_L = hcl.scalar(0, "dV_dx3_L")
                                    dV_dx3_R = hcl.scalar(0, "dV_dx3_R")
                                    dV_dx3 = hcl.scalar(0, "dV_dx3")
                                    dV_dx4_L = hcl.scalar(0, "dV_dx4_L")
                                    dV_dx4_R = hcl.scalar(0, "dV_dx4_R")
                                    dV_dx4 = hcl.scalar(0, "dV_dx4")
                                    dV_dx5_L = hcl.scalar(0, "dV_dx5_L")
                                    dV_dx5_R = hcl.scalar(0, "dV_dx5_R")
                                    dV_dx5 = hcl.scalar(0, "dV_dx5")

                                    # No tensor slice operation
                                    # dV_dx_L[0], dV_dx_R[0] = spa_derivX(i, j, k)
                                    dV_dx1_L[0], dV_dx1_R[0] = spa_derivX1_5d(i, j, k, l, m, V_init, g)
                                    dV_dx2_L[0], dV_dx2_R[0] = spa_derivX2_5d(i, j, k, l, m, V_init, g)
                                    dV_dx3_L[0], dV_dx3_R[0] = spa_derivX3_5d(i, j, k, l, m, V_init, g)
                                    dV_dx4_L[0], dV_dx4_R[0] = spa_derivX4_5d(i, j, k, l, m, V_init, g)
                                    dV_dx5_L[0], dV_dx5_R[0] = spa_derivX5_5d(i, j, k, l, m, V_init, g)

                    
                                    # Calculate average gradient
                                    dV_dx1[0] = (dV_dx1_L + dV_dx1_R) / 2
                                    dV_dx2[0] = (dV_dx2_L + dV_dx2_R) / 2
                                    dV_dx3[0] = (dV_dx3_L + dV_dx3_R) / 2
                                    dV_dx4[0] = (dV_dx4_L + dV_dx4_R) / 2
                                    dV_dx5[0] = (dV_dx5_L + dV_dx5_R) / 2

                                    # Find optimal control
                                    uOpt = my_object.opt_ctrl(t, (x1[i], x2[j], x3[k], x4[l], x5[m]), (
                                    dV_dx1[0], dV_dx2[0], dV_dx3[0], dV_dx4[0], dV_dx5[0]))
                                    # Find optimal disturbance
                                    dOpt = my_object.optDstb(t, (x1[i], x2[j], x3[k], x4[l], x5[m]),
                                        (dV_dx1[0], dV_dx2[0], dV_dx3[0], dV_dx4[0], dV_dx5[0]))

                                    #Ctrl[0,i,j,k,l,m] = uOpt[0]
                                    #Ctrl[1,i,j,k,l,m] = uOpt[1]

                                    
                                    with hcl.if_(uOpt[0] <= 0):
                                        Ctrl[0,i,j,k,l,m] = 0
                                    with hcl.else_():
                                        Ctrl[0,i,j,k,l,m] = 1
                                    with hcl.if_(uOpt[1] <= 0):
                                        Ctrl[1,i,j,k,l,m] = 0
                                    with hcl.else_():
                                        Ctrl[1,i,j,k,l,m] = 1
                                    
                                
                                    
                                   

    s = hcl.create_schedule([Ctrl, V_init, x1, x2, x3, x4, x5], graph_create)
    ##################### CODE OPTIMIZATION HERE ###########################
    print("Optimizing\n")

    # Accessing the hamiltonian and dissipation stage
    s_H = graph_create.Hamiltonian


    # Thread parallelize hamiltonian and dissipation
    s[s_H].parallel(s_H.i)


    # Inspect IR
    # if args.llvm:
    #    print(hcl.lower(s))

    # Return executable
    return (hcl.build(s))
