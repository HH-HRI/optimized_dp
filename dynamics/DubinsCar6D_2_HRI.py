import heterocl as hcl
import numpy as np

class DubinsCar6D_2_HRI:
    def __init__(self, x, accMax_R, accMax_H, thetaMax_R, thetaMax_H, talpha, uMode, dMode):
    
        self.x = x

        self.accMax_R = accMax_R
        self.accMax_H = accMax_H
        self.thetaMax_R = thetaMax_R
        self.thetaMax_H = thetaMax_H
        self.talpha = talpha
        assert(uMode in ["min", "max"])
        self.uMode = uMode
        if uMode == "min":
            assert(dMode == "max")
        else:
            assert(dMode == "min")
        self.dMode = dMode


    def opt_ctrl(self, t, state, spat_deriv):
        """
                :param  spat_deriv: tuple of spatial derivative in all dimensions
                        state: x1, x2, x3, x4
                        t: time
                :return: a tuple of optimal controls
        """

        ''' DYNAMICS
        x1_dot = vcos(u1)            x-vel Robot
        x2_dot = vcos(d1)            x-vel Human
        x3_dot = vsin(u1)            y-vel Robot
        x4_dot = vsin(d1)            y-vel Human
        x5_dot = u2 - talpha * x5    accel Robot
        x6_dot = d2 - talpha * x6    accel Human 
        '''

        # Graph takes in 4 possible inputs, by default, for now
        accOpt_R = hcl.scalar(self.accMax_R, "accOpt_R")
        thetaOpt_R = hcl.scalar(self.thetaMax_R, "thetaOpt_R")
        # Just create and pass back, even though they're not used
        in3 = hcl.scalar(0, "in3")
        in4 = hcl.scalar(0, "in4")
        
        # Declare a variable
        a_term = hcl.scalar(0, "a_term")
        b_term = hcl.scalar(0, "b_term")
        theta_1 = hcl.scalar(0, "theta_1")
        theta_2 = hcl.scalar(0, "theta_2")

        sum1 = hcl.scalar(-np.inf, "sum1")
        sum2 = hcl.scalar(-np.inf, "sum2")
        sum3 = hcl.scalar(-np.inf, "sum3")
        sum4 = hcl.scalar(-np.inf, "sum4")

        # use the scalar by indexing 0 everytime
        a_term[0] = spat_deriv[0] * state[4]
        b_term[0] = spat_deriv[2] * state[4]
        theta_1[0] = np.arctan2(spat_deriv[2] * state[4], spat_deriv[0] * state[4])
        theta_2[0] = np.arctan2(-(spat_deriv[2] * state[4]), -(spat_deriv[0] * state[4]))
        with hcl.if_(theta_1[0] <= self.thetaMax_R and theta_1[0] >= -self.thetaMax_R):
            sum1[0] = a_term[0] * hcl.cos(theta_1[0]) + b_term[0] * hcl.sin(theta_1[0])
        with hcl.else_():
            # keep the value at -inf
            pass
        with hcl.if_(theta_2[0] <= self.thetaMax_R and theta_2[0] >= -self.thetaMax_R):
            sum2[0] = a_term[0] * hcl.cos(theta_2[0]) + b_term[0] * hcl.sin(theta_2[0])
        with hcl.else_():
            # keep value at -inf
            pass
        sum3[0] = a_term[0] * hcl.cos(thetaOpt_R[0]) + b_term[0] * hcl.sin(thetaOpt_R[0])
        sum4[0] = a_term[0] * hcl.cos(-thetaOpt_R[0]) + b_term[0] * hcl.sin(-thetaOpt_R[0])
        
        

        if self.uMode == "min":
            with hcl.if_(spat_deriv[4] >= 0):
                accOpt_R[0] = -accOpt_R[0]

            with hcl.if_(sum1[0] < sum2[0]):
                with hcl.if_(sum1[0] < sum3[0]):
                    with hcl.if_(sum1[0] < sum4[0]):
                        thetaOpt_R[0] = theta_1[0]
                    with hcl.else_():
                        thetaOpt_R[0] = theta_4[0]
                with hcl.else_():
                    with hcl.if_(sum3[0] < sum4[0]):
                        pass
                        # keep initialized value
                    with hcl.else_():
                        thetaOpt_R[0] = -thetaOpt_R[0]
            with hcl.else_():
                with hcl.if_(sum2[0] < sum3[0]):
                    with hcl.if_(sum2[0] < sum4[0]):
                        thetaOpt_R[0] = theta_2[0]
                    with hcl.else_():
                        thetaOpt_R[0] = theta_4[0]
                with hcl.else_():
                    with hcl.if_(sum3[0] < sum4[0]):
                        pass
                        # keep initialized value
                    with hcl.else_():
                        thetaOpt_R[0] = -thetaOpt_R[0]
        else:
            with hcl.if_(spat_deriv[5] < 0):
                accOpt_R[0] = -accOpt_R[0]
                
            with hcl.if_(sum1[0] > sum2[0]):
                with hcl.if_(sum1[0] > sum3[0]):
                    with hcl.if_(sum1[0] > sum4[0]):
                        thetaOpt_R[0] = theta_1[0]
                    with hcl.else_():
                        thetaOpt_R[0] = theta_4[0]
                with hcl.else_():
                    with hcl.if_(sum3[0] > sum4[0]):
                        pass
                        # keep initialized value
                    with hcl.else_():
                        thetaOpt_R[0] = -thetaOpt_R[0]
            with hcl.else_():
                with hcl.if_(sum2[0] > sum3[0]):
                    with hcl.if_(sum2[0] > sum4[0]):
                        thetaOpt_R[0] = theta_2[0]
                    with hcl.else_():
                        thetaOpt_R[0] = theta_4[0]
                with hcl.else_():
                    with hcl.if_(sum3[0] > sum4[0]):
                        pass
                        # keep initialized value
                    with hcl.else_():
                        thetaOpt_R[0] = -thetaOpt_R[0]

        # return 3, 4 even if you don't use them
        return (thetaOpt_R[0], accOpt_R[0], in3[0], in4[0])






    def optDstb(self, t, state, spat_deriv):
        """
            :param spat_deriv: tuple of spatial derivative in all dimensions
                    state: x0, x1, x2, x3
                    t: time
            :return: a tuple of optimal disturbances
        """


        # Graph takes in 4 possible inputs, by default, for now
        accOpt_H = hcl.scalar(self.accMax_H, "accOpt_H")
        thetaOpt_H = hcl.scalar(self.thetaMax_H, "thetaOpt_H")
        # Just create and pass back, even though they're not used
        d3 = hcl.scalar(0, "d3")
        d4 = hcl.scalar(0, "d4")

        # Declare a variable
        a_term = hcl.scalar(0, "a_term")
        b_term = hcl.scalar(0, "b_term")
        theta_1 = hcl.scalar(0, "theta_1")
        theta_2 = hcl.scalar(0, "theta_2")

        sum1 = hcl.scalar(0, "sum1")
        sum2 = hcl.scalar(0, "sum2")
        sum3 = hcl.scalar(0, "sum3")
        sum4 = hcl.scalar(0, "sum4")

        # use the scalar by indexing 0 everytime
        a_term[0] = spat_deriv[1] * state[5]
        b_term[0] = spat_deriv[3] * state[5]
        theta_1[0] = np.atan2(spat_deriv[3] * state[5], spat_deriv[1] * state[5])
        theta_2[0] = np.atan2(-(spat_deriv[3] * state[5]), -(spat_deriv[1] * state[5]))
        with hcl.if_(theta_1[0] <= self.thetaMax_R and theta_1[0] >= -self.thetaMax_R):
            sum1[0] = a_term[0] * hcl.cos(theta_1[0]) + b_term[0] * hcl.sin(theta_1[0])
        with hcl.else_():
            sum1[0] = np.inf
        with hcl.if_(theta_2[0] <= self.thetaMax_R and theta_2[0] >= -self.thetaMax_R):
            sum2[0] = a_term[0] * hcl.cos(theta_2[0]) + b_term[0] * hcl.sin(theta_2[0])
        with hcl.else_():
            sum2[0] = np.inf
        sum3[0] = a_term[0] * hcl.cos(thetaOpt_H[0]) + b_term[0] * hcl.sin(thetaOpt_H[0])
        sum3[0] = a_term[0] * hcl.cos(-thetaOpt_H[0]) + b_term[0] * hcl.sin(-thetaOpt_H[0])

        if self.dMode == "min":
            with hcl.if_(spat_deriv[5] >= 0):
                accOpt_H[0] = -accOpt_H[0]

            with hcl.if_(sum1[0] < sum2[0]):
                with hcl.if_(sum1[0] < sum3[0]):
                    with hcl.if_(sum1[0] < sum4[0]):
                        thetaOpt_H[0] = theta_1[0]
                    with hcl.else_():
                        thetaOpt_H[0] = theta_4[0]
                with hcl.else_():
                    with hcl.if_(sum3[0] < sum4[0]):
                        pass
                        # keep initialized value
                    with hcl.else_():
                        thetaOpt_H[0] = -thetaOpt_H[0]
            with hcl.else_():
                with hcl.if_(sum2[0] < sum3[0]):
                    with hcl.if_(sum2[0] < sum4[0]):
                        thetaOpt_H[0] = theta_2[0]
                    with hcl.else_():
                        thetaOpt_H[0] = theta_4[0]
                with hcl.else_():
                    with hcl.if_(sum3[0] < sum4[0]):
                        pass
                        # keep initialized value
                    with hcl.else_():
                        thetaOpt_H[0] = -thetaOpt_H[0]



        else:
            with hcl.if_(spat_deriv[5] < 0):
                accOpt_H[0] = -accOpt_H[0]

            with hcl.if_(sum1[0] > sum2[0]):
                with hcl.if_(sum1[0] > sum3[0]):
                    with hcl.if_(sum1[0] > sum4[0]):
                        thetaOpt_H[0] = theta_1[0]
                    with hcl.else_():
                        thetaOpt_H[0] = theta_4[0]
                with hcl.else_():
                    with hcl.if_(sum3[0] > sum4[0]):
                        pass
                        # keep initialized value
                    with hcl.else_():
                        thetaOpt_H[0] = -thetaOpt_H[0]
            with hcl.else_():
                with hcl.if_(sum2[0] > sum3[0]):
                    with hcl.if_(sum2[0] > sum4[0]):
                        thetaOpt_H[0] = theta_2[0]
                    with hcl.else_():
                        thetaOpt_H[0] = theta_4[0]
                with hcl.else_():
                    with hcl.if_(sum3[0] > sum4[0]):
                        pass
                        # keep initialized value
                    with hcl.else_():
                        thetaOpt_H[0] = -thetaOpt_H[0]

        # return 3, 4 even if you don't use them
        return (thetaOpt_H[0], accOpt_H[0], d3[0], d4[0])





    def dynamics(self, t, state, uOpt, dOpt):
        dx0 = hcl.scalar(0, "dx0")
        dx1 = hcl.scalar(0, "dx1")
        dx2 = hcl.scalar(0, "dx2")
        dx3 = hcl.scalar(0, "dx3")
        dx4 = hcl.scalar(0, "dx4") 
        dx5 = hcl.scalar(0, "dx5")  
    
        dx0[0] = state[4]*hcl.cos(uOpt[0])
        dx1[0] = state[5]*hcl.cos(dOpt[0])
        dx2[0] = state[4]*hcl.sin(uOpt[0])
        dx3[0] = state[5]*hcl.sin(dOpt[0])
        dx4[0] = uOpt[1]- (self.talpha*state[4])
        dx5[0] = dOpt[1] - (self.talpha*state[5])

        return (dx0[0], dx1[0], dx2[0], dx3[0], dx4[0]. dx5[0])
