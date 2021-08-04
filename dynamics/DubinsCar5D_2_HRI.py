import heterocl as hcl
import numpy as np

class DubinsCar5D_2_HRI:
    def __init__(self, x, accMax_R, vLgtDev_H, vLgtNom_H, thetaMax_R, vLatMax_H, talpha, uMode, dMode):
    
        self.x = x

        self.accMax_R = accMax_R
        self.vLgtDev_H = vLgtDev_H
        self.vLgtNom_H = vLgtNom_H
        self.thetaMax_R = thetaMax_R
        self.vLatMax_H = vLatMax_H
        self.talpha = talpha
        assert(uMode in ["min", "max"])
        self.uMode = uMode
        if uMode == "min":
            assert(dMode == "max")
        else:
            assert(dMode == "min")
        self.dMode = dMode

    def atan2(self, y,x):
        x_prime = hcl.scalar(y/x, "x_prime")
        atan2 = hcl.scalar(0, "atan2")
        with hcl.if_(x_prime <-1):
            # technically this also depends on x and y sign, but won't matter
            atan2 = np.pi/2
            return atan2
        with hcl.elif_(x_prime >1):
            # technically this also depends on x and y sign, but won't matter
            atan2 = np.pi/2
            return atan2
        with hcl.else_():
            atan2 = (np.pi/4)*x_prime - x_prime*(np.abs(x_prime)-1)*(0.2447 + 0.0663*np.abs(x_prime))
            with hcl.if_(x>0):
                return atan2
            with hcl.if_(y>0):
                atan2 = atan2 + np.pi
            with hcl.else_():
                atan2 = atan2 - np.pi

    def opt_ctrl(self, t, state, spat_deriv):
        """
                :param  spat_deriv: tuple of spatial derivative in all dimensions
                        state: x1, x2, x3, x4
                        t: time
                :return: a tuple of optimal controls
        """

        ''' DYNAMICS
        x1_dot = vcos(u1)            x-vel Robot
        x2_dot = d1                  x-vel Human
        x3_dot = vsin(u1)            y-vel Robot
        x4_dot = d2                  y-vel Human
        x5_dot = u2 - talpha * x5    accel Robot
        
        '''

        # Graph takes in 4 possible inputs, by default, for now
        accOpt_R = hcl.scalar(self.accMax_R, "accOpt_R")
        thetaOpt_R = hcl.scalar(self.thetaMax_R, "thetaOpt_R")
        # Just create and pass back, even though they're not used
        in3 = hcl.scalar(0, "in3")
        in4 = hcl.scalar(0, "in4")
        in5 = hcl.scalar(0, "in5")
    
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
        a_term[0] = spat_deriv[0] * state[4]
        b_term[0] = spat_deriv[2] * state[4]

        # calculating theta from approximation
<<<<<<< HEAD
        #theta_1[0] = self.atan2(spat_deriv[2], spat_deriv[0])
        #theta_2[0] = self.atan2(-(spat_deriv[2]), -(spat_deriv[0]))
=======
        theta_1[0] = self.atan2(spat_deriv[2], spat_deriv[0])
        theta_2[0] = self.atan2(-(spat_deriv[2]), -(spat_deriv[0]))
>>>>>>> 7a6b563752a937661374771f2176f3dfdeeeab64


        
        sum3[0] = a_term[0] * hcl.cos(thetaOpt_R[0]) + b_term[0] * hcl.sin(thetaOpt_R[0])
        sum4[0] = a_term[0] * hcl.cos(-thetaOpt_R[0]) + b_term[0] * hcl.sin(-thetaOpt_R[0])
        
        

        if self.uMode == "min":
<<<<<<< HEAD
            '''
=======
>>>>>>> 7a6b563752a937661374771f2176f3dfdeeeab64
            # checks if theta from atan2 is within [-thetaMax, thetaMax]
            with hcl.if_(hcl.and_(theta_1[0] <= self.thetaMax_R, theta_1[0] >= -self.thetaMax_R)):
                sum1[0] = a_term[0] * hcl.cos(theta_1[0]) + b_term[0] * hcl.sin(theta_1[0])
            # if not, set to inf so it won't matter
            with hcl.else_():
                sum1[0] = np.inf
            # check if theta from atan2 is within [-thetaMax, thetaMax]
            with hcl.if_(hcl.and_(theta_2[0] <= self.thetaMax_R, theta_2[0] >= -self.thetaMax_R)):
                sum2[0] = a_term[0] * hcl.cos(theta_2[0]) + b_term[0] * hcl.sin(theta_2[0])
            # if not, set to inf so it won't matter
            with hcl.else_():
                sum2[0] = np.inf
<<<<<<< HEAD
            '''
            sum1[0] = np.inf
            sum2[0] = np.inf
=======
>>>>>>> 7a6b563752a937661374771f2176f3dfdeeeab64

            with hcl.if_(sum1[0] < sum2[0]):
                with hcl.if_(sum1[0] < sum3[0]):
                    with hcl.if_(sum1[0] < sum4[0]):
                        thetaOpt_R[0] = theta_1[0]
                    with hcl.else_():
                        thetaOpt_R[0] = -thetaOpt_R[0]
                with hcl.else_():
                    with hcl.if_(sum3[0] < sum4[0]):
<<<<<<< HEAD
                        thetaOpt_R[0] = thetaOpt_R[0]
=======
                        # keep thetaOpt = thetaMax
                        pass
>>>>>>> 7a6b563752a937661374771f2176f3dfdeeeab64
                    with hcl.else_():
                        thetaOpt_R[0] = -thetaOpt_R[0]
            with hcl.else_():
                with hcl.if_(sum2[0] < sum3[0]):
                    with hcl.if_(sum2[0] < sum4[0]):
                        thetaOpt_R[0] = theta_2[0]
                    with hcl.else_():
                        thetaOpt_R[0] = -thetaOpt_R[0]
                with hcl.else_():
                    with hcl.if_(sum3[0] < sum4[0]):
<<<<<<< HEAD
                        thetaOpt_R[0] = thetaOpt_R[0]
=======
                        # keep thetaOpt = thetaMax
                        pass
>>>>>>> 7a6b563752a937661374771f2176f3dfdeeeab64
                    with hcl.else_():
                        thetaOpt_R[0] = -thetaOpt_R[0]

            with hcl.if_(spat_deriv[4] >= 0):
                accOpt_R[0] = -accOpt_R[0]

        else:
<<<<<<< HEAD

            '''
=======
>>>>>>> 7a6b563752a937661374771f2176f3dfdeeeab64
            # check if theta from atan2 is within [-thetaMax, thetaMax]
            with hcl.if_(theta_1[0] <= self.thetaMax_R and theta_1[0] >= -self.thetaMax_R):
                sum1[0] = a_term[0] * hcl.cos(theta_1[0]) + b_term[0] * hcl.sin(theta_1[0])
            # if not, set to -inf so it won't matter
            with hcl.else_():
                sum1[0] = -np.inf
            # check if theta from atan2 is within [-thetaMax, thetaMax]
            with hcl.if_(theta_2[0] <= self.thetaMax_R and theta_2[0] >= -self.thetaMax_R):
                sum2[0] = a_term[0] * hcl.cos(theta_2[0]) + b_term[0] * hcl.sin(theta_2[0])
            # if not, set to -inf so it won't matter
            with hcl.else_():
                sum2[0] = -np.inf
<<<<<<< HEAD
            '''
            sum1[0] = -np.inf
            sum2[0] = -np.inf
=======

>>>>>>> 7a6b563752a937661374771f2176f3dfdeeeab64
            # finding maximizing theta                
            with hcl.if_(sum1[0] > sum2[0]):
                with hcl.if_(sum1[0] > sum3[0]):
                    with hcl.if_(sum1[0] > sum4[0]):
                        thetaOpt_R[0] = theta_1[0]
                    with hcl.else_():
                        thetaOpt_R[0] = -thetaOpt_R[0]
                with hcl.else_():
                    with hcl.if_(sum3[0] > sum4[0]):
<<<<<<< HEAD
                        thetaOpt_R[0] = thetaOpt_R[0]
=======
                        # keep thetaOpt = thetaMax
                        pass
>>>>>>> 7a6b563752a937661374771f2176f3dfdeeeab64
                    with hcl.else_():
                        thetaOpt_R[0] = -thetaOpt_R[0]
            with hcl.else_():
                with hcl.if_(sum2[0] > sum3[0]):
                    with hcl.if_(sum2[0] > sum4[0]):
                        thetaOpt_R[0] = theta_2[0]
                    with hcl.else_():
                        thetaOpt_R[0] = -thetaOpt_R[0]
                with hcl.else_():
                    with hcl.if_(sum3[0] > sum4[0]):
<<<<<<< HEAD
                        thetaOpt_R[0] = thetaOpt_R[0]
=======
                        # keep thetaOpt = thetaMax
                        pass
>>>>>>> 7a6b563752a937661374771f2176f3dfdeeeab64
                    with hcl.else_():
                        thetaOpt_R[0] = -thetaOpt_R[0]

            with hcl.if_(spat_deriv[4] < 0):
                accOpt_R[0] = -accOpt_R[0]
    

        # return 3, 4 even if you don't use them
        return (thetaOpt_R[0], accOpt_R[0], in3[0], in4[0], in5[0])






    def optDstb(self, t, state, spat_deriv):
        """
            :param spat_deriv: tuple of spatial derivative in all dimensions
                    state: x0, x1, x2, x3
                    t: time
            :return: a tuple of optimal disturbances
        """
        ''' DYNAMICS
        x1_dot = vcos(u1)            x-vel Robot
        x2_dot = d1                  x-vel Human
        x3_dot = vsin(u1)            y-vel Robot
        x4_dot = d2                  y-vel Human
        x5_dot = u2 - talpha * x5    accel Robot
        
        '''

        # Graph takes in 5 possible inputs, by default, for now
        vLgtDev_H = hcl.scalar(self.vLgtDev_H, "vLgtDev_H")
        vLatOpt_H = hcl.scalar(self.vLatMax_H, "vLatOpt_H")
        vLgtNom_H = hcl.scalar(self.vLgtNom_H, "vLgtNom_H")
        vLgtOpt_H = hcl.scalar(0, "vLgtOpt_H")
        # Just create and pass back, even though they're not used
        d3 = hcl.scalar(0, "d3")
        d4 = hcl.scalar(0, "d4")
        d5 = hcl.scalar(0, "d5")

        if self.dMode == "max":
            with hcl.if_(spat_deriv[1] <= 0):
                vLgtOpt_H[0] = vLgtNom_H-vLgtDev_H[0]
            with hcl.else_():
                vLgtOpt_H[0] = vLgtNom_H+vLgtDev_H[0]
            with hcl.if_(spat_deriv[3] <= 0):
                vLatOpt_H[0] = -vLatOpt_H[0]
        else:
            with hcl.if_(spat_deriv[1] > 0):
                vLgtOpt_H[0] = vLgtNom_H - vLgtDev_H[0]
            with hcl.else_():
                vLgtOpt_H[0] = vLgtNom_H + vLgtDev_H[0]

            with hcl.if_(spat_deriv[3] > 0):
                vLatOpt_H[0] = -vLatOpt_H[0]
        # return 3, 4 even if you don't use them
        return (vLgtOpt_H[0], vLatOpt_H[0], d3[0], d4[0], d5[0])




    def dynamics(self, t, state, uOpt, dOpt):
        dx0 = hcl.scalar(0, "dx0")
        dx1 = hcl.scalar(0, "dx1")
        dx2 = hcl.scalar(0, "dx2")
        dx3 = hcl.scalar(0, "dx3")
        dx4 = hcl.scalar(0, "dx4") 
    
        dx0[0] = state[4]*hcl.cos(uOpt[0])
        dx1[0] = dOpt[0]
        dx2[0] = state[4]*hcl.sin(uOpt[0])
        dx3[0] = dOpt[1]
        dx4[0] = uOpt[1] - (self.talpha*state[4])


        return (dx0[0], dx1[0], dx2[0], dx3[0], dx4[0])
