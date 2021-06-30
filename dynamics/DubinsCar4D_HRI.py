import heterocl as hcl

class DubinsCar4d_HRI:
    def __init__(self, x, accMax_R, accMax_H, vLatMax_R, vLatMax_H, talpha, uMode, dMode):
    
        self.x = x

        self.accMax_R = accMax_R
        self.accMax_H = accMax_H
        self.vLatMax_R = vLatMax_R
        self.vLatMax_H = vLatMax_H
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
        x1_dot = x4
        x2_dot = u2
        x3_dot = d2
        x4_dot = u1 - d1 - talpha * x4
        '''

        # Graph takes in 4 possible inputs, by default, for now
        accOpt_R = hcl.scalar(self.accMax_R, "accOpt_R")
        vLatOpt_R = hcl.scalar(self.vLatMax_R, "vLatOpt_R")
        # Just create and pass back, even though they're not used
        in3 = hcl.scalar(0, "in3")
        in4 = hcl.scalar(0, "in4")

        if self.uMode == "min":
            with hcl.if_(spat_deriv[3] >= 0):
                accOpt_R[0] = -accOpt_R[0]
            with hcl.if_(spat_deriv[1] >= 0):
                vLatOpt_R[0] = -vLatOpt_R[0]
        else:
            with hcl.if_(spat_deriv[3] < 0):
                accOpt_R[0] = -accOpt_R[0]
            with hcl.if_(spat_deriv[1] < 0):
                vLatOpt_R[0] = -vLatOpt_R[0]
        # return 3, 4 even if you don't use them
        return (accOpt_R[0], vLatOpt_R[0], in3[0], in4[0])




    def optDstb(self, t, state, spat_deriv):
        """
            :param spat_deriv: tuple of spatial derivative in all dimensions
                    state: x0, x1, x2, x3
                    t: time
            :return: a tuple of optimal disturbances
        """


        # Graph takes in 4 possible inputs, by default, for now
        accOpt_H = hcl.scalar(self.accMax_H, "accOpt_H")
        vLatOpt_H = hcl.scalar(self.vLatMax_H, "vLatOpt_H")
        # Just create and pass back, even though they're not used
        d3 = hcl.scalar(0, "d3")
        d4 = hcl.scalar(0, "d4")


        if self.dMode == "max":
            with hcl.if_(spat_deriv[3] >= 0):
                accOpt_H[0] = -accOpt_H[0]
            with hcl.if_(spat_deriv[2] <= 0):
                vLatOpt_H[0] = -vLatOpt_H[0]
        else:
            with hcl.if_(spat_deriv[3] < 0):
                accOpt_H[0] = -accOpt_H[0]
            with hcl.if_(spat_deriv[2] > 0):
                vLatOpt_H[0] = -vLatOpt_H[0]
        # return 3, 4 even if you don't use them
        return (accOpt_H[0], vLatOpt_H[0], d3[0], d4[0])



    def dynamics(self, t, state, uOpt, dOpt):
        dx0 = hcl.scalar(0, "dx0")
        dx1 = hcl.scalar(0, "dx1")
        dx2 = hcl.scalar(0, "dx2")
        dx3 = hcl.scalar(0, "dx3") 
    
        dx0[0] = state[3]
        dx1[0] = uOpt[1]
        dx2[0] = dOpt[1]
        dx3[0] = uOpt[0] - dOpt[0] - (self.talpha*state[3])

        return (dx0[0], dx1[0], dx2[0], dx3[0])
