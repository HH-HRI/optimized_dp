import heterocl as hcl

def my_sign(x):
    sign = hcl.scalar(0, "sign", dtype=hcl.Float())
    with hcl.if_(x == 0):
        sign[0] = 0
    with hcl.if_(x > 0):
        sign[0] = 1
    with hcl.if_(x < 0):
        sign[0] = -1
    return sign[0]


class Merge5D_HRI:
    def __init__(self, x, aMax_R, aMin_R, vyMax_R, vyMin_R, aMax_f, aMin_f, aMax_r, aMin_r, uMode='min', dMode='max'):
    
        self.x = x

        self.aMax_R  = aMax_R
        self.aMin_R  = aMin_R
        self.vyMax_R = vyMax_R
        self.vyMin_R = vyMin_R
        self.aMax_f  = aMax_f
        self.aMin_f  = aMin_f
        self.aMax_r  = aMax_r
        self.aMin_r  = aMin_r

        # assert(uMode in ["min", "max"])
        self.uMode = uMode
        # if uMode == "min":
        #     assert(dMode == "max")
        # else:
        #     assert(dMode == "min")
        self.dMode = dMode


    def opt_ctrl(self, t, state, spat_deriv):
        """
                :param  spat_deriv: tuple of spatial derivative in all dimensions
                        state: x1, x2, x3, x4, x5
                        t: time
                :return: a tuple of optimal controls
        """

        ''' DYNAMICS
                       0      1    2     3      4
            states: [px_ef  px_er  py  vx_ef  vx_er]
            ctrls:  [ax     vy]
            dstbs:  [ax_f   ax_r]

            x0_dot = x3
            x1_dot = x4
            x2_dot = u1
            x3_dot = d0 - u0
            x4_dot = u0 - d1
        '''

        accOpt = hcl.scalar(self.aMax_R, "accOpt")
        vLatOpt_R = hcl.scalar(self.vyMax_R, "vLatOpt_R")

        # Just create and pass back, even though they're not used
        in3 = hcl.scalar(0, "in3")
        in4 = hcl.scalar(0, "in4")
        in5 = hcl.scalar(0, "in5")

        with hcl.if_(self.uMode == "min"):
            with hcl.if_(spat_deriv[4] - spat_deriv[3] >= 0):
                accOpt[0] = -accOpt[0]
            with hcl.if_(spat_deriv[2] >= 0):
                vLatOpt_R[0] = -vLatOpt_R[0]

        with hcl.if_(self.uMode == "max"):
            with hcl.if_(spat_deriv[4] - spat_deriv[3] < 0):
                accOpt[0] = -accOpt[0]
            with hcl.if_(spat_deriv[2] < 0):
                vLatOpt_R[0] = -vLatOpt_R[0]
        # return 3, 4 even if you don't use them
        return (accOpt[0], vLatOpt_R[0], in3[0], in4[0], in5[0])



        # # Define hcl variables.
        # aMax_R_hcl  = hcl.scalar(self.aMax_R, "aMax_R_hcl")
        # aMin_R_hcl  = hcl.scalar(self.aMin_R, "aMin_R_hcl")
        # vyMax_R_hcl = hcl.scalar(self.vyMax_R, "vyMax_R_hcl")
        # vyMin_R_hcl = hcl.scalar(self.vyMin_R, "vyMin_R_hcl")

        # # Define dummy hcl variables.
        # u0 = hcl.scalar(0, "u0")
        # u1 = hcl.scalar(0, "u1")
        # u2 = hcl.scalar(0, "u2")
        # u3 = hcl.scalar(0, "u3")
        # u4 = hcl.scalar(0, "u4")

        # with hcl.if_(self.uMode == "min"):
        # # if self.uMode == "min": #!!!
        #     with hcl.if_(spat_deriv[4] - spat_deriv[3] >= 0):
        #         u0[0] = aMin_R_hcl[0]
        #     with hcl.else_():
        #         u0[0] = aMax_R_hcl[0]

        #     with hcl.if_(spat_deriv[2] >= 0):
        #         u1[0] = vyMin_R_hcl[0]
        #     with hcl.else_():
        #         u1[0] = vyMax_R_hcl[0]
        # with hcl.elif_(self.uMode == "max"):
        # # else: #!!!
        #     with hcl.if_(spat_deriv[4] - spat_deriv[3] < 0):
        #         u0[0] = aMin_R_hcl[0]
        #     with hcl.else_():
        #         u0[0] = aMax_R_hcl[0]

        #     with hcl.if_(spat_deriv[2] < 0):
        #         u1[0] = vyMin_R_hcl[0]
        #     with hcl.else_():
        #         u1[0] = vyMax_R_hcl[0]

        # # Return dummy variables even if you don't use them.
        # return (u0[0], u1[0], u2[0], u3[0], u4[0])


    def opt_dstb(self, t, state, spat_deriv):
        """
            :param  spat_deriv: tuple of spatial derivative in all dimensions
                    state: x1, x2, x3, x4, x5
                    t: time
            :return: a tuple of optimal disturbances
        """

        ''' DYNAMICS
                       0      1    2     3      4
            states: [px_ef  px_er  py  vx_ef  vx_er]
            ctrls:  [ax     vy]
            dstbs:  [ax_f   ax_r]
            
            x0_dot = x3
            x1_dot = x4
            x2_dot = u1
            x3_dot = d0 - u0
            x4_dot = u0 - d1
        '''

        # Define hcl variables.
        aMax_f_hcl = hcl.scalar(self.aMax_f, "aMax_f_hcl")
        aMin_f_hcl = hcl.scalar(self.aMin_f, "aMin_f_hcl")
        aMax_r_hcl = hcl.scalar(self.aMax_r, "aMax_r_hcl")
        aMin_r_hcl = hcl.scalar(self.aMin_r, "aMin_r_hcl")

        # Define dummy hcl variables.
        d0 = hcl.scalar(0, "d0")
        d1 = hcl.scalar(0, "d1")
        d2 = hcl.scalar(0, "d2")
        d3 = hcl.scalar(0, "d3")
        d4 = hcl.scalar(0, "d4")

        with hcl.if_(self.dMode == "max"):
            with hcl.if_(spat_deriv[3] >= 0):
                d0[0] = aMax_f_hcl[0]
            with hcl.else_():
                d0[0] = aMin_f_hcl[0]

            with hcl.if_(spat_deriv[4] >= 0):
                d1[0] = aMin_r_hcl[0]
            with hcl.else_():
                d1[0] = aMax_r_hcl[0]

        with hcl.elif_(self.dMode == "min"):
            with hcl.if_(spat_deriv[3] < 0):
                d0[0] = aMax_f_hcl[0]
            with hcl.else_():
                d0[0] = aMin_f_hcl[0]

            with hcl.if_(spat_deriv[4] < 0):
                d1[0] = aMin_r_hcl[0]
            with hcl.else_():
                d1[0] = aMax_r_hcl[0]

        # Return dummy variables even if you don't use them.
        return (d0[0], d1[0], d2[0], d3[0], d4[0])


    def dynamics(self, t, state, uOpt, dOpt):
        ''' DYNAMICS
                       0      1    2     3      4
            states: [px_ef  px_er  py  vx_ef  vx_er]
            ctrls:  [ax     vy]
            dstbs:  [ax_f   ax_r]
            
            x0_dot = x3
            x1_dot = x4
            x2_dot = u1
            x3_dot = d0 - u0
            x4_dot = u0 - d1
        '''

        dx0 = hcl.scalar(0, "dx0")
        dx1 = hcl.scalar(0, "dx1")
        dx2 = hcl.scalar(0, "dx2")
        dx3 = hcl.scalar(0, "dx3")
        dx4 = hcl.scalar(0, "dx4") 
    
        dx0[0] = state[3] 
        dx1[0] = state[4]
        dx2[0] = uOpt[1]
        dx3[0] = dOpt[0] - uOpt[0]
        dx4[0] = uOpt[0] - dOpt[1]

        return (dx0[0], dx1[0], dx2[0], dx3[0], dx4[0])

