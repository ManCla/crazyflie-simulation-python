import numpy as np
# from .lp2Filter import lp2Filter
import cython

'''
    PID class
'''
@cython.cclass
class PID():
    kp: cython.double
    ki: cython.double
    kd: cython.double
    dt: cython.double
    oldError: cython.double
    stateI: cython.double
    Iclamp: cython.double
    lp_enable: cython.int

    ## lp2filter implemented within PID class
    b0: cython.double
    b1: cython.double
    b2: cython.double
    a1: cython.double
    a2: cython.double
    x1: cython.double
    x2: cython.double


    def __init__(self, kp:float, ki:float, kd:float, dt:float, lp_enable:cython.int, lp_rate:float, lp_cutoff:float, Iclamp:float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt 
        self.oldError = 0
        self.stateI = 0
        self.Iclamp  = Iclamp
        self.lp_enable = lp_enable
        if self.lp_enable :
            ## lp2filter implemented within PID class
            fr: cython.double
            fr   = lp_rate/lp_cutoff
            ohm: cython.double
            ohm  = np.tan(np.pi/fr)
            ohm2: cython.double
            ohm2 = ohm**2
            c: cython.double
            c    = 1+2*np.cos(np.pi/4)*ohm+ohm2
            # coefficients
            self.b0 = ohm2/c
            self.b1 = 2*self.b0
            self.b2 = self.b0
            self.a1 = 2*(ohm2-1)/c
            self.a2 = (1-2*np.cos(np.pi/4)*ohm+ohm2)/c
            # init states
            self.x1 = 0
            self.x2 = 0

    @cython.ccall
    def run(self, ref: cython.double, measure: cython.double):
        error: cython.double
        error = ref-measure
        P: cython.double
        P = self.kp * error
        D: cython.double
        D = self.kd*(error-self.oldError)/self.dt
        if self.lp_enable :
            ## lp2filter implemented within PID class
            x0: cython.double
            x0  = D     - self.x1*self.a1 - self.x2*self.a2
            out: cython.double
            out = x0*self.b0 + self.x1*self.b1 + self.x2*self.b2
            self.x2 = self.x1
            self.x1 = x0
            D = out
        self.stateI = self.stateI + error * self.dt
        if not(self.Iclamp==0):
            self.stateI = np.clip(self.stateI, -self.Iclamp, self.Iclamp)
        I = self.ki * self.stateI
        self.oldError = error
        return P+D+I
