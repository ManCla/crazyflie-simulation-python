import numpy as np
from .lp2Filter import lp2Filter

'''
    PID class
'''
class PID():
    def __init__(self, kp, ki, kd, dt, lp_enable, lp_rate, lp_cutoff, Iclamp):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt 
        self.oldError = 0
        self.stateI = 0
        self.Iclamp  = Iclamp
        self.lp_enable = lp_enable
        if self.lp_enable :
            self.lpf = lp2Filter(lp_rate, lp_cutoff)

    def run(self, ref, measure):
        error = ref-measure
        P = self.kp * error
        D = self.kd*(error-self.oldError)/self.dt
        if self.lp_enable :
            D = self.lpf.filter(D)
        self.stateI = self.stateI + error * self.dt
        if not(self.Iclamp==0):
            self.stateI = np.clip(self.stateI, -self.Iclamp, self.Iclamp)
        I = self.ki * self.stateI
        self.oldError = error
        return P+D+I
