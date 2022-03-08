import numpy as np

'''
    Second order low pass filter used to filter derivative action
'''
class lp2Filter():
    def __init__(self, sample_freq, cutoff_freq):
        fr   = sample_freq/cutoff_freq
        ohm  = np.tan(np.pi/fr)
        ohm2 = ohm**2
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

    def filter(self, sample):
        x0  = sample     - self.x1*self.a1 - self.x2*self.a2
        out = x0*self.b0 + self.x1*self.b1 + self.x2*self.b2
        self.x2 = self.x1
        self.x1 = x0
        return out
