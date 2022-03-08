import numpy as np

class zTestCase():
    
    def __init__(self, refType, base=0.5, amplitude=0.2, omega=2):
        # reference trajectory desired type
        self.trajectoryType = refType
        self.base = base
        self.amplitude = amplitude
        self.omega = omega

    def refGen(self, t):
        if   (self.trajectoryType == "step") :
            z = self.base
        elif (self.trajectoryType == "sinus") :
            z = self.amplitude*np.sin(omega*t)+self.base
        elif (self.trajectoryType == "ramp") :
            z = self.amplitude*t+self.base
        return np.array([0,0,z])