import numpy as np

settle = 5 # wait this time to let the step settle before feeding sinusoidal

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
            if t>settle:
                z = self.amplitude*np.sin(self.omega*(t-settle))+self.base
            else : 
                z = self.base
        elif (self.trajectoryType == "ramp") :
            z = self.amplitude*t+self.base
        return np.array([0,0,z])
