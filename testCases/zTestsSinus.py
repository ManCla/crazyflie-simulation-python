import numpy as np

settle = 10 # wait this time to let the step settle before feeding sinusoidal

'''
this class implements a test case of the Z controller on the base of 
 * an amplitude parameter
 * a frequency parameter
 * a base offset parameter
'''
class zTestCaseSinus():
    
    def __init__(self, base=0.5, amplitude=0.2, omega=2):
        # reference trajectory desired type
        if omega==0:
            self.trajectoryType = "step"
        else: 
            self.trajectoryType = "sinus"
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
