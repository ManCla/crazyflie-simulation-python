import numpy as np

class Reference():
    
    def __init__(self, refType):
        # reference trajectory desired type
        self.trajectoryType = refType

    def refGen(self, t):
        if t<2 : 
            return np.array([0,0,0.5])
        if   (self.trajectoryType == "step") :
            if t<6:
                return np.array([0.2,0,0.5])
            return np.array([0,0.2,0.5])
        elif (self.trajectoryType == "zsinus") :
            return np.array([0,0,np.sin(0.2*t)+0.5])
        # NOTE: following wont work when using kalman filter since 
        #       position estimate is open loop
        elif (self.trajectoryType == "xsinus") :
            return np.array([np.sin(0.3*t),0,0.5])
        elif (self.trajectoryType == "ysinus") :
            return np.array([0,np.sin(0.3*t),0.5])
        elif (self.trajectoryType == "circle") :
            return np.array([np.cos(0.8*t),np.sin(0.8*t),0.5])
        elif (self.trajectoryType == "spiral") :
            return np.array([0.01*t*np.cos(1.2*t),0.01*t*np.sin(1.2*t),0.5])
