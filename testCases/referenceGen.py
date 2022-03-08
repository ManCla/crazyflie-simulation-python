import numpy as np

hovering_height = 1.5
size = 1.0
omega = 5.5

class Reference():
    
    def __init__(self, refType):
        # reference trajectory desired type
        self.trajectoryType = refType

    def refGen(self, t):
        # start by hovering 
        if t<2 : 
            return np.array([0,0,hovering_height])
        # actual flight sequence
        if   (self.trajectoryType == "step") :
            if t<6:
                return np.array([size,0,hovering_height])
            return np.array([0,size,hovering_height])
        elif (self.trajectoryType == "zsinus") :
            return np.array([0,0,size*np.sin(omega*t)+hovering_height])
        elif (self.trajectoryType == "zramp") :
            return np.array([0,0,size*t+hovering_height])
        # NOTE: following wont work when using kalman filter since 
        #       position estimate is open loop
        elif (self.trajectoryType == "xsinus") :
            return np.array([np.sin(0.3*t),0,hovering_height])
        elif (self.trajectoryType == "ysinus") :
            return np.array([0,np.sin(0.3*t),hovering_height])
        elif (self.trajectoryType == "circle") :
            return np.array([np.cos(0.8*t),np.sin(0.8*t),hovering_height])
        elif (self.trajectoryType == "spiral") :
            return np.array([0.01*t*np.cos(1.2*t),0.01*t*np.sin(1.2*t),hovering_height])
