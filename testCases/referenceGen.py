import numpy as np

hovering_height = 0.5

class Reference():
    
    def __init__(self, refType):
        # reference trajectory desired type
        self.trajectoryType = refType

    def refGen(self, t):
        if t<2 : 
            return np.array([0,0,hovering_height])
        if   (self.trajectoryType == "step") :
            step_size = 0.5
            if t<6:
                return np.array([step_size,0,hovering_height])
            return np.array([0,step_size,hovering_height])
        elif (self.trajectoryType == "zsinus") :
            return np.array([0,0,np.sin(0.2*t)+hovering_height])
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
