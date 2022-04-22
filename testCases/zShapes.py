import numpy as np


class zReferenceShapes():
    
    def __init__(self, refType):
        # reference trajectory desired type
        self.trajectoryType = refType

    def refGen(self, t):
        
        if   (self.trajectoryType == "step")   :return self.step(t)

    def step(self, t):
        if t<4:
            return np.array([0,0,2])
        if t<6:
            return np.array([0,0,4])
        if t<7:
            return np.array([0,0,2])
        if t<7.5:
            return np.array([0,0,4])
        if t<8.5:
            return np.array([0,0,2])
        if t<9:
            return np.array([0,0,4])
        if t<9.25:
            return np.array([0,0,2])
        if t<9.5:
            return np.array([0,0,4])
        if t<9.75:
            return np.array([0,0,2])
        if t<10:
            return np.array([0,0,4])
        if t<11:
            return np.array([0,0,2])
        if t<12:
            return np.array([0,0,4])
        else:
            return np.array([0,0,2])
