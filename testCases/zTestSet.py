import numpy as np

'''
Class that generates general test cases according to...
TODO: write class description
 - base period for shape functions is 10 seconds
 - base amplitude range for shape functions is [0,1]
 - avoid discontinuity at beginning of shape (after warm up)
'''

base_period = 10

class zShapes():
    
    # list of available shapes
    shapes = ['steps', 'trapezoidal', 'triangular','sinus']

    def __init__(self, shape, amplitude, time, offset, settle):
        
        if not(shape in self.shapes):
            print('Shape element must be present in zShapes.shapes')
            exit()
        self.shape = shape
        self.trajectoryType = shape # needed so we can store in test result object
        self.amplitude = amplitude
        self.time = time
        self.offset = offset
        self.settle = settle

    def refGen(self, t):
        # function called by the simulation object that returns the 
        # current value for the reference
        # This function also includes the computations that are
        # the same for the different shapes (amp,time scaling and offsets)

        if t<self.settle : # implement warm up
            return np.array([0,0,self.offset])

        t_scaled = self.time*(t-self.settle)    # scale time

        # switch statement over the different possible shapes
        if self.shape=='steps' :
            shape_term = self.steps(t_scaled)
        elif self.shape=='trapezoidal' :
            shape_term = self.trapezoidal(t_scaled)
        elif self.shape=='triangular' :
            shape_term = self.triangular(t_scaled)
        elif self.shape=='sinus' :
            shape_term = self.sinus(t_scaled)

        z = self.offset + self.amplitude * shape_term # apply offset and scaling
        return np.array([0,0,z])

    #######################
    ### SHAPE FUNCTIONS ###
    #######################

    def steps(self, t):
        if (t % base_period) < base_period/2 :        # if we are in first half of the period
            return 0
        else :                                        # if we are in second half of the period
            return 1

    def triangular(self, t):
        if (t % base_period) < base_period/2 :        # if we are in first half of the period
            return (t % base_period)/(base_period/2)
        else :                                        # if we are in second half of the period
            return 1-((t-base_period/2) % base_period)/(base_period/2)

    def trapezoidal(self, t):
        if (t % base_period) < base_period/4 :
            # if we are in first quarter of the period
            return (t % base_period)/(base_period/4)
        elif ((t % base_period) > base_period/4)   and ((t % base_period) < base_period/2) :
            return 1
        elif ((t % base_period) > base_period/2)   and ((t % base_period) < base_period*3/4) :
            return 1-((t-base_period/2) % base_period)/(base_period/4)
        else :
            return 0

    def sinus(self, t):
        # make sinus start from 270 degrees so that we avoid discontinuities in reference
        return 1+np.sin((np.pi*3/2)+((t % base_period)/base_period)*2*np.pi)
