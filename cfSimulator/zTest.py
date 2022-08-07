import numpy as np

'''
Class that generates general test cases according to...
TODO: write class description
 - base period for shape functions is 10 seconds
 - base amplitude range (max-min) for shape functions is [0,1]
 - avoid discontinuity at beginning of shape (after warm up)
To add a new shape:
 - add shape name to list
 - write utility function that creates pattern over a standard time of 1s
 - add elif selection in refGen function that calls the function above
'''

class zTest():
    
    # list of available shapes
    shapes = ['steps', 'ramp', 'trapezoidal', 'triangular', 'sinus', 'ud1', 'impulse']
    base_period = 1
    offset      = 1 # [m]
    settle      = 5 # [s]
    num_periods = 5 # [ ] number of periods of input to repeat

    def __init__(self, shape, amplitude, time_coef):
        
        if not(shape in self.shapes):
            print('Shape element must be present in zShapes.shapes')
            exit()
        self.shape = shape
        self.trajectoryType = shape # needed so we can store in test result object
        self.amplitude = amplitude
        self.time_coef = time_coef
        self.duration = self.settle + self.num_periods * (self.base_period/time_coef)

        self.initial_state = np.zeros(13)
        self.initial_state[2] = self.offset
        self.initial_state[6] = 1 # attitude quaternion has always norm 1

    def get_initial_state(self):
        return self.initial_state

    def refGen(self, t):
        # function called by the simulation object that returns the 
        # current value for the reference
        # This function also includes the computations that are
        # the same for the different shapes (amp,time scaling and offsets)

        t_scaled = self.time_coef*(t)    # scale time

        # switch statement over the different possible shapes
        if self.shape=='steps' :
            shape_term = self.steps(t_scaled)
        elif self.shape=='trapezoidal' :
            shape_term = self.trapezoidal(t_scaled)
        elif self.shape=='triangular' :
            shape_term = self.triangular(t_scaled)
        elif self.shape=='sinus' :
            shape_term = self.sinus(t_scaled)
        elif self.shape=='ud1' :
            shape_term = self.ud1(t_scaled)
        elif self.shape=='impulse' :
            shape_term = self.impulse(t_scaled)
        elif self.shape=='ramp' :
            shape_term = self.ramp(t_scaled)

        z = self.offset + self.amplitude * shape_term # apply offset and scaling
        return np.array([0,0,z])

    #######################
    ### SHAPE FUNCTIONS ###
    #######################
    # cannot pre-generate vector because we cannot know a priori the sampling

    def steps(self, t):
        if (t % self.base_period) < self.base_period/2 :        # if we are in first half of the period
            return 0
        else :                                        # if we are in second half of the period
            return 1

    def triangular(self, t):
        if (t % self.base_period) < self.base_period/2 :        # if we are in first half of the period
            return (t % self.base_period)/(self.base_period/2)
        else :                                        # if we are in second half of the period
            return 1-((t-self.base_period/2) % self.base_period)/(self.base_period/2)

    def trapezoidal(self, t):
        if (t % self.base_period) < self.base_period/4 :
            # if we are in first quarter of the period
            return (t % self.base_period)/(self.base_period/4)
        elif ((t % self.base_period) > self.base_period/4)   and ((t % self.base_period) < self.base_period/2) :
            return 1
        elif ((t % self.base_period) > self.base_period/2)   and ((t % self.base_period) < self.base_period*3/4) :
            return 1-((t-self.base_period/2) % self.base_period)/(self.base_period/4)
        else :
            return 0

    def sinus(self, t):
        # make sinus start from 270 degrees so that we avoid discontinuities in reference
        # amplitude is 0.5 because we want max-min=1
        return 0.5+0.5*np.sin((np.pi*3/2)+((t % self.base_period)/self.base_period)*2*np.pi)

    def ud1(self,t):
        percentage_period = (t % self.base_period) / self.base_period
        if percentage_period<=0.25 :
            return (percentage_period)/0.25
        if percentage_period<=0.5  :
            return 0.5+((percentage_period-0.25) % 0.05)*10
        if percentage_period<=0.65 :
            return 0.5
        return 0.5+0.5*np.sin(((percentage_period-0.15) % 1)*4*np.pi)

    def impulse(self,t):
        # basically a square wave with duty cycle at 2%
        percentage_period = (t % self.base_period) / self.base_period
        if percentage_period<=0.02 :
            return 1
        return 0

    def ramp(self,t):
        # basically a normalized modulo function
        percentage_period = (t % self.base_period) / self.base_period
        return percentage_period
