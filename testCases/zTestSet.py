import numpy as np

'''
Class that generates general test cases according to...
TODO: write class description
'''

class zShapes():
    
    # list of available shapes
    shapes = ['trapezoidal', 'steps', 'triangular']

    # minimum output
    base = 1

    def __init__(self):
        pass

    def refGen(self, t):
        # function called by the simulation object that returns the 
        # current value for the reference
        pass

    def step(self, t):
        pass
