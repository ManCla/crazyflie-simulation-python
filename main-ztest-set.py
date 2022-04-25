# for simulation
from cfSimulator import cfSimulation 
from cfSimulator import FlightDataHandler as fdh
from testCases.zTestSet import zShapes

# for nice iteration loop
import itertools

'''
Script to perform .... TODO: write script description
First idea: iterate over different:
    - shapes (defined by list field in refgen object)
    - amplitudes (defined by vector in this script)
    - time speed gains (defined by vector in this script)
'''

# name of directory where to store output data
# (under the directory pointed by FlightDataHandler)
z_test_directory    = 'z-test-set'

#######################
### TEST PARAMETERS ###
#######################

# Duration of test flight:
# should be defined according to the desired resolution
# on "frequency" axis
duration  = 2

### Shapes
# get from the test shapes class the list of available
# time shape: each of them will be used to generate
# test cases
shapes = zShapes.shapes

### Time Coefficients
# for each time coefficient tests will be performed
# using the coefficient to shrink a dilate time
time_speeds = [1,2]

### Amplitude Coefficients
# for each amplitude coefficient tests will be performed
# using the coefficient to scale the amplitude of the signal
amplitudes  = [1,2]

#####################
### MAIN FUNCTION ###
#####################

if __name__ == "__main__":

    for s, a, t in itertools.product(shapes,time_speeds,amplitudes):
        print(s,a,t)

