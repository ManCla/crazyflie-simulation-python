from os.path import exists # to check if test has already been performed
import time # for measurement of tests durations

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
duration  = 60

### Shapes
# get from the test shapes class the list of available
# time shape: each of them will be used to generate
# test cases
shapes = zShapes.shapes

### Time Coefficients
# for each time coefficient tests will be performed
# using the coefficient to shrink a dilate time
time_speeds = [1,2,3,4,5,6,7,8]

### Amplitude Coefficients
# for each amplitude coefficient tests will be performed
# using the coefficient to scale the amplitude of the signal
amplitudes  = [1,2,3,4,5,6,7,8]

####################
### TEST WARM UP ###
####################

### Offset
# parameter for basic hovering output
# just to get the drone flying
offset = 0.25

### Settle
# parameter that sets the time to wait in the tests
# till the drone starts hovering 
settle = 5

#####################
### MAIN FUNCTION ###
#####################

if __name__ == "__main__":

    for s, a, t in itertools.product(shapes,time_speeds,amplitudes):

        # retrieve location and name of test output file
        # name formed by shape, amplitude, and time
        file_name = s+'-'+str(a)+'-'+str(t)
        file_path = fdh.data_directory+'/'+z_test_directory+'/'+file_name

        if not(exists(file_path)):
            # If test has not been executed, run it
            print(" * Executing Test {}".format(file_name))
        
            ref = zShapes(s,a,t,offset,settle)
            sim = cfSimulation()
            start_test = time.perf_counter()
            storeObj   = sim.run(ref, duration)  # actual test execution
            end_test   = time.perf_counter()
            print(" >> {} took {} seconds".format(file_name, str(end_test-start_test)))
            storeObj.save(z_test_directory+'/'+file_name) # store simulation results
        else :
            # If test has been executed skip it
            print(" * Test {} already executed".format(file_name))