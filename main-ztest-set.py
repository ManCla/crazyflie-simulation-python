from os.path import exists # to check if test has already been performed
import time # for measurement of tests durations

# for simulation
from cfSimulator import cfSimulation 
from cfSimulator import ZAnalysis as fdh
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
# defined according to the desired resolution on "frequency" axis
# which is given by the number of periods used
min_duration  = 60
min_periods = 5

### Shapes
# get from the test shapes class the list of available
# time shape: each of them will be used to generate
# test cases
shapes = zShapes.shapes

### Time Coefficients
# for each time coefficient tests will be performed
# using the coefficient to shrink or dilate time
time_speeds = [0.25,0.5,0.75,1,\
               1.25,1.5,1.75,2,\
               2.25,2.5,2.75,3,\
               3.25,3.5,3.75,4,5,6]

### Amplitude Coefficients
# for each amplitude coefficient tests will be performed
# using the coefficient to scale the amplitude of the signal
amplitudes  = [0.5,0.75,1,\
               1.5,     2,\
               2.5,     3,\
               3.5,     4,\
               4.5,     5,\
                        6,\
                        7,\
                        8,\
                        9]

####################
### TEST WARM UP ###
####################

### Offset
# parameter for basic hovering output
# just to get the drone flying
offset = 1

### Settle
# parameter that sets the time to wait in the tests
# till the drone starts hovering 
settle = 5

#####################
### MAIN FUNCTION ###
#####################

if __name__ == "__main__":

    for s, a, t in itertools.product(shapes,amplitudes,time_speeds):

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
            # duration of a test is at least 5 times the period of the repeated input
            duration = max(min_duration, settle + min_periods*(zShapes.base_period/t))
            storeObj   = sim.run(ref, duration)  # actual test execution
            end_test   = time.perf_counter()
            print(" >> {} took {} seconds".format(file_name, str(end_test-start_test)))
            storeObj.save(z_test_directory+'/'+file_name) # store simulation results
        else :
            # If test has been executed skip it
            print(" * Test {} already executed".format(file_name))
