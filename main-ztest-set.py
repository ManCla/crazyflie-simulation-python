# for simulation
from cfSimulator import cfSimulation 
from cfSimulator import FlightDataHandler as fdh
from testCases.zTestsSinus import zTestCaseSinus

'''
Script to perform .... TODO: write script description
First idea: iterate over different:
	- shapes (defined by list in refgen object)
	- amplitudes (defined by vector in this script)
	- time speed gains (defined by vector in this script)
'''

# name of directory where to store output data
z_test_directory    = 'z-test-set'

# Duration of test flight
# should be defined according to desired resolution 
# on frequency axis
duration  = 60

time_speeds = [1,2]
amplitudes  = [1,2]

if __name__ == "__main__":
	pass
