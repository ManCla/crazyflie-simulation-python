import time # for measurement of tests durations
from os.path import exists # to check if test has already been performed
import numpy as np

# for simulation
from cfSimulator import cfSimulation
from cfSimulator import FlightDataHandler as fdh
from testCases.zTestsSinus import zTestCaseSinus

'''
Script to perform tests with purely sinusoidal inputs.
The script performs tests for each amplitude with each frequency
and stores them as a matrices in csv files in a results directory.
'''

# name of directory with sinus tests data
z_test_directory    = 'z-test-sinus'
# name of directory with aggregated heatmap data
z_results_directory = 'z-aggregated-sinus'

duration  = 50 # duration of flight

frequencies = [ .25, .5, .75, .875, 1, 1.125, 1.25, 1.5, 1.75, 2, 2.25, 2.5, 2.75, 3, 3.5, 4, 4.5]
amplitudes  = [.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 4.75, 5, 5.25, 5.5, 5.75, 6,\
                7, 8, 8.5, 9, 10, 11, 12, 13]

if __name__ == "__main__":

	# initialize store variables
	z_avg_error      = np.zeros((len(amplitudes),len(frequencies)))
	z_avg_error_rel  = np.zeros((len(amplitudes),len(frequencies)))
	z_max_error      = np.zeros((len(amplitudes),len(frequencies)))
	motors_saturated = np.zeros((len(amplitudes),len(frequencies)))
	hit_ground       = np.zeros((len(amplitudes),len(frequencies)))
	z_filtering      = np.zeros((len(amplitudes),len(frequencies)))
	behaviour        = np.zeros((len(amplitudes),len(frequencies)))

	# iterate over the test cases
	for i in range(len(frequencies)):
		freq = frequencies[i]
		for j in range(len(amplitudes)):
			amp = amplitudes[j]
			# compute base
			if (freq==0) :
				base = amp
			else :
				base = amp+1
			if i<=9 or j<=7: # skip tests that are known to fail. This is just a time-saving thing.
				ref = zTestCaseSinus(base, amp, freq)

				# retrieve location and name of test output file
				file_name = ref.trajectoryType+'-'+\
				            str(ref.base)+'-'+\
				            str(ref.amplitude)+'-'+\
				            str(ref.omega)
				file_path = fdh.data_directory+'/'+\
				            z_test_directory+'/'+\
				            file_name

				if not(exists(file_path)):
					# If test has not been executed, run it
					print(" * Executing Test {}".format(file_name))
					sim        = cfSimulation()          # initialize simulation object
					start_test = time.perf_counter()
					storeObj   = sim.run(ref, duration)  # actual test execution
					end_test   = time.perf_counter()
					print(" >> {} took {} seconds".format(file_name, str(end_test-start_test)))
					storeObj.save(z_test_directory+'/'+file_name) # store simulation results
					storeObj.unwrap(storeObj) # TODO -- not needed any longer
				else :
					# If test has been executed open the results file
					print(" * Test {} already executed".format(file_name))
					storeObj = fdh()
					storeObj.open(file_path,True)

				# store test results (independently if test was already executed or not)
				z_avg_error[j,i]      = storeObj.get_z_avg_error_abs()
				z_avg_error_rel[j,i]  = storeObj.get_z_avg_error_rel()
				z_max_error[j,i]      = storeObj.get_z_max_error()
				motors_saturated[j,i] = storeObj.get_motors_saturated()
				hit_ground[j,i]       = storeObj.get_hit_ground()
				z_filtering[j,i]      = storeObj.get_z_filtering()
				behaviour[j,i]        = storeObj.get_behaviour()

	## write results to csv
	out_path = fdh.data_directory+'/'+z_results_directory+'/'
	# frequencies and amplitudes vectors
	np.savetxt(out_path+'frequencies.csv', frequencies, delimiter=',')
	np.savetxt(out_path+'amplitudes.csv', amplitudes, delimiter=',')
	# actual test results
	np.savetxt(out_path+'z_avg_error_abs.csv', z_avg_error, delimiter=',')
	np.savetxt(out_path+'z_avg_error_rel.csv', z_avg_error_rel, delimiter=',')
	np.savetxt(out_path+'z_max_error.csv', z_max_error, delimiter=',')
	np.savetxt(out_path+'motors_saturated.csv', motors_saturated, delimiter=',')
	np.savetxt(out_path+'hit_ground.csv', hit_ground, delimiter=',')
	np.savetxt(out_path+'z_filtering.csv', z_filtering, delimiter=',')
	np.savetxt(out_path+'behaviour.csv', behaviour, delimiter=',')
