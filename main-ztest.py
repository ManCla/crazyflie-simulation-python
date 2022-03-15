import time
from os.path import exists
import code

# for simulation
from cfSimulator import cfSimulation
from cfSimulator import FlightDataHandler
from testCases.zControlTests import zTestCase

# for plotting
import matplotlib.pyplot as plt
import numpy as np

z_test_directory    = 'z-test'
z_results_directory = 'z-aggregated'

duration  = 30     # duration of flight

frequencies = [0, .25, .5, 1, 2, 3, 4, 8]
amplitudes  = [.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 20]

if __name__ == "__main__":

	z_error_cumulative     = np.zeros((len(amplitudes),len(frequencies)))
	z_error_rel_cumulative = np.zeros((len(amplitudes),len(frequencies)))
	motors_saturated       = np.zeros((len(amplitudes),len(frequencies)))
	hit_ground             = np.zeros((len(amplitudes),len(frequencies)))

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
			ref = zTestCase(base, amp, freq)

			# retrieve location and name of test output file
			file_name = ref.trajectoryType+'-'+\
			            str(ref.base)+'-'+\
			            str(ref.amplitude)+'-'+\
			            str(ref.omega)
			file_path = FlightDataHandler.data_directory+'/'+\
			            z_test_directory+'/'+\
			            file_name

			# If test has not been executed, do it otherwise, print performance
			if not(exists(file_path)):
				print(" * Executing Test {}".format(file_name))
				sim        = cfSimulation()          # initialize simulation object
				start_test = time.perf_counter()
				storeObj   = sim.run(ref, duration)  # actual test execution
				end_test   = time.perf_counter()
				print(" >> {} took {} seconds".format(file_name, str(end_test-start_test)))
				storeObj.save(z_test_directory+'/'+file_name) # store simulation results
			else :
				print(" * Test {} already executed".format(file_name.split('/')[-1]))
				storeObj = FlightDataHandler()
				storeObj.open(file_path, file_name.split('/')[-1],True)
				# show flight performance
				z_error_rel_cumulative[j,i] = storeObj.compute_z_error_rel_cumulative()
				motors_saturated[j,i]       = storeObj.motors_saturated()
				hit_ground[j,i]             = storeObj.hit_ground()

	## write results to csv
	out_path = FlightDataHandler.data_directory+'/'+z_results_directory+'/'
	np.savetxt(out_path+'frequencies.csv', frequencies, delimiter=',')
	np.savetxt(out_path+'amplitudes.csv', amplitudes, delimiter=',')
	np.savetxt(out_path+'z_error_rel_cumulative.csv', z_error_rel_cumulative, delimiter=',')
	np.savetxt(out_path+'motors_saturated.csv', motors_saturated, delimiter=',')
	np.savetxt(out_path+'hit_ground.csv', hit_ground, delimiter=',')
