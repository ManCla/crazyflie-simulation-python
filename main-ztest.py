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

duration  = 50     # duration of flight

frequencies = [ .25, .5, .75, .875, 1, 1.125, 1.25, 1.5, 1.75, 2, 2.25, 2.5, 2.75, 3, 3.5, 4, 4.5]
amplitudes  = [.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 4.75, 5, 5.25, 5.5, 5.75, 6,\
                7, 8, 8.5, 9, 10, 11, 12, 13]

if __name__ == "__main__":

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
			if i<=9 or j<=7:
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
					storeObj.unwrap(storeObj)
				else :
					print(" * Test {} already executed".format(file_name))
					storeObj = FlightDataHandler()
					storeObj.open(file_path,True)
					# show flight performance
				z_avg_error[j,i]      = storeObj.compute_z_avg_error_abs()
				z_avg_error_rel[j,i]  = storeObj.compute_z_avg_error_rel()
				z_max_error[j,i]      = storeObj.compute_z_max_error()
				motors_saturated[j,i] = storeObj.motors_saturated()
				hit_ground[j,i]       = storeObj.hit_ground()
				z_filtering[j,i]      = storeObj.compute_z_filtering()
				behaviour[j,i]        = storeObj.compute_behaviour()

	## write results to csv
	out_path = FlightDataHandler.data_directory+'/'+z_results_directory+'/'
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
