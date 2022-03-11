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

z_test_directory = 'z-test'

duration  = 20     # duration of flight

frequencies = [0, .25, .5, 1, 2, 3, 4, 8]
amplitudes  = [.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 5, 6, 7, 10, 20]

if __name__ == "__main__":

	error      = np.zeros((len(amplitudes),len(frequencies)))
	saturation = np.zeros((len(amplitudes),len(frequencies)))

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
				print(" * Test {} already executed".format(file_name.split('/')[-1]),end =".  ")
				storeObj = FlightDataHandler()
				storeObj.open(file_path, file_name.split('/')[-1],True)
				# show flight performance
				print("   Flight performance: {}".format(storeObj.compute_z_error()))
			error[j,i] = storeObj.compute_z_error()
			saturation[j,i] = storeObj.motors_saturated()

	# plot error heat map
	fig, ax = plt.subplots(1, 2)

	ax[0].imshow(error, cmap='hot', interpolation='nearest',origin='lower')
	ax[0].set_title('error')
	plt.xticks(range(len(frequencies)),list(map(str,frequencies)))
	plt.yticks(range(len(amplitudes)),list(map(str,amplitudes)))
	ax[1].imshow(saturation, cmap='hot', interpolation='nearest',origin='lower')
	ax[1].set_title('saturation')
	plt.xticks(range(len(frequencies)),list(map(str,frequencies)))
	plt.yticks(range(len(amplitudes)),list(map(str,amplitudes)))
	plt.show()
	code.interact(local=locals())