import time
from os.path import exists

from cfSimulator import cfSimulation
from cfSimulator import FlightDataHandler
from testCases.zControlTests import zTestCase

z_test_directory = 'z-test'

duration  = 20     # duration of flight

frequencies = [0, .25, .5, 1, 2, 3, 4, 8]
amplitudes  = [.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 5, 6, 7, 10, 20]

test_cases = [# STEPS -- OMEGA = 0
			  zTestCase(1,0,0),      # -- good tracking
			  zTestCase(2,0,0),      # -- small innocuous saturation
			  zTestCase(3,0,0),      # saturating -- recovering well
			  zTestCase(5,0,0),      # saturating -- overshoot
			  zTestCase(7,0,0),      # saturating -- eventually recover
			  zTestCase(10,0,0),     # saturating -- eventually recover
			  zTestCase(20,0,0),     # saturating -- crash?
			  # OMEGA = 0.25
			  zTestCase(1,.5 ,.25), # -- good tracking
			  zTestCase(2,1  ,.25), # -- good tracking
			  zTestCase(3,2  ,.25), # -- good tracking
			  zTestCase(3,2.5,.25), # -- good tracking
			  zTestCase(4,3  ,.25), # -- good tracking
			  zTestCase(5,4  ,.25), # saturating at step -- good tracking
			  zTestCase(6,5  ,.25), # saturating at step -- good tracking
			  # OMEGA = 0.5
			  zTestCase(1,.5 ,.5),  # -- good tracking
			  zTestCase(2,1  ,.5),  # -- good tracking
			  zTestCase(3,2  ,.5),  # -- good tracking
			  zTestCase(3,2.5,.5),  # -- good tracking
			  zTestCase(4,3  ,.5),  # -- good tracking
			  zTestCase(5,4  ,.5),  # saturating at step -- good tracking
			  zTestCase(6,5  ,.5),  # saturating at step -- good tracking
			  # OMEGA = 1
			  zTestCase(2,1  ,1),   # -- phase shift
			  zTestCase(2,1.5,1),   # -- phase shift
			  zTestCase(3,2  ,1),   # -- phase shift
			  zTestCase(3,2.5,1),   # -- phase shift
			  zTestCase(4,3.5,1),   # -- phase shift
			  zTestCase(5,4  ,1),   # saturating at step
			  # OMEGA = 2
			  zTestCase(2,1  ,2),   # -- phase shift
			  zTestCase(2,1.5,2),   # -- phase shift
			  zTestCase(3,2  ,2),   # saturating -- phase shift
			  zTestCase(4,3  ,2),   # saturating -- very funky patterns
			  zTestCase(5,4  ,2),   # saturating -- very funky patterns
			  # OMEGA = 3
			  zTestCase(2,1  ,3),   # saturating -- phase shift
			  zTestCase(2,1.5,3),   # saturating -- funny filtering
			  zTestCase(10,5 ,3),   # saturating -- too large step
			  # OMEGA = 4
			  zTestCase(2,0.5,4),   # -- filtering
			  zTestCase(2,1  ,4),   # saturating -- filtering
			  zTestCase(2,1.5,4),   # saturating -- filtering
			  # OMEGA = 8
			  zTestCase(2,0.5,8),   # -- filtering
			  zTestCase(2,1  ,8),   # saturating -- filtering
			  ]

if __name__ == "__main__":

	# iterate over the test cases
	for ref in test_cases:

		# retrieve location and name of test output file
		file_name=ref.trajectoryType+'-'+\
		          str(ref.base)+'-'+\
		          str(ref.amplitude)+'-'+\
		          str(ref.omega)
		file_path=FlightDataHandler.data_directory+'/'+\
		          z_test_directory+'/'+\
		          file_name

		# If test has not been executed, do it
		# otherwise, print performance
		if not(exists(file_path)):
			print(" * Executing Test {}".format(file_name))
			sim = cfSimulation()               # initialize simulation object
			start_test = time.perf_counter()
			storeObj = sim.run(ref, duration)  # actual test execution
			end_test = time.perf_counter()
			print(" >> {} took {} seconds".format(file_name, str(end_test-start_test)))
			storeObj.save(z_test_directory+'/'+file_name) # store simulation results
		else :
			print(" * Test {} already executed".format(file_name.split('/')[-1]),end =".  ")
			storeObj = FlightDataHandler()
			storeObj.open(file_path, file_name.split('/')[-1],True)
			# show flight performance
			print("   Flight performance: {}".format(storeObj.compute_z_error()))
