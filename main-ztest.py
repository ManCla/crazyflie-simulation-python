import time
from os.path import exists

from cfSimulator import cfSimulation
# from testCases.referenceGen import Reference
from testCases.zControlTests import zTestCase

duration  = 15     # duration of flight

test_cases = [# STEPS
			  zTestCase("step",1,0,0),      # -- good tracking
			  zTestCase("step",2,0,0),      # -- small innocuous saturation
			  zTestCase("step",3,0,0),      # saturating -- recovering well
			  zTestCase("step",5,0,0),      # saturating -- recovering well
			  zTestCase("step",10,0,0),     # saturating -- almost recover
			  zTestCase("step",20,0,0),     # saturating -- crash
			  # OMEGA = 0.25
			  zTestCase("sinus",2,1,.25),   # -- good tracking
			  # OMEGA = 0.5
			  zTestCase("sinus",1,.5 ,.5),  # -- good tracking
			  zTestCase("sinus",2,1  ,.5),  # -- good tracking
			  zTestCase("sinus",3,2  ,.5),  # -- good tracking
			  zTestCase("sinus",3,2.5,.5),  # -- good tracking
			  zTestCase("sinus",4,3  ,.5),  # -- good tracking
			  zTestCase("sinus",5,4  ,.5),  # saturating because of initial step
			  zTestCase("sinus",6,5  ,.5),  # saturating because of initial step
			  # OMEGA = 1
			  zTestCase("sinus",2,1  ,1),   # -- phase shift
			  zTestCase("sinus",2,1.5,1),   # -- phase shift
			  zTestCase("sinus",3,2  ,1),   # -- phase shift
			  zTestCase("sinus",3,2.5,1),   # -- phase shift
			  zTestCase("sinus",4,3.5,1),   # -- phase shift
			  zTestCase("sinus",5,4  ,1),   # saturating because of initial step
			  # OMEGA = 2
			  zTestCase("sinus",2,1  ,2),   # -- phase shift
			  zTestCase("sinus",2,1.5,2),   # -- phase shift
			  zTestCase("sinus",3,2  ,2),   # saturating -- phase shift
			  # OMEGA = 3
			  zTestCase("sinus",2,1  ,3),   # saturating -- phase shift
			  zTestCase("sinus",2,1.5,3),   # saturating -- funny filtering
			  zTestCase("sinus",10,5 ,3),   # saturating -- too large step
			  # OMEGA = 4
			  zTestCase("sinus",2,0.5,4),   # -- filtering
			  zTestCase("sinus",2,1  ,4),   # saturating -- filtering
			  zTestCase("sinus",2,1.5,4),   # saturating -- filtering
			  # OMEGA = 8
			  zTestCase("sinus",2,0.5,8),   # -- filtering
			  zTestCase("sinus",2,1  ,8),   # saturating -- filtering
			  ]

if __name__ == "__main__":

	# ref = Reference(reference)
	for ref in test_cases:
		file_name='z-test/'+ref.trajectoryType+'-'+\
		          str(ref.base)+'-'+str(ref.amplitude)+'-'+str(ref.omega)
		if not(exists('flightdata/'+file_name)):
			print(" * Executing Test {}".format(file_name.split('/')[-1]))
			sim = cfSimulation() # initialize simulation objects
			start_test = time.perf_counter()
			storeObj = sim.run(ref, duration) # actual test execution
			end_test = time.perf_counter()
			print(" -- This test took " + str(end_test-start_test) + " seconds")

			# store simulation results
			storeObj.save(file_name)
		else :
			print(" * Test {} already executed".format(file_name.split('/')[-1]))
