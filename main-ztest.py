import time

from cfSimulator import cfSimulation
# from testCases.referenceGen import Reference
from testCases.zControlTests import zTestCase

duration  = 5     # duration of flight

test_cases = [zTestCase("step",1,0,0),
			  zTestCase("step",20,0,0),]

if __name__ == "__main__":

	# initialize simulation objects
	sim = cfSimulation()
	# ref = Reference(reference)
	for ref in test_cases:
		# actual test execution
		start_test = time.perf_counter()
		storeObj = sim.run(ref, duration)
		end_test = time.perf_counter()
		print("This test took " + str(end_test-start_test) + " seconds")

		# store simulation results
		storeObj.save()
