import time

from cfSimulator import cfSimulation
from testCases.referenceGen import Reference

reference = "step" # type of reference sequence
duration  = 15     # duration of flight

if __name__ == "__main__":

	# initialize simulation objects
	sim = cfSimulation()
	ref = Reference(reference)

	# actual test execution
	start_test = time.perf_counter()
	storeObj = sim.run(ref, duration)
	end_test = time.perf_counter()
	print("This test took " + str(end_test-start_test) + " seconds")

	# store simulation results
	storeObj.save()
