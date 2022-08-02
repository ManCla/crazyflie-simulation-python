import time
import numpy as np
from cfSimulator import cfSimulation, zTest

reference = "ramp" # type of reference sequence

if __name__ == "__main__":

	# initialize simulation objects
	sim = cfSimulation()
	ref = zTest(reference,1,1)
	duration  = ref.duration

	# actual test execution
	start_test = time.perf_counter()
	storeObj = sim.run(ref, inital_drone_state=ref.get_initial_state())
	end_test = time.perf_counter()
	print("This test took " + str(end_test-start_test) + " seconds")

	# store simulation results
	storeObj.save()
