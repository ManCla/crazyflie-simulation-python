import numpy as np
from .Physics import cfPhysics
from .Controller import cfPIDController
from .StateEstimator import cfEKF
from .utils.FlightDataHandler import FlightDataHandler
import math
import cython

@cython.cclass
class cfSimulation():

	def __init__(self):
		pass

	def run(self, ref, t_final):
		### simulation parameters
		t_init: cython.double
		t_resolution: cython.double
		noise: cython.double
		useKalmanFilter: cython.int
		quantisation: cython.int
		t_init          = 0
		t_resolution    = 0.001
		noise           = 0      # if non-zero includes measurement noise with given gain
		useKalmanFilter = True   # if true the KF is used for feedback
		quantisation    = False  # if false removes quantisation from flow data


		n_steps: cython.int
		n_steps         = int((t_final-t_init)/t_resolution)

		##########################################
		# initialization of simulation variables #
		##########################################
		physics = cfPhysics()
		ctrl    = cfPIDController()
		est     = cfEKF()

		# initialize storage variables
		t       = np.linspace(t_init,t_final,n_steps)
		u_store = np.zeros((4, n_steps),dtype=long)  #  4 are the number of inputs to the physics model
		x_store = np.zeros((13, n_steps),dtype=long) # 13 are the number of states of the physics model
		acc     = np.zeros((3,n_steps),dtype=long) # inertial measurement
		gyro    = np.zeros((3,n_steps),dtype=long) # inertial measurement
		eta     = np.zeros((3,n_steps),dtype=long) # euler angles
		pxCount = np.zeros((2,n_steps),dtype=long) # pixel count measurement
		zrange  = np.zeros((n_steps),dtype=long)   # z ranging measurement
		set_pt  = np.zeros((3,n_steps),dtype=long) # setpoint fed to cf
		err_fd  = np.zeros((3,n_steps),dtype=long) # kalman innovation from flow measurements
		x_est   = np.zeros((9,n_steps),dtype=long) # state estimated by EKF [pos, vel, eta]

		# initialize loop counter
		i: cython.int
		i = 0

		###################
		# simulation loop #
		###################
		x_store[:,i] = physics.simulate(t[i], u_store[:,i]) # run first physics iteration
		i = i+1

		while i<n_steps: # loop over time steps
			if not i%1000: # progress printout
				print(" -- simulation at time " + str(t[i]))

			set_pt[:,i] = ref.refGen(t[i]) # get reference
			if useKalmanFilter :           # compute control action from estimated state
				u_store[:,i] = ctrl.ctrlCompute(set_pt[:,i],\
				                                x_est[0:3,i-1],\
				                                x_est[3:6,i-1],\
				                                x_est[6:9,i-1],\
				                                gyro[:,i-1])
			else:     # compute control action directly from physics
				u_store[:,i] = ctrl.ctrlCompute(set_pt[:,i],\
				                                x_store[0:3,i-1],\
				                                x_store[3:6,i-1],\
				                                # call to quaternionToEuler is broken for cython optimisations
				                                physics.quaternionToEuler(x_store[6:10,i-1]),\
				                                gyro[:,i-1])
			x_store[:,i] = physics.simulate(t[i], u_store[:,i]) # simulate physics
			
			# store measurements
			# def quaternionToEuler(self, q=x_store[6:10,i]):
			q: cython.double[4]
			q = [0,0,0,0]
			q[0] = x_store[6,i]
			q[1] = x_store[7,i]
			q[2] = x_store[8,i]
			q[3] = x_store[9,i]
			phi: cython.double
			phi1: cython.double
			theta: cython.double
			psi: cython.double
			psi1: cython.double
			phi   = 2*(q[0]*q[1] + q[2]*q[3])
			phi1  = 1-2*(q[1]**2+q[2]**2)
			theta = 2*(q[0]*q[2] - q[3]*q[1])
			psi   = 2*(q[0]*q[3] + q[1]*q[2])
			psi1  = 1-2*(q[2]**2+q[3]**2)
			phi   = math.atan2(phi,phi1)
			theta = math.asin(theta)
			psi   = math.atan2(psi, psi1)

			eta[:,i]     = [phi, theta, psi]
			acc[:,i]     = physics.readAcc(noise)
			gyro[:,i]    = physics.readGyro(noise)
			pxCount[:,i] = physics.readPixelcount(noise, quantisation)
			zrange[i]    = physics.readZRanging(noise)

			# run state estimator
			x_est[:,i], err_fd[:,i] = est.runEKF(acc[:,i],gyro[:,i],pxCount[:,i],zrange[i])

			i=i+1 # increase counter

		##############################################
		# store data as object attributes of storage #
		##############################################

		output = FlightDataHandler()
		output.store(ref.trajectoryType, t, x_store, u_store, eta, \
					 acc, pxCount, set_pt, zrange, err_fd, x_est)

		# return simulation data
		return output
