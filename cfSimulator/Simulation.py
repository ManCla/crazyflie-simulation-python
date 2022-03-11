import numpy as np
from .Physics import cfPhysics
from .Controller import cfPIDController
from .StateEstimator import cfEKF
from .utils.FlightDataHandler import FlightDataHandler

### simulation parameters
t_init          = 0
t_resolution    = 0.001
noise           = 0      # if non-zero includes measurement noise with given gain
useKalmanFilter = True   # if true the KF is used for feedback
quantisation    = False  # if false removes quantisation from flow data


class cfSimulation():

	def __init__(self):
		pass

	def run(self, ref, t_final):

		n_steps         = int((t_final-t_init)/t_resolution)

		##########################################
		# initialization of simulation variables #
		##########################################
		physics = cfPhysics()
		ctrl    = cfPIDController(physics.config, physics.b,\
		                          physics.I, physics.m, physics.g,\
		                          physics.k, physics.l)
		est     = cfEKF(physics.g)

		# initialize storage variables
		t       = np.linspace(t_init,t_final,n_steps)
		u_store = np.zeros((physics.n_inputs, n_steps))
		x_store = np.zeros((physics.n_states, n_steps))
		acc     = np.zeros((3,n_steps)) # inertial measurement
		gyro    = np.zeros((3,n_steps)) # inertial measurement
		eta     = np.zeros((3,n_steps)) # euler angles
		pxCount = np.zeros((2,n_steps)) # pixel count measurement
		zrange  = np.zeros((n_steps))   # z ranging measurement
		set_pt  = np.zeros((3,n_steps)) # setpoint fed to cf
		err_fd  = np.zeros((3,n_steps)) # kalman innovation from flow measurements
		x_est   = np.zeros((9,n_steps)) # state estimated by EKF [pos, vel, eta]

		# initialize loop counter
		i = 0

		###################
		# simulation loop #
		###################
		x_store[:,i] = physics.simulate(t[i], u_store[:,i]) # run first physics iteration
		i = i+1

		while i<n_steps: # loop over time steps
			if not i%500: # progress printout
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
				                                physics.quaternionToEuler(x_store[6:10,i-1]),\
				                                gyro[:,i-1])
			x_store[:,i] = physics.simulate(t[i], u_store[:,i]) # simulate physics
			
			# store measurements
			eta[:,i]     = physics.quaternionToEuler(x_store[6:10,i])
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
		output.type    = "mitl"
		output.test    = ref.trajectoryType
		output.t       = t
		output.x       = x_store
		output.u       = u_store

		# extract states
		output.pos     = x_store[0:3,:]
		output.vel     = x_store[3:6,:]
		output.eta     = eta
		output.gyro    = x_store[10:13,:]

		# measurements and other cf data
		output.acc     = acc
		output.pxCount = pxCount
		output.set_pt  = set_pt
		output.zrange  = zrange
		output.err_fd  = err_fd
		output.est_pos = x_est[0:3,:]
		output.est_vel = x_est[3:6,:]
		output.est_eta = x_est[6:9,:]

		# return simulation data
		return output
