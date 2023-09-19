"""
physical model of the drone. 
Implements a class that stores the physical state of the drone and 
allows to simulate its physic for an arbitrary dt
"""

import numpy as np
import math
import scipy.integrate as intgr
import sys

import random as rnd

## error classes
class Integration_Failed(BaseException):
    pass

class Drone_Crash(BaseException):
    pass

class cfPhysics():
	def __init__(self, seed=1, do_not_reset_seed=False):
		# Parameters
		self.g   = 9.81       # m/s^2 
		self.m   = 0.027+0.004      # kg
		self.l   = 0.046      # m
		self.k   = 2.2e-8     # N m s^2
		self.b   = 2e-9       # N s^2
		self.Ism = 3e-6       # kg*m^2
		self.I   = [1.66e-5, 1.66e-5, 2.93e-5]  # kg*m^2
		self.A   = [0.92e-6, 0.91e-6, 1.03e-6]  # kg/s
		self.config =  "cross" # quadcopter formation: "plus" or "cross" configuration
		
		# Rotor speed saturations
		self.omega_min_lim = 000.001   # rad/s
		self.omega_max_lim = 2500.0  # rad/s

		# Model size
		self.n_states = 13  # Number of states
		self.n_inputs = 4   # Number of inputs

		# States 
		# Initialize State Conditions
		self.x = np.zeros(self.n_states)
		self.x[6] = 1 # attitude quaternion has always norm 1
		self.currentTime = 0.0 # (relative) time at which the model is

		# Variales for measurements computation
		self.acc = np.array([0,0,0])   # acceleration
		self.R   = np.array([[1,0,0],
		                     [0,1,0],
		                     [0,0,1]]) # rotation matrix

		# Measurement Noise Parameters
		if not(do_not_reset_seed):
			rnd.seed(seed)
		self.accNoiseVar  = np.array([0.5,0.5,1.0]) # accelerometer noise variance
		self.gyroNoiseVar = np.array([0.1,0.1,0.1]) # gyro noise variance
		self.flowNoiseVar = np.array([2, 2])        # flowdeck noise variance
		# zRagner noise model coefficients
		self.expPointA = 2.5 
		self.expStdA   = 0.0025
		self.expCoeff  = 2.92135


	##############################
	### MATH UTILITY FUNCTIONS ###
	##############################

	def quatNormal(self, q):
		# utilitiy function: normalize a quaternion
		# input : quaternion -- np array 4x1
		# output: quaternion -- np array 4x1
		return q/np.sqrt(q[0]**2+q[1]**2+q[2]**2+q[3]**2)

	def skewSymmetricOp(self, x, y, z):
		# utilitiy function: compute the skew symmetric matrix associate to a 3-dim vector
		# input : three components of the vector (in order)
		# output: three-by-three skew symmetric matrix
		mcross = np.array([[ 0, -z,  y],\
	                       [ z,  0, -x],\
	                       [-y,  x,  0]])
		return mcross

	def quaternionToEuler(self, q):
		#utility function to translate quaternion in Euler angles for plotting
		phi   = math.atan2(2*(q[0]*q[1] + q[2]*q[3]), 1-2*(q[1]**2+q[2]**2))
		theta = math.asin(2*(q[0]*q[2] - q[3]*q[1]))
		psi   = math.atan2(2*(q[0]*q[3] + q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
		eta = np.array([phi, theta, psi])
		return eta

	def computeR(self, q):
		# computes rotation matrix from quaternion q
		q = self.quatNormal(q)
		qw = q[0]
		qx = q[1]
		qy = q[2]
		qz = q[3]
		R = np.array([[qw**2+qx**2-qy**2-qz**2,         2*(qx*qy-qw*qz),         2*(qx*qz+qw*qy)],\
	                  [        2*(qx*qy+qw*qz), qw**2-qx**2+qy**2-qz**2,         2*(qy*qz-qw*qx)],\
	                  [        2*(qx*qz-qw*qy),         2*(qy*qz+qw*qx), qw**2-qx**2-qy**2+qz**2]])
		return R

	######################################
	### MAPPING FUNCTIONS PWM<->THRUST ###
	###################################### 

	def pwmToThrust(self, pwm):
		# input : PWM signal to the 4 motors
		# output: thrust of each rotor
		# if ((pwm<0).any() or (pwm>65535).any()) :
		# 	print("pwm command out of range")
		# print(pwm)
		pwm = np.clip(pwm, 0, 65535)

		d = pwm/65535.0 # duty cycle 
		beta1 = 0.35
		beta2 = 0.26
		T = beta1*d + beta2*(d**2)
		return T 

	def thrustToOmega(self, T):
		# input : thrust generated by each motor
		# output: speed of each rotor
		# neglecting beta0 since it is of an order smaller
		beta1 = -1.97e-7
		beta2 =  9.78e-8
		# BUG have to account for the different direction of rotation of the different rotors
		T = -beta1/(2*beta2) + np.sqrt((beta1/(2*beta2))**2 + T/beta2)
		return T 

	def omegaToThrustPlusConfig(self, omega):
		# input : rotors speeds -- np array 4x1
		# output: T   : vertical thrust
		#         tau : torques in body frame
		self.Mw  = np.array([[        self.k,         self.k,        self.k,        self.k],  \
							 [             0, -self.k*self.l,             0, self.k*self.l],  \
							 [-self.k*self.l,              0, self.k*self.l,             0],  \
							 [       -self.b,         self.b,       -self.b,        self.b]])

		omega = np.clip(omega, self.omega_min_lim, self.omega_max_lim)
		omegasq = np.power(omega,2)

		Tbar = (self.Mw).dot(omegasq)
		return Tbar

	def omegaToThrustCrossConfig(self, omega):
		# input : rotors speeds -- np array 4x1
		# output: T   : vertical thrust
		#         tau : torques in body frame	

		# rotor speed saturation (not sure we want it....)
		omega = np.clip(omega, self.omega_min_lim, self.omega_max_lim)
		omegasq = np.power(omega,2)
		T        = self.k * sum(omegasq)
		tauPhi   = self.k*self.l/np.sqrt(2) * omegasq.dot([-1,-1, 1, 1])
		tauTheta = self.k*self.l/np.sqrt(2) * omegasq.dot([-1, 1, 1,-1])
		tauPsi   = self.b * omegasq.dot([-1, 1,-1, 1])
		Tbar     = np.array([T, tauPhi, tauTheta, tauPsi]) 
		return Tbar

	def pwdToForcesMap(self, u):
		# wrapper for mapping:
		# pwd -> rotor thrust -> rotor speed -> body forces
		if self.config=="plus" : # plus configuration
			return self.omegaToThrustPlusConfig(self.thrustToOmega((self.pwmToThrust(u))))
		else :                   # cross configuration
			return self.omegaToThrustCrossConfig(self.thrustToOmega((self.pwmToThrust(u))))

	############################
	### SIMULATION FUNCTIONS ###
	############################

	def quad_acceleration(self, T, tau, v, q, w):
		# input : system input, and states
		#         T   : vertical thrust
		#         tau : torques in body frame
		#         v   : speed -- np array 3x1
		#         q   : attitude quaternion -- np array 4x1
		#         w   : attitude rate -- np array 3x1
		# output: system accelerations
		#         vdot : xyz acceleration -- np array 3x1
		#         qdot : attitude acceleration (in quaternion) -- np array 4x1
		#         wdot : attitude acceleration
		#         R    : 
		q = self.quatNormal(q)
		qw = q[0]
		qx = q[1]
		qy = q[2]
		qz = q[3]
		qv = q[1:4]

		if self.x[2]<0.001 :
			Ga = np.array([0, 0, 0]) # gravity companesated by contact force
			self.x[2]=0.0
		else:
			Ga = np.array([0, 0, -self.g])
		Aa   = -1/self.m * np.diag(self.A).dot(v)       # Drag
		angM = np.array([2*(qx*qz + qw*qy), \
			             2*(qy*qz - qw*qx), \
			             qw**2 - qx**2 - qy**2 + qz**2])
		Ta = (T/self.m) * angM                          # vertical thrust?
		vdot = Ga + Ta + Aa
		
		J = np.diag(self.I);
		qcross = self.skewSymmetricOp(qx, qy, qz)
		tmp1 = np.array([[      0,      -qv[0],      -qv[1],      -qv[2]],\
			             [  qv[0], qcross[0,0], qcross[0,1], qcross[0,2]],\
			             [  qv[1], qcross[1,0], qcross[1,1], qcross[1,2]],\
			             [  qv[2], qcross[2,0], qcross[2,1], qcross[2,2]]])
		tmp2 = np.array([0,w[0],w[1],w[2]])
		qdot = 0.5*(np.identity(4)*qw  + tmp1).dot(tmp2) 
		
		wdot = np.linalg.inv(J).dot(tau - np.cross(w, J.dot(w)))

		return vdot, qdot, wdot

	def stateDerivative(self, t, xu):
		# compute derivative in given state
		# input : xu: states and inputs -- np array 17x1
		# output: derivative of the system. the zeros represent
		#         the fact that the control action doesn't change 
		#         between one simulation call and the next one

		# unwrap inputs
		T   = xu[13]
		tau = xu[14:17]
		# unwrap states
		v = xu[3:6]  
		q = xu[6:10]
		w = xu[10:13]
		vdot, qdot, wdot = self.quad_acceleration(T, tau, v, q, w)
		return np.concatenate((v,vdot, qdot, wdot,[0,0,0,0])) 

	def simulate(self, until, u):
		# input : until: absolute time until which the simulation should last
		#         u: PWD inputs to the 4 motors
		# output: returns unwrapped state
		
		if until<self.currentTime :  # check time input
			sys.exit("are you sure you want to simulate backward in time?")
		Tbar = self.pwdToForcesMap(u)
		sol  = intgr.solve_ivp(fun=self.stateDerivative, \
			                   t_span=(self.currentTime, until),\
			                   method="RK45" ,\
			                   y0=np.concatenate((self.x, Tbar.T)) \
			                  )
		# stop if integration failed
		if sol.success==False :
			print("integration of ODE failed")
			raise Integration_Failed
		self.currentTime = until
		self.x = sol.y[0:self.n_states,-1]
		# update measurements 
		xu = self.stateDerivative(0, np.concatenate((self.x, Tbar.T))) # first input is not used
		self.R    = self.computeR(self.x[6:10]) # rotation matrix
		self.acc  = xu[3:6] + self.R.dot(np.array([0, 0, self.g]))     # add gravity in body frame
		return self.x 

	#############################
	### MEASUREMENT FUNCTIONS ###
	#############################

	def readAcc(self, Noise=0):
		# accelerometer reading in m/s^2
		if Noise :
			nx = Noise * rnd.normalvariate(0,self.accNoiseVar[0])
			ny = Noise * rnd.normalvariate(0,self.accNoiseVar[1])
			nz = Noise * rnd.normalvariate(0,self.accNoiseVar[2])
			return self.acc + np.array([nx,ny,nz])
		return self.acc

	def readGyro(self, Noise=0):
		# gyro reading in rad/s
		if Noise : 
			nx = Noise * rnd.normalvariate(0,self.gyroNoiseVar[0])
			ny = Noise * rnd.normalvariate(0,self.gyroNoiseVar[1])
			nz = Noise * rnd.normalvariate(0,self.gyroNoiseVar[2])
			return self.x[10:13] + np.array([nx,ny,nz])
		return self.x[10:13]

	def readZRanging(self, Noise=0):
		# z ranging data reading 		
		if self.x[2]<0 : # can read only positive distances from the floor
			return 0
		else : # if positive distance
			angle = abs(np.arccos(self.R[2,2])) - (np.pi/180)*15/2 # alpha - theta_pz/2
			if angle<0 :
				angle = 0
			if angle>np.pi/2 :
				print("ERROR: drone too much tilted, zranging data corrupted")
				raise Drone_Crash
				angle = np.pi-0.001 # send out a very large reading (firmware has to handle it)
			if Noise :
				nz = self.expStdA * (1 + np.exp(self.expCoeff * (self.x[2] - self.expPointA)))
				ret = self.x[2]/np.cos(angle) + Noise * rnd.normalvariate(0,nz)
				if ret<0 :
					return 0
				else :
					return ret
			return self.x[2]/np.cos(angle)

	def readPixelcount(self, Noise=0, Quantisation=True):
		# function implementing the optical flow measure
		# it returns the pixelcount in the x and y direction
		# parameters
		# TODO: use dt from firmware to detect poor timing properties of 
		#       flowdeck task implementation
		dt      = 0.01 #technically the firmware uses a measured one
		Npx     = 30
		thetapx = 4.2*np.pi/180.0
		R22     = self.R[2,2]
		wFactor = 1.25
		velBF   = self.x[3:6] #speed in body frame
		h = self.x[2] if self.x[2]>0.01 else 0.01
		# predictedNX 
		dnx = (dt * Npx / thetapx) * ((velBF[0]*R22 / h) - wFactor * self.x[11]) 
		# predictedNY 
		dny = (dt * Npx / thetapx) * ((velBF[1]*R22 / h) + wFactor * self.x[10])
		if Noise :
			dnx = dnx + Noise * rnd.normalvariate(0,self.flowNoiseVar[0])
			dny = dny + Noise * rnd.normalvariate(0,self.flowNoiseVar[1])
		if Quantisation:
			return np.array([int(np.rint(dnx)), int(np.rint(dny))])
		else:
			return np.array([dnx, dny])
