"""
physical model of the drone. 
Implements a class that stores the physical state of the drone and 
allows to simulate its physic for an arbitrary dt
"""

import numpy as np
import math
import scipy.integrate as intgr
import sys
import cython

import random as rnd

@cython.cclass
class cfPhysics():
	# States 
	x: cython.double[13]
	currentTime: cython.double

	# Variales for measurements computation
	acc: cython.double[3]
	R: cython.double[3][3]

	# Measurement Noise Parameters
	accNoiseVar: cython.double[3]
	gyroNoiseVar: cython.double[3]
	flowNoiseVar: cython.double[2]
	# zRagner noise model coefficients
	expPointA: cython.float
	expStdA: cython.float
	expCoeff: cython.float

	def __init__(self, seed=1):
		
		# States 
		# Initialize State Conditions
		self.x = np.zeros(13,dtype=long)
		self.x[6] = 1 # attitude quaternion has always norm 1
		self.currentTime = 0.0 # (relative) time at which the model is

		# Variales for measurements computation
		acc = [0,0,0]   # acceleration
		self.R = [[1,0,0],[0,1,0],[0,0,1]] # rotation matrix

		# Measurement Noise Parameters
		rnd.seed(seed)

	######################################
	### MAPPING FUNCTIONS PWM<->THRUST ###
	###################################### 

	@cython.cdivision(True)
	def pwmToThrust(self, pwm):
		# input : PWM signal to the 4 motors
		# output: thrust of each rotor
		# extract input for compilation efficiency
		pwm0: cython.double
		pwm1: cython.double
		pwm2: cython.double
		pwm3: cython.double
		pwm0 = pwm[0]
		pwm1 = pwm[1]
		pwm2 = pwm[2]
		pwm3 = pwm[3]
		if pwm0 < 0 :
			pwm0 = 0
		elif pwm0 > 65535 :
			pwm0 = 65535
		if pwm1 < 0 :
			pwm1 = 0
		elif pwm1 > 65535 :
			pwm1 = 65535
		if pwm2 < 0 :
			pwm2 = 0
		elif pwm2 > 65535 :
			pwm2 = 65535
		if pwm3 < 0 :
			pwm3 = 0
		elif pwm3 > 65535 :
			pwm3 = 65535

		pwm0 = pwm0/65535.0
		pwm1 = pwm1/65535.0
		pwm2 = pwm2/65535.0
		pwm3 = pwm3/65535.0
		beta1: cython.float
		beta2: cython.float
		beta1 = 0.35
		beta2 = 0.26
		T: cython.double[4]
		T[0] = beta1*pwm0 + beta2*(pwm0*pwm0)
		T[1] = beta1*pwm1 + beta2*(pwm1*pwm1)
		T[2] = beta1*pwm2 + beta2*(pwm2*pwm2)
		T[3] = beta1*pwm3 + beta2*(pwm3*pwm3)
		return T 

	@cython.cdivision(True)
	def thrustToOmega(self, T):
		Tret: cython.double[4]
		Tret[0] = T[0]
		Tret[1] = T[1]
		Tret[2] = T[2]
		Tret[3] = T[3]
		# input : thrust generated by each motor
		# output: speed of each rotor
		# neglecting beta0 since it is of an order smaller
		beta1: cython.float
		beta2: cython.float
		beta1 = -1.97e-7
		beta2 =  9.78e-8
		tmp1: cython.double
		tmp2: cython.double
		tmp1 = beta1/(2*beta2)
		tmp2 = (tmp1)**2
		# BUG have to account for the different direction of rotation of the different rotors
		Tret[0] = -tmp1 + math.sqrt(tmp2 + Tret[0]/beta2)
		Tret[1] = -tmp1 + math.sqrt(tmp2 + Tret[1]/beta2)
		Tret[2] = -tmp1 + math.sqrt(tmp2 + Tret[2]/beta2)
		Tret[3] = -tmp1 + math.sqrt(tmp2 + Tret[3]/beta2)
		return Tret

	def omegaToThrustCrossConfig(self, omega):
		# input : rotors speeds -- np array 4x1
		# output: T   : vertical thrust
		#         tau : torques in body frame
		# Rotor speed saturations
		omega_min_lim: cython.float
		omega_max_lim: cython.float
		omega_min_lim = 000.001   # rad/s
		omega_max_lim = 2500.0  # rad/s
		# extract input vars to local typed ones for C compilation
		omega0: cython.double
		omega1: cython.double
		omega2: cython.double
		omega3: cython.double
		omega0 = omega[0]
		omega1 = omega[1]
		omega2 = omega[2]
		omega3 = omega[3]
		# coefficients to go from rotor speed to vertical thrust
		k: cython.float
		l: cython.float
		b: cython.float
		k   = 2.2e-8     # N m s^2
		l   = 0.046      # m
		b   = 2e-9       # N s^2
		# rotor speed saturation (not sure we want it....)
		if omega0 < omega_min_lim :
			omega0 = omega_min_lim
		elif omega0 > omega_max_lim :
			omega0 = omega_max_lim
		if omega1 < omega_min_lim :
			omega1 = omega_min_lim
		elif omega1 > omega_max_lim :
			omega1 = omega_max_lim
		if omega2 < omega_min_lim :
			omega2 = omega_min_lim
		elif omega2 > omega_max_lim :
			omega2 = omega_max_lim
		if omega3 < omega_min_lim :
			omega3 = omega_min_lim
		elif omega3 > omega_max_lim :
			omega3 = omega_max_lim
		# omegasq = np.power(omega,2)
		omega0sq = omega0*omega0
		omega1sq = omega1*omega1
		omega2sq = omega2*omega2
		omega3sq = omega3*omega3
		Tbar: cython.double[4]
		Tbar[0]        = k * (omega0sq+omega1sq+omega2sq+omega3sq)
		tmp: cython.double
		tmp = k*l/(1.4142135623730951)  # sqrt(2) = 1.4142135623730951
		Tbar[1] = tmp * (-omega0sq-omega1sq+omega2sq+omega3sq) # omegasq.dot([-1,-1, 1, 1])
		Tbar[2] = tmp * (-omega0sq+omega1sq+omega2sq-omega3sq) # omegasq.dot([-1, 1, 1,-1])
		Tbar[3] = b   * (-omega0sq+omega1sq-omega2sq+omega3sq) # omegasq.dot([-1, 1,-1, 1])
		# Tbar     = [T, tauPhi, tauTheta, tauPsi]
		return Tbar

	############################
	### SIMULATION FUNCTIONS ###
	############################

	@cython.cdivision(True)
	def quad_acceleration(self, T:cython.double, tau, v, q, w):
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

		# typing for cython
		qw: cython.double
		qx: cython.double
		qy: cython.double
		qz: cython.double
		Ga: cython.double[3]
		Ta: cython.double[3]
		Aa: cython.double[3]
		vdot: cython.double[3]
		# constants for cython
		m: cython.float
		m = 0.027+0.004 # kg

		# local typed copy of w for efficient C compilation
		w0: cython.double
		w1: cython.double
		w2: cython.double
		w0 = w[0]
		w1 = w[1]
		w2 = w[2]

		# local typed copy of v for efficient C compilation
		v0: cython.double
		v1: cython.double
		v2: cython.double
		v0 = v[0]
		v1 = v[1]
		v2 = v[2]

		# local typed copy of tau for efficient C compilation
		tau0: cython.double
		tau1: cython.double
		tau2: cython.double
		tau0 = tau[0]
		tau1 = tau[1]
		tau2 = tau[2]

		qw = q[0]
		qx = q[1]
		qy = q[2]
		qz = q[3]
		qmod: cython.double
		qmod = math.sqrt(qw*qw+qx*qx+qy*qy+qz*qz)
		qw = qw/qmod
		qx = qx/qmod
		qy = qy/qmod
		qz = qz/qmod

		if self.x[2]<0.001 :
			Ga = [0, 0, 0] # gravity compensated by contact force
			self.x[2]=0.0
		else:
			Ga = [0, 0, -9.81] #g=9.81
		# Aa   = -1/self.m * np.diag(self.A).dot(v)       # Drag
		Aa   = [0.92e-6*v0/m, 0.91e-6*v1/m, 1.03e-6*v2/m]
		angM: cython.double[3]
		angM = [2*(qx*qz+qw*qy), 2*(qy*qz-qw*qx), qw**2-qx**2-qy**2+qz**2]
		Ta[0] = (T/m) * angM[0] # vertical thrust?
		Ta[1] = (T/m) * angM[1] # vertical thrust?
		Ta[2] = (T/m) * angM[2] # vertical thrust?
		vdot[0] = Ga[0] + Ta[0] - Aa[0]
		vdot[1] = Ga[1] + Ta[1] - Aa[1]
		vdot[2] = Ga[2] + Ta[2] - Aa[2]
		
		J: cython.double[3][3]
		J = [[1.66e-5,      0,       0],\
		     [      0,1.66e-5,       0],\
			 [      0,      0, 2.93e-5]];
		qcross: cython.double[3][3]
		qcross = [[ 0, -qz,  qy],\
	              [ qz,  0, -qx],\
	              [-qy,  qx,  0]]
		tmp1: cython.double[4][4]
		# merged this summation in the definition of tmp1: np.identity(4)*qw  + tmp1
		tmp1 = [[  qw,             -qx,             -qy,             -qz],\
			    [  qx, qw+qcross[0][0],    qcross[0][1],    qcross[0][2]],\
			    [  qy,    qcross[1][0], qw+qcross[1][1],    qcross[1][2]],\
			    [  qz,    qcross[2][0],    qcross[2][1], qw+qcross[2][2]]]
		tmp2: cython.double[4]
		tmp2 = [0,w0,w1,w2]
		qdot: cython.double[4]
		# qdot = 0.5*(np.identity(4)*qw  + tmp1).dot(tmp2)
		qdot[0] = 0.5*(tmp1[0][0]*tmp2[0]+tmp1[0][1]*tmp2[1]+tmp1[0][2]*tmp2[2]+tmp1[0][3]*tmp2[3])
		qdot[1] = 0.5*(tmp1[1][0]*tmp2[0]+tmp1[1][1]*tmp2[1]+tmp1[1][2]*tmp2[2]+tmp1[1][3]*tmp2[3])
		qdot[2] = 0.5*(tmp1[2][0]*tmp2[0]+tmp1[2][1]*tmp2[1]+tmp1[2][2]*tmp2[2]+tmp1[2][3]*tmp2[3])
		qdot[3] = 0.5*(tmp1[3][0]*tmp2[0]+tmp1[3][1]*tmp2[1]+tmp1[3][2]*tmp2[2]+tmp1[3][3]*tmp2[3])
		
		# wdot = np.linalg.inv(J).dot(tau - np.cross(w, J.dot(w)))

		Jw: cython.double[3]
		# J.dot(w)		
		Jw[0] = J[0][0]*w0+J[0][1]*w1+J[0][2]*w2
		Jw[1] = J[1][0]*w0+J[1][1]*w1+J[1][2]*w2
		Jw[2] = J[2][0]*w0+J[2][1]*w1+J[2][2]*w2
		
		wCrossJ: cython.double[3]
		# tau - np.cross(w, J.dot(w))
		wCrossJ[0] = tau0 - w1*Jw[2]-w2*Jw[1]
		wCrossJ[1] = tau1 - w2*Jw[0]-w0*Jw[2]
		wCrossJ[2] = tau2 - w0*Jw[1]-w1*Jw[0]
		
		invJ: cython.double[3][3]
		invJ = [[1/1.66e-5,        0,       0],\
		        [        0,1/1.66e-5,       0],\
			    [        0,        0,1/2.93e-5]];
		wdot: cython.double[3]
		wdot[0] = invJ[0][0]*wCrossJ[0]+invJ[0][1]*wCrossJ[1]+invJ[0][2]*wCrossJ[2]
		wdot[1] = invJ[1][0]*wCrossJ[0]+invJ[1][1]*wCrossJ[1]+invJ[1][2]*wCrossJ[2]
		wdot[2] = invJ[2][0]*wCrossJ[0]+invJ[2][1]*wCrossJ[1]+invJ[2][2]*wCrossJ[2]

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

	def simulate(self, until: cython.double, u):
		# input : until: absolute time until which the simulation should last
		#         u: PWD inputs to the 4 motors
		# output: returns unwrapped state
		
		if until<self.currentTime :  # check time input
			sys.exit("are you sure you want to simulate backward in time?")
		Tbar = self.omegaToThrustCrossConfig(self.thrustToOmega((self.pwmToThrust(u))))
		sol  = intgr.solve_ivp(fun=self.stateDerivative, \
			                   t_span=(self.currentTime, until),\
			                   method="RK45" ,\
			                   y0=np.concatenate((self.x, Tbar)) \
			                  )
		# stop if integration failed
		if sol.success==False :
			sys.exit("integration of ODE failed")
		self.currentTime = until
		self.x = sol.y[0:13,-1]
		# update measurements 
		xu: cython.double[17]
		xu = self.stateDerivative(0, np.concatenate((self.x, Tbar))) # first input is not used
		# computes rotation matrix from quaternion q=self.x[6:10]
		qw: cython.double
		qx: cython.double
		qy: cython.double
		qz: cython.double
		qw = self.x[6]
		qx = self.x[7]
		qy = self.x[8]
		qz = self.x[9]
		a: cython.double[3]
		b: cython.double[3]
		c: cython.double[3]
		self.R = [[qw*qw+qx*qx-qy*qy-qz*qz,         2*(qx*qy-qw*qz),         2*(qx*qz+qw*qy)],\
		          [        2*(qx*qy+qw*qz), qw*qw-qx*qx+qy*qy-qz*qz,         2*(qy*qz-qw*qx)],\
		          [        2*(qx*qz-qw*qy),         2*(qy*qz+qw*qx), qw*qw-qx*qx-qy*qy+qz*qz]]
		self.acc[0] = xu[3] + 9.81*self.R[0][2]
		self.acc[1] = xu[4] + 9.81*self.R[1][2]
		self.acc[2] = xu[5] + 9.81*self.R[2][2]
		return self.x 

	#############################
	### MEASUREMENT FUNCTIONS ###
	#############################

	def readAcc(self, Noise=0):
		accNoiseVar: cython.double[3]
		accNoiseVar  = [0.5,0.5,1.0] # accelerometer noise variance
		# accelerometer reading in m/s^2
		if Noise :
			nx = Noise * rnd.normalvariate(0,accNoiseVar[0])
			ny = Noise * rnd.normalvariate(0,accNoiseVar[1])
			nz = Noise * rnd.normalvariate(0,accNoiseVar[2])
			return self.acc + np.array([nx,ny,nz])
		return self.acc

	def readGyro(self, Noise=0):
		gyroNoiseVar: cython.double[3]
		gyroNoiseVar = [0.1,0.1,0.1] # gyro noise variance
		# gyro reading in rad/s
		if Noise :
			nx = Noise * rnd.normalvariate(0,gyroNoiseVar[0])
			ny = Noise * rnd.normalvariate(0,gyroNoiseVar[1])
			nz = Noise * rnd.normalvariate(0,gyroNoiseVar[2])
			return [self.x[10]+nx, self.x[11]+ny, self.x[12]+nz ]
		return [self.x[10], self.x[11], self.x[12] ]

	@cython.cdivision(True)
	def readZRanging(self, Noise=0):
		pi: cython.float
		pi=3.1415926535
		# zRagner noise model coefficients
		expPointA: cython.double
		expStdA: cython.double
		expCoeff: cython.double
		expPointA = 2.5
		expStdA   = 0.0025
		expCoeff  = 2.92135

		angle: cython.double # local var
		x2: cython.double    # extract state once for efficiency
		x2 = self.x[2]

		# z ranging data reading 		
		if x2<0 : # can read only positive distances from the floor
			return 0
		else : # if positive distance
			angle = abs(np.arccos(self.R[2][2])) - (pi/180)*15/2 # alpha - theta_pz/2
			if angle<0 :
				angle = 0
			if angle>pi/2 :
				print("ERROR: drone too much tilted, zranging data corrupted")
				angle = pi-0.001 # send out a very large reading (firmware has to handle it)
			if Noise :
				ret: cython.double
				nz = expStdA * (1 + np.exp(expCoeff * (x2 - expPointA)))
				ret = x2/np.cos(angle) + Noise * rnd.normalvariate(0,nz)
				if ret<0 :
					return 0
				else :
					return ret
			return x2/np.cos(angle)

	@cython.cdivision(True)
	def readPixelcount(self, Noise: cython.int =0 , Quantisation: cython.int =True):
		pi: cython.double
		pi=3.1415926535
		flowNoiseVar: cython.double[2]
		flowNoiseVar = [2, 2]        # flowdeck noise variance
		# function implementing the optical flow measure
		# it returns the pixelcount in the x and y direction
		# parameters
		# TODO: use dt from firmware to detect poor timing properties of 
		#       flowdeck task implementation
		dt: cython.double
		Npx: cython.double
		thetapx: cython.double
		R22: cython.double
		wFactor: cython.double
		dt      = 0.01 #technically the firmware uses a measured one
		Npx     = 30
		thetapx = 4.2*pi/180.0
		R22     = self.R[2][2]
		wFactor = 1.25
		velBF: cython.double[3] #speed in body frame
		velBF[0] = self.x[3]
		velBF[1] = self.x[4]
		velBF[2] = self.x[5]
		h: cython.double
		h = self.x[2] if self.x[2]>0.01 else 0.01
		# predictedNX 
		dnx = (dt * Npx / thetapx) * ((velBF[0]*R22 / h) - wFactor * self.x[11]) 
		# predictedNY 
		dny = (dt * Npx / thetapx) * ((velBF[1]*R22 / h) + wFactor * self.x[10])
		if Noise :
			dnx = dnx + Noise * rnd.normalvariate(0,flowNoiseVar[0])
			dny = dny + Noise * rnd.normalvariate(0,flowNoiseVar[1])
		if Quantisation:
			return np.array([int(np.rint(dnx)), int(np.rint(dny))])
		else:
			return np.array([dnx, dny])
