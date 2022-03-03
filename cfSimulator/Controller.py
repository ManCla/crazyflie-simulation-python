# class implementing the a controller for the crazyflie
# mainly needed for testing of the python model
import numpy as np

# controller "macros"
rateMain = 1000
rateAttitude = 500
ratePosition = 100
posDT = 1/ratePosition
attDT = 1/rateAttitude

def rateDo(rate, tick):
	#utility function to trigger cascaded control loops at different rates
	return not( tick % (rateMain/rate))

class PID():
	def __init__(self, kp, ki, kd, dt):
		self.kp = kp
		self.ki = ki
		self.kd = kd
		self.dt = dt 
		self.oldError = 0
		self.stateI = 0

	def run(self, ref, measure):
		# TODO?: in firmware they have low pass filter on D action
		error = ref-measure
		P = self.kp * error
		D = self.kd*(error-self.oldError)/self.dt
		self.stateI = self.stateI + error * self.dt
		I = self.ki * self.stateI
		self.oldError = error
		return P+D+I

class cfPIDController():
	
	def __init__(self, refType, config, b, I, m, g, k, l):
		#drone parameters
		self.b = b
		self.I = I
		self.g = g
		self.m = m
		self.k = k
		self.l = l
		self.config = config

		# reference trajectory desired type
		self.trajectoryType = refType

		# controller states
		self.tick = 1
		self.T    = 0
		self.tau  = np.array([0,0,0])
		self.etaDesired = np.array([0,0,0])

		# PID controllers - position
		self.xPID  = PID(2.0, 0  ,0,posDT) 
		self.yPID  = PID(2.0, 0  ,0,posDT)
		self.zPID  = PID(2.0, 0.5,0,posDT)
		self.vxPID = PID(25.0,1.0,0,posDT)
		self.vyPID = PID(25.0,1.0,0,posDT)
		self.vzPID = PID(25.0,15 ,0,posDT)
		# PID controllers - attitude
		self.phiPID    = PID(6,3,0,attDT)
		self.thetaPID  = PID(6,3,0,attDT)
		self.psiPID    = PID(6,1,0.35,attDT)
		self.phidPID   = PID(250,500,2.5,attDT)
		self.thetadPID = PID(250,500,2.5,attDT)
		self.psidPID   = PID(120,16.7,0,attDT)
		
	#############################
	### TORQUE -> PWM MAPPING ###
	#############################

	def forcesToPWMcrossConfig(self, T, tau):
		r = tau[0] / 2
		p = tau[1] / 2
		y = tau[2]
		pwm1 = T - r + p + y
		pwm2 = T - r - p - y
		pwm3 = T + r - p + y
		pwm4 = T + r + p - y
		return np.array([pwm1, pwm2, pwm3, pwm4])

	############################
	### CONTROLLER FUNCTIONS ###
	############################

	def positionCtrl(self, ref, pos, vel, eta):
		# position controller: from desired and estimated position to desired attitude
		v_ref = np.array([0.0,0.0,0.0])
		v_ref[0] = self.xPID.run(ref[0], pos[0])
		v_ref[1] = self.yPID.run(ref[1], pos[1])
		v_ref[2] = self.zPID.run(ref[2], pos[2])
		# NOTE: firmware inverts the naming of roll and pitch for -Raw variables
		pitchRaw = self.vxPID.run(v_ref[0], vel[0])
		rollRaw  = self.vyPID.run(v_ref[1], vel[1])
		roll  = - rollRaw  * np.cos(eta[2]*np.pi/180.0) - pitchRaw * np.sin(eta[2]*np.pi/180.0)
		pitch = - pitchRaw * np.cos(eta[2]*np.pi/180.0) + rollRaw  * np.sin(eta[2]*np.pi/180.0)
		thrust = self.vzPID.run(v_ref[2], vel[2])
		thrustScale = 1000
		thrustBase   = 36000
		thrust = thrust * thrustScale + thrustBase
		return thrust, np.array([roll, pitch, 0])

	def attitudeCtrl(self, etaDesired, eta, etadot):
		etadot_ref = np.array([0.0,0.0,0.0])
		etadot_ref[0] = self.phiPID.run(etaDesired[0], eta[0])
		etadot_ref[1] = self.thetaPID.run(etaDesired[1], eta[1])
		etadot_ref[2] = self.psiPID.run(etaDesired[2], eta[2])
		tau = np.array([0,0,0])
		tau[0] = self.phidPID.run(etadot_ref[0], etadot[0])
		tau[1] = self.thetadPID.run(etadot_ref[1], etadot[1])
		tau[2] = self.psidPID.run(etadot_ref[2], etadot[2])
		return tau

	def ctrlCompute(self, pos_r, pos, vel, eta, gyro):
		# main controller function 
		# note that it gets attitude in quaternions from x but also euler angles from eta

		# _fw variables are the translation of states into firmware conventions:
		# 1) controller tuning is based on angles in degrees
		# 2) pitch sign reference in KF is inverted
		# 3) yaw sign reference in KF is inverted
		eta_fw    =  (eta*180.0/np.pi)*np.array([1,-1,-1])
		etadot_fw = (gyro*180.0/np.pi)*np.array([1,-1,-1])
		# position control
		if rateDo(ratePosition, self.tick):
			self.T, self.etaDesired = self.positionCtrl(pos_r, pos, vel, eta_fw)
		# attitude control
		if rateDo(rateAttitude, self.tick):
			self.tau = self.attitudeCtrl(self.etaDesired, eta_fw, etadot_fw)
		self.tick = self.tick + 1
		# output PWM values
		return self.forcesToPWMcrossConfig(self.T, self.tau)

	############################
	### REFERENCE GENERATION ###
	############################

	def referenceGen(self, t):
		if t<2 : 
			return np.array([0,0,0.5])
		if self.trajectoryType=="step":
			if t<6:
				return np.array([0.2,0,0.5])
			return np.array([0,0.2,0.5])
		elif self.trajectoryType=="zsinus":
			return np.array([0,0,np.sin(0.2*t)+0.5])
		# NOTE: following wont work when using kalman filter since 
		#       position estimate is open loop
		elif self.trajectoryType=="xsinus":
			return np.array([np.sin(0.3*t),0,0.5])
		elif self.trajectoryType=="ysinus":
			return np.array([0,np.sin(0.3*t),0.5])
		elif self.trajectoryType=="circle":
			return np.array([np.cos(0.8*t),np.sin(0.8*t),0.5])
		elif self.trajectoryType=="spiral":
			return np.array([0.01*t*np.cos(1.2*t),0.01*t*np.sin(1.2*t),0.5])



