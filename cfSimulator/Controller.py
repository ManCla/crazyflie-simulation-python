# class implementing the PID controller of the Crazyflie
import numpy as np

# controller "macros"
rate_main = 1000
rate_attitude = 500
rate_position = 100
posDT = 1/rate_position
attDT = 1/rate_attitude

# cutoff frequencies for lp filters on derivative action
pos_lpf_enable     = True
pos_filter_cutoff  = 20
vel_filter_cutoff  = 20
posZ_filter_cutoff = 20
velZ_filter_cutoff = 20
att_lpf_enable        = False
att_filter_cutoff     = 15
attRate_filter_cutoff = 30

# saturations
thrust_min = 20000
thrust_max = 65535
roll_limit  = 20   # degrees
pitch_limit = 20   # degrees

def rateDo(rate, tick):
	#utility function to trigger cascaded control loops at different rates
	return not( tick % (rate_main/rate))

'''
	Second order low pass filter used to filter derivative action
'''

class lp2Filter():
	def __init__(self, sample_freq, cutoff_freq):
		fr   = sample_freq/cutoff_freq
		ohm  = np.tan(np.pi/fr)
		ohm2 = ohm**2
		c    = 1+2*np.cos(np.pi/4)*ohm+ohm2
		# coefficients
		self.b0 = ohm2/c
		self.b1 = 2*self.b0
		self.b2 = self.b0
		self.a1 = 2*(ohm2-1)/c
		self.a2 = (1-2*np.cos(np.pi/4)*ohm+ohm2)/c
		# init states
		self.x1 = 0
		self.x2 = 0

	def filter(self, sample):
		x0  = sample     - self.x1*self.a1 - self.x2*self.a2
		out = x0*self.b0 + self.x1*self.b1 + self.x2*self.b2
		self.x2 = self.x1
		self.x1 = x0
		return out

'''
	PID class
'''

class PID():
	def __init__(self, kp, ki, kd, dt, lp_enable, lp_rate, lp_cutoff, Iclamp):
		self.kp = kp
		self.ki = ki
		self.kd = kd
		self.dt = dt 
		self.oldError = 0
		self.stateI = 0
		self.Iclamp  = Iclamp
		self.lp_enable = lp_enable
		if self.lp_enable :
			self.lpf = lp2Filter(lp_rate, lp_cutoff)

	def run(self, ref, measure):
		error = ref-measure
		P = self.kp * error
		D = self.kd*(error-self.oldError)/self.dt
		if self.lp_enable :
			D = self.lpf.filter(D)
		self.stateI = self.stateI + error * self.dt
		if not(self.Iclamp==0):
			self.stateI = np.clip(self.stateI, -self.Iclamp, self.Iclamp)
		I = self.ki * self.stateI
		self.oldError = error
		return P+D+I

'''
	Actual controller class
'''

class cfPIDController():
	
	def __init__(self, config, b, I, m, g, k, l):
		#drone parameters
		self.b = b
		self.I = I
		self.g = g
		self.m = m
		self.k = k
		self.l = l
		self.config = config

		# controller states
		self.tick = 1
		self.T    = 0
		self.tau  = np.array([0,0,0])
		self.etaDesired = np.array([0,0,0])

		# PID controllers - position
		self.xPID  = PID(2.0,0,0,posDT,pos_lpf_enable,rate_position,pos_filter_cutoff,5000)
		self.yPID  = PID(2.0,0,0,posDT,pos_lpf_enable,rate_position,pos_filter_cutoff,5000)
		self.zPID  = PID(2.0,0.5,0,posDT,pos_lpf_enable,rate_position,posZ_filter_cutoff,5000)
		self.vxPID = PID(25.0,1.0,0,posDT,pos_lpf_enable,rate_position,vel_filter_cutoff,5000)
		self.vyPID = PID(25.0,1.0,0,posDT,pos_lpf_enable,rate_position,vel_filter_cutoff,5000)
		self.vzPID = PID(25.0,15 ,0,posDT,pos_lpf_enable,rate_position,velZ_filter_cutoff,5000)
		# PID controllers - attitude
		self.phiPID    = PID(6,3,0,attDT,att_lpf_enable,rate_attitude,att_filter_cutoff,20)
		self.thetaPID  = PID(6,3,0,attDT,att_lpf_enable,rate_attitude,att_filter_cutoff,20)
		self.psiPID    = PID(6,1,0.35,attDT,att_lpf_enable,rate_attitude,att_filter_cutoff,360)
		self.phidPID   = PID(250,500,2.5,attDT,att_lpf_enable,rate_attitude,attRate_filter_cutoff,33.3)
		self.thetadPID = PID(250,500,2.5,attDT,att_lpf_enable,rate_attitude,attRate_filter_cutoff,33.3)
		self.psidPID   = PID(120,16.7,0,attDT,att_lpf_enable,rate_attitude,attRate_filter_cutoff,166.7)
		
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
		# saturate to actuator limits
		return np.clip(np.array([pwm1, pwm2, pwm3, pwm4]),0,65535)

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
		roll  = np.clip(roll ,-roll_limit,roll_limit)
		pitch = np.clip(pitch,-pitch_limit,pitch_limit)
		thrust = self.vzPID.run(v_ref[2], vel[2])
		thrustScale = 1000
		thrustBase   = 36000
		thrust = np.clip(thrust*thrustScale+thrustBase, thrust_min, thrust_max)
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
		if rateDo(rate_position, self.tick):
			self.T, self.etaDesired = self.positionCtrl(pos_r, pos, vel, eta_fw)
		# attitude control
		if rateDo(rate_attitude, self.tick):
			self.tau = self.attitudeCtrl(self.etaDesired, eta_fw, etadot_fw)
		self.tick = self.tick + 1
		# output PWM values
		return self.forcesToPWMcrossConfig(self.T, self.tau)
