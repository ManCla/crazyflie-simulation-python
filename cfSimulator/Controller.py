# class implementing the PID controller of the Crazyflie
import numpy as np
from .utils import PID
import cython

'''
	Actual controller class
'''
@cython.cdivision(True) 
class cfPIDController():

	# execution of different control loops
	rate_attitude: cython.int
	rate_position: cython.int
	posDT: cython.int
	attDT: cython.int

	# low pass filters on derivative action
	pos_lpf_enable: cython.int
	pos_filter_cutoff: cython.int
	vel_filter_cutoff: cython.int
	posZ_filter_cutoff: cython.int
	velZ_filter_cutoff: cython.int
	att_lpf_enable: cython.int
	att_filter_cutoff: cython.int
	attRate_filter_cutoff: cython.int


	# controller parameters
	b: cython.double
	I: cython.double
	g: cython.double
	m: cython.double
	k: cython.double
	l: cython.double

	# controller states
	tick: cython.int
	T: cython.int
	tau: cython.double[3]
	etaDesired: cython.double[3]
	
	@cython.cdivision(True) 
	def __init__(self):
		'''
		controller "macros"
		'''
		# execution of different control loops
		self.rate_attitude = 500
		self.rate_position = 100
		self.posDT = 1/self.rate_position
		self.attDT = 1/self.rate_attitude
		# low pass filters on derivative action
		self.pos_lpf_enable     = True
		self.pos_filter_cutoff  = 20
		self.vel_filter_cutoff  = 20
		self.posZ_filter_cutoff = 20
		self.velZ_filter_cutoff = 20
		self.att_lpf_enable        = False
		self.att_filter_cutoff     = 15
		self.attRate_filter_cutoff = 30

		# controller states
		self.tick = 1
		self.T    = 0
		self.tau1 = 0
		self.tau2 = 0
		self.tau3 = 0
		self.etaDesired = [0,0,0]

		# PID controllers - position
		self.xPID  = PID(2.0,0,0,self.posDT,self.pos_lpf_enable,self.rate_position,self.pos_filter_cutoff,5000)
		self.yPID  = PID(2.0,0,0,self.posDT,self.pos_lpf_enable,self.rate_position,self.pos_filter_cutoff,5000)
		self.zPID  = PID(2.0,0.5,0,self.posDT,self.pos_lpf_enable,self.rate_position,self.posZ_filter_cutoff,5000)
		self.vxPID = PID(25.0,1.0,0,self.posDT,self.pos_lpf_enable,self.rate_position,self.vel_filter_cutoff,5000)
		self.vyPID = PID(25.0,1.0,0,self.posDT,self.pos_lpf_enable,self.rate_position,self.vel_filter_cutoff,5000)
		self.vzPID = PID(25.0,15 ,0,self.posDT,self.pos_lpf_enable,self.rate_position,self.velZ_filter_cutoff,5000)
		# PID controllers - attitude
		self.phiPID    = PID(6,3,0,self.attDT,self.att_lpf_enable,self.rate_attitude,self.att_filter_cutoff,20)
		self.thetaPID  = PID(6,3,0,self.attDT,self.att_lpf_enable,self.rate_attitude,self.att_filter_cutoff,20)
		self.psiPID    = PID(6,1,0.35,self.attDT,self.att_lpf_enable,self.rate_attitude,self.att_filter_cutoff,360)
		self.phidPID   = PID(250,500,2.5,self.attDT,self.att_lpf_enable,self.rate_attitude,self.attRate_filter_cutoff,33.3)
		self.thetadPID = PID(250,500,2.5,self.attDT,self.att_lpf_enable,self.rate_attitude,self.attRate_filter_cutoff,33.3)
		self.psidPID   = PID(120,16.7,0,self.attDT,self.att_lpf_enable,self.rate_attitude,self.attRate_filter_cutoff,166.7)
		
	#############################
	### TORQUE -> PWM MAPPING ###
	#############################

	@cython.cdivision(True)
	def forcesToPWMcrossConfig(self, T:cython.double, tau1:cython.double,tau2:cython.double,tau3:cython.double):
		r: cython.double
		p: cython.double
		y: cython.double
		r = tau1 / 2
		p = tau2 / 2
		y = tau3
		pwm1: cython.double
		pwm2: cython.double
		pwm3: cython.double
		pwm4: cython.double
		pwm1 = T - r + p + y
		pwm2 = T - r - p - y
		pwm3 = T + r - p + y
		pwm4 = T + r + p - y
		# saturate to actuator limits
		if pwm1>65535:
			pwm1=65535
		elif pwm1<0:
			pwm1=0
		if pwm2>65535:
			pwm2=65535
		elif pwm2<0:
			pwm2=0
		if pwm3>65535:
			pwm3=65535
		elif pwm3<0:
			pwm3=0
		if pwm4>65535:
			pwm4=65535
		elif pwm4<0:
			pwm4=0
		return np.array([pwm1, pwm2, pwm3, pwm4])

	############################
	### CONTROLLER FUNCTIONS ###
	############################

	@cython.cdivision(True) 
	def positionCtrl(self, ref, pos, vel, eta):
		# position controller: from desired and estimated position to desired attitude
		v_ref: cython.double[3]
		roll: cython.double
		pitch: cython.double
		thrust: cython.double
		thrustScale: cython.int
		thrustBase: cython.int
		pitchRaw: cython.double
		rollRaw: cython.double
		pi: cython.float
		pi=3.1415926535
		# saturations - since they are used only here, declared local makes for more C code
		thrust_min: cython.int
		thrust_max: cython.int
		roll_limit: cython.int
		pitch_limit: cython.int
		thrust_min = 20000
		thrust_max = 65535
		roll_limit  = 20   # degrees
		pitch_limit = 20   # degrees


		v_ref = [0.0,0.0,0.0]
		v_ref[0] = self.xPID.run(ref[0], pos[0])
		v_ref[1] = self.yPID.run(ref[1], pos[1])
		v_ref[2] = self.zPID.run(ref[2], pos[2])
		# NOTE: firmware inverts the naming of roll and pitch for -Raw variables
		pitchRaw = self.vxPID.run(v_ref[0], vel[0])
		rollRaw  = self.vyPID.run(v_ref[1], vel[1])
		roll  = - rollRaw  * np.cos(eta[2]*pi/180.0) - pitchRaw * np.sin(eta[2]*pi/180.0)
		pitch = - pitchRaw * np.cos(eta[2]*pi/180.0) + rollRaw  * np.sin(eta[2]*pi/180.0)
		if roll>roll_limit :
			roll = roll_limit
		elif roll<-roll_limit:
			roll = -roll_limit
		if pitch>pitch_limit :
			pitch = pitch_limit
		elif pitch<-pitch_limit:
			pitch = -pitch_limit
		thrust = self.vzPID.run(v_ref[2], vel[2])
		thrustScale = 1000
		thrustBase   = 36000
		thrust = thrust*thrustScale+thrustBase
		if thrust<thrust_min :
			thrust=thrust_min
		elif thrust>thrust_max :
			thrust=thrust_max
		return thrust, [roll, pitch, 0]

	def attitudeCtrl(self, etaDesired, eta, etadot):
		etadot_ref: cython.double[3]
		tau: cython.double[3]
		etadot_ref = [0.0,0.0,0.0]
		etadot_ref[0] = self.phiPID.run(etaDesired[0], eta[0])
		etadot_ref[1] = self.thetaPID.run(etaDesired[1], eta[1])
		etadot_ref[2] = self.psiPID.run(etaDesired[2], eta[2])
		tau = [0,0,0]
		tau[0] = self.phidPID.run(etadot_ref[0], etadot[0])
		tau[1] = self.thetadPID.run(etadot_ref[1], etadot[1])
		tau[2] = self.psidPID.run(etadot_ref[2], etadot[2])
		return tau[0],tau[1],tau[2]

	@cython.cdivision(True)
	def ctrlCompute(self, pos_r, pos, vel, eta, gyro):
		# local variables for efficiency
		pi: cython.float
		tmp: cython.int[3]
		rate_main: cython.int
		tmp=[1,-1,-1]
		pi=3.1415926535
		rate_main = 1000

		# main controller function 
		# note that it gets attitude in quaternions from x but also euler angles from eta

		# _fw variables are the translation of states into firmware conventions:
		# 1) controller tuning is based on angles in degrees
		# 2) pitch sign reference in KF is inverted
		# 3) yaw sign reference in KF is inverted
		eta_fw    =  (eta*180.0/pi)*tmp
		etadot_fw = (gyro*180.0/pi)*tmp
		# position control
		if not( self.tick % (rate_main/self.rate_position)) :
			self.T, self.etaDesired = self.positionCtrl(pos_r, pos, vel, eta_fw)
		# attitude control
		if not( self.tick % (rate_main/self.rate_attitude)) :
			self.tau1, self.tau2, self.tau3 = self.attitudeCtrl(self.etaDesired, eta_fw, etadot_fw)
		self.tick = self.tick + 1
		# output PWM values
		return self.forcesToPWMcrossConfig(self.T, self.tau1, self.tau2, self.tau3)
