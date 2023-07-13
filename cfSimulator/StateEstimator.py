# class implementing the extended Kalman filter that is run on the Crazyflie

import numpy as np
import math
import scipy.linalg as spl
import cython

#########################
### UTILITY FUNCTIONS ###
#########################
@cython.cfunc
@cython.inline
def cross(x):
    # utility function: compute skew symmetric matrix from 3-dim vector
    # input : three dimensional vector
    # output: three-by-three skew symmetric matrix
    mcross = np.array([[ 0,   -x[2],  x[1]],\
                       [ x[2],    0, -x[0]],\
                       [-x[1], x[0],    0]])
    return mcross

@cython.cfunc
@cython.inline
def quatMult(dq, q):
    dq0: cython.double
    dq1: cython.double
    dq2: cython.double
    dq3: cython.double
    dq0 = dq[0]
    dq1 = dq[1]
    dq2 = dq[2]
    dq3 = dq[3]
    q0: cython.double
    q1: cython.double
    q2: cython.double
    q3: cython.double
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]
    out: cython.double[4]
    out = np.zeros(4)
    out[0] = dq0*q0-dq1*q1-dq2*q2-dq3*q3
    out[1] = dq1*q0+dq0*q1+dq3*q2-dq2*q3
    out[2] = dq2*q0-dq3*q1+dq0*q2+dq1*q3
    out[3] = dq3*q0+dq2*q1-dq1*q2+dq0*q3
    return out

class cfEKF():
    tick: cython.int
    x: cython.double[9]
    q: cython.double[4]
    R: cython.double[3][3]
    P: cython.double[9][9]
    stateExternal: cython.double[9]
    flowerror: cython.double[2]
    zerror: cython.double

    def __init__(self):

        # filter states
        self.x = np.zeros(9)     # estimated state: position, speed, attitude error
        self.q = np.array([1,0,0,0])
        self.R = np.identity(3)
        self.P = np.diag([100,100,1,0.01,0.01,0.01,0.01,0.01,0.01])  # covar matrix init

        self.stateExternal = np.zeros(9)
        self.flowerror = np.zeros(2)
        self.zerror = 0

        self.tick = 0 # counter 

    def sanityCheckP(self):
        # saturate
        if self.P[0,0]<0 :
            self.P[0,0] = 0
        elif self.P[0,0]>100 :
            self.P[0,0] = 100
        if self.P[1,1]<0 :
            self.P[1,1] = 0
        elif self.P[1,1]>100 :
            self.P[1,1] = 100
        if self.P[2,2]<0 :
            self.P[2,2] = 0
        elif self.P[2,2]>100 :
            self.P[2,2] = 100
        if self.P[3,3]<0 :
            self.P[3,3] = 0
        elif self.P[3,3]>100 :
            self.P[3,3] = 100
        if self.P[4,4]<0 :
            self.P[4,4] = 0
        elif self.P[4,4]>100 :
            self.P[4,4] = 100
        if self.P[5,5]<0 :
            self.P[5,5] = 0
        elif self.P[5,5]>100 :
            self.P[5,5] = 100
        if self.P[6,6]<0 :
            self.P[6,6] = 0
        elif self.P[6,6]>100 :
            self.P[6,6] = 100
        if self.P[7,7]<0 :
            self.P[7,7] = 0
        elif self.P[7,7]>100 :
            self.P[7,7] = 100
        if self.P[8,8]<0 :
            self.P[8,8] = 0
        elif self.P[8,8]>100 :
            self.P[8,8] = 100
        # enforce symmetry
        self.P = (self.P+self.P.transpose())/2

    ###########################
    ### ALGORITHM FUNCTIONS ###
    ###########################

    def predictionStep(self, acc, gyro, dt):
        # use IMU data to predict state forward
        d: cython.double[3]
        d[0] = -gyro[0]*dt/2
        d[1] = -gyro[1]*dt/2
        d[2] = -gyro[2]*dt/2
        # build A (linearised dynamics)
        A = np.zeros((9,9))
        A[0:3,0:3] = [[1,0,0],[0,1,0],[0,0,1]]
        A[0:3,3:6] = self.R*dt
        A[0:3,6:9] = np.matmul(self.R,cross(-self.x[3:6]))*dt
        A[3:6,3:6] = [[1,0,0],[0,1,0],[0,0,1]]+cross(-gyro)*dt
        A[3:6,6:9] = 9.81*cross(-self.R[2,:])*dt # g=9.81
        A[6:9,6:9] = spl.expm(cross(d))

        # covariance update according to system dynamics 
        AP = np.matmul(A,self.P)             # compute A*P
        self.P = np.matmul(AP,A.transpose()) # compute A*P*A'

        # prediction
        dt2 = pow(dt,2)
        v   = self.x[3:6]*dt+acc*(dt2/2)
        self.x[0:3] = self.x[0:3]+self.R.dot(v)+np.array([0,0,-9.81*dt2/2]) # g=9.81
        self.x[3:6] = self.x[3:6]+dt*(acc-(cross(gyro).dot(self.x[3:6]))-9.81*self.R[2,:]) # g=9.81
        angle  = np.linalg.norm(gyro*dt)
        ca     = np.cos(angle/2)
        sa     = np.sin(angle/2)
        if  angle==0:
            dq = np.array([1,0,0,0])
        else:
            dq = np.array([ca, sa*dt*gyro[0]/angle, sa*dt*gyro[1]/angle, sa*dt*gyro[2]/angle])
        self.q = quatMult(dq,self.q)      # rotation in quaternions
        self.q = self.q/np.linalg.norm(self.q) # normalize

    @cython.cdivision
    def scalarUpdate(self, error, H, std):
        # given the measurement Jacobian 'H' and the innovation 'error'
        # update state and covariance
        R: cython.double
        R      = std*std
        PHt    = self.P.dot(H)                 # no need to transpose in numpy
        HPHtR  = H.dot(PHt)+R                  # HPH'+R
        K      = PHt/HPHtR                     # kalman gain
        self.x = self.x+K*error                # measurement update
        IKH    = np.identity(9)-np.outer(K,H)  #
        IKHP   = np.matmul(IKH,self.P)
        self.P = np.matmul(IKHP,IKH.transpose())+np.outer(K,K)*R
        self.sanityCheckP()

    def correctionZranging(self, meas):
        # zRagner noise model coefficients
        expPointA: cython.double
        expStdA: cython.double
        expCoeff: cython.double
        expPointA = 2.5
        expStdA   = 0.0025
        expCoeff  = 2.92135

        # update estimate with Z laser ranging data
        pred = self.x[2]/self.R[2,2]
        H    = np.array([0,0,1/self.R[2,2],0,0,0,0,0,0])
        std  = expStdA * (1 + np.exp(expCoeff * (self.x[2] - expPointA)))
        self.scalarUpdate(meas-pred, H, std)
        # externalize error
        self.zerror = meas-pred

    @cython.cdivision
    def correctionFlow(self, dpxl, gyro, dt):
        # constants for typing and cython optimisation
        # optical flow noise model
        flowStd: cython.float
        flowStd   = 2 # used for both directions
        pi: cython.float
        pi=3.1415926535

        # extract relevant states (so that compiler can optimise later)
        R22: cython.double
        R22 = self.R[2,2]
        x3: cython.double
        x3 = self.x[3]
        x4: cython.double
        x4 = self.x[4]

        # update estimate with flow data
        Npx: cython.float
        thetapx: cython.float
        wFactor: cython.float
        coeff: cython.float
        z: cython.float
        Npx     = 30.0            #
        thetapx = 4.2*pi/180.0 #
        wFactor = 1.25            #
        coeff   = Npx*dt/thetapx
        z       = self.x[2] if self.x[2]>0.1 else 0.1
        # X direction
        predicted_x = coeff*((x3*R22/z) - wFactor * gyro[1])
        Hx = np.array([0, 0, coeff*((R22*x3)/(-z*z)),\
                       coeff*R22/z, 0, 0,\
                       0, 0, 0])
        self.scalarUpdate(dpxl[0]-predicted_x, Hx, flowStd)
        # Y direction
        predicted_y = coeff*((x4*R22/z) + wFactor * gyro[0])
        Hy = np.array([0, 0, coeff*((R22*x4)/(-z*z)),\
                       0, coeff*R22/z, 0,\
                       0, 0, 0])
        self.scalarUpdate(dpxl[1]-predicted_y, Hy, flowStd)
        # externalize error
        self.flowerror = [dpxl[0]-predicted_x, dpxl[1]-predicted_y]

    def finalize(self):
        # update quaternion according to quaternion error (from kalman state)
        angle:cython.float
        ca:cython.float
        sa:cython.float
        angle  = np.linalg.norm(self.x[6:9])
        ca     = np.cos(angle/2)
        sa     = np.sin(angle/2)
        if  angle==0:
            dq = np.array([1,0,0,0])
        else:
            dq = np.array([ca, sa*self.x[6]/angle, sa*self.x[7]/angle, sa*self.x[8]/angle])
        self.q = quatMult(dq,self.q)      # rotation in quaternions
        self.q = self.q/np.linalg.norm(self.q) # normalize

        # rotate covariance since we rotated body
        d = self.x[6:9]/2
        # build A (linearised dynamics)
        A = np.zeros((9,9))
        A[0:3,0:3] = [[1,0,0],[0,1,0],[0,0,1]]
        A[3:6,3:6] = [[1,0,0],[0,1,0],[0,0,1]]
        A[6:9,6:9] = spl.expm(cross(-d))
        AP = np.matmul(A,self.P)             # compute A*P
        self.P = np.matmul(AP,A.transpose()) # compute A*P*A'
        self.sanityCheckP()

        # computes rotation matrix from quaternion q
        qw: cython.double
        qx: cython.double
        qy: cython.double
        qz: cython.double
        qw = self.q[0]
        qx = self.q[1]
        qy = self.q[2]
        qz = self.q[3]
        self.R = np.array([[qw**2+qx**2-qy**2-qz**2,        2*(qx*qy-qw*qz),         2*(qx*qz+qw*qy)],\
                          [         2*(qx*qy+qw*qz),qw**2-qx**2+qy**2-qz**2,         2*(qy*qz-qw*qx)],\
                          [         2*(qx*qz-qw*qy),        2*(qy*qz+qw*qx),qw**2-qx**2-qy**2+qz**2]])
        self.x[6:9] = [0,0,0] # reset attitude error

        # compute externalised state
        self.stateExternal[0:3] = self.x[0:3] # position
        self.stateExternal[3:6] = self.R.dot(self.x[3:6]) # speed in world frame
        # quaternionToEuler angles
        #utility function to translate quaternion in Euler angles for plotting
        phi: cython.double
        theta: cython.double
        psi: cython.double
        phi   = math.atan2(2*(qw*qx + qy*qz), 1-2*(qx**2+qy**2))
        theta = math.asin(2*(qw*qy - qz*qx))
        psi   = math.atan2(2*(qw*qz + qx*qy), 1-2*(qy**2+qz**2))
        self.stateExternal[6:9] = np.array([phi, theta, psi])
        # self.stateExternal[6:9] = self.quaternionToEuler(self.q)

    @cython.cdivision(True)
    def runEKF(self, acc, gyro, pxCount, zrange):
        # synchronization macros
        mainRate: cython.float
        predictionRate: cython.float
        zrangingRate: cython.float
        flowRate: cython.float
        mainDT: cython.float
        predDT: cython.float
        zrDT: cython.float
        flowDT: cython.float
        mainRate        = 1000 #[Hz]
        predictionRate  = 100  #[Hz]
        zrangingRate    = 40   #[Hz]
        flowRate        = 100  #[Hz]
        mainDT = 1/mainRate
        predDT = 1/predictionRate
        zrDT   = 1/zrangingRate
        flowDT = 1/flowRate

        # main function called by main loop that takes care 
        # of all the timings of the kalman filter steps
        update = False

        if not(self.tick % (mainRate/predictionRate)):
            self.predictionStep(acc,gyro,predDT)
            update = True

        if not(self.tick % (mainRate/zrangingRate)):
            self.correctionZranging(zrange)
            update = True

        if not(self.tick % (mainRate/flowRate)):
            self.correctionFlow(pxCount,gyro,flowDT)
            update = True

        # call finalize state is update has been made
        if update:
            self.finalize()

        ## ADD PROCESS NOISE ## (used to be a function)
        # noise params
        procNoiseAcc_xy: cython.float
        procNoiseAcc_z: cython.float
        procNoiseVel: cython.float
        velNoiseForSim_xy: cython.float
        procNoisePos: cython.float
        procNoiseAtt: cython.float
        measNoiseGyro_rollpitch: cython.float
        measNoiseGyro_yaw: cython.float
        procNoiseAcc_xy = 0.5
        procNoiseAcc_z = 1.0
        procNoiseVel = 0
        velNoiseForSim_xy =0.1 # NOTE: this is different from the firmware!
        procNoisePos = 0
        procNoiseAtt = 0
        measNoiseGyro_rollpitch = 0.1
        measNoiseGyro_yaw = 0.1
        # update covariance matrix according to process-noise
        self.P[0][0] = self.P[0][0] + (procNoiseAcc_xy*mainDT*mainDT + procNoiseVel*mainDT + procNoisePos)**2
        self.P[1][1] = self.P[1][1] + (procNoiseAcc_xy*mainDT*mainDT + procNoiseVel*mainDT + procNoisePos)**2
        self.P[2][2] = self.P[2][2] + (procNoiseAcc_z*mainDT*mainDT + procNoiseVel*mainDT + procNoisePos)**2
        self.P[3][3] = self.P[3][3] + (procNoiseAcc_xy*mainDT + procNoiseVel+velNoiseForSim_xy)**2
        self.P[4][4] = self.P[4][4] + (procNoiseAcc_xy*mainDT + procNoiseVel+velNoiseForSim_xy)**2
        self.P[5][5] = self.P[5][5] + (procNoiseAcc_z*mainDT + procNoiseVel)**2
        self.P[6][6] = self.P[6][6] + (measNoiseGyro_rollpitch * mainDT + procNoiseAtt)**2
        self.P[7][7] = self.P[7][7] + (measNoiseGyro_rollpitch * mainDT + procNoiseAtt)**2
        self.P[8][8] = self.P[8][8] + (measNoiseGyro_yaw * mainDT + procNoiseAtt)**2
        self.sanityCheckP()

        self.tick = self.tick + 1 # increase counter
        return self.stateExternal, np.concatenate(([self.zerror],self.flowerror))
