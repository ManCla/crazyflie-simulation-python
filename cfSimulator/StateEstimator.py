# class implementing the extended Kalman filter that is run on the Crazyflie

import numpy as np
import math
import scipy.linalg as spl

# synchronization macros
mainRate        = 1000 #[Hz]
predictionRate  = 100  #[Hz]
zrangingRate    = 40   #[Hz]
flowRate        = 100  #[Hz]
mainDT = 1/mainRate
predDT = 1/predictionRate
zrDT   = 1/zrangingRate
flowDT = 1/flowRate

def rateDo(rate, tick):
    #utility function to trigger events at expected rate
    return not(tick % (mainRate/rate))

class cfEKF():

    def __init__(self):
        # drone parameters
        self.g = 9.81

        # filter params
        self.procNoiseAcc_xy = 0.5
        self.procNoiseAcc_z = 1.0
        self.procNoiseVel = 0
        self.velNoiseForSim_xy =0.1 # NOTE: this is different from the firmware!
        self.procNoisePos = 0
        self.procNoiseAtt = 0
        self.measNoiseGyro_rollpitch = 0.1
        self.measNoiseGyro_yaw = 0.1
        # zRagner noise model coefficients
        self.expPointA = 2.5
        self.expStdA   = 0.0025
        self.expCoeff  = 2.92135
        # optical flow noise model
        self.flowStd   = 2 # used for both directions

        # filter states
        self.x = np.zeros(9)     # estimated state: position, speed, attitude error
        self.q = np.array([1,0,0,0])
        self.R = np.identity(3)
        self.P = np.diag([100,100,1,0.01,0.01,0.01,0.01,0.01,0.01])  # covar matrix init

        self.stateExternal = np.zeros(9)
        self.flowerror = np.zeros(2)
        self.zerror = 0

        self.tick = 0 # counter 

    #########################
    ### UTILITY FUNCTIONS ###
    #########################
    def cross(self, x):
        # utility function: compute skew symmetric matrix from 3-dim vector
        # input : three dimensional vector
        # output: three-by-three skew symmetric matrix
        mcross = np.array([[ 0,   -x[2],  x[1]],\
                           [ x[2],    0, -x[0]],\
                           [-x[1], x[0],    0]])
        return mcross

    def updateR(self):
        # computes rotation matrix from quaternion q
        qw = self.q[0]
        qx = self.q[1]
        qy = self.q[2]
        qz = self.q[3]
        self.R = np.array([[qw**2+qx**2-qy**2-qz**2,        2*(qx*qy-qw*qz),         2*(qx*qz+qw*qy)],\
                          [         2*(qx*qy+qw*qz),qw**2-qx**2+qy**2-qz**2,         2*(qy*qz-qw*qx)],\
                          [         2*(qx*qz-qw*qy),        2*(qy*qz+qw*qx),qw**2-qx**2-qy**2+qz**2]])

    def sanityCheckP(self):
        # saturate
        for i in range(9):
            self.P[i,i] = min(max(self.P[i,i],0),100)
        # enforce symmetry
        self.P = (self.P+self.P.transpose())/2

    def quatMult(self, dq, q):
        out = np.zeros(4)
        out[0] = dq[0]*q[0]-dq[1]*q[1]-dq[2]*q[2]-dq[3]*q[3]
        out[1] = dq[1]*q[0]+dq[0]*q[1]+dq[3]*q[2]-dq[2]*q[3]
        out[2] = dq[2]*q[0]-dq[3]*q[1]+dq[0]*q[2]+dq[1]*q[3]
        out[3] = dq[3]*q[0]+dq[2]*q[1]-dq[1]*q[2]+dq[0]*q[3]
        return out

    def quaternionToEuler(self, q):
        #utility function to translate quaternion in Euler angles for plotting
        phi   = math.atan2(2*(q[0]*q[1] + q[2]*q[3]), 1-2*(q[1]**2+q[2]**2))
        theta = math.asin(2*(q[0]*q[2] - q[3]*q[1]))
        psi   = math.atan2(2*(q[0]*q[3] + q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
        eta = np.array([phi, theta, psi])
        return eta

    ###########################
    ### ALGORITHM FUNCTIONS ###
    ###########################

    def predictionStep(self, acc, gyro, dt):
        # use IMU data to predict state forward

        d = gyro*dt/2
        # build A (linearised dynamics)
        A = np.zeros((9,9))
        A[0:3,0:3] = np.identity(3)
        A[0:3,3:6] = self.R*dt
        A[0:3,6:9] = np.matmul(self.R,self.cross(-self.x[3:6]))*dt
        A[3:6,3:6] = np.identity(3)+self.cross(-gyro)*dt
        A[3:6,6:9] = self.g*self.cross(-self.R[2,:])*dt
        A[6:9,6:9] = spl.expm(self.cross(-d))

        # covariance update according to system dynamics 
        AP = np.matmul(A,self.P)             # compute A*P
        self.P = np.matmul(AP,A.transpose()) # compute A*P*A'

        # prediction
        dt2 = pow(dt,2)
        v   = self.x[3:6]*dt+acc*(dt2/2)
        self.x[0:3] = self.x[0:3]+self.R.dot(v)+np.array([0,0,-self.g*dt2/2])
        self.x[3:6] = self.x[3:6]+dt*(acc-(self.cross(gyro).dot(self.x[3:6]))-self.g*self.R[2,:])
        angle  = np.linalg.norm(gyro*dt)
        ca     = np.cos(angle/2)
        sa     = np.sin(angle/2)
        if  angle==0:
            dq = np.array([1,0,0,0])
        else:
            dq = np.array([ca, sa*dt*gyro[0]/angle, sa*dt*gyro[1]/angle, sa*dt*gyro[2]/angle])
        self.q = self.quatMult(dq,self.q)      # rotation in quaternions
        self.q = self.q/np.linalg.norm(self.q) # normalize
 

    def addProcessNoise(self, dt):
        # update covariance matrix according to process-noise
        self.P = self.P + np.diag(np.power([self.procNoiseAcc_xy*dt*dt + self.procNoiseVel*dt + self.procNoisePos,\
                                            self.procNoiseAcc_xy*dt*dt + self.procNoiseVel*dt + self.procNoisePos,\
                                            self.procNoiseAcc_z*dt*dt + self.procNoiseVel*dt + self.procNoisePos,\
                                            self.procNoiseAcc_xy*dt + self.procNoiseVel+self.velNoiseForSim_xy,\
                                            self.procNoiseAcc_xy*dt + self.procNoiseVel+self.velNoiseForSim_xy,\
                                            self.procNoiseAcc_z*dt + self.procNoiseVel,\
                                            self.measNoiseGyro_rollpitch * dt + self.procNoiseAtt,\
                                            self.measNoiseGyro_rollpitch * dt + self.procNoiseAtt,\
                                            self.measNoiseGyro_yaw * dt + self.procNoiseAtt ],2))
        self.sanityCheckP()

    def scalarUpdate(self, error, H, std):
        # given the measurement Jacobian 'H' and the innovation 'error'
        # update state and covariance
        R      = pow(std,2)
        PHt    = self.P.dot(H)                 # no need to transpose in numpy
        HPHtR  = H.dot(PHt)+R                  # HPH'+R
        K      = PHt/HPHtR                     # kalman gain
        self.x = self.x+K*error                # measurement update
        IKH    = np.identity(9)-np.outer(K,H)  #
        IKHP   = np.matmul(IKH,self.P)
        self.P = np.matmul(IKHP,IKH.transpose())+np.outer(K,K)*R
        self.sanityCheckP()

    def correctionZranging(self, meas):
        # update estimate with Z laser ranging data
        pred = self.x[2]/self.R[2,2]
        H    = np.array([0,0,1/self.R[2,2],0,0,0,0,0,0])
        std  = self.expStdA * (1 + np.exp(self.expCoeff * (self.x[2] - self.expPointA)))
        self.scalarUpdate(meas-pred, H, std)
        # externalize error
        self.zerror = meas-pred

    def correctionFlow(self, dpxl, gyro, dt):
        # update estimate with flow data
        Npx     = 30.0            #
        thetapx = 4.2*np.pi/180.0 #
        wFactor = 1.25            #
        coeff   = Npx*dt/thetapx
        z       = self.x[2] if self.x[2]>0.1 else 0.1
        # X direction
        predicted_x = coeff*((self.x[3]*self.R[2,2]/z) - wFactor * gyro[1])
        Hx = np.array([0, 0, coeff*((self.R[2,2]*self.x[3])/(-z*z)),\
                       coeff*self.R[2,2]/z, 0, 0,\
                       0, 0, 0])
        self.scalarUpdate(dpxl[0]-predicted_x, Hx, self.flowStd)
        # Y direction
        predicted_y = coeff*((self.x[4]*self.R[2,2]/z) + wFactor * gyro[0])
        Hy = np.array([0, 0, coeff*((self.R[2,2]*self.x[4])/(-z*z)),\
                       0, coeff*self.R[2,2]/z, 0,\
                       0, 0, 0])
        self.scalarUpdate(dpxl[1]-predicted_y, Hy, self.flowStd)
        # externalize error
        self.flowerror = np.array([dpxl[0]-predicted_x, dpxl[1]-predicted_y])

    def finalize(self):
        # update quaternion according to quaternion error (from kalman state)
        angle  = np.linalg.norm(self.x[6:9])
        ca     = np.cos(angle/2)
        sa     = np.sin(angle/2)
        if  angle==0:
            dq = np.array([1,0,0,0])
        else:
            dq = np.array([ca, sa*self.x[6]/angle, sa*self.x[7]/angle, sa*self.x[8]/angle])
        self.q = self.quatMult(dq,self.q)      # rotation in quaternions
        self.q = self.q/np.linalg.norm(self.q) # normalize

        # rotate covariance since we rotated body
        d = self.x[6:9]/2
        # build A (linearised dynamics)
        A = np.zeros((9,9))
        A[0:3,0:3] = np.identity(3)
        A[3:6,3:6] = np.identity(3)
        A[6:9,6:9] = spl.expm(self.cross(-d))
        AP = np.matmul(A,self.P)             # compute A*P
        self.P = np.matmul(AP,A.transpose()) # compute A*P*A'
        self.sanityCheckP()

        self.updateR()
        self.x[6:9] = np.zeros(3) # reset attitude error

        # compute externalised state
        self.stateExternal[0:3] = self.x[0:3] # position
        self.stateExternal[3:6] = self.R.dot(self.x[3:6]) # speed in world frame
        self.stateExternal[6:9] = self.quaternionToEuler(self.q)

    def runEKF(self, acc, gyro, pxCount, zrange):
        # main function called by main loop that takes care 
        # of all the timings of the kalman filter steps
        update = False

        if rateDo(predictionRate, self.tick):
            self.predictionStep(acc,gyro,predDT)
            update = True

        if rateDo(zrangingRate, self.tick):
            self.correctionZranging(zrange)
            update = True

        if rateDo(flowRate, self.tick):
            self.correctionFlow(pxCount,gyro,flowDT)
            update = True

        # call finalize state is update has been made
        if update:
            self.finalize()

        self.addProcessNoise(mainDT)

        self.tick = self.tick + 1 # increase counter
        return self.stateExternal, np.concatenate(([self.zerror],self.flowerror))
