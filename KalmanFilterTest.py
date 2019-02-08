# This program is a virtual Kalman Filter for positional tracking. It is meant for simulation and performance measurements.



import time
import numpy as np
import math



##########          **********          Kalman Filter Class Begin          **********          ##########		
	

class KalmanFilterPosVelAcc:

	def __init__(self, dt, ProcPosVarXYZ, ProcVelVarXYZ, ObsPosVarXYZ, ObsVelVarXYZ):

		#Define in seconds the time between process/update cycles
		self.dt = dt #Based on time elapsed per cycle
		# THE A MATRIX - USED FOR STATE CALCULATION
		#We are tracking position and velocity so we use the formula x = x + vt + 0.5at^2
		#State is a 2x1 matrix with position at (0,0) and velocity at (1,0)
		#The product of A with the initial state should yield a 2x1 matrix with position and velocity
		#Using the formula we find that the A matrix is a 2x2 matrix which calculates the new "expected" positions
		#Thus A = [[1, dt],[0,1]] as it adds velocity change to the position and "keeps" the velocity as we are still tracking it
		self.Amat = np.array([[1, dt],[0, 1]]) #Based on the system
		# THE B MATRIX - USED FOR STATE CALCULATION
		#We need to incorporate how the observed acceleration affects the states position and velocity
		#We see we need to multiply the acceleration (1x1 matrix) by Bmat to acheive this
		#Using the position formula above, Bmat = [0.5dt^2, dt]
		self.Bmat = np.array([[(0.5*(dt**2))],[dt]]) #Based on the system
		# THE Q MATRIX - ERROR IN PCOV CALC
		#Qmat contains the error in calculating the error in the process covariance matrix
		#We are ignoring it here for simplicity
		# THE C MATRIX - TRANSFORMATION MATRIX
		#Cmat transforms the observation into the format of the state
		#Defined by the system, here it is a 2x2 identity matrix
		self.Cmat = np.array([[1,0,0],[0,1,0],[1,0,0]])
		# THE H MATRIX - TRANSFORMATION MATRIX
		#Hmat transforms PCOV into the format of the Kalman Gain
		#Defined by the system, here it is a 2x2 identity matrix
		self.Hmat = np.array([[1,0],[0,1]])
		# THE I MATRIX - IDENTITY MATRIX
		#Used to update PCOV
		self.Imat = np.identity(2)
		# THE R MATRIX - OBSERVATION COVARIANCE MATRIX
		#Rmat contains the variance and covariance of observed quantities by the system
		#Again, like with the PCOV below, we are ignoring the covariance for simplicity
		xRpos = ObsPosVarXYZ[0]
		yRpos = ObsPosVarXYZ[1]
		zRpos = ObsPosVarXYZ[2]
		xRvel = ObsVelVarXYZ[0]
		yRvel = ObsVelVarXYZ[1]
		zRvel = ObsVelVarXYZ[2]
		xRmat = [[xRpos**2,0],[0,xRvel**2]]
		yRmat = [[yRpos**2,0],[0,yRvel**2]]
		zRmat = [[zRpos**2,0],[0,zRvel**2]]
		self.Rmat = np.array([xRmat, yRmat, zRmat])
		#Here we use assumptions/information to create the initial process error covariance matrix
		#Recall, deviation from the average is the difference between the average of all observations and a single observation
		#Recall, the square of the deviation from the average is the square of the deviation
		#Recall, the sum of all square of the deviations divided by the number of data points is the variance
		#Recall, the square root of the variance is the standard deviation
		#Recall, the covariance between TYPES of observation is the sum of the products of each corresponding observation's 
			#standard deviation, divided by the number of observation taken
		#A variance/covariance matrix has the variance of each variable along the diagonal and respective covariances in all other positions
		#The 3D form of this is as follows:
		#[[VAR(X), COV(X,Y), COV(X,Z)]
		#[COV(Y,X), VAR(Y), COV(Y,Z)]
		#[COV(Z,X), COV(Z,Y), VAR(Z)]]
		#If we know there is no correlation between X,Y,Z the COV terms can be zero'd
		#For simplicity we ignore covariance terms
		xProcPosVar = ProcPosVarXYZ[0]
		yProcPosVar = ProcPosVarXYZ[1]
		zProcPosVar = ProcPosVarXYZ[2]
		xProcVelVar = ProcVelVarXYZ[0]
		yProcVelVar = ProcVelVarXYZ[1]
		zProcVelVar = ProcVelVarXYZ[2]
		xPCOV = [[xProcPosVar**2, 0],[0, xProcVelVar**2]]
		yPCOV = [[yProcPosVar**2, 0],[0, yProcVelVar**2]]
		zPCOV = [[zProcPosVar**2, 0],[0, zProcVelVar**2]]
		self.PCOV = np.array([xPCOV,yPCOV,zPCOV])
		#Here we initialize the state as zeros so that we can track it as a class attribute
		shape = (2,3)
		self.State = np.zeros(shape)
		#Here we initialize the Kalman gain as zeros so that we can track it as a class attribute
		shape = (2,2)
		self.KG = np.zeros(shape)


	#This can be used to find the initial state by using the initial observations
	def predictState(self, PosVelObservations, AccObservations):

		dt = self.dt ##########***************YOU SHOULD BE ABLE TO CHANGE THIS EACH CYCLE BY TRACKING IT!!!! DYNAMIC DT!!!
		Amat = self.Amat
		Bmat = self.Bmat
		#The observations are used to predict the next state, they are split into position/velocity and acceleration
		PV = PosVelObservations
		A = AccObservations

		#Thus, we can predict the next state
		#Looking at just the x dimension with x=4000, vx=280, ax=2
		PVmat = np.matmul(Amat,PV)
		Amat = np.matmul(Bmat,A)
		predictedState = PVmat+Amat
		self.State = predictedState

		return predictedState


	def predictPCOV(self):

		Amat = self.Amat
		AmatT = np.transpose(Amat)

		#Cross-terms are ignored for simplicity
		xPrevPCOV = self.PCOV[0]
		yPrevPCOV = self.PCOV[1]
		zPrevPCOV = self.PCOV[2]
		xPCOVnoERR = np.matmul(np.matmul(Amat,xPrevPCOV),AmatT)
		xPCOVnoERR[0][1] = 0
		xPCOVnoERR[1][0] = 0
		yPCOVnoERR = np.matmul(np.matmul(Amat,yPrevPCOV),AmatT)
		yPCOVnoERR[0][1] = 0
		yPCOVnoERR[1][0] = 0
		zPCOVnoERR = np.matmul(np.matmul(Amat,zPrevPCOV),AmatT)
		zPCOVnoERR[0][1] = 0
		zPCOVnoERR[1][0] = 0

		#Qmat is ignored. If it weren't, we would add it to the following matrix
		self.PCOV = np.array([xPCOVnoERR,yPCOVnoERR,zPCOVnoERR])

		return self.PCOV


	def calcKalmanGain(self):

		Hmat = self.Hmat
		HmatT = np.transpose(Hmat)
		Rmat = self.Rmat
		xRmat = Rmat[0]
		yRmat = Rmat[1]
		zRmat = Rmat[2]
		PCOV = self.PCOV
		xPCOV = PCOV[0]
		yPCOV = PCOV[1]
		zPCOV = PCOV[2]

		#xKalmanGain
		xKalmanGainNumerator = np.matmul(xPCOV, Hmat)
		xKalmanGainDenomenator = np.matmul(np.matmul(Hmat,xPCOV),HmatT)+xRmat
		xKalmanGain = np.array([[xKalmanGainNumerator[0][0]/xKalmanGainDenomenator[0][0],0],[0,xKalmanGainNumerator[1][1]/xKalmanGainDenomenator[1][1]]])
		
		yKalmanGainNumerator = np.matmul(yPCOV, Hmat)
		yKalmanGainDenomenator = np.matmul(np.matmul(Hmat,yPCOV),HmatT)+yRmat
		yKalmanGain = np.array([[yKalmanGainNumerator[0][0]/yKalmanGainDenomenator[0][0],0],[0,yKalmanGainNumerator[1][1]/yKalmanGainDenomenator[1][1]]])
		
		zKalmanGainNumerator = np.matmul(zPCOV, Hmat)
		zKalmanGainDenomenator = np.matmul(np.matmul(Hmat,zPCOV),HmatT)+zRmat
		zKalmanGain = np.array([[zKalmanGainNumerator[0][0]/zKalmanGainDenomenator[0][0],0],[0,zKalmanGainNumerator[1][1]/zKalmanGainDenomenator[1][1]]])

		KalmanGain = np.array([xKalmanGain,yKalmanGain,zKalmanGain])
		self.KG = KalmanGain

		return KalmanGain


	def Observe(self, i):

		i+=1

		#As Cmat is the identity matrix, we ignore multiplying at as outcome is the same
		Cmat = self.Cmat

		#We pull each iterations "obesrvation" from data.txt
		pullData = open('data.txt','r').read()
		lines = pullData.split('\n')
		ObservationRaw = np.array(lines[i].split('~'))
		XYZpos = np.array(ObservationRaw[0].split(','))
		XYZvel = np.array(ObservationRaw[1].split(','))
		XYZacc = np.array(ObservationRaw[2].split(','))

		Observation = np.array([XYZpos, XYZvel, XYZacc])
		# print(Observation)
		for i in range(len(Observation)):
			for j in range(len(Observation[i])):
				Observation[i][j] = float(Observation[i][j])

		print(Observation[0][0])

		# #It is assumed Observation is composed of 2x1 matrices with position and velocity in x,y,z at positions 0,1,2 respectively
		xObs = np.array([Observation[0][0],Observation[0][1],Observation[0][2]])
		yObs = np.array([Observation[1][0],Observation[1][1],Observation[1][2]])
		zObs = np.array([Observation[2][0],Observation[2][1],Observation[2][2]])

		# #Here we would then add any observation errors due to lag time etc. as a 2x1 matrix Zmat.
		# #ie. xObs = xObs+xZmat
		# #This is ignored for simplicity

		Observation = np.array([xObs,yObs,zObs])
		print(Observation)
		return Observation


	def calcCurrentState(self, PosVelObservation):

		Hmat = self.Hmat
		KG = self.KG
		predictedState = self.State

		Adjustment = np.matmul(KG, (PosVelObservation - np.matmul(Hmat,predictedState)))

		currState = predictedState + Adjustment

		self.State = currState

		return currState

	def updatePCOV(self):

		Hmat = self.Hmat
		Imat = self.Imat
		predPCOV = self.PCOV
		KalmanGain = self.KG

		Adjust = (Imat - np.matmul(KalmanGain, Hmat))
		updatedPCOV = np.matmul(Adjust,predPCOV)

		self.PCOV = updatedPCOV

		return updatedPCOV


##########          **********          Kalman Filter Class End          **********          ##########		

#Define process errors
ProcPosVarXYZ = np.array([20,20,20]) #Process Position Variance
ProcVelVarXYZ = np.array([5,3,1]) #Process Velocity Variance

#Define process errors
ObsPosVarXYZ = np.array([3,25,25]) #Observation Position Variance
ObsVelVarXYZ = np.array([6,4,2]) #Observation Velocity Variance

#Setup Object & Initial Variables
KF = KalmanFilterPosVelAcc(1, ProcPosVarXYZ, ProcVelVarXYZ, ObsPosVarXYZ, ObsVelVarXYZ)

t0 = time.time()
#KF.Observe here in lieu of below
XYZposition = np.array([4000,4000,4000])
XYZvelocity = np.array([280,280,280])
XYZacceleration = np.array([2,1,0])
Obs = np.array([XYZposition,XYZvelocity,XYZacceleration])
PosVelObs = np.array([Obs[0],Obs[1]])
AccObs = np.array([Obs[2]])
i = 0
x=KF.Observe(1)

t1 = time.time()
while(i<3):
	t0=time.time()
	KF.predictPCOV()
	KF.predictState(PosVelObs, AccObs)
	KF.calcKalmanGain()
	# print(f'Kalman Gain X: \n{KF.KG[0][0][0]}\n')
	#KF.Observe here in lieu of below***********************************************************************************************
#NEED TO PUT IT HERE OR ELSE CURRENT STATE IS JIBJAB********************************************************************************
	#KF.Observe in lieu of above****************************************************************************************************
	KF.calcCurrentState(PosVelObs)
	#Graph or something
	KF.updatePCOV()
	# print(f'Current X State: \n{KF.State[0][0][0]},{KF.State[0][1][0]}\n')
	# print(f'Current X PCOV: \n{KF.PCOV[0]}\n\n')
	t1=time.time()
	dt=t1-t0
	#Once actual data extraction and server COMS are setup, add each cycles dt here by directly changing KF.dt
	i+=1


#Calc init State & PCOV**********************************************
#Begin Loop**********************************************
	#Predict State**********************************************
	#Calc KG**********************************************
	#Import Observation**********************************************
	#Calc new state**********************************************
	#Graph or something**********************************************