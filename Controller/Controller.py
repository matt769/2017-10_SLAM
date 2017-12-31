from msvcrt import getch
import serial
import time
import random
import math
import numpy as np
import matplotlib.pyplot as plt
from msvcrt import getch

# Supporting functions
from findingCorners import *
from manageLandmarks import *
from slam import *

# Connection settings
port = 'COM6'
baudrate = 115200
timeoutNum = 0.2
print "Settings can be configured in .py file"
print 'Current settings...'
print 'port:',port
print 'baudrate:',baudrate

######################################
# SETUP #########################
######################################

# connect to robot
try:
	ser = serial.Serial(port, baudrate, timeout=timeoutNum)
	print 'Connected. Go!'
except serial.SerialException:
	print "Error: Could not connect to",port
	print "Exiting program"
	exit()
	
# charting interactive mode on
# note that plt.pause(0.001) is required after each plot type command for this to work properly
plt.ion()

	
######################################
# VARIABLES ##########################
######################################

# MODE
mode = 0	# 0 = Manual, 1 = Automatic (doesn't require any user input)
# TO BE IMPLEMENTED

# robot state
globalPosition = (0.0, 0.0)
globalHeading = 0.0
angleTurned = 0.0
distanceMoved = 0.0
dgx = 0.0	#global coords
dgy = 0.0	#global coords
dlx = 0.0	#local coords
dly = 0.0	#local coords

# map state
landmarksInUse = list()		# index = (position, observedCount)
landmarksPotential = list()	# index = (position, observedCount)
newLandmarkCandidates = list()
lx = 0.0	# distance to landmark
ly = 0.0	# distance to landmark

#charting
chartMin = -500
chartMax = 500

#program state
robotReadyForNewCommand = False
waitingForRobotResponse = False
robotResponseReceived = False
needNewUserInput = False

# sensor and position data
readingAnglesDegrees = range(80,-81,-1)
readingAnglesRadians = [x*math.pi/180 for x in readingAnglesDegrees]
sensorReadings = [0 for x in range(len(readingAnglesDegrees))]
sensedPositions = list()
allSensorReadings = list()
positionHistory = list()

# communication package types
MOTION = 1
SENSOR = 2
CONTROL = 3

######################################
# FUNCTIONS# #########################
######################################

# for sending
lastSent = time.clock()
sendFreq = 10	# in seconds

def decideNextMove():
	turn = 0.0
	move = 0.0
	return turn, move

def send():
	global lastSent
	global robotReadyForNewCommand
	turn, move = decideNextMove()
	package = "2" + "\t" + str(turn) + "\t" + str(move) + "\n"
	ser.write(package)
	# "Sent:",package.strip('\n')
	lastSent += sendFreq

# for receiving
def listenAndReceive(timeout=1):
	start = time.clock()
	dataReceived = False
	inputString = ''
	while time.clock() - start < timeout:
		inputString = ser.readline()
		if len(inputString) != 0:
			dataReceived = True
			break
	return dataReceived, inputString


######################################
# HANDLE INPUT DATA ##################
######################################

# if data is parsed/updated successfully then this function will return the data packet type
# if any failure then it will return 0
def splitAndRoute(str):
	#print str
	status = False
	inputArray = str.split()
	outputArray = []
	packetType = int(inputArray[0])
	data = inputArray[3:]	#removing the metadata
	if packetType == MOTION:
		status = parseMotionPackage(data)
	elif packetType == SENSOR:
		#print len(data)
		#print data
		status = parseSensorPackage(data)
	elif packetType == CONTROL:
		status = parseControlPackage(data)
	if status:
		return packetType
	else:
		return 0
	
def parseMotionPackage(data):
	# add sense checks and exception handling
	global angleTurned
	global dlx
	global dly
	angleTurned = float(data[0])
	dlx = float(data[1])
	dly = float(data[2])
	#print angleTurned, dlx, dly
	return True	#default return True until there is error checking
	
def parseSensorPackage(data):
	# add sense checks and exception handling
	global sensorReadings
	for idx in range(len(sensorReadings)):
		sensorReadings[idx] = int(data[idx])
	#print sensorReadings
	return True	#default return True until there is error checking

def parseControlPackage(data):
	# add sense checks and exception handling
	return True	#default return True until there is error checking


######################################
# PROCESS MOTION DATA ################
######################################

def processNewMotionData():
	global globalPosition
	global globalHeading
	global positionHistory
	print "Received motion data:",angleTurned, dlx, dly
	globalPosition, globalHeading = updatePositionAfterMovement(angleTurned, dlx, dly)
	positionHistory.append((globalPosition,globalHeading))
	# dgx, dgy already updated during updatePositionAfterMovement()
	print "New position:", globalPosition, globalHeading

def updatePositionAfterMovement(angleTurned, dlx, dly):
	global dgx
	global dgy
	# for the global change, rotate the local change by the current heading of the robot
	dgx = dlx*math.cos(globalHeading) - dly*math.sin(globalHeading)
	dgy = dlx*math.sin(globalHeading) + dly*math.cos(globalHeading)
	x = globalPosition[0] + dgx
	y = globalPosition[1] + dgy
	# and then udpdate the heading of the robot
	heading = (globalHeading + angleTurned) % (math.pi * 2)
	return (x, y), heading

######################################
# PROCESS MEASUREMENT DATA ###########
######################################

# *** split this into separate functions ***
def processNewSensorData():
	global chartMin
	global chartMax
	print "Received sensor data:",sensorReadings
	allSensorReadings.extend(sensorReadings)	# keep full list (this will need to be changed in future)
	#print len(allSensorReadings)
	readingPositions = calculateSensorReadingPositions(readingAnglesRadians, sensorReadings)
	xp, yp = convertReadingsForPlot(readingPositions)
	plt.scatter(xp, yp, c = 'b', marker = '.')
	plt.pause(0.001)
	chartMin = min(chartMin,min(xp),min(yp))
	chartMax = max(chartMax,max(xp),max(yp))
	plt.axis([chartMin,chartMax,chartMin,chartMax])
	plt.pause(0.001)
	# check for corner and show if found
	cornerFound, cornerPositions, cornerPositionsIdx = getCorners(readingPositions,sensorReadings)
	#print cornerFound, cornerPositions
	for corner in cornerPositions:
		plt.plot(corner[0],corner[1],'r.')	# show corner if found
		plt.pause(0.001)
	return cornerPositions, cornerPositionsIdx

def calculateSensorReadingPositions(angles, distances):
	global sensedPositions
	readingPositions = []
	for idx in range(len(distances)):#
		# add sense check
		if distances[idx] <= 0.02:
			readingPositions.append(None)
			continue
		positionHeading = (globalHeading + angles[idx]) % (math.pi * 2)
		x = globalPosition[0] + distances[idx] * math.cos(positionHeading)
		y = globalPosition[1] + distances[idx] * math.sin(positionHeading)
		#print x,y,positionHeading
		readingPositions.append((x,y))
	sensedPositions = readingPositions	#adding this in here so I can use it elsewhere
	#print readingPositions
	return readingPositions

def createMeasurementsForSlam(sensedLandmarks):
	measurementsForSlam = list()
	for sensorIdx, landmarkIdx in sensedLandmarks:
		# calculate distance to landmark (based on sensor reading)
		# don't want the global coords that I have previously calculated
		reading = sensorReadings[sensorIdx]
		angle = readingAnglesRadians[sensorIdx]
		lx = reading*math.cos(angle)
		ly = reading*math.sin(angle)
		measurementsForSlam.append((landmarkIdx, (lx, ly)))
		#print landmarkIdx, lx, ly
	return measurementsForSlam

			
######################################
# OTHER ##############################
######################################

def updateHeading(sensedLandmarks):
	count = 0.0
	calculatedHeading = 0.0
	vx = 0
	vy = 0
	for sensorIdx, landmarkIdx in sensedLandmarks:
		#print "calculate heading"
		# landmark is at global angle...
		#print globalPosition, landmarksInUse[landmarkIdx]
		dx = landmarksInUse[landmarkIdx][0][0] - globalPosition[0]
		dy = landmarksInUse[landmarkIdx][0][1] - globalPosition[1]
		atGlobalAngle = math.atan2(dy,dx)
		atLocalAngle = readingAnglesRadians[sensorIdx]
		# therefore robot heading is...
		calculatedHeading = atGlobalAngle - atLocalAngle
		#print atGlobalAngle, atLocalAngle,calculatedHeading
		# accumulate vectors
		vx += math.cos(calculatedHeading)
		vy += math.sin(calculatedHeading)
		count += 1
	vx /= count
	vy /= count
	averageAngle = math.atan2(vy,vx)
	print "calculated heading:", averageAngle
	return averageAngle

def updateLandmarkPositions(landmarkPositions):
	updatedLandmarksInUse = list()
	for landmarkIdx in range(len(landmarkPositions)/2):
		x, y = landmarkPositions.item(landmarkIdx*2), landmarkPositions.item(landmarkIdx*2+1)
		count = landmarksInUse[landmarkIdx][1]
		updatedLandmarksInUse.append(((x,y),count))
	return updatedLandmarksInUse



######################################
# PLOT HELPER ########################
######################################

# for charting
def convertReadingsForPlot(data):
	x = list()
	y = list()
	for idx in range(len(data)):
		if data[idx] is None:
			continue
		x.append(data[idx][0])
		y.append(data[idx][1])
	return x,y

def plotRobotPositionAfterMotion():
	plt.plot(globalPosition[0],globalPosition[1],'gs')
	plt.pause(0.001)
	return

def plotRobotPositionAfterSlam():
	plt.plot(globalPosition[0],globalPosition[1],'rp')
	plt.pause(0.001)
	return

def plotLandmarks():
	x = [a[0][0] for a in landmarksInUse]
	y = [a[0][1] for a in landmarksInUse]
	plt.plot(x,y,'yo')
	plt.pause(0.001)
	return

def replot():
	plt.clf()
	plotLandmarks()
	plotRobotPositionAfterSlam()
	plt.axis([chartMin,chartMax,chartMin,chartMax])
	plt.pause(0.001)
	return

######################################
# MAIN START #########################
######################################
print "Starting position:", globalPosition, globalHeading

needNewUserInput = True

while True:
	if needNewUserInput:
		userInput = raw_input("Enter next command, Enter for default, q to quit\n")
		if userInput == '':
			# default path
			robotReadyForNewCommand = True
			needNewUserInput = False
		if userInput == 'q':
			break
		if userInput != '':
			print "Not valid command"
			# add possibility to give custom commands

	if robotReadyForNewCommand:
		send()
		robotReadyForNewCommand = False
		waitingForRobotResponse = True
	if waitingForRobotResponse:
		(result,input) = listenAndReceive(1)
		#print result, input
		if result:
			packetType = splitAndRoute(input)
			#print packetType
			if packetType == MOTION:
				#processNewMotionData()
				placeholder = None
			elif packetType == SENSOR:
				#processNewSensorData()
				waitingForRobotResponse = False	# rethink placement
				robotResponseReceived = True
			elif packetType == CONTROL:
				print "Packet not yet defined. You shouldn't be here."
			else:
				print "Data receipt failure"

	if robotResponseReceived:
		processNewMotionData()
		plotRobotPositionAfterMotion()	# plot estimated position after movement
		newLandmarkCandidates, newLandmarkCandidatesIdx  = processNewSensorData()
		sensedLandmarks = processAllLandmarks(landmarksInUse, landmarksPotential, newLandmarkCandidates, newLandmarkCandidatesIdx)
		# landmarksInUse and landmarksPotential are also updated
		# TO DO - MOVE BELOW TO PLOT FUNCTION
		print "sensedLandmarks", sensedLandmarks
		if len(sensedLandmarks)>0:
			for sensorIdx, landmarkIdx in sensedLandmarks:
				x,y = landmarksInUse[landmarkIdx][0]
				plt.plot(x,y,'yo')	# show corner if found
				plt.pause(0.001)
		# SLAM
		motionForSlam = (dgx,dgy)
		measurementsForSlam = createMeasurementsForSlam(sensedLandmarks)
		robotPosition, landmarkPositions = runSlam(motionForSlam,measurementsForSlam)
		print "robotPosition:\n", robotPosition
		print "landmarkPositions:\n", landmarkPositions
		globalPosition = (robotPosition.item(0), robotPosition.item(1))	# update robot position
		# Update the lain landmark list - this would be simpler if I didn't save the count (which probably isn't required)
		landmarksInUse = updateLandmarkPositions(landmarkPositions)
		#print landmarksInUse
		
		plotRobotPositionAfterSlam()
		# TO DO...
		# Replot landmarks
		# need to also replot the individual readings
		# but need to adjust these given the landmarks have moved
		# will need to record their relationship to landmarks
		
		# Update heading based on updated position and angle/distance to known landmark
		if len(sensedLandmarks) > 0:
			globalHeading = updateHeading(sensedLandmarks)
		
		robotResponseReceived = False
		needNewUserInput = True


######################################
# MAIN END ###########################
######################################



try:
	#ser.write('x')
	ser.close()
	print "Closing connection"
except serial.SerialException:
	print "Connection lost"
	pass

print "Exiting program"
exit()