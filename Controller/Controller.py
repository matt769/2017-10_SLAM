# add button to quit and close connection
# choose when to send? or have option for either

# movement noise
# sensor noise


from msvcrt import getch
import serial
import time
import random
import math
import numpy as np
import matplotlib.pyplot as plt
from msvcrt import getch


from findingCorners import *
from manageLandmarks import *
from slam import *

port = 'COM6'
baudrate = 115200
timeoutNum = 0.2
print "Settings can be configured in .py file"
print 'Current settings...'
print 'port:',port
print 'baudrate:',baudrate

# don't think this is perfect but intended to deal with situation where connection is already opon
try:
	ser = serial.Serial(port, baudrate, timeout=timeoutNum)
	print 'Connected. Go!'
except serial.SerialException:
	print "Error: Could not connect to",port
	print "Exiting program"
	exit()

	

######################################
# MAIN SETUP #########################
######################################

# MODE
mode = 0	# 0 = Manual, 1 = Automatic (doesn't require any user input)
# TO BE IMPLEMENTED


# robot state
globalPosition = (0.0, 0.0)
globalHeading = 0.0
angleTurned = 0.0
distanceMoved = 0.0
dx = 0.0
dy = 0.0

# map state
landmarksInUse = list()		# index = (position, observedCount)
landmarksPotential = list()	# index = (position, observedCount)
newLandmarkCandidates = list()
lx = 0.0	# distance to landmark
ly = 0.0	# distance to landmark

#program state
robotReadyForNewCommand = False
waitingForRobotResponse = False
robotResponseReceived = False
needNewUserInput = False

## change this to match 160 
readingAnglesDegrees = range(80,-81,-1)
readingAnglesRadians = [x*math.pi/180 for x in readingAnglesDegrees]
sensorReadings = [0 for x in range(len(readingAnglesDegrees))]
allSensorReadings = list()
positionHistory = list()

#package types
MOTION = 1
SENSOR = 2
CONTROL = 3

# charting interactive mode on
plt.ion()

######################################
# FUNCTIONS# #########################
######################################

# for sending
lastSent = time.clock()
sendFreq = 10	# in seconds
defaultTurn = 0.0		# radians
defaultMove = 200.0		# millimeters

def decideNextMove2():
	turn = random.randint(-100,100)/100.0
	move = random.randint(300,500)
	return turn, move
	
def decideNextMove3():
	turn = 0.0
	move = 0.0
	return turn, move

def decideNextMove():
	turn = 1.57
	move = 200.0
	return turn, move

def send():
	global lastSent
	global robotReadyForNewCommand
	#if time.clock() - lastSent > sendFreq:
	#if robotReadyForNewCommand:
	turn, move = decideNextMove()
	#package = "1" + "\t" + str(random.randint(1,100)) + "\n"
	package = "2" + "\t" + str(turn) + "\t" + str(move) + "\n"
	#package = "2" + "\t" + str(defaultTurn) + "\t" + str(defaultMove) + "\n"
	#package = "2" + "\t" + str(1.5) + "\t" + str(800.0) + "\n"
	ser.write(package)
	print "Sent:",package.strip('\n')
	lastSent += sendFreq
	#robotReadyForNewCommand = False	# set back to true when valid sensor data is received

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
	global distanceMoved
	angleTurned = float(data[0])
	distanceMoved = float(data[1])
	#print angleTurned, distanceMoved
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
	

def updatePositionAfterMovement(angleTurned, distanceMoved):
	heading = (globalHeading + angleTurned) % (math.pi * 2)
	x = globalPosition[0] + distanceMoved * math.sin(heading)
	y = globalPosition[1] + distanceMoved * math.cos(heading)
	return (x, y), heading


def calculateSensorReadingPositions(angles, distances):
	readingPositions = []
	for idx in range(len(distances)):#
		# add sense check
		if distances[idx] <= 0.02:
			readingPositions.append(None)
			continue
		positionHeading = (globalHeading + angles[idx]) % (math.pi * 2)
		x = globalPosition[0] + distances[idx] * math.sin(positionHeading)
		y = globalPosition[1] + distances[idx] * math.cos(positionHeading)
		#print x,y,positionHeading
		readingPositions.append((x,y))
	#print readingPositions
	return readingPositions


def updatePositionAfterSense():
	# this will be one of the final functions to implement
	return False


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


######################################
# MOTION UPDATE ######################
######################################
def processNewMotionData():
	global globalPosition
	global globalHeading
	global positionHistory
	global dx
	global dy
	print "Received motion data:",angleTurned, distanceMoved
	globalPosition, globalHeading = updatePositionAfterMovement(angleTurned, distanceMoved)
	positionHistory.append((globalPosition,globalHeading))
	#addition to incorporate slam
	dx = distanceMoved * math.sin(globalHeading)
	dy = distanceMoved * math.cos(globalHeading)
	print "New position:", globalPosition, globalHeading




######################################
# MEASUREMENT UPDATE #################
######################################
def processNewSensorData():
	print "Received sensor data:",sensorReadings
	allSensorReadings.extend(sensorReadings)	# keep full list (this will need to be changed in future)
	#print len(allSensorReadings)
	readingPositions = calculateSensorReadingPositions(readingAnglesRadians, sensorReadings)
	xp, yp = convertReadingsForPlot(readingPositions)
	plt.scatter(xp, yp, c = 'b', marker = '.')
	plt.pause(0.001)

	# check for corner and show if found
	cornerFound, cornerPositions, cornerPositionsIdx = getCorners(readingPositions,sensorReadings)
	print cornerFound, cornerPositions
	for corner in cornerPositions:
		plt.plot(corner[0],corner[1],'r.')	# show corner if found
		plt.pause(0.001)
	return cornerPositions, cornerPositionsIdx




######################################
# MAIN START #########################
######################################
print "Starting position:", globalPosition, globalHeading

needNewUserInput = True
#robotReadyForNewCommand = True

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
		newLandmarkCandidates, newLandmarkCandidatesIdx  = processNewSensorData()

		# DEAL WITH LANDMARKS
		sensedLandmarks = processAllLandmarks(landmarksInUse, landmarksPotential, newLandmarkCandidates, newLandmarkCandidatesIdx)
		# landmarksInUse and landmarksPotential are also updated
		print "sensedLandmarks", sensedLandmarks
		if len(sensedLandmarks)>0:
			for sensorIdx, landmarkIdx in sensedLandmarks:
				x,y = landmarksInUse[landmarkIdx][0]
				plt.plot(x,y,'yo')	# show corner if found
				plt.pause(0.001)
		
		# RUN SLAM
		motionForSlam = (dx,dy)
		#measurements = landmark idx, distance x, distance y
		measurementsForSlam = list()
		for sensorIdx, landmarkIdx in sensedLandmarks:
			# calculate distance to landmark (based on sensor reading)
			# don't want the global coords that I have previously calculated
			reading = sensorReadings[sensorIdx]
			angle = readingAnglesRadians[sensorIdx]
			lx = reading*math.sin(angle)
			ly = reading*math.cos(angle)
			measurementsForSlam.append((landmarkIdx, (lx, ly)))
			#print landmarkIdx, lx, ly
		# I need to know which sensor reading was associated with each landmark found
		# pass motion, plus list of landmark indices and distances
		robotPosition, landmarkPositions = runSlam(motionForSlam,measurementsForSlam)
		print robotPosition, landmarkPositions
		
		# UPDATE HEADING
		# based on updated position and angle/distance to known landmark
		
		
		robotResponseReceived = False
		needNewUserInput = True




######################################
# MAIN END ###########################
######################################


# WHAT's THE FLOW?
#send command
#wait for data to come back
#do some calculations
# repeat
# (in future may get multiple data receipts before deciding to take action)
# will need to understand when to stop receiving



try:
	#ser.write('x')
	ser.close()
	print "Closing connection"
except serial.SerialException:
	print "Connection lost"
	pass

print "Exiting program"
exit()