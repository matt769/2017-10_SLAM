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

globalPosition = (0.0, 0.0)
globalHeading = 0.0
angleTurned = 0.0
distanceMoved = 0.0
readingAnglesDegrees = [-80,-70,-60,-50,-40,-30,-20,-10,0,10,20,30,40,50,60,70,80]
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
robotReadyForNewCommand = True;
defaultTurn = 0.0		# radians
defaultMove = 200.0		# millimeters

def decideNextMove():
	turn = random.randint(-100,100)/100.0
	move = random.randint(300,1000))
	return turn, move


def send():
	global lastSent
	global robotReadyForNewCommand
	#if time.clock() - lastSent > sendFreq:
	if robotReadyForNewCommand:
		turn, move = decideNextMove():
		#package = "1" + "\t" + str(random.randint(1,100)) + "\n"
		package = "2" + "\t" + str(turn) + "\t" + str(move) + "\n"
		#package = "2" + "\t" + str(defaultTurn) + "\t" + str(defaultMove) + "\n"
		#package = "2" + "\t" + str(1.5) + "\t" + str(800.0) + "\n"
		ser.write(package)
		print "Sent:",package.strip('\n')
		lastSent += sendFreq
		robotReadyForNewCommand = False	# set back to true when valid sensor data is received

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
		if distances[idx] == 0.0: continue
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
# FINDING CORNERS ####################
######################################

segmentLength = 1

# points is a list of (x,y) coordinates
# n is number of points a segment should go between (basically a smoother)
def segmentLines(points, n=1):
	angles = list()
	segmentsLeft = list()
	segmentsRight = list()
	for idx in range(len(points)):
		if idx < n or idx > (len(points)-n-1):
			segmentsLeft.append(None)
			segmentsRight.append(None)
			continue
		if points[idx] is None:
			segmentsLeft.append(None)
			segmentsRight.append(None)
			continue
		if points[idx-n] is None:
			segmentsLeft.append(None)
			continue
		if points[idx+n] is None:
			segmentsRight.append(None)
			continue
		
		x,y = points[idx]
		xLeft,yLeft = points[idx - n]
		xRight,yRight = points[idx + n]
		dyLeft = y - yLeft
		dyRight = yRight - y
		dxLeft = x - xLeft
		dxRight = xRight - x
		segmentsLeft.append((dxLeft,dyLeft))
		segmentsRight.append((dxRight,dyRight))
	return segmentsLeft, segmentsRight

def calculateIntersects(segmentsLeft,segmentsRight):
	angles = list()
	for idx in range(len(segmentsLeft)):
		#print segmentsLeft[idx]
		if segmentsLeft[idx] is None or segmentsRight[idx] is None:
			angles.append(None)
			continue
		dxLeft, dyLeft = segmentsLeft[idx]
		dxRight, dyRight = segmentsRight[idx]
		if dyLeft != 0 and dyRight != 0:
			angleLeft = dxLeft / dyLeft
			angleRight = dxRight / dyRight
			intersect = atan((angleLeft-angleRight)/(1+angleLeft*angleRight))
			angles.append(intersect)
		else:
			angles.append(None)
	return angles


	# CONSIDER CASE OF INSIDE CORNER

# estimated point at which there is (or may be) a corner
# 1 if there is a series of points with segments at particular angles
# 2 or if there is a loss in readings within one sweep
# 3 or if there is a large change in distance suddenly (is this basically the same as 1?)
# FUTURE
# include corner orientation
# 	return the 'heading' which is the way it points

# readingCountThreshold - how many missing readings required on one side to suggest a corner
def findCorner(points, angles, n=1, angleThreshold = 1.22, readingCountThreshold = 2):
	# for now just pick a threshold
	# and limit to a single result (the nearest)
	cornerFound = False
	cornerPosition = False
	cornerHeading = False
	maxValIdxPos = -1
	maxVal = 0
	# check if the largest angle change between adjacent segments is over the threshold
	# could move to separate function
	for idx in range(len(angles)):
		if angles[idx] is None: continue
		if abs(angles[idx]) > maxVal: 
			maxVal = angles[idx]
			maxValIdxPos = idx
	if maxValIdxPos >= 0 and maxVal >= angleThreshold:
		cornerFound = True
		cornerPosition = points[maxValIdxPos]

	# also check if the readings 'disappear' before the end, could be sign of a corner
	# could move to separate function
	if not(cornerFound):
		lookLeftRange = range(-readingCountThreshold,0)
		lookRightRange = range(1,readingCountThreshold+1)
		for idx in range(len(points)):
			if idx < readingCountThreshold or idx > (len(angles)-readingCountThreshold-1):
				continue	# if reading at beginning or end then can't do this check
			if points[idx] is None:
				continue	# if reading does not exist then no way to classify as corner
			# check if there are missing readings to the left
			leftMissingCount = 0
			for relativeIdx in lookLeftRange:
				if points[idx+relativeIdx] is None: 
					leftMissingCount += 1
			if leftMissingCount >= readingCountThreshold:
				cornerFound = True
				cornerPosition = points[idx]
				# add step to check if this is the closest candidate for a corner
			rightMissingCount = 0
			for relativeIdx in lookRightRange:
				if points[idx+relativeIdx] is None: 
					rightMissingCount += 1
			if rightMissingCount >= readingCountThreshold:
				cornerFound = True
				cornerPosition = points[idx]
				# add step to check if this is the closest candidate for a corner

	return cornerFound, cornerPosition, cornerHeading





######################################
# MAIN START #########################
######################################
print "Starting position:", globalPosition, globalHeading
while True:
	#key = getch()
	# exit program with 'q'
	#if key == 'q':
	#	break

	send()
	(result,input) = listenAndReceive(1)
	#print result, input
	
	if result:
		packetType = splitAndRoute(input)
		#print packetType
		if packetType == MOTION:
			print "Received motion data:",angleTurned, distanceMoved
			globalPosition, globalHeading = updatePositionAfterMovement(angleTurned, distanceMoved)
			positionHistory.append((globalPosition,globalHeading))
			print "New position:", globalPosition, globalHeading
		elif packetType == SENSOR:
			print "Received sensor data:",sensorReadings
			allSensorReadings.extend(sensorReadings)	# keep full list (this will need to be changed in future)
			#print len(allSensorReadings)
			x, y = convertReadingsForPlot(calculateSensorReadingPositions(readingAnglesRadians, sensorReadings))
			#plt.scatter(x, y)
			#plt.pause(0.001)
			robotReadyForNewCommand = True	# re-think where this goes. leaving here for now.
		elif packetType == CONTROL:
			print "Packet not yet defined. You shouldn't be here."
		else:
			print "Data receipt failure"






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
	print "Sending final stop command"
	ser.close()
	print "Closing connection"
except serial.SerialException:
	print "Connection lost"
	pass

print "Exiting program"
exit()