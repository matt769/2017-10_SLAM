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
readingAnglesDegrees = [10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170]
readingAnglesRadians = [x*math.pi/180 for x in readingAnglesDegrees]
sensorReadings = [0 for x in range(len(readingAnglesDegrees))]

#package types
MOTION = 1
SENSOR = 2
CONTROL = 3

######################################
# FUNCTIONS# #########################
######################################

# for sending
lastSent = time.clock()
sendFreq = 5	# in seconds
def send():
	global lastSent
	if time.clock() - lastSent > sendFreq:
		#package = "1" + "\t" + str(random.randint(1,100)) + "\n"
		package = "2" + "\t" + str(random.randint(-100,100)) + "\t" + str(random.randint(1,100)) + "\n"
		ser.write(package)
		print "Sent:",package.strip('\n')
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
	heading = (globalHeading + angleTurned) % 360
	x = globalPosition[0] + distanceMoved * math.sin(globalHeading)
	y = globalPosition[1] + distanceMoved * math.cos(globalHeading)
	return x, y, heading


def calculateSensorReadingPositions(angles, distances):
	readingPositions = []
	for idx in distances:#
		# add sense check
		x = globalPosition[0] + distances[idx] * math.sin(angles[idx])
		y = globalPosition[1] + distances[idx] * math.cos(angles[idx])
		readingPositions.append((x,y))
	return readingPositions

def updatePositionAfterSense():
	# this will be one of the final functions to implement
	return False


######################################
# MAIN START #########################
######################################
while True:
	send()
	(result,input) = listenAndReceive(1)
	#print result, input
	if result:
		packetType = splitAndRoute(input)
		#print packetType
		if packetType == MOTION:
			print "Received motion data:",angleTurned, distanceMoved
		elif packetType == SENSOR:
			print "Received sensor data:",sensorReadings
		elif packetType == CONTROL:
			print "Packet not yet defined. YOu shouldn't be here."
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