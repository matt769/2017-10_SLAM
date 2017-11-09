# add button to quit and close connection
# stop sending regularly (choose when to send)
# add multiple package types


from msvcrt import getch
import serial
import time
import random

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
angleTurned = 0
distanceMoved = 0
readingAngles = [10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170]
# modify parser to extract the motion information

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

def parseData(str):
	inputArray = str.split()
	inputArrayNum = []
	#print inputArray
	try:
		for i in inputArray:
			#print inputArray[i]
			#tmp = int(inputArray[i])
			#print tmp
			inputArrayNum.append(int(i))
	except:
		print "conversion failed"
		return False
	return True, inputArrayNum

# don't think this is the best solution but go with it for now
# prob change to return some data instead of modifying globals ??
def parseData2(str):
	global angleTurned
	global distanceMoved
	inputArray = str.split()
	inputArrayNum = []
	packetType = 0
	#print inputArray
	try:
		if int(inputArray[0]) == 1:	# motion packet
			packetType = 1
			angleTurned = float(inputArray[3])
			distanceMoved = float(inputArray[4])
		elif int(inputArray[0]) == 2:	# sensor packet
			packetType = 2
			for i in inputArray:
				#print inputArray[i]
				#tmp = int(inputArray[i])
				#print tmp
				inputArrayNum.append(int(i))
	except:
		print "conversion failed"
		return 0
	return packetType, inputArrayNum

def updatePositionAfterMovement(angleTurned, distanceMoved):
	globalHeading = (globalHeading + angleTurned) % 360
	globalPosition[0] = globalPosition[0] + distanceMoved * sin(globalHeading)	# CHECK THIS
	globalPosition[1] = globalPosition[1] + distanceMoved * cos(globalHeading)	# CHECK THIS


def calculateSensorReadingPositions(angles, distances):
	readingPositions = []
	for idx in distances:#
		# add sense check
		x = globalPosition[0] + distances[idx] * sin(angles[idx])	# CHECK THIS
		y = globalPosition[1] + distances[idx] * cos(angles[idx])	# CHECK THIS
		readingPositions.append((x,y))



globalPosition = (0.0, 0.0)
globalHeading = 0.0
angleTurned = 0
distanceMoved = 0





######################################
# MAIN START #########################
######################################
while True:
	send()
	(result,input) = listenAndReceive(1)
	#print result, input
	if result:
		(result, data) = parseData(input)
	if result > 0:
		print data

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