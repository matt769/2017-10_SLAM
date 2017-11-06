# add button to quit and close connection
# stop sending regularly (choose when to send)
# add multiple package types


from msvcrt import getch
import serial
import time

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

# for sending
lastSent = time.clock()
sendFreq = 1	# in seconds
def send():
	global lastSent
	if time.clock() - lastSent > sendFreq:
		ser.write('x')
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


while True:
	send()
	(result,input) = listenAndReceive(1)
	#print result, input
	if result:
		(result, data) = parseData(input)
	if result:
		print data


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