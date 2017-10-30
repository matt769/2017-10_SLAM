
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
	inputArray = str.split('\t')
	try:
		tm = int(inputArray[0])
		inc = int(inputArray[1])
	except:
		print "conversion failed"
		return False
	return True, tm, inc


while True:
	send()
	(result,input) = listenAndReceive(1)
	#print result, input
	if result:
		(result, tm, inc) = parseData(input)
	if result:
		print tm, inc


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