import math

######################################
# FINDING CORNERS ####################
######################################

segmentLength = 5
angleThreshold = 1.22	# radians
missingReadingCountThreshold = 3	#how many missing readings required on one side to suggest a corner
readingChangeThreshold = 100	# millimeters # somewhat based on sensor noise and scan resolution

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
			intersect = math.atan((angleLeft-angleRight)/(1+angleLeft*angleRight))
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

# missingThreshold - how many missing readings required on one side to suggest a corner
def findCorner(points, angles, rawReadings, n=1, angleThreshold = angleThreshold, missingThreshold = missingReadingCountThreshold, changeThreshold = readingChangeThreshold):
	# for now just pick a threshold
	# and limit to a single result (the nearest)
	cornerFound = False
	cornerPosition = False
	cornerPositions = list()
	cornerHeading = False
	maxValIdxPos = -1
	maxVal = 0
	# check if each angle change between adjacent segments is over the threshold
	# if so, add to list of corners
	# could move to separate function
	for idx in range(len(angles)):
		if angles[idx] is None: continue
		if abs(angles[idx]) > angleThreshold: 
			cornerPosition = points[idx]
			cornerPositions.append(cornerPosition)

	# also check if the readings 'disappear' before the end, could be sign of a corner
	# could move to separate function
	lookLeftRange = range(-missingThreshold,0)
	lookRightRange = range(1,missingThreshold+1)
	for idx in range(len(points)):
		if idx < missingThreshold or idx > (len(angles)-missingThreshold-1):
			continue	# if reading at beginning or end then can't do this check
		if points[idx] is None:
			continue	# if reading does not exist then no way to classify as corner
		# check if there are missing readings to the left
		leftMissingCount = 0
		for relativeIdx in lookLeftRange:
			if points[idx+relativeIdx] is None: 
				leftMissingCount += 1
		if leftMissingCount >= missingThreshold:
			cornerPosition = points[idx]
			cornerPositions.append(cornerPosition)
		# check if there are missing readings to the right
		rightMissingCount = 0
		for relativeIdx in lookRightRange:
			if points[idx+relativeIdx] is None: 
				rightMissingCount += 1
		if rightMissingCount >= missingThreshold:
			cornerPosition = points[idx]
			cornerPositions.append(cornerPosition)

	# also check for 'spike' landmarks
	# don't really need the coordinates, just raw readings

	for idx in range(len(points)):
		if idx < 1 or idx > (len(points)-1-1): continue	# if reading at beginning or end then can't do this check
		a = rawReadings[idx-1]
		b = rawReadings[idx]
		c = rawReadings[idx+1]
		if a == 0 or b == 0 or c == 0: continue
		
		# have included a check that the signs are the same, otherwise could be straight line at close to 90deg angle to sensor
		if abs(a-b) > changeThreshold and abs(c-b) > changeThreshold and (a<b)==(c<b):
			cornerPosition = points[idx]
			cornerPositions.append(cornerPosition)
		
		
	if len(cornerPositions)>0: cornerFound = True
	return cornerFound, cornerPositions

def getCorners(readingPositions,rawReadings):
	sl,sr = segmentLines(readingPositions, segmentLength)
	a = calculateIntersects(sl,sr)
	cornerFound, cornerPositions = findCorner(readingPositions, a, rawReadings)
	return cornerFound, cornerPositions