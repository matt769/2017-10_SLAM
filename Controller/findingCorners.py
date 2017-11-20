import math

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
