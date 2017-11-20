import math

######################################
# LANDMARK INFO ######################
######################################
	

# structure
# coordinate mean, coordinate variance
# ((xbar,ybar),(xvar,yvar))
# need to track how many times each landmark is seen
#	but if looking at a guassian, then how to stop them 'drifting'
# use first observation as static, and subsequent observations must fit in?
#	what if wrongly assigning a subsequent observation?

# I guess these inaccuracies are just a consequence of landmark 'resolution'

# when searching through the list of landmarks, would it be possible to cut down that list a bit
#	i.e. to possible points within some range of current estimated position

# how do I decide whether a landmark I've just see is the same as a landmark in my map
# for the moment, I will just apply some threshold on the euclidean distance between their means

def calcDistance(point1,point2):
	x1,y1 = point1
	x2,y2 = point2
	dist = math.sqrt((x1-x2)**2 + (y1-y2)**2)
	return dist

	
	
def checkIfLandmarkExists(pointToCheck, checkList, threshold = 1.0):
	minDist = threshold + 1
	exists = False
	landmark = None
	for checkPoint in checkList:
		dist = calcDistance(pointToCheck,checkPoint)
		if dist < minDist:
			minDist = dist
			minPoint = checkPoint
	if minDist <= threshold:
		exists = True
		landmark = minPoint
	return exists, landmark



def processFoundLandmark(pointToCheck):
	landmarkCountThreshold = 4
	# check if point is already on landmark list
	result1, position = checkIfLandmarkExists(pointToCheck,landmarksInUse)
	# update count (may not be required)
	if result1:
		landmarksInUse[position] = landmarksInUse[position] + 1
		print "landmark exists:", position
	else:
		result2, position = checkIfLandmarkExists(pointToCheck,landmarksPotential)
		if result2:
			print "potential landmark exists:", position
			landmarksPotential[position] = landmarksPotential[position] + 1
			if landmarksPotential[position] >= landmarkCountThreshold:
				landmarksInUse[position] = landmarksPotential.pop(position)	# add to landmark list
				print "added to main list"
		else:
			print "nothing found, add to potential list"
			landmarksPotential[pointToCheck] = 1
