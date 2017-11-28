import math

######################################
# LANDMARK INFO ######################
######################################

# need to track how many times each landmark is seen
#	but if looking at a guassian, then how to stop them 'drifting'
# use first observation as static, and subsequent observations must fit in?
#	what if wrongly assigning a subsequent observation?

# I guess these inaccuracies are just a consequence of landmark 'resolution'

# when searching through the list of landmarks, would it be possible to cut down that list a bit
#	i.e. to possible points within some range of current estimated position

# how do I decide whether a landmark I've just see is the same as a landmark in my map
# for the moment, I will just apply some threshold on the euclidean distance between their means

#consider situation where I have multiple landmarks in the latest scan
# only one of them can be matched to an existing landmark


landmarkCountThreshold = 3
landmarkDistanceThreshold = 50.0

#landmarksInUse = dict()		# index = (position, observedCount)
#landmarksPotential = dict	# index = (position, observedCount)
#landmarksFalse = dict()	# prevent any landmarks from being found in these areas # not sure if needed


def calcDistance(point1,point2):
	x1,y1 = point1
	x2,y2 = point2
	dist = math.sqrt((x1-x2)**2 + (y1-y2)**2)
	return dist


def checkIfLandmarkExists(pointToCheck, checkList, threshold = landmarkDistanceThreshold):
	minDist = threshold + 1
	exists = False
	landmarkIdx = None
	for idx in range(len(checkList)):
		checkListPoint = checkList[idx][0]
		dist = calcDistance(pointToCheck,checkListPoint)
		if dist < minDist:
			minDist = dist
			minPointIdx = idx
	if minDist <= threshold:
		exists = True
		landmarkIdx = minPointIdx
	return exists, landmarkIdx


def processSensedLandmark(landmarksInUse, landmarksPotential, pointToCheck, pointIdx):
	landmarkIdx = None
	# check if point is already on landmark list
	existsInUse, landmarkIdx = checkIfLandmarkExists(pointToCheck,landmarksInUse)
	# update count (may not be required)
	if existsInUse:
		position, count = landmarksInUse[landmarkIdx][0],landmarksInUse[landmarkIdx][1]
		landmarksInUse[landmarkIdx] = position, count+1
		#print "landmark exists:", position, count
	else:
		existsPotential, positionIdx = checkIfLandmarkExists(pointToCheck,landmarksPotential)
		if existsPotential:
			position, count = landmarksPotential[positionIdx][0],landmarksPotential[positionIdx][1]
			#print "potential landmark exists:", position, count
			landmarksPotential[positionIdx] = position, count+1
			if count+1 >= landmarkCountThreshold:
				landmarksInUse.append(landmarksPotential.pop(positionIdx))	# add to landmark list
				landmarkIdx = len(landmarksInUse)-1	# the last index in the list
				#print "added to main list"
		else:
			#print "nothing found, add to potential list"
			position = pointToCheck
			count = 1
			landmarksPotential.append((pointToCheck,count))
	return landmarkIdx

# need to keep the idx as this will be used to tie the landmark back to a specific sensor reading
def processAllLandmarks(landmarksInUse, landmarksPotential, allPoints, pointIdxs):
	results = list()
	for idx in range(len(allPoints)):
		#print allPoints[idx], pointIdxs[idx]
		landmarkIdx = processSensedLandmark(landmarksInUse, landmarksPotential, allPoints[idx], pointIdxs[idx])
		if landmarkIdx is not None:
			results.append((pointIdxs[idx], landmarkIdx))	# sensor index, and landmark index
	return results
