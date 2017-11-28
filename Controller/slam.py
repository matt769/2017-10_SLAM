import numpy as np

# in general, where you see the number 2, it relates to the number of dimensions in each states
# i.e. x and y
# index is x1, y1, x2, y2, Lx1, Ly1, Lx2, Ly2, L...
# list of landmarks may grow but will never hold more than 2 positions

# initial state
initialised = False
omega = None
xi = None
numberOfDims = 2 #	x and y	# I've been a bit inconsistent using this
numLandmarks = 0
idxOfFirstLandmark = 4

# parameters
motionNoise = 1.0
measurementNoise = 1.0


def expand(m, newDimRows, newDimCols, placeOldRows, placeOldCols = []):
	#print m
	dimRows, dimCols = m.shape
	if placeOldCols == []:
		placeOldCols = placeOldRows
	if len(placeOldRows) > dimRows or len(placeOldCols) > dimCols:
		raise ValueError, "list invalid in expand()"
	res = np.matrix(np.zeros((newDimRows, newDimCols)))
	#print placeOldRows, placeOldCols
	for i in range(len(placeOldRows)):
		for j in range(len(placeOldCols)):
			#print i,j, placeOldRows[i], placeOldCols[i]
			res[placeOldRows[i],placeOldCols[j]] = m[i,j]
	return res

# runs without errors but have not validated numeric value of output
def reduce(omega, xi, dims = 2):
	dimRows, dimCols = omega.shape
	omegaTmp = omega[dims:,dims:]	# takes all but the first dims number of rows/cols
	xiTmp = xi[dims:]	# takes all but the first dims number of rows
	A = omega[:dims,dims:]
	B = omega[:dims,:dims]
	C = xi[:dims]
	omega = omegaTmp - (A.T * B.I * A)
	xi = xiTmp - (A.T * B.I * C)
	return omega, xi

def initialiseSlam(initialPosition = (0,0), noise = 1.0):
	x, y = initialPosition
	omega = np.matrix(np.zeros((2,2)))
	xi = np.matrix(np.zeros((2,1)))
	# updates
	# x0 = x
	# y0 = y
	omega[0,0] += 1.0 / noise
	omega[1,1] += 1.0 / noise
	xi[0]      += x / noise
	xi[1]      += y / noise
	return omega, xi

	
def expandMotion(omega, xi, numberOfDims=numberOfDims):
	# expand omega and xi to hold the new motion data
	dimRows, dimCols = omega.shape
	newDimRows = dimRows + numberOfDims
	placeOldRows = range(numberOfDims) + range(numberOfDims*2,newDimRows)
	omega = expand(omega,newDimRows,newDimRows,placeOldRows,placeOldRows)
	xi = expand(xi,newDimRows,1,placeOldRows,[0])
	return omega, xi

def expandNewLandmark(omega, xi, numberOfDims=numberOfDims):
	# expand omega and xi to hold the new motion data
	dimRows, dimCols = omega.shape
	newDimRows = dimRows + numberOfDims
	placeOldRows = range(dimRows)
	omega = expand(omega,newDimRows,newDimRows,placeOldRows,placeOldRows)
	xi = expand(xi,newDimRows,1,placeOldRows,[0])
	return omega, xi

	
# remember this is relative motion
def motionUpdateSlam(omega, xi, motion, noise = motionNoise):
	# the motion will always be from x0 to x1
	dx, dy = motion
	# updates
	# + x0 - x1 = -dx
	# - x0 + x1 = +dx
	# + y0 - y1 = -dy
	# - y0 + y1 = +dy
	for idx in range(4):
		omega[idx,idx] += 1.0 / noise	# update the positive elements of x and y
	# not that if x0 at index 0, x1 will be at index 0 + 2
	for idx in range(2):
		omega[idx,idx+2] += -1.0 / noise # update the negative elements of x and y
		omega[idx+2,idx] += -1.0 / noise # update the negative elements of x and y
		xi[idx,0]        += -motion[idx] / noise
		xi[idx+2,0]      +=  motion[idx] / noise
	return omega, xi
	

def measurementUpdateSlam(omega, xi, measurement, noise = measurementNoise):
	# measurement will need to specify which landmark it relates to
	landmarkIdx, (x, y) = measurement
	m = (landmarkIdx*2) + 4	# account for the 4 rows at beginning of matrix/vector
	# m = landmarkIdx
	# updates
	# + L?x - x1 = +dx
	# + x1 - L?x = -dx
	# + L?y - y1 = +dy
	# + y1 - L?y = -dy
	for idx in range(2):	# index 0 will do the xs, index 1 will do the ys
		omega[2+idx,2+idx] +=  1.0 / noise	# updates diagonals for x1 and y1
		omega[m+idx,m+idx] +=  1.0 / noise	# updates diagonals for L?x and L?y
		omega[2+idx,m+idx] += -1.0 / noise	# updates non-diagonals for x1 and y1
		omega[m+idx,2+idx] += -1.0 / noise	# updates non-diagonals for L?x and L?y
		xi[2+idx,0]      += -measurement[1][idx] / noise
		xi[m+idx,0]      +=  measurement[1][idx] / noise
	return omega, xi

def calculateMu(omega, xi):
	mu = omega.I * xi
	return mu

# update to handle multiple landmarks in one step
def runSlam(motion,measurements):
	global initialised
	global omega
	global xi
	global numLandmarks
	if not(initialised):
		omega, xi = initialiseSlam()
		initialised = True
	omega, xi = expandMotion(omega, xi)
	omega, xi = motionUpdateSlam(omega, xi, motion)
	for measurement in measurements:
		landmarkIdx = measurement[0]
		while (landmarkIdx >= numLandmarks):
			omega, xi = expandNewLandmark(omega, xi)
			numLandmarks += 1
		omega, xi = measurementUpdateSlam(omega, xi, measurement)
	omega, xi = reduce(omega, xi)
	mu = calculateMu(omega, xi)
	return mu[:2], mu[2:]	# robot position, landmarks



# to run...
#mu = runSlam(motion, measurements)

