import numpy as np

# in general, where you see the number 2, it relates to the number of dimensions in each states
# i.e. x and y
# index is x1, y1, x2, y2, Lx1, Ly1, Lx2, Ly2, L...
# list of landmarks may grow but there will never be more than 2 robot positions held

class Slam():

	dims = 2

	def __init__(self, motion_noise, sensor_noise):
		self.motionNoise = motion_noise
		self.measurementNoise = sensor_noise
		self.initialised = False
		self.omega = None
		self.xi = None
		self.mu = None
		self.numLandmarks = 0
		self.idxOfFirstLandmark = 4
		self.initialiseMatrices()
	
	def initialiseMatrices(self):
		x, y = (0,0)
		self.omega = np.matrix(np.zeros((2,2)))
		self.xi = np.matrix(np.zeros((2,1)))
		# updates
		# x0 = x
		# y0 = y
		self.omega[0,0] += 1.0 / 1.0
		self.omega[1,1] += 1.0 / 1.0
		self.xi[0]      += x / 1.0
		self.xi[1]      += y / 1.0
		return

	@staticmethod
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

	def reduce(self):
		dimRows, dimCols = self.omega.shape
		self.omegaTmp = self.omega[self.dims:,self.dims:]	# takes all but the first dims number of rows/cols
		self.xiTmp = self.xi[self.dims:]	# takes all but the first dims number of rows
		A = self.omega[:self.dims,self.dims:]
		B = self.omega[:self.dims,:self.dims]
		C = self.xi[:self.dims]
		self.omega = self.omegaTmp - (A.T * B.I * A)
		self.xi = self.xiTmp - (A.T * B.I * C)
		return

	def expandMotion(self):
		# expand self.omega and self.xi to hold the new motion data
		dimRows, dimCols = self.omega.shape
		newDimRows = dimRows + self.dims
		placeOldRows = range(self.dims) + range(self.dims*2,newDimRows)
		self.omega = self.expand(self.omega,newDimRows,newDimRows,placeOldRows,placeOldRows)
		self.xi = self.expand(self.xi,newDimRows,1,placeOldRows,[0])
		return

	def expandNewLandmark(self):
		# expand self.omega and self.xi to hold the new motion data
		dimRows, dimCols = self.omega.shape
		newDimRows = dimRows + self.dims
		placeOldRows = range(dimRows)
		self.omega = self.expand(self.omega,newDimRows,newDimRows,placeOldRows,placeOldRows)
		self.xi = self.expand(self.xi,newDimRows,1,placeOldRows,[0])
		return

		
	# remember this is relative motion
	def motionUpdate(self, motion):
		# the motion will always be from x0 to x1
		dx, dy = motion
		# updates
		# + x0 - x1 = -dx
		# - x0 + x1 = +dx
		# + y0 - y1 = -dy
		# - y0 + y1 = +dy
		for idx in range(4):
			self.omega[idx,idx] += 1.0 / self.motionNoise	# update the positive elements of x and y
		# not that if x0 at index 0, x1 will be at index 0 + 2
		for idx in range(2):
			self.omega[idx,idx+2] += -1.0 / self.motionNoise # update the negative elements of x and y
			self.omega[idx+2,idx] += -1.0 / self.motionNoise # update the negative elements of x and y
			self.xi[idx,0]        += -motion[idx] / self.motionNoise
			self.xi[idx+2,0]      +=  motion[idx] / self.motionNoise
		return
		

	def measurementUpdate(self, measurement):
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
			self.omega[2+idx,2+idx] +=  1.0 / self.measurementNoise	# updates diagonals for x1 and y1
			self.omega[m+idx,m+idx] +=  1.0 / self.measurementNoise	# updates diagonals for L?x and L?y
			self.omega[2+idx,m+idx] += -1.0 / self.measurementNoise	# updates non-diagonals for x1 and y1
			self.omega[m+idx,2+idx] += -1.0 / self.measurementNoise	# updates non-diagonals for L?x and L?y
			self.xi[2+idx,0]      += -measurement[1][idx] / self.measurementNoise
			self.xi[m+idx,0]      +=  measurement[1][idx] / self.measurementNoise
		return

	def calculateMu(self):
		self.mu = self.omega.I * self.xi
		return

	# update to handle multiple landmarks in one step
	def run(self, motion,measurements):
		self.expandMotion()
		self.motionUpdate(motion)
		for measurement in measurements:
			landmarkIdx = measurement[0]
			while (landmarkIdx >= self.numLandmarks):
				self.expandNewLandmark()
				self.numLandmarks += 1
			self.measurementUpdate(measurement)
		self.reduce()
		self.calculateMu()
		return self.mu[:2], self.mu[2:]	# robot position, landmarks

# add method to return data in more convenient form than numpy matrix

