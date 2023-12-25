class SlidingContactConstraintLCP(object):
	def __init__(self, plant, context, vars, conPairsDict, nBasis):
		self.plant = plant
		self.vars = vars 
		self.context = context
		self.conPairsDict = conPairsDict
		self.nBasis = nBasis
		nContact = self.GetnContact(self.conPairsDict)
    	q, v, lambdasAndSlack = np.split(vars, [self.nq, self.nq+self.nv])

    	# split the contact vars for each contact
    	markPoints = [(nBasis+1+1)*i for i in range(1, nContact)]
    	self.lambdasAndSlack_set = [np.split(lambdasAndSlack, markPoints)]

		# the components needed for LCP
		self.signedDistance_set = [] # phi(q)
    	self.frictionCone = [] # miu*fn - sum(all lambdas on basis)
    	self.v_tang_ACb_W_set = []

    	""" the cone basis for each contact
    		from list to numpy, the shape will be [nc, nbasis, 3] """
    	self.cone_set = [] # this is [ [[x,y,z]_1,...,[]_6]_1, []_2, ..., []_nc]

    	# call the computation
    	self.ComputeComponents()

    def GetnContact(self, conPairsDict):
		nContact = 0
		for key in conPairsDict:
			nContact += len(conPairsDict[key].pts_in_A)
		return nContact

	def ComputeComponents(self):
    	"""
    	It is important to note the relative indexing of the complementarity and dynamical constraints. 
    	Over the interval [tk,tk+1], the contact impulse can be non-zero if and only if φ(qk+1) = 0; 
    	that is, the bodies must be in contact at the end of the given interval.
    	"""
    	if isinstance(vars[0], AutoDiffXd):
			if not autoDiffArrayEqual(v, self.plant.GetVelocities(context)):
				self.plant.SetVelocities(self.context, v)
			if not autoDiffArrayEqual(q, self.plant.GetPositions(context)):
				self.plant.SetPositions(self.context, q)

    	# signed distance between hand and object
    	counter = 0

    	# compute tangential velocities and cone basis for each contact
    	for key, value in self.conPairDict:
			frameA = self.plant.GetFrameByName(value.bodyA)
			X_WA = frameA.CalcPoseInWorld(self.context)
			X_WB = frameB.CalcPoseInWorld(self.context)
			for i, (ptA, ptB) in enumerate(value.pts_in_A, value.pts_in_B):
				pA_in_W = X_WA @ value.pts_in_A[i]
				pB_in_W = X_WB @ value.pts_in_B[i]
				self.signedDistance_set.append(pA_in_W - pB_in_W)

				# frictional cone, eq(31)
				self.frictionCone.append(value.miu*self.lambdasAndSlack_set[counter][0] - \
								    (np.sum(self.lambdasAndSlack_set[counter][1:-1])))

				# onto eq(15-16) in posa paper, relative tangential velocity
				""" This computation of the relative tangential velocity is a repetition! """
				self.v_tang_ACb_W_i = self.ComputeContactTangentialVelocity( \
																		  value.bodyA, \
																		  value.bodyB, \
																		  pt_in_B, \
																		  value.nhat_BA_W)
				v_tang_ACb_W_set.append(v_tang_ACb_W_i)
				cone_set.append(self.ComputeConeBasis(v_tang_ACb_W_i))
				counter += 1

		counter = 0

	def ComputeContactTangentialVelocity(self, bodyA, bodyB, ptB, nhat_BA_W):
		frameA = self.plant.GetFrameByName(bodyA)
		frameB = self.plant.GetFrameByName(bodyB)
		V_AB_W = frameB.CalcSpatialVelocity(self.context, \
                                                    frameA, \
                                                    self.plant.world_frame())
		R_w_B = self.plant.CalcRelativeTransform(self.context, \
                                                    self.plant.world_frame(), \
                                                    frameB).rotation() # DCM from body B pose in {W}
		p_BCb_W = R_w_B @ ptB # the contact points in {B} expressed in {W}
		v_ACb_W = V_AB_W.Shift(p_BCb_W).translational() # compute the contact point's velocity in {A}
        v_tang_ACb_W = v_ACb_W - np.dot(v_ACb_W, nhat_BA_W) * nhat_BA_W
        return v_tang_ACb_W

	def ComputeConeBasis(self, v_tang_ACb_W):
		vhat_tang_ACb_W = v_tang_ACb_W/np.linalg.norm(v_tang_ACb_W)
		wrench = np.zeros((6, 1))
		angles = np.linspace(0, 360, lambdas.size-1) # exclude the lambdaN
		basis = []
        for i, ang in enumerate(angles):
            rot = np.array([[np.cos(ang), -np.sin(ang)],
                            [np.sin(ang),  np.cos(ang)]])
            basis.append(rot @ vhat_tang_ACb_W)
        return

    def MiuVecForCone(self):
    	mius = []
    	for key, value in self.conPairDict:
    		for i in range(len(value.pts_in_A)):
    			mius.append(value.miu)
    	return np.array(miu)

    def LCPEqualityConstraint(self):
    	# Eq(13): phi(q) dot lambda = 0
    	lambdaNs = np.array([self.lambdasAndSlack_set[i][0] for i in range(len(self.lambdasAndSlack_set))])
    	eq_Cst1 = np.dot(np.array(self.signedDistance_set), lambdaNs)

    	# Eq(33): cone constraint
    	# mius = self.MiuVecForCone()
    	slacks = np.array([self.lambdasAndSlack_set[i][-1] for i in range(len(self.lambdasAndSlack_set))])
    	# tangLambdasSums = np.array([np.sum(self.lambdasAndSlack_set[i][1:-1]) for i in range(len(self.lambdasAndSlack_set))])
    	# eq_Cst2 = np.dot(mius * lambdaNs - tangLambdasSums, \
    	# 				slacks)
    	eq_Cst2 = np.array(self.frictionCone) * slacks

    	# Eq(34): sliding
    	reshaped = self.cone_set.reshape(self.cone_set.shape[0]*self.cone_set.shape[1], self.cone_set.shape[2])
    	using np.einsum to vectorize this calculation!!!!

