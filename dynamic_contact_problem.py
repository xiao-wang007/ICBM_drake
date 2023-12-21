from pydrake.all import (
    AddMultibodyPlantSceneGraph, DiagramBuilder, 
    FindResourceOrThrow, GenerateHtml, InverseDynamicsController, 
    MultibodyPlant, Parser, Simulator)
from pydrake.multibody.jupyter_widgets import MakeJointSlidersThatPublishOnCallback
import pydrake
from pydrake import geometry
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix 
from pydrake.solvers import MathematicalProgram, Solve
from pydrake.systems.jupyter_widgets import PoseSliders, WidgetSystem
from ipywidgets import ToggleButton, ToggleButtons
from functools import partial
from pydrake.all import (
    JointIndex, PiecewisePolynomial, JacobianWrtVariable,
    eq, ge,  AutoDiffXd, SnoptSolver, IpoptSolver,  
    AddUnitQuaternionConstraintOnPlant, PositionConstraint, OrientationConstraint
    )
import numpy as np

"""
a function to return the scene with context and plant.
"""
#



"""
Contact data containers
"""
from collections import OrderedDict, namedtuple
ContactPairs = namedtuple("ContactPairs", "bodyA, bodyB, pts_in_A, pts_in_B, \
						   nhat_BA_W_list, v_tang_ACb_W_list, contact_wrenches_W")

conPairs["h-obj"] = ContactPairs("panda_rightfinger", \
                                 "base_link_cracker", \
                                 [np.array([[0.1],[0.1],[0.1]])], \
                                 [np.array([[0.2],[0.2],[0.2]])], \
                                 [np.array([[0],[0],[1]])], \
                                 [], \
                                 [])

conPairs["obj-env"] = ContactPairs("base_link_cracker", \
                                   "table_top_link", \
                                   [np.array([[0.1],[0.1],[0.1]])], \
                                   [np.array([[0.2],[0.2],[0.2]])], \
                                   [np.array([[1],[0],[1]])], \
                                   [], \
                                   [])

"""
init the prog and setting decision variable
"""
prog = MathematicalProgram()
T = 2.
N = 100
q = prog.NewContinuousVariables(nq, N, "positions")
v = prog.NewContinuousVariables(nv, N, "velocities")
u = prog.NewContinuousVariables(nu, N, "joint_torques")
lambdas = prog.NewContinuousVariables(nc*(nBasis+1+1), N, "contact_force_scalars") # 1 for fn, 1 for slack
decVars = np.vstack((q,v,u,lambdas))


"""
Constraint functions
"""

# 1. (joint space + task space) dynamics 
def generalized_dynamics_constraint(plant, contextList, decVars, contactPairs):
	# unpack the decision variables
	for ti in range(decVars.shape[1]):

"""
"A class for the case"
"""

class Push3D(object):
	def __init__(self, stepSize, nStep, tabelFile, pandaFile, boxFile, tipFile=None):
		self.builder = DiagramBuilder()
		self.N = nStep
		self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, time_step=stepSize)
		self.modelInstances = {"table_top" : parser.AddModels(tableFile)[0], \
							   "panda"     : parser.AddModels(pandaFile)[0], \
							   "box"       : parser.AddModels(boxFile)[0],
							   "tip"	   : parser.AddModels(tipFile)[0]}

	def SetTheScene(self, X_W_table, X_table_pandaBase):
		table_top_frame = self.plant.GetFrameByName("table_top_center")
		robot_base_frame = self.plant.GetFrameByName("panda_link0")
		box_base_frame = self.plant.GetFrameByName("base_link_cracker")
		self.plant.WeldFrames(self.plant.world_frame(), table_top_frame, X_W_table)
		self.plant.WeldFrames(table_top_frame, robot_base_frame, X_table_pandaBase)
		self.plant.Finalize()
		self.plant.set_name("Push3D")

		# Two versions: float
		self.diagram = self.builder.Build()
		self.mutable_context_list = [self.plant.CreateDefaultContext() for i in range(self.N)]
		# AutoDiff
		self.ad_diagram = self.diagram.ToAutoDiffXd()
		self.ad_plant = self.ad_diagram.GetSubsystemByName("Push3D")
		self.ad_mutable_context_list = [self.ad_plant.CreateDefaultContext() for i in range(self.N)]

	def SetDevisionVariables(self, case, nContact, nBasis):
		self.prog = MathematicalProgram()
		self.nDecVar_ti = nq + nv + nu + nContact*(nBasis+1+1) # the offset for indexing
		nDecVar = (nq + nv + nu + nContact*(nBasis+1+1))*self.N

		if case == "jointspace":
			# -2 is to offset the two fixed panda fingers
			nq = self.plant.num_positions(self.modelInstances["panda"])	- 2 + \
				 self.plant.num_positions(self.modelInstances["box"])
			nv = self.plant.num_velocities(self.modelInstances["panda"]) - 2 + \
				 self.plant.num_velocities(self.modelInstances["box"])
			nu = self.plant.num_actuators(self.modelInstances["panda"])

			# notice here, decVars is 1D vector
			self.decVars = self.plant.NewContinuousVariables(nDecVar, "decision_variable_set")

		elif case == "taskspace":
			nq = self.plant.num_positions(self.modelInstances["tip"])	+ \
				 self.plant.num_positions(self.modelInstances["box"])
			nv = self.plant.num_velocities(self.modelInstances["tip"]) + \
				 self.plant.num_velocities(self.modelInstances["box"])
			nu = self.plant.num_actuators(self.modelInstances["tip"])

			self.qVars = self.prog.NewContinuousVariables(nq, self.N, "positions")
			self.vVars = self.prog.NewContinuousVariables(nv, self.N, "velocities")
			self.uVars = self.prog.NewContinuousVariables(nu, self.N, "actuations")

			self.decVars = self.plant.NewContinuousVariables(nDecVar, "decision_variable_set")

	def SetBoundaryConditions(self, decVars_t0, decVars_tf=None):
		self.decVars[0:self.ndecVars_ti] = decVars_t0

		if decVars_tf is not None:
			self.decVars[-self.nDecVar_ti: ] = decVars_tf

	def SceneVisualizer(self, context_index=None):
		# this is intended for visualization of the trajectory
		# as well as just visualize the scene at a time point
		pass

	def SetConstraints(self, cst_list):
		"""
		"Note! The solver side expects a flattened vector, not 2D decVars"
		
		:param      cst_list:  The cst list
		:type       cst_list:  { type_description }
		"""
		for cst in cst_list:
			self.prog.AddConstraint(cst, lb=cst.lb, ub=cst.ub, vars=self.decVars[cst.indices])

	def SetObjective(self, indices):
		"""
		"Assuming we are using Drake's .solvers.Cost()"
		
		:param      objFunction:  The object function
		:type       objFunction:  { type_description }
		"""
		self.prog.AddCost(objFunction, vars=self.decVars[indices])

	def Solve(self, decVar_init=None):
		if decVar_init is not None:
			Solve(self.prog, initial_guess=initial_init)
		else:
			pass # zeros init

	def AutoDiffArrayEqual(self, a, b):
    	return np.array_equal(a, b) and np.array_equal(ExtractGradient(a), ExtractGradient(b))
