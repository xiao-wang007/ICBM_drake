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
	def __init__(self, stepSize, nStep, tabelFile, pandaFile, boxFile):
		self.builder = DiagramBuilder()
		self.nStep = nStep
		self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, time_step=stepSize)
		self.modelInstances = [parser.AddModels(table_file)[0], \
							   parser.AddModels(panda_file)[0], \
							   parser.AddModels(box_file)[0]]

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
		self.mutable_context_list = [self.plant.CreateDefaultContext() for i in range(self.nStep)]
		# AutoDiff
		self.ad_diagram = self.diagram.ToAutoDiffXd()
		self.ad_plant = self.ad_diagram.GetSubsystemByName("Push3D")
		self.ad_mutable_context_list = [self.ad_plant.CreateDefaultContext() for i in range(self.nStep)]

	def SetDevisionVariables(self, ):
		# HERE!!!!

	def SceneVisualizer(self, qs_init):
		pass

	def AutoDiffArrayEqual(self, a, b):
    	return np.array_equal(a, b) and np.array_equal(ExtractGradient(a), ExtractGradient(b))
