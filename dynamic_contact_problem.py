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
from functools import partial

"""
a function to return the scene with context and plant.
"""
#



"""
Contact data containers

Need to use a class to contain the contact pairs.

Dict as data holder 
Namedtuple as dict values
class methods to get some utilities.

"""


from collections import OrderedDict, namedtuple
ContactPairs = namedtuple("ContactPairs", "bodyA, bodyB, pts_in_A, pts_in_B, \
						   nhat_BA_W_list, v_tang_ACb_W_list")

conPairs["h-obj"] = ContactPairs("panda_rightfinger", \
                                 "base_link_cracker", \
                                 [np.array([[0.1],[0.1],[0.1]])], \
                                 [np.array([[0.2],[0.2],[0.2]])], \
                                 [np.array([[0],[0],[1]])], \
                                 [])

conPairs["obj-env"] = ContactPairs("base_link_cracker", \
                                   "table_top_link", \
                                   [np.array([[0.1],[0.1],[0.1]])], \
                                   [np.array([[0.2],[0.2],[0.2]])], \
                                   [np.array([[1],[0],[1]])], \
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
Some utility functions
"""
def AutoDiffArrayEqual(self, a, b):
	return np.array_equal(a, b) and np.array_equal(ExtractGradient(a), ExtractGradient(b))

"""
"A class for the case"
"""

class Push3D(object):
	def __init__(self, T, nStep, tabelFile, \
				 pandaFile, boxFile, tipFile=None, \
				 X_W_table=None, X_table_pandaBase=None):
		self.builder = DiagramBuilder()
		self.N = nStep
		self.h = T/nStep
		self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, time_step=self.h)
		self.modelInstances = {"table_top" : parser.AddModels(tableFile)[0], \
							   "panda"     : parser.AddModels(pandaFile)[0], \
							   "box"       : parser.AddModels(boxFile)[0],
							   "tip"	   : parser.AddModels(tipFile)[0]}
		self.SetTheScene(X_W_table, X_table_pandaBase)

	def SetTheScene(self, X_W_table, X_table_pandaBase):
		table_top_frame = self.plant.GetFrameByName("table_top_center")
		robot_base_frame = self.plant.GetFrameByName("panda_link0")
		box_base_frame = self.plant.GetFrameByName("base_link_cracker")

		if X_W_table is not None:
			self.plant.WeldFrames(self.plant.world_frame(), table_top_frame, X_W_table)
		if X_table_pandaBase is not None:
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
		self.nContactParams = nContact * (nBasis + 1 + 1)

		if case == "jointspace":
			# -2 is to offset the two fixed panda fingers
			self.nq = self.plant.num_positions(self.modelInstances["panda"])	- 2 + \
				 self.plant.num_positions(self.modelInstances["box"])
			self.nv = self.plant.num_velocities(self.modelInstances["panda"]) - 2 + \
				 self.plant.num_velocities(self.modelInstances["box"])
			self.nu = self.plant.num_actuators(self.modelInstances["panda"])

			self.qVars = self.prog.NewContinuousVariables(self.nq, self.N, "positions")
			self.vVars = self.prog.NewContinuousVariables(self.nv, self.N, "velocities")
			self.uVars = self.prog.NewContinuousVariables(self.nu, self.N, "actuations")
			self.contactVars = self.prog.NewContinuousVariables(self.nContactParams, N, "lambdas&slack")
			# 1 is for fn, 1 is for slack

		elif case == "taskspace":
			self.nq = self.plant.num_positions(self.modelInstances["tip"])	+ \
				 	  self.plant.num_positions(self.modelInstances["box"])
			self.nv = self.plant.num_velocities(self.modelInstances["tip"])  + \
				 	  self.plant.num_velocities(self.modelInstances["box"])
			self.nu = self.plant.num_actuators(self.modelInstances["tip"])

			self.qVars = self.prog.NewContinuousVariables(self.nq, self.N, "positions")
			self.vVars = self.prog.NewContinuousVariables(self.nv, self.N, "velocities")
			self.uVars = self.prog.NewContinuousVariables(self.nu, self.N, "actuations")
			self.contactVars = self.prog.NewContinuousVariables(self.nContactParams, N, "lambdas&slack")

	def SetBoundaryConditions(self, decVars_t0, decVars_tf=None):
		"""
		Sets the boundary conditions.
		
		:param      decVars_t0:  initial condition 
		:type       decVars_t0:  { 1D array}
		:param      decVars_tf:  final condition
		:type       decVars_tf:  { 1D array }
		"""

		self.qVars[:, 0], self.vVars[:, 0], self.uVars[:, 0], self.contactVars[:, 0] = \ 
			np.split(decVars_t0, [self.nq, self.nq+self.nv, self.nq+self.nv+self.nContactParams])

		if decVars_tf is not None:
			self.qVars[:, -1], self.vVars[:, -1], self.uVars[:, -1], self.contactVars[:, -1] = \ 
			np.split(decVars_tf, [self.nq, self.nq+self.nv, self.nq+self.nv+self.nContactParams])

	def SceneVisualizer(self, context_index=None):
		# this is intended for visualization of the trajectory
		# as well as just visualize the scene at a time point
		pass

	def SetSystemDynamicConstraint(self, conPairsDict):
		"""
		"Note! The solver side expects a flattened vector, not 2D decVars"
		"""
		# adding defect constraints 
		for i in range(self.N-1):
			self.prog.AddConstraint(partial(self.SystemDynamicConstraint, \
								    context_index=i, conPairsDict=conPairsDict), \
									lb=[0]*self.nv, \
									ub=[0]*self.nv,\
									vars=np.concatenate(self.vVars[:, i], \
														self.qVars[:, i+1],
														self.vVars[:, i+1],
														self.uVars[:, i+1],
														self.contactVars[:, i+1]))

	def SetObjective(self):
		pass

	def Solve(self, decVar_init=None):
		if decVar_init is not None:
			Solve(self.prog, initial_guess=initial_init)
		else:
			pass # zeros init

    def SystemDynamicConstraint(self, vars, context_index, conPairsDict):
    	v, q_next, v_next, u_next, lambdasAndSlack_next = np.split(vars, \
    					[self.nv, self.nv+self.nq, self.nq+self.nv+self.nv, self.nq+self.nv+self.nv+self.nu])
    	if isinstance(vars[0], AutoDiffXd):
			if not autoDiffArrayEqual(v, self.ad_plant.GetVelocities(self.mutable_context_list[context_index])):
				self.ad_plant.SetVelocities(self.mutable_context_list[context_index], v)
			if not autoDiffArrayEqual(q_next, self.ad_plant.GetPositions(self.mutable_context_list[context_index+1])):
				self.ad_plant.SetPositions(self.mutable_context_list[context_index+1], q_next)
			if not autoDiffArrayEqual(v_next, self.ad_plant.GetVelocities(self.mutable_context_list[context_index+1])):
				self.ad_plant.SetVelocities(self.mutable_context_list[context_index+1], v_next)

			B = self.ad_plant.MakeActuationMatrix()
			y = B @ u_next

			G_next = self.ad_plant.CalcGravityGeneralizedForces(self.mutable_context_list[context_index+1])
			y += G_next

			C_next = self.ad_plant.CalcBiasTerm(self.mutable_context_list[context_index+1])
			y -= C_next

			M_next = self.ad_plant.CalcMassMatrixViaInverseDynamics(self.mutable_context_list[context_index+1])

			# now loop to get contact Jacobians
			for key, value in conPairDict:
				if key is not "obj-env":
					for i, (pt_in_A, pt_in_B) in zip(value.pts_in_A, value.pts_in_B):
						Jv_V_W_Ca = self.ad_plant.CalcJacobianSpatialVelocity( \
													  self.mutable_context_list[context_index], 
                                                      JacobianWrtVariable.kV, 
                                                      self.ad_plant.GetFrameByName(value.bodyA), 
                                                      p_in_A, 
                                                      eval_plant.world_frame(), 
                                                      eval_plant.world_frame())
						Jv_V_W_Cb = self.ad_plant.CalcJacobianSpatialVelocity( \
													  self.mutable_context_list[context_index], 
                                                      JacobianWrtVariable.kV, 
                                                      self.ad_plant.GetFrameByName(value.bodyB), 
                                                      p_in_B, 
                                                      eval_plant.world_frame(), 
                                                      eval_plant.world_frame())
						v_tang_ACb_W = self.ComputeContactTangentialVelocity(context_index, \
																			 value.bodyA, \
																			 value.bodyB, \
																			 pt_in_B, \
																			 value.nhat_BA_W)
						F_AB_W = self.ContactWrenchEvaluator(lambdasAndSlack_next[:-1], \
															 value.nhat_BA_W, v_tang_ACb_W)
						
						y += Jv_V_W_Ca @ F_AB_W
						y += Jv_V_W_Cb @ (-F_AB_W)
				else:
					for i, (pt_in_A, pt_in_B) in zip(value.pts_in_A, value.pts_in_B):
						# only wrenches on the object is needed as table is fixed.
						Jv_V_W_Ca = self.ad_plant.CalcJacobianSpatialVelocity( \
													  self.mutable_context_list[context_index], 
                                                      JacobianWrtVariable.kV, 
                                                      self.ad_plant.GetFrameByName(value.bodyA), 
                                                      p_in_A, 
                                                      eval_plant.world_frame(), 
                                                      eval_plant.world_frame())

						v_tang_ACb_W = self.ComputeContactTangentialVelocity(context_index, \
																			 value.bodyA, \
																			 value.bodyB, \
																			 pt_in_B, \
																			 value.nhat_BA_W)
						F_AB_W = self.ContactWrenchEvaluator(lambdasAndSlack_next[:-1], \
															 value.nhat_BA_W, v_tang_ACb_W)

						y += Jv_V_W_Ca @ F_AB_W # the wrench of A applied to B in expressed in {W}

			y = y * self.h - M_next @ (v_next - v)
		
		else:
			# here goes a version for float datatype
			pass

	def ContactWrenchEvaluator(self, lambdas, nhat_BA_W, v_tang_ACb_W):
		# get the tangential velocity unit vector
		vhat_tang_ACb_W = v_tang_ACb_W/np.linalg.norm(v_tang_ACb_W)
		wrench = np.zeros((6, 1))
		angles = np.linspace(0, 360, lambdas.size-1) # exclude the lambdaN
        for i, ang in enumerate(angles):
            rot = np.array([[np.cos(ang), -np.sin(ang)],
                            [np.sin(ang),  np.cos(ang)]])
            wrench[3:] += (rot @ vhat_tang_ACb_W) * lambdas[i+1]

        wrench[3:] += lambdas[0] * nhat_BA_W  # contact normal
		return wrench

	def ComputeContactTangentialVelocity(self, context_index, bodyA, bodyB, ptB, nhat_BA_W):
		frameA = self.ad_plant.GetFrameByName(bodyA)
		frameB = self.ad_plant.GetFrameByName(bodyB)
		V_AB_W = frameB.CalcSpatialVelocity(self.mutable_context_list[context_index], \
                                                    frameA, \
                                                    plant.world_frame())
		R_w_B = self.ad_plant.CalcRelativeTransform(self.mutable_context_list[context_index], \
                                                    plant.world_frame(), \
                                                    frameB).rotation() # DCM from body B pose in {W}
		p_BCb_W = R_w_B @ ptB # the contact points in {B} expressed in {W}
		v_ACb_W = V_AB_w.shift(p_BCb_W).translational() # compute the contact point's velocity in {A}
        v_tang_ACb_W = v_ACb_W - np.dot(v_ACb_W, nhat_BA_W) * nhat_BA_W
        return v_tang_ACb_W














"""
To be used to get the system input, i.e. torques

The following snippet shows how per model instance actuation can be set:
Click to expand C++ code...

ModelInstanceIndex model_instance_index = ...;
VectorX<T> u_instance(plant.num_actuated_dofs(model_instance_index));
int offset = 0;
for (JointActuatorIndex joint_actuator_index :
plant.GetJointActuatorIndices(model_instance_index)) {
const JointActuator<T>& actuator = plant.get_joint_actuator(
joint_actuator_index);
const Joint<T>& joint = actuator.joint();
VectorX<T> u_joint = ... my_actuation_logic_for(joint) ...;
ASSERT(u_joint.size() == joint_actuator.num_inputs());
u_instance.segment(offset, u_joint.size()) = u_joint;
offset += u_joint.size();
}
plant.get_actuation_input_port(model_instance_index).FixValue(
plant_context, u_instance);


"""






