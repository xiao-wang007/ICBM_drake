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
from pydrake.geometry import (MeshcatVisualizer, StartMeshcat)
from pydrake.visualization import (VisualizationConfig, ApplyVisualizationConfig, AddDefaultVisualization)

import numpy as np
from functools import partial

"""
Contact data containers

Need to use a class to contain the contact pairs.

Dict as data holder 
Namedtuple as dict values
class methods to get some utilities.

"""

from collections import OrderedDict, namedtuple
ContactPairs = namedtuple("ContactPairs", "bodyA, bodyB, pts_in_A, pts_in_B, \
                           nhat_BA_W_list, miu")

conPairs = dict()
conPairs["h-obj"] = ContactPairs("panda_rightfinger", \
                                 "base_link_cracker", \
                                 [np.array([[0.1],[0.1],[0.1]])], \
                                 [np.array([[0.2],[0.2],[0.2]])], \
                                 [np.array([[0],[0],[1]])], \
                                 0.5)

conPairs["obj-env"] = ContactPairs("base_link_cracker", \
                                   "table_top_link", \
                                   [np.array([[0.1],[0.1],[0.1]])], \
                                   [np.array([[0.2],[0.2],[0.2]])], \
                                   [np.array([[1],[0],[1]])], \
                                   0.5)

"""
init the prog and setting decision variable
"""
# prog = MathematicalProgram()
# T = 2.
# N = 100
# q = prog.NewContinuousVariables(nq, N, "positions")
# v = prog.NewContinuousVariables(nv, N, "velocities")
# u = prog.NewContinuousVariables(nu, N, "joint_torques")
# lambdas = prog.NewContinuousVariables(nc*(nBasis+1+1), N, "contact_force_scalars") # 1 for fn, 1 for slack
# decVars = np.vstack((q,v,u,lambdas))


"""
Constraint functions
"""

# 1. (joint space + task space) dynamics 
# def generalized_dynamics_constraint(plant, contextList, decVars, contactPairs):
#     for ti in range(decVars.shape[1]):

"""
Some utility functions
"""
def AutoDiffArrayEqual(self, a, b):
    return np.array_equal(a, b) and np.array_equal(ExtractGradient(a), ExtractGradient(b))

class SceneFactory(object):
    def __init__(self, h, tableFile, pandaFile, boxFile, X_W_table, X_table_pandaBase, X_table_box, tipFile=None):
        self.builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, time_step=h)
        self.parser = Parser(self.plant)

        if tipFile == None:
            self.modelInstances= {"table_top" : self.parser.AddModels(tableFile)[0], \
                                   "panda"     : self.parser.AddModels(pandaFile)[0], \
                                   "box"       : self.parser.AddModels(boxFile)[0]}
        else:
            self.modelInstances = {"table_top" : self.parser.AddModels(tableFile)[0], \
                               "panda"     : self.parser.AddModels(pandaFile)[0], \
                               "box"       : self.parser.AddModels(boxFile)[0],
                               "tip"       : self.parser.AddModels(tipFile)[0]}

        self.table_top_frame = self.plant.GetFrameByName("table_top_center")
        self.robot_base_frame = self.plant.GetFrameByName("panda_link0")
        self.box_base_frame = self.plant.GetFrameByName("base_link_cracker")

        
        self.plant.WeldFrames(self.plant.world_frame(), self.table_top_frame, X_W_table)
        self.plant.WeldFrames(self.table_top_frame, self.robot_base_frame, X_table_pandaBase)
        self.plant.WeldFrames(self.table_top_frame, self.box_base_frame, X_table_box)

        self.plant.Finalize()
        # self.plant.set_name("Push3D")

        self.diagram = self.builder.Build()
        self.diagram_context = self.diagram.CreateDefaultContext()
        self.mutable_plant_context = self.plant.GetMyContextFromRoot(self.diagram_context)


"""
"A class for the case"
"""

class Push3D(object):
    def __init__(self, T, nStep, tableFile, \
                 pandaFile, boxFile, tipFile=None, \
                 X_W_table=None, X_table_pandaBase=None, X_table_box=None, visualize=False):
        self.builder = DiagramBuilder()
        self.tableFile = tableFile
        self.pandaFile = pandaFile
        self.boxFile = boxFile
        self.tipFile = tipFile
        self.isvisualized = visualize
        self.N = nStep
        self.h = T/nStep
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, time_step=self.h)
        self.parser = Parser(self.plant)

        if tipFile == None:
            self.modelInstances = {"table_top" : self.parser.AddModels(self.tableFile)[0], \
                                   "panda"     : self.parser.AddModels(self.pandaFile)[0], \
                                   "box"       : self.parser.AddModels(self.boxFile)[0]}
        else:
            self.modelInstances = {"table_top" : self.parser.AddModels(self.tableFile)[0], \
                               "panda"     : self.parser.AddModels(self.pandaFile)[0], \
                               "box"       : self.parser.AddModels(self.boxFile)[0],
                               "tip"       : self.parser.AddModels(self.tipFile)[0]}
        self.SetTheScene(X_W_table, X_table_pandaBase, X_table_box)

    def SetTheScene(self, X_W_table, X_table_pandaBase, X_table_box):
        self.table_top_frame = self.plant.GetFrameByName("table_top_center")
        self.robot_base_frame = self.plant.GetFrameByName("panda_link0")
        self.box_base_frame = self.plant.GetFrameByName("base_link_cracker")

        if X_W_table != None:
            self.plant.WeldFrames(self.plant.world_frame(), self.table_top_frame, X_W_table)
        if X_table_pandaBase != None:
            self.plant.WeldFrames(self.table_top_frame, self.robot_base_frame, X_table_pandaBase)

        self.plant.Finalize()
        # self.plant.set_name("Push3D")

        # Two versions: float
        if self.isvisualized:
            self.meshcat = StartMeshcat()
            self.visualizer = MeshcatVisualizer.AddToBuilder(self.builder, self.scene_graph, self.meshcat)
            self.visualization_config = VisualizationConfig()
            self.visualization_config.publish_contacts = True
            self.visualization_config.publish_proximity = True
            ApplyVisualizationConfig(self.visualization_config, self.builder, meshcat=self.meshcat)
        self.diagram = self.builder.Build()
        self.diagram_context_list = [self.diagram.CreateDefaultContext() for i in range(self.N)]
        self.mutable_plant_context_list = [self.plant.GetMyContextFromRoot(context) for context in self.diagram_context_list]

        # set free body pose
        boxBody = self.plant.GetBodyByName("base_link_cracker")
        self.plant.SetFreeBodyPose(self.mutable_plant_context_list[0], boxBody, X_table_box)
        
        """ How to let ad diagram to co-exit with double diagram?? """
        # AutoDiff 
        # self.ad_diagram = self.diagram.ToAutoDiffXd()
        # self.ad_plant = self.ad_diagram.GetSubsystemByName("Push3D")
        # self.ad_mutable_context_list = [self.ad_plant.CreateDefaultContext() for i in range(self.N)]
    
    def CreateSceneForInvKinProg(self, X_W_table, X_table_pandaBase, X_table_box):
        """ create the same scene but weld the box as well, this is for testing the my own contacts 
            This can be substituted by locking the joints in class pydrake.multibody.inverse_kinematics.InverseKinematics
            using the overloaded constructor with Joint::Lock
        """
        return SceneFactory(self.h, self.tableFile, self.pandaFile, self.boxFile, X_W_table, X_table_pandaBase, X_table_box)

    def SetDevisionVariables(self, case, nContact, nBasis):
        self.prog = MathematicalProgram()
        self.nContactParams = nContact * (nBasis + 1 + 1)

        if case == "jointspace":
            # -2 is to offset the two fixed panda fingers
            self.nq = self.plant.num_positions(self.modelInstances["panda"]) - 2 + \
                 self.plant.num_positions(self.modelInstances["box"])
            self.nv = self.plant.num_velocities(self.modelInstances["panda"]) - 2 + \
                 self.plant.num_velocities(self.modelInstances["box"])
            self.nu = self.plant.num_actuators(self.modelInstances["panda"])

            self.qVars = self.prog.NewContinuousVariables(self.nq, self.N, "positions")
            self.vVars = self.prog.NewContinuousVariables(self.nv, self.N, "velocities")
            self.uVars = self.prog.NewContinuousVariables(self.nu, self.N, "actuations")
            self.contactVars = self.prog.NewContinuousVariables(self.nContactParams, self.N, "lambdas&slack")
            # 1 is for fn, 1 is for slack

        elif case == "taskspace":
            self.nq = self.plant.num_positions(self.modelInstances["tip"])  + \
                      self.plant.num_positions(self.modelInstances["box"])
            self.nv = self.plant.num_velocities(self.modelInstances["tip"])  + \
                      self.plant.num_velocities(self.modelInstances["box"])
            self.nu = self.plant.num_actuators(self.modelInstances["tip"])

            self.qVars = self.prog.NewContinuousVariables(self.nq, self.N, "positions")
            self.vVars = self.prog.NewContinuousVariables(self.nv, self.N, "velocities")
            self.uVars = self.prog.NewContinuousVariables(self.nu, self.N, "actuations")
            self.contactVars = self.prog.NewContinuousVariables(self.nContactParams, self.N, "lambdas&slack")

        self.nDecVar_ti = self.nq + self.nv + self.nu + nContact*(nBasis+1+1) # the offset for indexing
        self.nDecVar_all = (self.nq + self.nv + self.nu + nContact*(nBasis+1+1))*self.N

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

        if decVars_tf != None:
            self.qVars[:, -1], self.vVars[:, -1], self.uVars[:, -1], self.contactVars[:, -1] = \
                np.split(decVars_tf, [self.nq, self.nq+self.nv, self.nq+self.nv+self.nContactParams])

    def GetnContact(self, conPairsDict):
        nContact = 0
        for key in conPairsDict:
            nContact += len(conPairsDict[key].pts_in_A)
        return nContact

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
        if decVar_init != None:
            Solve(self.prog, initial_guess=initial_init)
        else:
            pass # zeros init

    def SystemDynamicConstraint(self, vars, context_index, conPairsDict, nBasis):
        nContact = self.GetnContact(conPairsDict)
        v, q_next, v_next, u_next, lambdasAndSlack_next = np.split(vars, \
                        [self.nv, self.nv+self.nq, self.nq+self.nv+self.nv, self.nq+self.nv+self.nv+self.nu])

        # split the contact vars for each contact
        markPoints = [(nBasis+1+1)*i for i in range(1, nContact)]
        lambdasAndSlack_set = [np.split(lambdasAndSlack_next, markPoints)]

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
            counter = 0
            for key, value in conPairDict:
                if key != "obj-env":
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

                        # need the offset the number of contact parameters each time
                        F_AB_W = self.ContactWrenchEvaluator(lambdasAndSlack_set[counter][:-1], \
                                                             value.nhat_BA_W, v_tang_ACb_W)
                        
                        y += Jv_V_W_Ca @ F_AB_W
                        y += Jv_V_W_Cb @ (-F_AB_W)
                        counter += 1
                    
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
                        F_AB_W = self.ContactWrenchEvaluator(lambdasAndSlack_next[counter][:-1], \
                                                             value.nhat_BA_W, v_tang_ACb_W)

                        y += Jv_V_W_Ca @ F_AB_W # the wrench of A applied to B in expressed in {W}
                        counter += 1

            counter = 0
            y = y * self.h - M_next @ (v_next - v)
            
        else:
            # here goes a version for float datatype
            pass

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
        """
        "NOTE: ptB and nhat_BA_W are expected to be 1D vector, i,e (3,)"
        
        :param      context_index:  The context index
        :type       context_index:  { type_description }
        :param      bodyA:          The body a
        :type       bodyA:          { type_description }
        :param      bodyB:          The body b
        :type       bodyB:          { type_description }
        :param      ptB:            The point b
        :type       ptB:            { type_description }
        :param      nhat_BA_W:      The nhat ba w
        :type       nhat_BA_W:      { type_description }
        """
        frameA = self.ad_plant.GetFrameByName(bodyA)
        frameB = self.ad_plant.GetFrameByName(bodyB)
        V_AB_W = frameB.CalcSpatialVelocity(self.mutable_context_list[context_index], \
                                                    frameA, \
                                                    plant.world_frame())
        R_w_B = self.ad_plant.CalcRelativeTransform(self.mutable_context_list[context_index], \
                                                    plant.world_frame(), \
                                                    frameB).rotation() # DCM from body B pose in {W}
        p_BCb_W = R_w_B @ ptB # the contact points in {B} expressed in {W}
        v_ACb_W = V_AB_W.Shift(p_BCb_W).translational() # compute the contact point's velocity in {A}
        v_tang_ACb_W = v_ACb_W - np.dot(v_ACb_W, nhat_BA_W) * nhat_BA_W
        return v_tang_ACb_W

    def SceneVisualizer(self, context_index, jointPositions=None):
        # this is intended for visualization of the trajectory
        # as well as just visualize the scene at a time point
        if jointPositions is not None:
            plant_context_ti = self.mutable_plant_context_list[context_index]
            self.plant.SetPositions(plant_context_ti, self.modelInstances["panda"], jointPositions)
            self.plant.get_actuation_input_port().FixValue(plant_context_ti, np.zeros(9))
            self.diagram.ForcedPublish(self.diagram_context_list[context_index]) # publish the corresponding diagram
        else: 
            plant_context_ti = self.mutable_plant_context_list[context_index]
            self.plant.get_actuation_input_port().FixValue(plant_context_ti, np.zeros(9))
            self.diagram.ForcedPublish(self.diagram_context_list[context_index]) # publish the corresponding diagram


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






