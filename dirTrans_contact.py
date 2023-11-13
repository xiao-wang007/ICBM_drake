# Import some basic libraries and functions for this tutorial.
import numpy as np
import os

from pydrake.common import temp_directory
from pydrake.geometry import StartMeshcat
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization, ModelVisualizer

table_dir = ("/Users/xiao/0_codes/usingDrake/models/objects")
table_top_sdf_file = os.path.join(table_dir, "table_top.sdf")

from pydrake.systems.primitives import ConstantVectorSource

# set h
sim_time_step = 0.0001

# start building the diagram
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)
parser = Parser(plant)

# load Franka Panda
parser.AddModels("/Users/xiao/0_codes/usingDrake/models/franka_description/urdf/panda_arm_hand.urdf")

# load table top
parser.AddModels(table_top_sdf_file)

# load a box
parser.AddModels("/Users/xiao/0_codes/usingDrake/models/ycb/sdf/003_cracker_box.sdf")

# fix the table in {world}
table_frame = plant.GetFrameByName("table_top_center")
plant.WeldFrames(plant.world_frame(), table_frame)

# fix the robot onto the table.
robot_base = plant.GetFrameByName("panda_link0")
plant.WeldFrames(table_frame, robot_base, RigidTransform(RotationMatrix.Identity(), [0., -0.4, 0.]))

# Finalize the plant after loading the scene.
plant.Finalize()

# use the default context to calculate the transformation of the table in {world}
# but this is NOT the context the Diagram consumes.
plant_context = plant.CreateDefaultContext() # fetch the default context

# add robot actuation 
""" ?? Do I need this for my case? Is this only for the controller not planner?? """
torques = builder.AddSystem(ConstantVectorSource(np.zeros(plant.num_actuators())))
builder.Connect(torques.get_output_port(), plant.get_actuation_input_port())

# Set the initial pose for the free bodies
X_WorldTable = table_frame.CalcPoseInWorld(plant_context)







