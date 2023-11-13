from pydrake.symbolic import Variable
from pydrake.systems.primitives import SymbolicVectorSystem

""" continuous vs discrete system """

# Define a new symbolic Variable
x = Variable("x")

# Define the System.  
continuous_vector_system = SymbolicVectorSystem(state=[x], dynamics=[-x + x**3], output=[x])


# Define a new symbolic Variable
x = Variable("x")

# Define the System.  Note the additional argument specifying the time period.
discrete_vector_system = SymbolicVectorSystem(state=[x], dynamics=[x**3], output=[x], time_period=1.0)

######################################################################

""" Using LeafSystem """
from pydrake.systems.framework import LeafSystem

# Define the continous system.
class SimpleContinuousTimeSystem(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)

        state_index = self.DeclareContinuousState(1)  # One state variable.
        self.DeclareStateOutputPort("y", state_index)  # One output: y=x.

    # xdot(t) = -x(t) + x^3(t)
    def DoCalcTimeDerivatives(self, context, derivatives):
        x = context.get_continuous_state_vector().GetAtIndex(0)
        xdot = -x + x**3
        derivatives.get_mutable_vector().SetAtIndex(0, xdot)

# Instantiate the System
continuous_system = SimpleContinuousTimeSystem()


# Define the discrete system.
class SimpleDiscreteTimeSystem(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)

        state_index = self.DeclareDiscreteState(1)  # One state variable.
        self.DeclareStateOutputPort("y", state_index)  # One output: y=x.
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=1.0,  # One second time step.
            offset_sec=0.0,  # The first event is at time zero.
            update=self.Update) # Call the Update method defined below.

    # x[n+1] = x^3[n]
    def Update(self, context, discrete_state):
        x = context.get_discrete_state_vector().GetAtIndex(0)
        x_next = x**3
        discrete_state.get_mutable_vector().SetAtIndex(0, x_next)

# Instantiate the System
discrete_system = SimpleDiscreteTimeSystem()

""" Simulation after the continous dynamics is defined """
import matplotlib.pyplot as plt
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import LogVectorOutput

# Create a simple block diagram containing our system.
builder = DiagramBuilder()
system = builder.AddSystem(SimpleContinuousTimeSystem()) # here is the dynamics
logger = LogVectorOutput(system.get_output_port(0), builder)
diagram = builder.Build()

# Set the initial conditions, x(0).
context = diagram.CreateDefaultContext()
context.SetContinuousState([0.9])

# Create the simulator, and simulate for 10 seconds.
simulator = Simulator(diagram, context)
simulator.AdvanceTo(10)

# Plot the results.
log = logger.FindLog(context)
plt.figure()
plt.plot(log.sample_times(), log.data().transpose())
plt.xlabel('t')
plt.ylabel('y(t)')


""" Simulation after the discrete dynamics is defined """
# Create a simple block diagram containing our system.
builder = DiagramBuilder()
system = builder.AddSystem(discrete_vector_system)  # using SymbolicVectorSystem also work, just as Leafsystem 
logger = LogVectorOutput(system.get_output_port(0), builder)
diagram = builder.Build()

# Create the simulator.
simulator = Simulator(diagram)

# Set the initial conditions, x(0).
state = simulator.get_mutable_context().get_mutable_discrete_state_vector()  # receive the Context built by the Simulator
state.SetFromVector([0.9])

# Simulate for 10 seconds.
simulator.AdvanceTo(10)

# Plot the results.
log = logger.FindLog(simulator.get_context())
plt.figure()
plt.stem(log.sample_times(), log.data().transpose())
plt.xlabel('n')
plt.ylabel('y[n]');


"""
The System "Context":
	A core concept in Drake systems framework:
	- The Context captures all of the (potentially) dynamic information that a System requires to implement
	  its core methods: this includes the time, state, any inputs, and any system parameters.
	- The Context of a System is everything you need to know for simulation (or control design, ...), and 
	  given a Context all methods called on a System should be completely deterministic/repeatable.
	- Simulator(diagram) in the above example creates the Context.
	- Note that a Context is not completely defined unless all of the input ports are connected 
	  (simulation and other method calls will fail if they are not).
	- For input ports that are not directly tied to the output of another system, consider using the port's FixValue 
	  method.

"""

######################################################################

""" The REAL POWER of Drake is system-wise prototyping and testing!!!!!!
	Such as:

	Combinations of Systems: Diagram and DiagramBuilder

	The real modeling power of Drake comes from combining many smaller systems together into more complex systems.  
	The concept is very simple: we use the DiagramBuilder class to AddSystem()s and to Connect() input ports to 
	output ports or to expose them as inputs/output of the diagram. Then we call Build() to generate the new Diagram 
	instance, which is just another System in the framework, and can be simulated or analyzed using the entire suite of tools.

	The example below contains 3 sub-systems, i.e. a plant, a controller, and a logger.
"""

import matplotlib.pyplot as plt
import numpy as np
import pydot
from IPython.display import SVG, display
from pydrake.examples import PendulumPlant
from pydrake.systems.analysis import Simulator
from pydrake.systems.controllers import PidController
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import LogVectorOutput

builder = DiagramBuilder()

# First add the pendulum.
pendulum = builder.AddNamedSystem("pendulum", PendulumPlant())

# Add a PID controller.
controller = builder.AddNamedSystem("controller",
                                    PidController(kp=[10.], ki=[1.], kd=[1.]))

# Now "wire up" the controller to the plant.
builder.Connect(pendulum.get_state_output_port(),
                controller.get_input_port_estimated_state())
builder.Connect(controller.get_output_port_control(), pendulum.get_input_port())

# Make the desired_state input of the controller an input to the diagram.
builder.ExportInput(controller.get_input_port_desired_state())
# Make the pendulum state an output from the diagram.
builder.ExportOutput(pendulum.get_state_output_port())

# Log the state of the pendulum.
logger = LogVectorOutput(pendulum.get_state_output_port(), builder)
logger.set_name("logger")

diagram = builder.Build()
diagram.set_name("diagram")

# Visualize the diagram.
display(SVG(pydot.graph_from_dot_data(
    diagram.GetGraphvizString(max_depth=2))[0].create_svg()))

"""
Just like any other System, a Diagram has a Context. You can always extract the context of a subsystem using 
Diagram.GetSubSystemContext() or Diagram.GetMutableSubsystemContext()
"""

######################################################################
""" Simulating the PID controlled system and visulize the results """

# Set up a simulator to run this diagram.
simulator = Simulator(diagram)
context = simulator.get_mutable_context()

# We'll try to regulate the pendulum to a particular angle.
desired_angle = np.pi/2.

# First we extract the subsystem context for the pendulum.
pendulum_context = diagram.GetMutableSubsystemContext(pendulum, context)
# Then we can set the pendulum state, which is (theta, thetadot).
pendulum_context.get_mutable_continuous_state_vector().SetFromVector(
    [desired_angle + 0.1, 0.2])

# The diagram has a single input port (port index 0), which is the desired_state.
diagram.get_input_port(0).FixValue(context, [desired_angle, 0.])

# Clear the logger only because we've written this notebook with the opportunity to
# simulate multiple times (in this cell) using the same logger object.  This is
# often not needed.
logger.FindMutableLog(context).Clear()

# Simulate for 10 seconds.
simulator.AdvanceTo(20);

# Plot the results.
log = logger.FindLog(simulator.get_context())
t = log.sample_times()
plt.figure()
# Plot theta.
plt.plot(t, log.data()[0,:],'.-')
# Draw a line for the desired angle.
plt.plot([t[0], t[-1]], [desired_angle, desired_angle], 'g' )
plt.xlabel('time (seconds)')
plt.ylabel('theta (rad)')
plt.title('PID Control of the Pendulum');

plt.show()