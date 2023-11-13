import numpy as np
from pydrake.common.containers import namedview
from pydrake.common.value import Value
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import BasicVector, LeafSystem
from pydrake.trajectories import PiecewisePolynomial


""" Input and Output Ports """
######################################################################

# Vector-valued Ports
class MyAdder(LeafSystem):
    def __init__(self):
        super().__init__()  # Don't forget to initialize the base class.
        self._a_port = self.DeclareVectorInputPort(name="a", size=2)
        self._b_port = self.DeclareVectorInputPort(name="b", size=2)
        self.DeclareVectorOutputPort(name="sum", size=2, calc=self.CalcSum)
        self.DeclareVectorOutputPort(name="difference",
                                     size=2,
                                     calc=self.CalcDifference)

    def CalcSum(self, context, output):
        # Evaluate the input ports to obtain the 2x1 vectors.
        a = self._a_port.Eval(context)
        b = self._b_port.Eval(context)

        # Write the sum into the output vector.
        output.SetFromVector(a + b)

    def CalcDifference(self, context, output):
        # Evaluate the input ports to obtain the 2x1 vectors.
        a = self._a_port.Eval(context)
        b = self._b_port.Eval(context)

        # Write the difference into output vector.
        output.SetFromVector(a - b)

# Construct an instance of this system and a context.
system = MyAdder()
context = system.CreateDefaultContext()

# Fix the input ports to some constant values.
system.GetInputPort("a").FixValue(context, [3, 4])
system.GetInputPort("b").FixValue(context, [1, 2])

# Evaluate the output ports.
print(f"sum: {system.GetOutputPort('sum').Eval(context)}")
print(f"difference: {system.GetOutputPort('difference').Eval(context)}")


# Abstract-valued Ports
class RotateAboutZ(LeafSystem):
    def __init__(self):
        super().__init__()  # Don't forget to initialize the base class.
        self.DeclareAbstractInputPort(name="in",
                                      model_value=Value(RigidTransform()))
        self.DeclareAbstractOutputPort(
            name="out",
            alloc=lambda: Value(RigidTransform()),
            calc=self.CalcOutput)

    def CalcOutput(self, context, output):
        # Evaluate the input port to obtain the RigidTransform.
        X_1 = system.get_input_port().Eval(context)

        X_2 = RigidTransform(RotationMatrix.MakeZRotation(np.pi / 2)) @ X_1

        # Set the output RigidTransform.
        output.set_value(X_2)

# Construct an instance of this system and a context.
system = RotateAboutZ()
context = system.CreateDefaultContext()

# Fix the input port to a constant value.
system.get_input_port().FixValue(context, RigidTransform())

# Evaluate the output port.
print(f"output: {system.get_output_port(0).Eval(context)}")


""" State Variables:
	- Systens store their state in the Context.
	- Abstract state can only be updated in a discrete-time fashion.
	- Vector-valued state can be declared either dicrete or continuous.
	- For a system to share its state, it must do that using an output port.
	  to do this, use DeclareStateOutputPort().
	- While it is possible and convenient to declare many different groups of
	  discrete state variables (they may even be updated at different rates by different events),
	  only zero or one continuous state vector for a system can be defined. And this is done by
	  overloading LeafSystem::DoCalcTimeDerivatives().
"""
######################################################################

# Discrete-time system
class SimpleDiscreteTimeSystem(LeafSystem):
    def __init__(self):
        super().__init__()

        state_index = self.DeclareDiscreteState(1)  # One state variable.
        self.DeclareStateOutputPort("y", state_index)  # One output: y=x.
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=1.0,  # One second time step.
            offset_sec=0.0,  # The first event is at time zero.
            update=self.Update) # Call the Update method defined below.

    # x[n+1] = x^3[n].
    def Update(self, context, discrete_state):
        x = context.get_discrete_state_vector().GetAtIndex(0)
        x_next = x**3
        discrete_state.get_mutable_vector().SetAtIndex(0, x_next)

# Instantiate the System.
system = SimpleDiscreteTimeSystem()
simulator = Simulator(system)
context = simulator.get_mutable_context()

# Set the initial conditions: x[0] = [0.9].
context.get_mutable_discrete_state_vector().SetFromVector([0.9])

# Run the simulation.
simulator.AdvanceTo(4.0)
print(context.get_discrete_state_vector())


# Abstract-valued state
class AbstractStateSystem(LeafSystem):
    def __init__(self):
        super().__init__()

        self._traj_index = self.DeclareAbstractState(
            Value(PiecewisePolynomial()))
        self.DeclarePeriodicUnrestrictedUpdateEvent(period_sec=1.0,
                                                    offset_sec=0.0,
                                                    update=self.Update)

    def Update(self, context, state):
        t = context.get_time()
        traj = PiecewisePolynomial.FirstOrderHold(
            [t, t + 1],
            np.array([[-np.pi / 2.0 + 1., -np.pi / 2.0 - 1.], [-2., 2.]]))
        # Update the state
        state.get_mutable_abstract_state(int(self._traj_index)).set_value(traj)



system = AbstractStateSystem()
simulator = Simulator(system)
context = simulator.get_mutable_context()

# Set an initial condition for the abstract state.
context.SetAbstractState(0, PiecewisePolynomial())

# Run the simulation.
simulator.AdvanceTo(4.0)
traj = context.get_abstract_state(0).get_value()
print(f"breaks: {traj.get_segment_times()}")
print(f"traj({context.get_time()}) = {traj.value(context.get_time())}")


""" Parameters:
	- vector-valued or abstract.
	- never updated, unlike states.
"""
######################################################################

class SystemWithParameters(LeafSystem):
    def __init__(self):
        super().__init__()  # Don't forget to initialize the base class.

        self.DeclareNumericParameter(BasicVector([1.2, 3.4]))
        self.DeclareAbstractParameter(
            Value(RigidTransform(RotationMatrix.MakeXRotation(np.pi / 6))))

        # Declare output ports to demonstrate how to access the parameters in
        # system methods.
        self.DeclareVectorOutputPort(name="numeric",
                                     size=2,
                                     calc=self.OutputNumeric)
        self.DeclareAbstractOutputPort(
            name="abstract",
            alloc=lambda: Value(RigidTransform()),
            calc=self.OutputAbstract)

    def OutputNumeric(self, context, output):
        output.SetFromVector(context.get_numeric_parameter(0).get_value())

    def OutputAbstract(self, context, output):
        output.set_value(context.get_abstract_parameter(0).get_value())

# Construct an instance of this system and a context.
system = SystemWithParameters()
context = system.CreateDefaultContext()

# Evaluate the output ports.
print(f"numeric: {system.get_output_port(0).Eval(context)}")
print(f"abstract: {system.get_output_port(1).Eval(context)}")


""" Systems can "publish":
	- a callback to "publish".
	- cannot modify any states.
	- useful for broadcasting data outside of the systems framework 
	  (e.g. via a message-passing protocol like in ROS), terminating a simulation, for detecting errors,
	  and for forcing boundaries between integration steps.

"""
######################################################################

class MyPublishingSystem(LeafSystem):
    def __init__(self):
        super().__init__()

        # Calling `ForcePublish()` will trigger the callback.
        self.DeclareForcedPublishEvent(self.Publish)

        # Publish once every second.
        self.DeclarePeriodicPublishEvent(period_sec=1,
                                         offset_sec=0,
                                         publish=self.Publish)
        
    def Publish(self, context):
        print(f"Publish() called at time={context.get_time()}")

system = MyPublishingSystem()
simulator = Simulator(system)
simulator.AdvanceTo(5.3)

# We can also "force" a publish at a arbitrary time.
print("\ncalling ForcedPublish:")
system.ForcedPublish(simulator.get_context())

""" Naming Vector Values:
	- Abstract-valued ports, state, and/or parameters can be used to work with structured data. 
	  But it can also be convenient to use string names to reference the individual elements of 
	  vector-valued data. In Python, we recommend using a namedview workflow, which was inspired 
	  by Python's namedtuple and is demonstrated in the example below.
"""
######################################################################

# Define the system.
class NamedViewDemo(LeafSystem):
    MyDiscreteState = namedview("MyDiscreteState", ["a", "b"])
    MyContinuousState = namedview("MyContinuousState", ["x", "z", "theta"])
    MyOutput = namedview("MyOutput", ["x","a"])

    def __init__(self):
        super().__init__()

        self.DeclareDiscreteState(2)
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=1.0,
            offset_sec=0.0,
            update=self.DiscreteUpdate)
        self.DeclareContinuousState(3)
        self.DeclareVectorOutputPort(name="out", size=2, calc=self.CalcOutput)

    def DiscreteUpdate(self, context, discrete_values):
        discrete_state = self.MyDiscreteState(
            context.get_discrete_state_vector().value())
        continuous_state = self.MyContinuousState(
            context.get_continuous_state_vector().CopyToVector())
        next_state = self.MyDiscreteState(discrete_values.get_mutable_value())
        # Now we can compute the next state by referencing each element by name.
        next_state.a = discrete_state.a + 1
        next_state.b = discrete_state.b + continuous_state.x

    def DoCalcTimeDerivatives(self, context, derivatives):
        continuous_state = self.MyContinuousState(
            context.get_continuous_state_vector().CopyToVector())
        dstate_dt = self.MyContinuousState(continuous_state[:])
        dstate_dt.x = -continuous_state.x
        dstate_dt.z = -continuous_state.z
        dstate_dt.theta = -np.arctan2(continuous_state.z, continuous_state.x)
        derivatives.SetFromVector(dstate_dt[:])

    def CalcOutput(self, context, output):
        discrete_state = self.MyDiscreteState(
            context.get_discrete_state_vector().value())
        continuous_state = self.MyContinuousState(
            context.get_continuous_state_vector().CopyToVector())
        out = self.MyOutput(output.get_mutable_value())
        out.x = continuous_state.x
        out.a = discrete_state.a

# Instantiate the System.
system = NamedViewDemo()
simulator = Simulator(system)
context = simulator.get_mutable_context()

# Set the initial conditions.
initial_discrete_state = NamedViewDemo.MyDiscreteState([3, 4])
context.SetDiscreteState(initial_discrete_state[:])
initial_continuous_state = NamedViewDemo.MyContinuousState.Zero()
initial_continuous_state.x = 0.5
initial_continuous_state.z = 0.92
initial_continuous_state.theta = 0.23
context.SetContinuousState(initial_continuous_state[:])

# Run the simulation.
simulator.AdvanceTo(4.0)
print(
    NamedViewDemo.MyDiscreteState(context.get_discrete_state_vector().value()))
print(
    NamedViewDemo.MyContinuousState(
        context.get_continuous_state_vector().CopyToVector()))
print(NamedViewDemo.MyOutput(system.get_output_port().Eval(context)))


""" Supporting Scalar Type Conversion (double, AutoDiff, and Symbolic):
	The important steps are:
		- Add the @TemplateSystem decorator.
    	- Derive from LeafSystem_[T] instead of simply LeafSystem.
    	- Implement the _construct method instead of the typical __init__ method.
    	- Implement the _construct_copy method, which needs to populate the same member 
    	  fields as _construct (as we did with self.Q in this example).
    	- Add the default instantiation, so that you can continue to refer to the system as, 
    	  e.g., RunningCost in addition to using RunningCost_[float].

For further details, you can find the related documentation for scalar conversion in C++ here 
and the documentation for the @TemplateSystem decorator.

"""
######################################################################

from pydrake.systems.framework import LeafSystem_
from pydrake.systems.scalar_conversion import TemplateSystem
from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression

@TemplateSystem.define("RunningCost_")
def RunningCost_(T):

    class Impl(LeafSystem_[T]):

        def _construct(self, converter=None, Q=np.eye(2)):
            super().__init__(converter)
            self._Q = Q
            self._state_port = self.DeclareVectorInputPort("state", 2)
            self._command_port = self.DeclareVectorInputPort("command", 1)
            self.DeclareVectorOutputPort("cost", 1, self.CostOutput)

        def _construct_copy(self, other, converter=None):
            # Any member fields (e.g. configuration values) need to be
            # transferred here from `other` to `self`.
            Impl._construct(self, converter=converter, Q=other._Q)

        def CostOutput(self, context, output):
            x = self._state_port.Eval(context)
            u = self._command_port.Eval(context)[0]
            output[0] = x.dot(self._Q.dot(x)) + u**2

    return Impl

RunningCost = RunningCost_[None]  # Default instantiation.
