#calculate optimal trajectory and simulation
initial_state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# target state for trajectory optimization
final_state = [0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# limit of thrust force for each individual propellor
thrust_limit = 10.0

#we want to have an odd number
num_time_samples = 40
# number of samples into future we will run trajectory optimization
time_horizon = 40

initial_thrust = [0., 0., 0., 0.]
time_step = 0.1


""" 
VectorSystem: A base class that specializes LeafSystem for use with 
only zero or one vector input ports, and only zero or one vector output ports.
"""
class MpcController(VectorSystem): 
    def __init__(self, quadrotor_plant, init_state, target_state, time_horizon, 
                max_time_samples, thrust_limit, obstacle_list, initial_thrust, time_step):

        # define this system as having 12 inputs and 4 outputs
        VectorSystem.__init__(self, 12, 4)
        self.quadrotor_plant = quadrotor_plant
        self.init_state = init_state
        self.target_state = target_state
        self.time_horizon = time_horizon
        self.max_time_samples = max_time_samples
        self.thrust_limit = thrust_limit
        self.obstacle_list = obstacle_list
        self.current_step = 0

        self.current_output = inital_thrust
        
        self.time_step = time_step

        # initialize optimization
        self.prog = MathematicalProgram()

        # define the quadrotor plant using drake built in quadrotor plant class:
        # pydrake.examples.QuadrotorPlant 
        quadrotor_plant = QuadrotorPlant() # this object has base: pydrake.systems.framework.LeafSystem
        quadrotor_plant = builder.AddSystem(quadrotor_plant)

        mpc_controller = MpcController(quadrotor_plant, initial_state, final_state, time_horizon, 
        num_time_samples, thrust_limit, obstacles, initial_thrust, time_step)
        mpc_controller = builder.AddSystem(mpc_controller)

        # connect the MPC controller to the quadrotor
        builder.Connect(mpc_controller.get_output_port(0), quadrotor_plant.get_input_port(0))
        builder.Connect(quadrotor_plant.get_output_port(0), mpc_controller.get_input_port(0))

    def DoCalcVectorOutput(self, context, inp, state, output):

        # quadrotor_mutable_context = self.quadrotor_plant.GetMyMutableContextFromRoot(self.sim_context)
        quadrotor_context = self.quadrotor_plant.CreateDefaultContext()

        # fix input port of quadrotor with output of MPC controller, in order to perform linearization
        # this is the output from the previous iteration of the sim
        self.quadrotor_plant.get_input_port(0).FixValue(quadrotor_context, self.current_output)

        # input into the controller is the state of the quadrotor
        # set the context equal to the current state
        quadrotor_context.SetContinuousState(inp)
        current_state = inp

        ##################
        # Linearize system dynamics - Take first order taylor series expansions of system
        # around the operating point defined by quadrotor context
        ##################
        eq_check_tolerance = 10e6 # we do not need to be at an equilibrium point
        linear_quadrotor = Linearize(self.quadrotor_plant, quadrotor_context, \
        equilibrium_check_tolerance = eq_check_tolerance )

        #### get A & B matrices of linearized continuous dynamics
        A = linear_quadrotor.A()
        B = linear_quadrotor.B()

        # print(f"len A = {len(A)}")
        # print(f"len B = {len(B)}")
        # print(f"A = {A}")
        # print(f"B = {B}")

        # number of time steps in optimization should decrease as we get closer to target
        # num_samples = min(self.time_horizon, self.max_time_samples - self.current_step +1)
        num_samples = self.time_horizon

        # print(f"cur_quad_state = {current_state}")
        # print(f"quadrotor current input = {self.current_output}")
        # print(f"num_samples = {num_samples}")
        # print(f"target_state = {self.target_state}")

        # optimization variables
        # input to controller is state, x
        self.x = self.prog.NewContinuousVariables(self.time_horizon + 1, 12, 'state')
        # output from controller is thrust, u
        self.u = self.prog.NewContinuousVariables(self.time_horizon, 4, 'thrust')

        ## Add starting state constraint based on current state
        self.prog.AddBoundingBoxConstraint(current_state, current_state, self.x[0,:])


        ### add quadratic cost on state error ###
        Q = 100000*np.diag([100.,100.,100.,100.,100.,100.,10.,10.,10.,10.,10.,10.])
        #### Add quadratic cost on input effort ###
        R = np.diag([1.,1.,1.,1.,])

        for t in range(num_samples):
            
            # calculate residual using Implicit Euler integration
            residuals = self.quadrotor_discrete_dynamics(A, B, self.x[t], 
            self.x[t+1], self.u[t], self.time_step)
            for residual in residuals:
                self.prog.AddLinearConstraint(residual == 0)

            # calculate quadratic state error cost
            # error  = self.x[t] - self.target_state
            # self.prog.AddCost(error.T.dot(Q.dot(error)) )
            self.prog.AddQuadraticErrorCost(Q, self.target_state, self.x[t])

            self.prog.AddCost(self.u[t].dot(R.dot(self.u[t])))


        #### Define initial guess trajectory by linearly interpolating between current state and target 
        initial_x_trajectory = self.generate_initial_guess(current_state, self.target_state, 
            num_samples, self.time_step )

        self.prog.SetInitialGuess(self.x, initial_x_trajectory)

        # print("running solver")
        result = Solve(self.prog)
        assert (result.is_success())

        # # retrieve optimal solution
        u_trajectory = result.GetSolution(self.u)
        # x_trajectory = result.GetSolution(x)
        

        #### set output to first step of thrust trajectory ###
        print(u_trajectory[0])
        output[:] = u_trajectory[0]

        # output[:] = [10.,10.,10.,10.]
        self.current_output = output
        print(output)

        ##### increment current timestep
        self.current_step += 1

    def quadrotor_discrete_dynamics(self, A, B, state, state_next, thrust, time_step):
        '''This method assumes quadrotor dynamics have been linearized, and A & B 
        matrices have been passed in from linear quadrotor'''

        # continuous-time dynamics evaluated at the next time step
        # in the form state_dot_continous = f(state, thrust)
        state_dot_continous = A.dot(state_next) + B.dot(thrust)

        # implicit-Euler state update
        # enforce x[n+1] = x[n] + f(x[n], u[n]) * time_step
        residuals = state_next - state - time_step * state_dot_continous

        return residuals


        # generate initial guess of x trajectory by interpolating between current state and target
    def generate_initial_guess(self, current_state, target_state, num_samples, time_interval ):
        # np.random.seed(0)

        time_limits = [0., num_samples * time_interval ]
        state_samples = np.column_stack((self.init_state, self.target_state))
        initial_x_trajectory = PiecewisePolynomial.FirstOrderHold(time_limits, state_samples)

        # sample state on the time grid and add small random noise
        x_trajectory_guess = np.vstack([initial_x_trajectory.value(t * time_interval).T for t in range(num_samples + 1)])
        # x_trajectory_guess += np.random.rand(*x_trajectory_guess.shape) * 5e-6

        return x_trajectory_guess
