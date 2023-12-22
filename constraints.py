"""
Constraints will be implemented as functions for the ease of use
"""
def SystemDynamicConstraint(vars, context_index, plant, context_list, nq, nv, nu):
	v, q_next, v_next, u_next, lambdasAndSlack = np.split(vars, [nv, nv+nq, nq+nv+nv, nq+nv+nv+nu])

	if isinstance(vars[0], AutoDiffXd):
		if not autoDiffArrayEqual(v, plant.GetVelocities()):

 """
 The little-dog example:

    def velocity_dynamics_constraint(vars, context_index):
        h, q, v, qn = np.split(vars, [1, 1+nq, 1+nq+nv])
        if isinstance(vars[0], AutoDiffXd):
            if not autoDiffArrayEqual(
                    q,
                    ad_plant.GetPositions(
                        ad_velocity_dynamics_context[context_index])):
                ad_plant.SetPositions(
                    ad_velocity_dynamics_context[context_index], q)
            v_from_qdot = ad_plant.MapQDotToVelocity(
                ad_velocity_dynamics_context[context_index], (qn - q) / h)
        else:
            if not np.array_equal(q, plant.GetPositions(
                    context[context_index])):
                plant.SetPositions(context[context_index], q)
            v_from_qdot = plant.MapQDotToVelocity(context[context_index],
                                                  (qn - q) / h)
        return v - v_from_qdot
    for n in range(N-1):
        prog.AddConstraint(partial(velocity_dynamics_constraint,
                                   context_index=n),
                           lb=[0] * nv,
                           ub=[0] * nv,
                           vars=np.concatenate(
                               ([h[n]], q[:, n], v[:, n], q[:, n + 1])))
"""
