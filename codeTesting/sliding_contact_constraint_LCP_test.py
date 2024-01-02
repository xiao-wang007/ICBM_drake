import sys
sys.path.append("/Users/xiao/0_codes/ICBM_drake")

import dynamic_contact_problem as dcp
import sliding_contact_constraint_LCP as scc

# initialize the scene
period = 2. # in seconds
N = 100
nContact = 5
nBasis = 6
case = "jointspace"

pandafile = "/Users/xiao/0_codes/ICBM_drake/models/franka_description/urdf/panda_arm_hand.urdf"
boxfile = "/Users/xiao/0_codes/ICBM_drake/models/ycb/sdf/003_cracker_box.sdf"
tablefile =  "/Users/xiao/0_codes/ICBM_drake/models/objects/table_top.sdf"

traj = dcp.Push3D(period, N, tablefile, pandafile, boxfile, visualize=True)
traj.SetDevisionVariables(case, nContact, nBasis)

# testing decision variable dimensions
if traj.nq + traj.nv + traj.nu + nContact * (nBasis+1+1) == traj.nDecVar_ti:
	print("The dim of decision variables at ti matches! \n")
if (traj.nq + traj.nv + traj.nu + nContact * (nBasis+1+1)) * N == traj.nDecVar_all:
	print("The dim of total decision variables matches! \n")


# using a dummy context from one of the contextList
dummyContext = traj.mutable_context_list[0] # at t = 0

# visualize the case in meshcat 
traj.SceneVisualizer()


