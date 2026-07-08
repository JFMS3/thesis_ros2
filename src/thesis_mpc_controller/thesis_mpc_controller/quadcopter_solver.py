# Adapted from https://github.com/acados/acados/blob/main/examples/acados_python/getting_started/minimal_example_closed_loop.py

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver, AcadosSim
from quadcopter_model import export_quadcopter_ode_model
import numpy as np
import scipy.linalg

def setup_ocp_solver(x0, N_horizon, Tf, Ax, Ay, Az, tau_phi, tau_theta, m):
    ocp = AcadosOcp()
    model = export_quadcopter_ode_model(Ax, Ay, Az, tau_phi, tau_theta, m)
    ocp.model = model

    nx = model.x.rows()
    nu = model.u.rows()
    ny = nx + nu
    ny_e = nx

    # Set cost
    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'
    
    Q_mat = np.diag([5, 5, 5, 0, 0, 0, 0, 0])
    R_mat = np.diag([0.1, 0.1, 0.1])

    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)
    ocp.cost.W_e = Q_mat

    ocp.cost.Vx = np.vstack([np.eye(nx), np.zeros((nu, nx))])
    ocp.cost.Vu = np.vstack([np.zeros((nx, nu)), np.eye(nu)])
    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((ny_e,))

    # constraints
    phi_max = np.deg2rad(15)
    theta_max = np.deg2rad(15)
    Tdev_max = 10.0

    ocp.constraints.idxbu = np.array([0, 1, 2])
    ocp.constraints.lbu = np.array([-phi_max, -theta_max, -Tdev_max])
    ocp.constraints.ubu = np.array([phi_max, theta_max, Tdev_max])
    ocp.constraints.x0 = x0

    ocp.solver_options.N_horizon = N_horizon
    ocp.solver_options.tf = Tf

    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'

    ocp.code_export_directory = 'c_generated_code_quad_ocp'
    ocp_solver = AcadosOcpSolver(ocp)
    
    return ocp, ocp_solver


def generate_reference(t, N_horizon, h, nx, f_platform=0.1, z_hover=2.0):
    omega = 2 * np.pi*f_platform # note f is frequency of platform motion
    refs = np.zeros((N_horizon + 1, nx))

    for k in range(N_horizon + 1):
        t_k = t + k*h
        refs[k, 0] = np.sin(omega * t_k)
        refs[k, 1] = np.cos(omega * t_k)
        refs[k, 2] = z_hover
        refs[k, 3] = omega * np.cos(omega * t_k)
        refs[k, 4] = -omega * np.sin(omega * t_k)
        refs[k, 5] = 0.0
    
    return refs


# for simulating the quadcopter
def setup_integrator(ocp: AcadosOcp):
    sim = AcadosSim.from_ocp(ocp)
    sim.solver_options.num_steps = 2
    sim.code_export_directory = 'c_generated_code_quad_sim'
    return AcadosSimSolver(sim)


def run_closed_loop_tracking(ocp, ocp_solver, integrator, x0, N_sim, h, N_horizon):
    nx = ocp.dims.nx
    nu = ocp.dims.nu
    
    simX = np.zeros((N_sim+1, nx))
    simU = np.zeros((N_sim, nu))
    simX[0, :] = x0

    for i in range(N_sim):
        t = i * h
        refs = generate_reference(t, N_horizon, h, nx)

        for k in range(N_horizon):
            yref_k = np.concatenate([refs[k, :], np.zeros(nu)])
            ocp_solver.cost_set(k, "yref", yref_k)
        ocp_solver.cost_set(N_horizon, "yref", refs[N_horizon, :])

        simU[i, :] = ocp_solver.solve_for_x0(x0_bar=simX[i, :])
        simX[i+1, :] = integrator.simulate(x=simX[i, :], u=simU[i, :])

    return simX, simU

