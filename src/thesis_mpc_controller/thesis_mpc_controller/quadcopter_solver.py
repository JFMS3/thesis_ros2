# Adapted from https://github.com/acados/acados/blob/main/examples/acados_python/getting_started/minimal_example_closed_loop.py

from acados_template import AcadosOcp, AcadosOcpSolver
from .quadcopter_model import export_quadcopter_ode_model
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
    
    Q_mat = np.diag([5, 5, 5, 2, 2, 2, 0, 0])
    R_mat = np.diag([0.1, 0.1, 0.1])

    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)
    ocp.cost.W_e = Q_mat

    ocp.cost.Vx = np.vstack([np.eye(nx), np.zeros((nu, nx))])
    ocp.cost.Vu = np.vstack([np.zeros((nx, nu)), np.eye(nu)])
    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((ny_e,))

    # constraints
    phi_max = np.deg2rad(5)
    theta_max = np.deg2rad(5)
    Tdev_max = 0.05

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


