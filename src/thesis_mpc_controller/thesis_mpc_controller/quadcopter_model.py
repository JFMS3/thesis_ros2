# Adapted from: https://github.com/acados/acados/blob/main/examples/acados_python/getting_started/pendulum_model.py
from acados_template import AcadosModel
from casadi import SX, vertcat

def export_quadcopter_ode_model(Ax, Ay, Az, tau_phi, tau_theta, m) -> AcadosModel:
    model_name = 'quadcopter_ode'

    # constants
    g=9.81 # gravity constant [m/s^2]

    # states
    x = SX.sym('x')
    y = SX.sym('y')
    z = SX.sym('z')
    vx = SX.sym('vx')
    vy = SX.sym('vy')
    vz = SX.sym('vz')
    phi = SX.sym('phi')
    theta = SX.sym('theta')
    state = vertcat(x, y, z, vx, vy, vz, phi, theta)

    # controls
    phi_cmd = SX.sym('phi_cmd')
    theta_cmd = SX.sym('theta_cmd')
    T_dev = SX.sym('T_dev')
    u = vertcat(phi_cmd, theta_cmd, T_dev)

    x_dot = SX.sym('x_dot')
    y_dot = SX.sym('y_dot')
    z_dot = SX.sym('z_dot')
    vx_dot = SX.sym('vx_dot')
    vy_dot = SX.sym('vy_dot')
    vz_dot = SX.sym('vz_dot')
    phi_dot = SX.sym('phi_dot')
    theta_dot = SX.sym('theta_dot')
    state_dot = vertcat(x_dot, y_dot, z_dot, vx_dot, vy_dot, vz_dot, phi_dot, theta_dot)

    # dynamics
    f_expl = vertcat(
        vx, vy, vz,
        -Ax*vx + g*phi, -Ay*vy - g*theta, -Az*vz + T_dev/m,
        (phi_cmd-phi)/tau_phi, (theta_cmd-theta)/tau_theta
    )
    f_impl = state_dot - f_expl

    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = state
    model.xdot = state_dot
    model.u = u
    model.name = model_name
    
    return model