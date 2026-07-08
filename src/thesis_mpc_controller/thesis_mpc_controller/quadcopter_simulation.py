
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver, AcadosSim
import numpy as np
import matplotlib.pyplot as plt
from quadcopter_solver import generate_reference, setup_ocp_solver, setup_integrator, run_closed_loop_tracking



def plot_tracking(simX, simU, h):
    Nsim = simU.shape[0]
    t_grid = np.arange(Nsim + 1) * h

    ref = np.array([generate_reference(t, 0, h, 8)[0] for t in t_grid])

    fig, axs = plt.subplots(2, 2, figsize=(10, 7))
    axs[0, 0].plot(t_grid, ref[:, 0], '--', label='ref x')
    axs[0, 0].plot(t_grid, simX[:, 0], label='tracked x')
    axs[0, 0].plot(t_grid, ref[:, 1], '--', label='ref y')
    axs[0, 0].plot(t_grid, simX[:, 1], label='tracked y')
    axs[0, 0].set_title('Horizontal position tracking')
    axs[0, 0].legend()

    axs[0, 1].plot(t_grid, simX[:, 2])
    axs[0, 1].set_title('z (height)')

    axs[1, 0].plot(t_grid[:-1], np.rad2deg(simU[:, 0]), label='phi_cmd')
    axs[1, 0].plot(t_grid[:-1], np.rad2deg(simU[:, 1]), label='theta_cmd')
    axs[1, 0].set_title('Commanded attitude (deg)')
    axs[1, 0].legend()

    axs[1, 1].plot(t_grid[:-1], simU[:, 2])
    axs[1, 1].set_title('Tdev (N)')

    plt.tight_layout()
    plt.show()


def plot_tracking_error(simX, h):
    Nsim_plus1 = simX.shape[0]
    t_grid = np.arange(Nsim_plus1) * h

    ref = np.array([generate_reference(t, 0, h, 8)[0] for t in t_grid])

    pos_error = simX[:, 0:2] - ref[:, 0:2]
    pos_error_mag = np.linalg.norm(pos_error, axis=1)

    z_error = simX[:, 2] - ref[:, 2]

    fig, axs = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

    axs[0].plot(t_grid, pos_error[:, 0], label='x error')
    axs[0].plot(t_grid, pos_error[:, 1], label='y error')
    axs[0].plot(t_grid, pos_error_mag, 'k--', label='|xy error|', linewidth=1.5)
    axs[0].axhline(0, color='gray', linewidth=0.5)
    axs[0].set_title('Horizontal tracking error')
    axs[0].set_ylabel('error (m)')
    axs[0].legend()

    axs[1].plot(t_grid, z_error, color='tab:green')
    axs[1].axhline(0, color='gray', linewidth=0.5)
    axs[1].set_title('Vertical (z) tracking error')
    axs[1].set_ylabel('error (m)')
    axs[1].set_xlabel('time (s)')

    plt.tight_layout()
    plt.show()


def main():
    x0 = np.array([0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, ])

    fs = 20
    h = 1/fs
    N_horizon = 40
    Tf = N_horizon*h

    Ax, Ay, Az = 0.2, 0.2, 0.4
    tau_phi, tau_theta = 0.2, 0.2
    m = 40e-3

    ocp, ocp_solver = setup_ocp_solver(x0, N_horizon, Tf, Ax, Ay, Az, tau_phi, tau_theta, m)
    integrator = setup_integrator(ocp)

    N_sim = 400
    simX, simU = run_closed_loop_tracking(ocp, ocp_solver, integrator, x0, N_sim, h, N_horizon)
    plot_tracking(simX, simU, h)
    plot_tracking_error(simX, h)


if __name__ == '__main__':
    main()