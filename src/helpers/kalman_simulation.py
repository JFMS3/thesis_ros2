import yaml
import numpy as np
from thesis_mpc_controller.thesis_mpc_controller.kalman_filter import PositionVelocityKalmanFilter
import os
from scipy.stats import chi2
import matplotlib.pyplot as plt

current_dir = os.path.dirname(os.path.abspath(__file__))
yaml_path = os.path.abspath(os.path.join(current_dir, '..', "thesis_mpc_controller", "config", "matrix_params.yaml"))
with open(yaml_path, 'r') as f:
    params = yaml.safe_load(f)
    kf_params = params.get("/kalman_filter").get('ros_parameters')
    mpc_params = params.get("/mpc_solver").get('ros_parameters')

R_pos_diag = [float(v) for v in kf_params.get('R_pos_diag')]
sigma_accel = [float(v) for v in kf_params.get('sigma_accel')]
P_pos_init = float(kf_params.get('P_pos_init'))
P_vel_init = float(kf_params.get('P_vel_init'))
R_pos_std = np.sqrt(R_pos_diag)
nis_threshold = kf_params.get('nis_threshold')
dt = 0.01
duration = 30.0
print(R_pos_diag, R_pos_std, sigma_accel, P_pos_init, P_vel_init, nis_threshold)


def simulate_truth(rng, n_steps, dt, sigma_accel):
    pos = np.array([0.0, 0.0, 1.0])
    vel = np.array([0.0, 0.2, 0.0])
    positions, velocities = [], []

    for _ in range(n_steps):
        accel = np.array(sigma_accel) * rng.standard_normal(3)
        pos = pos + vel*dt + 0.5*accel*dt**2
        vel = vel + accel*dt
        positions.append(pos.copy())
        velocities.append(vel.copy())
    return np.array(positions), np.array(velocities)


def single_trial(seed, noise_prob=0.01):
    rng = np.random.default_rng(seed)
    n_steps = int(duration / dt)
    kf = PositionVelocityKalmanFilter(R_pos_diag, sigma_accel, P_pos_init, P_vel_init)

    time, truth_pos, truth_vel, accepted = [], [], [], []
    mean_pos, est_pos, est_vel, nis_hist = [], [], [], []

    t = 0.0
    for _ in range(n_steps):
        t += dt
        pos, vel = truth(t)
        noisy_pos = pos + R_pos_std * rng.standard_normal(3)
        if rng.random() < noise_prob:
            noisy_pos[rng.integers(3)] += 0.05

        if not kf.initialised:
            kf.initialise(noisy_pos)
        else:
            kf.predict(dt)
            nis, ok = kf.update(noisy_pos, nis_threshold=nis_threshold)
            accepted.append(ok)

        x = kf.get_state()
        time.append(t)
        truth_pos.append(pos)
        truth_vel.append(vel)
        mean_pos.append(noisy_pos)
        est_pos.apend(x[:3])
        est_vel.apend(x[3:])
    
    time, truth_pos, truth_vel, accepted = np.array(time), np.array(truth_pos), np.array(truth_vel), np.array(accepted)
    mean_pos, est_pos, est_vel, nis_hist = np.array(mean_pos), np.array(est_pos), np.array(est_vel), np.array(nis_hist)

    pos_rmse = np.sqrt(np.mean((est_pos - truth_pos) ** 2, axis=0))
    vel_rmse = np.sqrt(np.mean((est_vel - truth_vel) ** 2, axis=0))
    print(f"Position RMSE: {pos_rmse}")
    print(f"Velocity RMSE: {vel_rmse}")
    print(f"Measurements rejected by NIS threshold: {np.sum(~accepted)} / {len(accepted)}")

    return pos_rmse, vel_rmse, nis_hist

'''
Goal of monte carlo test is to check error covariance. By running many trials we can see if the Kalman filter's
actual error is within the expected uncertainty claimed by the filter. Want something like 95%+ for NIS and position NEES in 95% consistency bounds

Recall NEES (normalised estimation error squared) evaluates the accuracy of the full state space by comparing estimation error
against state error covariance matrix.
Normalised Innovation Squared checks accuracy of merasurementt by looking at innovation (difference between actual and predicted measurement)
normalised by iinnovation covariance matrix.
'''


def run_monte_carlo(num_runs=100, noise_prob=0.0):
    all_nis, all_nees_pos = [], []

    for run in range(num_runs):
        kf = PositionVelocityKalmanFilter(R_pos_diag, sigma_accel, P_pos_init, P_vel_init)
        rng = np.random.default_rng(run)
        n_steps = int(duration/dt)
        t = 0.0
        run_nis, run_nees = [], []
        pos_arr, vel_arr = simulate_truth(rng, n_steps, dt, sigma_accel)

        for i in range(n_steps):
            t += dt
            noisy_pos = pos_arr[i] + R_pos_std * rng.standard_normal(3)
            if rng.random() < noise_prob:
                noisy_pos[rng.integers(3)] += 0.05
    
            if not kf.initialised:
                kf.initialise(noisy_pos)
                continue
            
            kf.predict(dt)
            nis, _ = kf.update(noisy_pos, nis_threshold=None)
            run_nis.append(nis)

            x = kf.get_state()
            err_pos = x[:3] - pos_arr[i]
            P_pos = kf.kf.P[:3, :3]
            run_nees.append(float(err_pos @ np.linalg.solve(P_pos, err_pos)))

        all_nis.append(run_nis)
        all_nees_pos.append(run_nees)

    
    all_nis = np.array(all_nis)
    all_nees_pos = np.array(all_nees_pos)
 
    mean_nis = all_nis.mean(axis=0)
    mean_nees = all_nees_pos.mean(axis=0)
 
    dof = 3 * num_runs
    lower_nis, upper_nis = chi2.ppf([0.025, 0.975], dof) / num_runs
    lower_nees, upper_nees = chi2.ppf([0.025, 0.975], dof) / num_runs
 
    frac_nis_ok = np.mean((mean_nis >= lower_nis) & (mean_nis <= upper_nis))
    frac_nees_ok = np.mean((mean_nees >= lower_nees) & (mean_nees <= upper_nees))
 
    print(f"Mean NIS in 95% consistency bounds [{lower_nis}, {upper_nis}]: {frac_nis_ok*100}% of timesteps")
    print(f"Mean position NEES in 95% consistency bounds [{lower_nees}, {upper_nees}]: {frac_nees_ok*100}% of timesteps")
 
    fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    t_grid = np.arange(len(mean_nis)) * dt
 
    axs[0].plot(t_grid, mean_nis, label='mean NIS')
    axs[0].axhline(lower_nis, color='r', linestyle='--')
    axs[0].axhline(upper_nis, color='r', linestyle='--', label='95% bounds')
    axs[0].set_title(f'Mean NIS over {num_runs} runs')
    axs[0].legend()
 
    axs[1].plot(t_grid, mean_nees, label='mean position NEES')
    axs[1].axhline(lower_nees, color='r', linestyle='--')
    axs[1].axhline(upper_nees, color='r', linestyle='--', label='95% bounds')
    axs[1].set_title(f'Mean position NEES over {num_runs} runs')
    axs[1].set_xlabel('time (s)')
    axs[1].legend()
 
    plt.tight_layout()
    plt.savefig('kf_monte_carlo.png', dpi=150)
    print("Saved plot to kf_monte_carlo.png")


run_monte_carlo(num_runs=100)

'''
cd src and then run python -m helpers.kalman_simulation
'''