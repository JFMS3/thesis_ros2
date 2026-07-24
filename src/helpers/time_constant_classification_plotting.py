import pandas as pd
from pathlib import Path
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import numpy as np
import os
import re


def first_order_step(t, theta_ss, tau):
    return theta_ss * (1 - np.exp(-t / tau))


def plot_fit(csv_path, result, commanded_deg=None):
    plt.figure(figsize=(7, 4))
    plt.plot(result['t'], result['theta'], '.', alpha=0.6, label='measured')
    plt.plot(result['t'], result['theta_pred'], '-',
              label=f"fit (tau={result['tau']*1000:.0f} ms, R2={result['r_squared']:.3f})")
    if commanded_deg is not None:
        plt.axhline(commanded_deg, linestyle='--', color='gray', alpha=0.5, label='commanded')
    plt.xlabel('time since step onset [s]')
    plt.ylabel(f'Roll [deg]')
    plt.title(f"Step response fit — {os.path.basename(csv_path)}")
    plt.legend()
    plt.tight_layout()
    out_png = csv_path.with_suffix('').name + '_fit.png'
    out_png = csv_path.parent / out_png
    plt.savefig(out_png, dpi=150)
    print(f"Saved plot: {out_png}")


def main():
    FILENAME = "step_response_20260724_174824_r15p0_1s.csv"
    script_dir = Path(__file__).resolve().parent
    project_root = script_dir.parents[1]
    csv_path = project_root / "log" / FILENAME

    df = pd.read_csv(csv_path)
    step_df = df[df['sequence_state'] == 'STEP']
    t = step_df['timestamp_ms'].values
    t = (t - t[0])/1000.0
    theta = step_df['roll'].values
    theta0 = theta[0]
    theta_rel = theta - theta0

    theta_ss_guess = theta_rel[-5:].mean()
    tau_guess = max(t[-1] / 3, 1e-3)

    popt, pcov = curve_fit(
        first_order_step, t, theta_rel,
        p0=[theta_ss_guess, tau_guess],
        maxfev=5000
    )
    theta_ss, tau = popt
    perr = np.sqrt(np.diag(pcov))

    theta_pred = first_order_step(t, *popt)
    ss_res = np.sum((theta_rel - theta_pred) ** 2)
    ss_tot = np.sum((theta_rel - theta_rel.mean()) ** 2)
    r_squared = 1 - ss_res / ss_tot if ss_tot > 0 else float('nan')
    result = {
        'theta0': theta0,
        'theta_ss': theta_ss + theta0,
        'tau': tau,
        'tau_stderr': perr[1],
        'r_squared': r_squared,
        't': t,
        'theta': theta,
        'theta_pred': theta_pred + theta0,
    }

    match = re.search(r'r([0-9]+)p[0-9]+', FILENAME)
    commanded_degrees = match.group(1)
    plot_fit(csv_path, result, commanded_degrees)

if __name__ == '__main__':
    main()

'''
source /home/control_thesis_lab/thesis_ros2_fresh/analysis-venv/bin/activate
deactivate
'''