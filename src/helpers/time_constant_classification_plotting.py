import pandas as pd
from pathlib import Path
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import numpy as np
import os
import re


def first_order_step(t, theta_ss, tau):
    return theta_ss * (1 - np.exp(-t / tau))


def find_response_start(theta_rel, theta_ss_guess, abs_thresh=0.5, frac_thresh=0.05, consecutive=3):
    threshold = max(abs_thresh, frac_thresh * abs(theta_ss_guess))
    above = np.abs(theta_rel) >= threshold
    count = 0
    for i, val in enumerate(above):
        if val:
            count += 1
            if count >= consecutive:
                return i - consecutive + 1
        else:
            count = 0
    return 0

def weighted_average(values, stderrs):
    values = np.asarray(values, dtype=float)
    stderrs = np.asarray(stderrs, dtype=float)
    weights = 1.0 / stderrs**2
    mean = np.sum(weights * values) / np.sum(weights)
    mean_stderr = np.sqrt(1.0 / np.sum(weights))
    return mean, mean_stderr


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
    script_dir = Path(__file__).resolve().parent
    project_root = script_dir.parents[1]
    classification_dir = project_root / "log" / "step_response_classification"
    results = []

    for csv_path in classification_dir.glob("*.csv"):
        filename = csv_path.name
        df = pd.read_csv(csv_path)
        step_df = df[df['sequence_state'] == 'STEP']
        t = step_df['timestamp_ms'].values
        t = (t - t[0])/1000.0
        theta = step_df['roll'].values
        theta0 = theta[0]
        theta_rel = theta - theta0

        theta_ss_guess = theta_rel[-5:].mean()

        start_idx = find_response_start(theta_rel, theta_ss_guess,
                                        abs_thresh=0.5,
                                        frac_thresh=0.05,
                                        consecutive=3)

        t = t[start_idx:]
        theta = theta[start_idx:]
        t = t - t[0]
        theta0 = theta[0]
        theta_rel = theta - theta0

        tau_guess = max(t[-1] / 3, 1e-3)
        theta_ss_guess = theta_rel[-5:].mean()

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
        theta_ss = theta_ss + theta0

        match = re.search(r'r([0-9]+)p[0-9]+', filename)
        commanded_degrees = float(match.group(1)) if match else None
        result = {
            'filename': filename,
            'commanded_deg': commanded_degrees,
            'tau_ms': tau * 1000,
            'tau_stderr_ms': perr[1] * 1000,
            'gain': (theta_ss / commanded_degrees) if commanded_degrees else None,
            'theta0': theta0,
            'theta_ss': theta_ss,
            'tau': tau,
            'tau_stderr': perr[1],
            'r_squared': r_squared,
            't': t,
            'theta': theta,
            'theta_pred': theta_pred + theta0,
        }        

        results.append(result)
        plot_fit(csv_path, result, commanded_degrees)

    summary = pd.DataFrame(results)
    tau_w, tau_w_err = weighted_average(summary['tau_ms'], summary['tau_stderr_ms'])
    print(f"Overall tau = {tau_w:.1f} +/- {tau_w_err:.1f} ms")

    if summary['commanded_deg'].notna().any():
        print("\nBy amplitude:")
        for amp, group in summary.groupby('commanded_deg'):
            amp_tau, amp_err = weighted_average(group['tau_ms'], group['tau_stderr_ms'])
            print(f"  {amp} deg: tau = {amp_tau:.1f} +/- {amp_err:.1f} ms (n={len(group)})")


if __name__ == '__main__':
    main()



# RESULTS: tau = 94.6 +- 1.1ms

'''
source /home/control_thesis_lab/thesis_ros2_fresh/analysis-venv/bin/activate
deactivate
'''