import pandas as pd
from pathlib import Path
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import numpy as np
import os


def exponential_decay(t, v0, A, v_inf):
    return (v0 - v_inf) * np.exp(-A * t) + v_inf


def find_decay_start(v):
    return int(np.argmax(np.abs(v)))



def weighted_average(values, stderrs):
    values = np.asarray(values, dtype=float)
    stderrs = np.asarray(stderrs, dtype=float)
    weights = 1.0 / stderrs**2
    mean = np.sum(weights * values) / np.sum(weights)
    mean_stderr = np.sqrt(1.0 / np.sum(weights))
    return mean, mean_stderr


def plot_fit(csv_path, result, axis):
    plt.figure(figsize=(7, 4))
    plt.plot(result['t'], result['v'], '.', alpha=0.6, label='measured')
    plt.plot(result['t'], result['v_pred'], '-',
              label=f"fit (A={result['A']:.3f} 1/s, R2={result['r_squared']:.3f})")
    plt.xlabel('time since decay onset [s]')
    plt.ylabel(f'{axis} [m/s]')
    plt.title(f"Drag decay fit — {os.path.basename(csv_path)}")
    plt.legend()
    plt.tight_layout()
    out_png = csv_path.with_suffix('').name + '_fit.png'
    out_png = csv_path.parent / out_png
    plt.savefig(out_png, dpi=150)
    print(f"Saved plot: {out_png}")


def main():
    script_dir = Path(__file__).resolve().parent
    project_root = script_dir.parents[1]
    axis = 'vx'
    classification_dir = project_root / "log" / "drag_classification"/ axis
    results = []

    for csv_path in classification_dir.glob("*.csv"):
        filename = csv_path.name
        df = pd.read_csv(csv_path)
        coast_df = df[df['sequence_state'] == 'COAST']
        t = coast_df['timestamp_ms'].values
        t = (t - t[0]) / 1000.0
        v = coast_df[axis].values

        start_idx = find_decay_start(v)

        t = t[start_idx:]
        v = v[start_idx:]
        t = t - t[0]

        v0_guess = v[0]
        A_guess = 1.0
        v_inf_guess = v[-5:].mean()
        popt, pcov = curve_fit(
            exponential_decay, t, v,
            p0=[v0_guess, A_guess, v_inf_guess],
            maxfev=5000
        )
        v0, A, v_inf = popt
        perr = np.sqrt(np.diag(pcov))

        v_pred = exponential_decay(t, *popt)
        ss_res = np.sum((v - v_pred) ** 2)
        ss_tot = np.sum((v - v.mean()) ** 2)
        r_squared = 1 - ss_res / ss_tot if ss_tot > 0 else float('nan')

        result = {
            'filename': filename,
            'axis': axis,
            'v0': v0,
            'A': A,
            'A_stderr': perr[1],
            'r_squared': r_squared,
            't': t,
            'v': v,
            'v_pred': v_pred,
        }

        results.append(result)
        plot_fit(csv_path, result, axis)

    summary = pd.DataFrame(results)
    A_w, A_w_err = weighted_average(summary['A'], summary['A_stderr'])
    print(f"Overall A_{axis[1]} = {A_w:.3f} +/- {A_w_err:.3f} 1/s")


if __name__ == '__main__':
    main()


'''
source /home/control_thesis_lab/thesis_ros2_fresh/analysis-venv/bin/activate
deactivate
'''