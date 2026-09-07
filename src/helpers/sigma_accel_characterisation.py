import pandas as pd
from pathlib import Path
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import numpy as np
from scipy.signal import savgol_filter

script_dir = Path(__file__).resolve().parent
project_root = script_dir.parents[1]
classification_dir = project_root / "log" / "sigma_accel_classification" / "11_10"
log_path = classification_dir / "position.csv"
df = pd.read_csv(log_path)


SETTLE_TIME = 10.0
SMOOTHING_WINDOW = 0.5
MAX_GAP_S = 0.15

measurements = df[df['source'] == 'extpos_sent'].sort_values('t').reset_index(drop=True)
hover = measurements[measurements['sequence'] == 'HOVERING'].copy()
t0 = hover['t'].iloc[0]
hover = hover[hover['t'] >= t0 + SETTLE_TIME].copy()
hover['t'] -= hover['t'].iloc[0]


# split hover df into smaller segments, split where time gap is large
dt = np.diff(hover['t'].values)
bounds = [0] + list(np.where(dt > MAX_GAP_S)[0] + 1) + [len(hover)]
segments = [hover.iloc[bounds[i]:bounds[i+1]] for i in range(len(bounds) - 1)]
segments = [s for s in segments if len(s) >= 20]

all_accel, plot_data = [], []
for seg in segments:
    t = seg['t'].values
    pos = seg[['x', 'y', 'z']].values

    median_dt = np.median(np.diff(t))
    window = int(round(SMOOTHING_WINDOW/median_dt))
    window += 1 - window%2
    window = max(window, 5)

    pos_smooth = savgol_filter(pos, window, 3, axis=0)
    vel = np.gradient(pos_smooth, t, axis=0)
    accel = np.gradient(vel, t, axis=0)

    all_accel.append(accel)
    plot_data.append((t, pos, pos_smooth, vel, accel))

all_accel = np.vstack(all_accel)
sigma_accel = all_accel.std(axis=0)


print("Measured sigma_accel:")
print(sigma_accel)
print(f"(pooled from {len(all_accel)} samples across {len(segments)} segment(s))")


fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
for t, pos, pos_smooth, vel, accel in plot_data:
    for a in range(3):
        axs[0].plot(t, pos[:, a], alpha=0.3)
        axs[0].plot(t, pos_smooth[:, a])
        axs[1].plot(t, vel[:, a])
        axs[2].plot(t, accel[:, a])
axs[0].set_title('Position: raw vs smoothed')
axs[1].set_title('Estimated velocity (should hover near zero)')
axs[2].set_title('Estimated disturbance acceleration')
axs[2].set_xlabel('time (s)')
plt.tight_layout()
out_path = classification_dir / 'sigma_accel_hover_analysis.png'
plt.savefig(out_path, dpi=150)
print(f"Saved diagnostic plot to {out_path}")