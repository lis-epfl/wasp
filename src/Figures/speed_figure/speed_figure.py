import matplotlib.pyplot as plt
import numpy as np
import matplotlib as mpl
import csv
from scipy.signal import savgol_filter
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch

# Enable LaTeX rendering
mpl.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif": ["Computer Modern Roman"],
    "axes.unicode_minus": False,
    "axes.unicode_minus": False  # ensure minus sign renders correctly with LaTeX
})


# ENCODER DATA
csv_path = ("src/Figures/speed_figure/data_2025-09-19_16-12-47.csv")

time_vals, position_vals, velocity_vals, current_vals, voltage_vals = [], [], [], [], []
with open(csv_path, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        time_vals.append(float(row['Timestamp [s]']))
        position_vals.append(float(row['Linear position [m]']))
        velocity_vals.append(float(row['Linear speed [m/s]']))
        current_vals.append(float(row['Current [A]']))
        voltage_vals.append(float(row['Voltage [V]']))

time_vals = np.array(time_vals)
position_vals = np.array(position_vals)
velocity_vals = np.array(velocity_vals)
current_vals = np.array(current_vals)
voltage_vals = np.array(voltage_vals)

index_start, index_end = 0, 80
time_alignement = 1
t = [time - (time_vals[index_start]+time_alignement) for time in time_vals[index_start:index_end]]
pos = position_vals[index_start:index_end]
vel = velocity_vals[index_start:index_end]
curr = current_vals[index_start:index_end]
volt = voltage_vals[index_start:index_end]
time_start = -0.5
time_stop = 8.5

power = - curr * volt
# power = savgol_filter(power, 11, 3)

# acc = np.gradient(vel, t)
dt = np.mean(np.diff(t))
# acc = savgol_filter(vel, 5, 3, deriv=1, delta=dt)
acc = np.diff(vel, prepend=vel[0]) / dt


run_distance_metre = np.max(pos)
print(f"Run distance in metres: {run_distance_metre}")



# VISION DATA
csv_path_vision = ("src/Figures/speed_figure/input_track.csv")
vision_time_vals, vision_x_vals = [], []
with open(csv_path_vision, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        vision_time_vals.append(float(row['t_sec']))
        vision_x_vals.append(float(row['x_center']))
vision_time_vals = np.array(vision_time_vals)
vision_x_vals = np.array(vision_x_vals)

csv_path_vision_ground_truth = ("src/Figures/speed_figure/slow_ground_truth_track.csv")
vision_time_ground_truth_vals = []
with open(csv_path_vision_ground_truth, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        vision_time_ground_truth_vals.append(float(row['x_center']))
vision_time_ground_truth_vals = np.array(vision_time_ground_truth_vals)

run_distance_pixels = vision_x_vals[-1] - vision_x_vals[0]
print(f"Run distance in pixels (fast run): {run_distance_pixels}")

run_distance_pixels_ground_truth = vision_time_ground_truth_vals[-1] - vision_time_ground_truth_vals[0]
print(f"Run distance in pixels (slow run/ground truth): {run_distance_pixels_ground_truth}")

vision_x_vals_meters = (vision_x_vals - vision_x_vals[0]) * (run_distance_metre / run_distance_pixels_ground_truth)
time_vision_alignement = 2.75
vision_time_vals_aligned = vision_time_vals - time_vision_alignement

vision_ground_truth = np.max(vision_x_vals_meters)


# PLOTTING

colormap = mpl.colormaps.get_cmap('plasma')

t = np.asarray(t)
pos = np.asarray(pos)
vel = np.asarray(vel)
acc = np.asarray(acc)
power = np.asarray(power)

if __name__ == "__main__":
    fig, axes = plt.subplots(4, 1, figsize=(5, 6), sharex=True)

    # --- POSITION ---
    axes[0].plot(t, pos, color='black', linewidth=1, label='Motor measurements')
    axes[0].set_ylabel(r"Position [m]", fontsize=11)
    axes[0].set_ylim(-10, 85)
    axes[0].set_yticks(np.arange(0, 71, 10))
    axes[0].grid(True, linestyle='-.', color='#BBBBBB', linewidth=0.4)
    axes[0].plot([7.5, 8.5], [vision_ground_truth, vision_ground_truth], linestyle="-.", linewidth=1, color=colormap(0.8))
    axes[0].plot([-2.5, 0.5], [0, 0], linestyle="-.", linewidth=1, color=colormap(0.8), label='External vision ground truth')
    axes[0].legend(loc='upper left', fontsize=9)



    # --- VELOCITY ---
    axes[1].plot(t, vel, color='black', linewidth=1)
    axes[1].set_ylabel(r"Speed [m/s]", fontsize=11)
    axes[1].set_ylim(-1, 16)
    axes[1].set_yticks(np.arange(0, 15, 2))
    axes[1].grid(True, linestyle='-.', color='#BBBBBB', linewidth=0.4)


    # --- ACCELERATION ---
    axes[2].plot(t, acc, color='black', linewidth=1)
    axes[2].set_ylabel(r"Acceleration [m/s$^2$]", fontsize=11)
    axes[2].set_ylim(-6, 10)
    axes[2].set_yticks(np.arange(-4, 9, 2))
    axes[2].grid(True, linestyle='-.', color='#BBBBBB', linewidth=0.4)

    # --- POWER ---
    axes[3].plot(t, power, color='black', linewidth=1)
    axes[3].set_xlabel(r"Time [s]", fontsize=11)
    axes[3].set_ylabel(r"Power [W]", fontsize=11)
    axes[3].set_ylim(-250, 750)
    axes[3].set_yticks(np.arange(-250, 750, 250))
    axes[3].grid(True, linestyle='-.', color='#BBBBBB', linewidth=0.4)
    axes[3].set_xticks(np.arange(0, 9, 1))
    axes[3].plot([], [], color=colormap(0.4), linestyle='-.', linewidth=1.0, label='Mean over region')
    axes[3].legend(loc='upper right', fontsize=9)

    shaded_regions = [
        (-1, 0, None, "Idle"),
        (0, 1.8, '#BBBBBB', "Acceleration / Deceleration"),
        (1.8, 4.15, None, "Cruising"),
        (4.15, 7.7, '#BBBBBB', "Acceleration / Deceleration"),
        (7.7, 8.5, None, "Idle"),
    ]

    region_means = []

    for x_start, x_end, _, _ in shaded_regions:
        mask = (t >= x_start) & (t <= x_end)
        if np.any(mask):
            mean_power = np.mean(power[mask])
            region_means.append((x_start, x_end, mean_power))

            # horizontal mean line (red dotted)
            axes[3].hlines(
                mean_power, x_start, x_end,
                color=colormap(0.4), linestyle='-.',
                linewidth=1.0, zorder=3
            )

    # --- Draw shaded regions ---
    for ax in axes:
        for x_start, x_end, color, label in shaded_regions:
            if color is not None:
                ax.axvspan(
                    x_start, x_end, color=color,
                    alpha=0.30, linewidth=0, zorder=0
                )
        ax.tick_params(labelsize=9)
        ax.set_xlim(time_start, time_stop)

    plt.tight_layout()
    plt.savefig("src/Figures/speed_figure/speed_figure.png", dpi=400, bbox_inches='tight')
    plt.show()