import matplotlib.pyplot as plt
import matplotlib as mpl
import csv
import numpy as np
from scipy.spatial.transform import Rotation as R
from matplotlib.ticker import FixedLocator


# Enable LaTeX rendering
mpl.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",  # or "sans-serif" or another family
    "font.serif": ["Computer Modern Roman"],  # the default LaTeX font
    "axes.unicode_minus": False  # ensure minus sign renders correctly with LaTeX
})

# Data
# csv_path = "src/Figures/wind_figure/wind_data_2025-10-09_15-59-18.csv"  # dataset1
csv_path = "src/Figures/wind_figure/wind_data_2026-01-07_14-39-18.csv"    # dataset2

with open(csv_path, newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    data = []
    for row in reader:
        try:
            # Ensure all required values exist and are valid floats
            timestamp = float(row['Timestamp [s]'])
            u = float(row['U Vector [m/s]']) 
            v = -float(row['V Vector [m/s]']) # minus for convention (positive forward)
            w = float(row['W Vector [m/s]'])
            pitch = float(row['Pitch [°]'])
            roll = float(row['Roll [°]'])
            heading = float(row['Heading [°]'])

            norm3D = float(row['Wind 3D norm [m/s]'])
            data.append({
                'timestamp': timestamp,
                'u': u,
                'v': v,
                'w': w,
                'pitch': pitch,
                'roll': roll,
                'heading': heading,
            })
        except (ValueError, KeyError):
            continue  # Skip rows with missing or invalid data

# Time
timestamps = [row['timestamp'] for row in data]

# Speed data
# csv_path_speed = "src/Figures/wind_figure/data_2025-10-09_15-59-24.csv"  # dataset1
csv_path_speed = "src/Figures/wind_figure/data_2026-01-07_14-39-24.csv"    # dataset2

speed = []
with open(csv_path_speed, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        speed.append(-float(row['Linear speed [m/s]'])) # minus for convention (positive forward)

# --- Match lengths ---
len_t = len(timestamps)
len_s = len(speed)

if len_s < len_t:
    # Prepend zeros at the start
    padding = [0.0] * (len_t - len_s)
    speed = padding + speed
elif len_s > len_t:
    # Truncate extra elements if necessary
    speed = speed[:len_t]

# Manual alignment offset (in samples) 
speed_offset = 4  

if speed_offset > 0:
    # speed delayed → shift right
    speed = [0.0]*speed_offset + speed[:-speed_offset]
elif speed_offset < 0:
    # speed advanced → shift left
    speed = speed[-speed_offset:] + [0.0]*(-speed_offset)


# Data extraction
u = np.array([row['v'] for row in data]) # axis inverted
v = np.array([row['u'] for row in data])
w = np.array([row['w'] for row in data])
pitch = [row['pitch'] for row in data]
roll = [row['roll'] for row in data]
yaw = [row['heading'] for row in data]
pitch = np.radians(np.array(pitch) - pitch[0]) # remove initial offset and convert to radians
roll = np.radians(np.array(roll) - roll[0])
yaw = np.radians(np.array(yaw) - yaw[0])


true_wind_world = []

for i in range(len(u)):
    wind_sensor = np.array([u[i], v[i], w[i]])

    rotation = R.from_euler('xyz', [roll[i], pitch[i], yaw[i]])
    wind_world = rotation.apply(wind_sensor)

    sensor_motion_sensor = np.array([speed[i], 0, 0])
    sensor_motion_world = rotation.apply(sensor_motion_sensor)

    true_wind = wind_world + sensor_motion_world
    true_wind_world.append(true_wind)

true_wind_world = np.array(true_wind_world)
true_wind_x = true_wind_world[:, 0]
true_wind_y = true_wind_world[:, 1]
true_wind_z = true_wind_world[:, 2]

colormap = mpl.colormaps.get_cmap('inferno')


#index_start = 975  # dataset1
#index_end = 1075   # dataset1

index_start = 4932  # dataset2
index_end = 5040   # dataset2

timestamps = timestamps[index_start:index_end]
timestamps = [t - timestamps[0] for t in timestamps]  # Normalize to start at 0
speed = speed[index_start:index_end]
u = u[index_start:index_end]
v = v[index_start:index_end]
w = w[index_start:index_end]
true_wind_x = true_wind_x[index_start:index_end]
true_wind_y = true_wind_y[index_start:index_end]
true_wind_z = true_wind_z[index_start:index_end]

if __name__ == "__main__":
    fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1, figsize=(3.5, 4.5), sharex=True)  # same width, double height

    # First plot
    ax1.plot(timestamps, speed, color='black', linewidth=1.5, label='WASP speed')
    ax1.plot(timestamps, u, color=colormap(0.5), linewidth=1.5, label='Raw x-wind') 
    ax1.plot(timestamps, v, color=colormap(0.7), linewidth=1.5, label='Raw y-wind') 
    ax1.plot(timestamps, w, color=colormap(0.9), linewidth=1.5, label='Raw z-wind')
    # Second plot
    ax2.plot(timestamps, true_wind_x, color=colormap(0.5), linewidth=1.5, label='Estimated true x-wind') 
    ax2.plot(timestamps, true_wind_y, color=colormap(0.7), linewidth=1.5, label='Estimated true y-wind') 
    ax2.plot(timestamps, true_wind_z, color=colormap(0.9), linewidth=1.5, label='Estimated true z-wind')

    for ax in [ax1, ax2]:
        ax.set_xlim(0, 10.8)
        ax.set_ylim(-12, 19.5)
        ax.set_ylabel('Amplitude [m/s]', fontsize=11)
        ax.grid(True, linestyle='-.', color='#BBBBBB', linewidth=0.5)

    ax1.legend(loc='upper left', ncol=2, fontsize=9)
    ax2.legend(loc='upper left', fontsize=9)
    ax2.set_xlabel('Time [s]', fontsize=11)

    plt.tight_layout()
    plt.savefig('src/Figures/wind_figure/wind_figure.png', dpi=600, bbox_inches='tight')
    plt.show()
