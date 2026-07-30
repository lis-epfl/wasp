import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')  # Use a non-interactive backend
import matplotlib.pyplot as plt
import random
from matplotlib.ticker import MaxNLocator, FuncFormatter
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.animation as animation
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
import sys
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from matplotlib.lines import Line2D
import matplotlib.cm as cm
import matplotlib.colors as mcolors


import config


def plot_data(csv_path):
    # Read and filter the CSV file
    with open(csv_path, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        valid_data = []
        for row in reader:
            try:
                # Check that at least the timestamp can be parsed
                float(row['Unix Timestamp [s]'])  # raises ValueError if invalid
                valid_data.append(row)
            except (ValueError, KeyError):
                continue

    if not valid_data:
        print("No valid rows found in CSV.")
        return

    # Extract timestamp from filename
    timestamp = Path(csv_path).stem.replace("data_", "")
    csv_path = Path(csv_path)
    
    # First figure
    fig1, axs = plt.subplots(3, 6, figsize=(14, 8))
    axs = axs.flatten()

    # Generate a random color for each field
    def generate_random_color():
        return [random.random() for _ in range(3)]  # RGB values between 0 and 1

    for idx, (short_key, full_name) in enumerate(config.LI550_MAPPING.items()):
        if full_name in valid_data[0]:
            timestamps = []
            values = []
            for row in valid_data:
                try:
                    t = float(row['Unix Timestamp [s]'])
                    val = float(row[full_name])
                    timestamps.append(t)
                    values.append(val)
                except (ValueError, KeyError):
                    continue  # Skip this row if conversion fails
            if timestamps and values:
                random_color = generate_random_color()
                axs[idx].plot(timestamps, values, color=random_color)
                axs[idx].set_ylabel(full_name)
                axs[idx].grid(True)
                axs[idx].tick_params(axis='x', labelrotation=0)
                axs[idx].yaxis.set_major_formatter(FuncFormatter(lambda val, pos: f'{val:.1f}'))
                axs[idx].xaxis.set_major_formatter(FuncFormatter(lambda val, pos: f'{val:.0f}'))
                if idx < 12:
                    axs[idx].set_xticks([])

    # Set the x-axis labels only for the bottom row (indices 12 to 16)
    for i in range(12, 17):  # Avoid the last subplot
        axs[i].set_xlabel("Unix Timestamp [s]")
        try:
            timestamps = [float(row['Unix Timestamp [s]']) for row in valid_data]
            ticks = sorted(set(round(t, 1) for t in timestamps))
            ticks = ticks[::int(5 / config.DT_WIND)] if config.DT_WIND > 0 else ticks  # Avoid division by 0
            axs[i].set_xticks(ticks)
            axs[i].tick_params(axis='x', labelrotation=0)
            axs[i].grid(axis='x')
        except Exception as e:
            print(f"Could not set x-axis ticks for subplot {i}: {e}")

    # Remove the last subplot (bottom-right corner)
    axs[17].axis('off')

    fig1.suptitle('LI-550 Sensor Measurements')
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(csv_path.parent / f"plot_li550_data_{timestamp}.png", dpi=300)


def create_video_from_data(csv_path):
    # Read the CSV file
    with open(csv_path, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        data = []
        for row in reader:
            try:
                # Ensure all required values exist and are valid floats
                timestamp = float(row['Unix Timestamp [s]'])
                u = float(row['U Vector [m/s]'])
                v = float(row['V Vector [m/s]'])
                w = float(row['W Vector [m/s]'])
                norm3D = float(row['Wind 3D norm [m/s]'])
                data.append({
                    'timestamp': timestamp,
                    'u': u,
                    'v': v,
                    'w': w,
                    'norm3D': norm3D
                })
            except (ValueError, KeyError):
                continue  # Skip rows with missing or invalid data

    if not data:
        print("No valid data found in CSV.")
        return

    timestamps = [row['timestamp'] for row in data]
    u_values = [row['u'] for row in data]
    v_values = [row['v'] for row in data]
    w_values = [row['w'] for row in data]
    norms_3D = [row['norm3D'] for row in data]
    max_norm = np.max(norms_3D)

    # Extract timestamp from filename
    timestamp = Path(csv_path).stem.replace("data_", "")
    csv_path = Path(csv_path)

    # Setup figure
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlim([-config.WIND_AXIS_LENGTH, config.WIND_AXIS_LENGTH])
    ax.set_ylim([-config.WIND_AXIS_LENGTH, config.WIND_AXIS_LENGTH])
    ax.set_zlim([-config.WIND_AXIS_LENGTH, config.WIND_AXIS_LENGTH])
    ax.set_xlabel('Y [m/s]')
    ax.set_ylabel('X [m/s]')
    ax.set_zlabel('Z [m/s]')
    # ax.set_title('3D wind vectors animation')

    # Create ScalarMappable for the colorbar
    # max_norm = int(max(norms_3D))
    max_norm = 12
    step = 2
    ticks = list(range(0, max_norm, step))
    if ticks[-1] != max_norm:
        ticks.append(max_norm)

    norm = mcolors.Normalize(vmin=0, vmax=max_norm)
    sm = cm.ScalarMappable(cmap='viridis', norm=norm)
    sm.set_array([])  # required for colorbar

    # Create colorbar
    cbar = plt.colorbar(sm, ax=ax, ticks=ticks)
    cbar.ax.set_yticklabels([str(t) for t in ticks])  # Optional
    cbar.set_label('Wind magnitude [m/s]', fontsize=11)
    cbar.ax.tick_params(labelsize=9)

    # Initialize empty list for quivers
    quivers = []

    # Update function for animation
    def update(frame):
        nonlocal quivers
        for q in quivers:
            q.remove()
        quivers = []

        # Plot last config.NUM_PAST_VECTORS vectors up to current frame
        start = max(0, frame - config.NUM_PAST_VECTORS + 1)
        end = frame + 1

        for idx, i in enumerate(range(start, end)):
            color = plt.cm.viridis(norms_3D[i] / max_norm)

            # Dynamically compute alpha from min_alpha to 1.0
            min_alpha = 0.1
            fraction = (idx + 1) / (end - start)
            alpha = min_alpha + (1.0 - min_alpha) * fraction

            q = ax.quiver(0, 0, 0,
                          u_values[i], v_values[i], w_values[i],
                          length=1.0, normalize=False, arrow_length_ratio=0.1, linewidth=2,
                          color=color, alpha=alpha)
            quivers.append(q)

        return quivers

    # Create animation and save it
    ani = animation.FuncAnimation(fig, update, frames=len(timestamps), interval=config.DT_WIND * 1000, blit=False)
    ani.save(csv_path.parent / f"wind_animation_better_{timestamp}.mp4", writer='ffmpeg', fps=1 / config.DT_WIND, dpi=200)

    
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Choose either 'plot_data' or 'create_video_from_data'")
    elif sys.argv[1] == "plot_data":
        if len(sys.argv) >= 3:
            plot_data(sys.argv[2])
        else:
            print("Please provide the path to the CSV file.")
    elif sys.argv[1] == "create_video_from_data":
        if len(sys.argv) >= 3:
            create_video_from_data(sys.argv[2])
        else:
            print("Please provide the path to the CSV file.")
    else:
        print(f"Unknown function: {sys.argv[1]}")