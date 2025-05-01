import csv
import matplotlib.pyplot as plt
import random
from matplotlib.ticker import MaxNLocator, FuncFormatter
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.animation as animation
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
import sys

import config

def plot_data(csv_path):
    # Read the CSV file
    with open(csv_path, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        data = [row for row in reader]
    
    # First figure
    fig1, axs = plt.subplots(3, 6, figsize=(14, 8))
    axs = axs.flatten()

    # Tags to plot and their y-axis labels
    fields_with_units = {
        'S': 'Wind 3D norm [m/s]',
        'S2': 'Wind 2D norm [m/s]',
        'D': 'Horizontal wind direction [°]',
        'DV': 'Vertical wind direction [°]',
        'U': 'U Vector [m/s]',
        'V': 'V Vector [m/s]',
        'W': 'W Vector [m/s]',
        'T': 'Temperature [°C]',
        'C': 'Speed of sound [m/s]',
        'H': 'Humidity [%]',
        'DP': 'Dew point [°C]',
        'P': 'Pressure [hPa]',
        'AD': 'Air density [kg/cm³]', 
        'PI': 'Pitch [°]',
        'RO': 'Roll [°]',
        'MD': 'Heading [°]',
        'TD': 'TrueHead [°]'
    }

    # Generate a random color for each field
    def generate_random_color():
        return [random.random() for _ in range(3)]  # RGB values between 0 and 1

    for idx, (field, name) in enumerate(fields_with_units.items()):
        if field in data[0]:
            timestamps = [float(row['Timestamp']) for row in data]
            values = [float(row[field]) for row in data]
            random_color = generate_random_color()  # Generate a random color for each plot
            axs[idx].plot(timestamps, values, color=random_color)
            axs[idx].set_ylabel(f"{name}")
            axs[idx].grid(True)
            axs[idx].tick_params(axis='x', labelrotation=0)  # Ensure labels are horizontal

            # Force one decimal for y-ticks using FuncFormatter
            axs[idx].yaxis.set_major_formatter(FuncFormatter(lambda val, pos: f'{val:.1f}'))

            # Remove x-axis ticks for all plots except the bottom row
            if idx < 12:  # First 12 plots (non-bottom row)
                axs[idx].set_xticks([])

    # Set the x-axis labels only for the bottom row (indices 12 to 17)
    for i in range(12, 17):  # Adjusted to avoid the last subplot
        axs[i].set_xlabel("Time [s]")
        
        # Extract the first decimal of the timestamp for ticks
        timestamps = [float(row['Timestamp']) for row in data]
        
        # Create ticks every 5th timestamp value
        ticks = sorted(set([round(t, 1) for t in timestamps]))  # Remove duplicates and sort
        
        # Get only every 5th timestamp for the ticks
        ticks = ticks[::10]  # Take every 10th value
        
        axs[i].set_xticks(ticks)  # Set x-axis ticks to the first decimal of the timestamp
        axs[i].tick_params(axis='x', labelrotation=0)  # Ensure labels are horizontal

        # Remove vertical grid lines for the bottom subplots (indices 12 to 16)
        axs[i].grid(axis='x')  # Remove grid lines from the y-axis

    # Remove the last subplot (bottom-right corner)
    axs[17].axis('off')

    fig1.suptitle('LI-550 Sensor Measurements')
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig('li550_data_plot.png', dpi=300)


def create_video_from_data(csv_path):
    # Read the CSV file
    with open(csv_path, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        data = [row for row in reader]

    timestamps = [float(row['Timestamp']) for row in data]
    u_values = [float(row['U']) for row in data]
    v_values = [float(row['V']) for row in data]
    w_values = [float(row['W']) for row in data]
    norms_3D = [float(row['S']) for row in data]
    max_norm = np.max(norms_3D)

    # Setup figure
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlim([-config.WIND_AXIS_LENGTH, config.WIND_AXIS_LENGTH])
    ax.set_ylim([-config.WIND_AXIS_LENGTH, config.WIND_AXIS_LENGTH])
    ax.set_zlim([-config.WIND_AXIS_LENGTH, config.WIND_AXIS_LENGTH])
    ax.set_xlabel('U [m/s]')
    ax.set_ylabel('V [m/s]')
    ax.set_zlabel('W [m/s]')
    ax.set_title('3D wind vectors animation')

    # Create ScalarMappable for the colorbar
    norm = Normalize(vmin=min(norms_3D), vmax=max(norms_3D))
    sm = ScalarMappable(cmap='viridis', norm=norm)
    sm.set_array([])

    # Add the colorbar
    cbar = plt.colorbar(sm, ax=ax, pad=0.1)
    cbar.set_label('Wind magnitude [m/s]')

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
                        length=1.0, normalize=False, arrow_length_ratio=0.1, linewidth = 2,
                        color=color, alpha=alpha)
            quivers.append(q)

        return quivers

    # Create animation and save it
    ani = animation.FuncAnimation(fig, update, frames=len(timestamps), interval=config.DT * 1000, blit=False)
    ani.save('wind_vectors_animation_2.mp4', writer='ffmpeg', fps=1/config.DT, dpi=300)
    

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