import csv
import math
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
import sys
import numpy as np
import matplotlib as mpl
from matplotlib.patches import Patch

import config   


def plot_data(csv_path):
    # Read data from CSV
    time_vect = []
    velocity_vect = []
    position_vect = []
    torque_vect = []
    voltage_vect = []
    current_vect = []
    x_ref_vect = []
    x_aruco_est_vect = []
    v_aruco_est_vect = []
    vel_ref_vect = []
    x_aruco_vect = []
    y_aruco_vect = []
    z_aruco_vect = []
    roll_aruco_vect = []
    pitch_aruco_vect = []
    yaw_aruco_vect = []

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            time_vect.append(float(row['Timestamp [s]']))
            velocity_vect.append(float(row['Linear speed [m/s]']))
            position_vect.append(float(row['Linear position [m]']))
            torque_vect.append(float(row['Torque [Nm]']))
            voltage_vect.append(float(row['Voltage [V]']))
            current_vect.append(float(row['Current [A]']))
            x_ref_vect.append(float(row['Reference x_ref [m]']))
            x_aruco_est_vect.append(float(row['Estimated plane position [m]']))
            v_aruco_est_vect.append(float(row['Estimated plane velocity [m/s]']))
            vel_ref_vect.append(float(row['Reference velocity [m/s]']))
            x_aruco_vect.append(float(row['x ArUco [m]']))
            y_aruco_vect.append(float(row['y ArUco [m]']))
            z_aruco_vect.append(float(row['z ArUco [m]']))
            roll_aruco_vect.append(float(row['roll ArUco [deg]']))
            pitch_aruco_vect.append(float(row['pitch ArUco [deg]']))
            yaw_aruco_vect.append(float(row['yaw ArUco [deg]']))

    t_max = max(time_vect)

    t_start = time_vect[0]

    # Compute plane_position_vect
    plane_position_vect = []
    for pos, error in zip(position_vect, x_aruco_vect):
        if math.isnan(error):
            plane_position_vect.append(float('nan'))
        else:
            plane_position_vect.append(pos + error)
    
    colormap = mpl.colormaps.get_cmap('magma')
    colormap2 = mpl.colormaps.get_cmap('cividis')
    colormap3 = mpl.colormaps.get_cmap('viridis')

    # Extract timestamp from filename
    timestamp = Path(csv_path).stem.replace("data_", "")

    # Create one figure with 4 subplots
    fig, axs = plt.subplots(3, 3, figsize=(14, 10), sharex=True)


    # Plot 1: Position over time
    valid_mask = ~np.isnan(plane_position_vect)
    regions = []
    in_region = False
    for i, valid in enumerate(valid_mask):
        if valid and not in_region:
            start_idx = i
            in_region = True
        elif not valid and in_region:
            end_idx = i
            regions.append((start_idx, end_idx))
            in_region = False
    # Handle case where last region reaches the end
    if in_region:
        regions.append((start_idx, len(valid_mask)))

    # Now draw shaded areas like you did with label
    for start_idx, end_idx in regions:
        start_time = time_vect[start_idx]
        end_time = time_vect[end_idx-1] if end_idx < len(time_vect) else time_vect[-1] + (time_vect[1] - time_vect[0])
        axs[0, 0].axvspan(start_time, end_time, facecolor='blue', alpha=0.15, zorder=0, edgecolor='none')
    
    axs[0, 0].plot(time_vect, x_aruco_est_vect, label="Estimated UAV position", color="royalblue")
    axs[0, 0].plot(time_vect, position_vect, label="WASP position", color="crimson", linestyle=(0, (5, 1)))
    axs[0, 0].set_ylabel("Position [m]")
    axs[0, 0].set_xlim(t_start, t_max)
    axs[0, 0].yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    axs[0, 0].grid(True)
    axs[0, 0].legend(loc='lower left', fontsize=9)

    shaded_patch = Patch(facecolor='blue', alpha=0.15, label='True UAV position through vision')
    axs[0, 0].legend(handles=[
        shaded_patch,
        axs[0, 0].lines[0],
        axs[0, 0].lines[1],  
    ])


    # Plot 2: Velocity over time
    axs[0, 1].grid(True)
    axs[0, 1].plot(time_vect, v_aruco_est_vect, label="Estimated UAV speed", color="royalblue")
    axs[0, 1].plot(time_vect, velocity_vect, label="WASP speed", color="crimson", linestyle=(0, (5, 1)))
    axs[0, 1].plot(time_vect, vel_ref_vect, label="Reference speed", color="green", linestyle=(0, (5, 1)))
    axs[0, 1].set_ylabel("Speed [m/s]", fontsize=11)
    axs[0, 1].set_xlim(t_start, t_max)
    axs[0, 1].yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    axs[0, 1].legend(loc='lower left', ncol=2, fontsize=9, columnspacing=1.0)


    # Plot 3: Reference over time
    axs[0, 2].plot(time_vect, position_vect, label="WASP position", color="crimson", linestyle=(0, (5, 1)))
    axs[0, 2].plot(time_vect, x_ref_vect, label="Reference", color="green")
    axs[0, 2].set_xlabel("Time [s]")
    axs[0, 2].set_ylabel("Position [m]")
    axs[0, 2].set_xlim(t_start, t_max)
    axs[0, 2].yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    axs[0, 2].grid(True)
    axs[0, 2].legend(loc='lower right')


    # Plot 4: Torque over time
    axs[1, 0].plot(time_vect, torque_vect, color=colormap(0.5))
    axs[1, 0].set_ylabel("Torque [Nm]")
    axs[1, 0].yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    axs[1, 0].set_ylim(-2, 2)
    axs[1, 0].set_xlim(t_start, t_max)
    axs[1, 0].grid(True)


    # Plot 5: Voltage over time
    axs[1, 1].plot(time_vect, voltage_vect, color=colormap(0.6))
    axs[1, 1].set_ylabel("Voltage [V]", fontsize=11)
    axs[1, 1].yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    axs[1, 1].grid(True)
    axs[1, 1].set_ylim(config.MIN_VOLTAGE-5, config.MAX_VOLTAGE+5)
    axs[1, 1].set_xlim(t_start, t_max)
    axs[1, 1].hlines(config.MIN_VOLTAGE, xmin=0, xmax=t_max, color=colormap(0.3), linestyle='--', label='Upper/lower battery voltage limits')
    axs[1, 1].hlines(config.MAX_VOLTAGE, xmin=0, xmax=t_max, color=colormap(0.3), linestyle='--')
    axs[1, 1].legend(loc='upper left', fontsize=9)


    # Plot 6: Current over time
    axs[1, 2].plot(time_vect, current_vect, color=colormap(0.7))
    axs[1, 2].set_xlabel("Time [s]")
    axs[1, 2].set_ylabel("Current [A]", fontsize=11)
    axs[1, 2].yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    axs[1, 2].grid(True)
    axs[1, 2].set_ylim(-config.HARD_MAX_CURRENT-40, config.HARD_MAX_CURRENT+40)
    axs[1, 2].set_xlim(t_start, t_max)
    axs[1, 2].hlines(-config.HARD_MAX_CURRENT, xmin=0, xmax=t_max, color=colormap(0.3), linestyle='--', label='Upper/lower motor current limits')
    axs[1, 2].hlines(config.HARD_MAX_CURRENT, xmin=0, xmax=t_max, color=colormap(0.3), linestyle='--')
    axs[1, 2].legend(loc='upper left', fontsize=9)


    # Plot 7: ArUco position over time
    axs[2, 0].plot(time_vect, x_aruco_vect, label="x ArUco", color=colormap2(0.1))
    axs[2, 0].plot(time_vect, y_aruco_vect, label="y ArUco", color=colormap2(0.5))
    axs[2, 0].plot(time_vect, z_aruco_vect, label="z ArUco", color=colormap2(0.8))
    axs[2, 0].set_ylabel("Position [m]")
    axs[2, 0].set_xlim(t_start, t_max)
    axs[2, 0].yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    axs[2, 0].grid(True)
    axs[2, 0].legend(loc='upper right')


    # Plot 8: ArUco orientation over time
    axs[2, 1].plot(time_vect, roll_aruco_vect, label="roll ArUco", color=colormap3(0.1))
    axs[2, 1].plot(time_vect, pitch_aruco_vect, label="pitch ArUco", color=colormap3(0.4))
    axs[2, 1].plot(time_vect, yaw_aruco_vect, label="yaw ArUco", color=colormap3(0.8))
    axs[2, 1].set_ylabel("Angle [deg]")
    axs[2, 1].set_xlim(t_start, t_max)
    axs[2, 1].yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    axs[2, 1].grid(True)
    axs[2, 1].legend(loc='upper right')


    # Plot 9: 
    axs[2, 2].set_xlabel("Time [s]")
    axs[2, 2].set_xlim(t_start, t_max)


    plt.suptitle(f"System Overview - {timestamp}", fontsize=16)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(csv_path.parent / f"plot_motor_data_{timestamp}.png", dpi=300)
    plt.close(fig)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python script.py <path_to_csv_file>")
    else:
        plot_data(sys.argv[1])