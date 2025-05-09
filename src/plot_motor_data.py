import csv
import math
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
import sys


def plot_data(csv_path):
    # Read data from CSV
    time_vals = []
    velocity_vals = []
    position_vals = []
    torque_vals = []
    tracking_error = []
    voltage_vals = []
    current_vals = []

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            time_vals.append(float(row['Timestamp [s]']))
            velocity_vals.append(float(row['Linear speed [m/s]']))
            position_vals.append(float(row['Linear position [m]']))
            torque_vals.append(float(row['Torque [Nm]']))
            tracking_error.append(float(row['Tracking error [m]']))
            voltage_vals.append(float(row['Voltage [V]']))
            current_vals.append(float(row['Current [A]']))

    # Compute plane_position_vals
    plane_position_vals = []
    for pos, error in zip(position_vals, tracking_error):
        if math.isnan(error):
            plane_position_vals.append(float('nan'))
        else:
            plane_position_vals.append(pos + error)

    # Extract timestamp from filename
    timestamp = Path(csv_path).stem.replace("data_", "")

    # Create one figure with 4 subplots
    fig, axs = plt.subplots(3, 3, figsize=(14, 10))

    # Plot 1: Cart and Plane Position over Time
    axs[0, 0].plot(time_vals, position_vals, label="Cart position", color="tab:red")
    axs[0, 0].plot(time_vals, plane_position_vals, label="Plane position", color="tab:blue")
    axs[0, 0].set_xlabel("Time [s]")
    axs[0, 0].set_ylabel("Position [m]")
    axs[0, 0].yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    axs[0, 0].grid(True)
    axs[0, 0].legend(loc='upper right')
    axs[0, 0].set_title("Position vs Time")

    # Plot 2: Velocity over Time
    axs[0, 1].plot(time_vals, velocity_vals, label="Linear velocity", color="tab:purple")
    axs[0, 1].set_xlabel("Time [s]")
    axs[0, 1].set_ylabel("Velocity [m/s]")
    axs[0, 1].yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    axs[0, 1].grid(True)
    axs[0, 1].legend(loc='upper right')
    axs[0, 1].set_title("Velocity vs Time")

    # Plot 3: Torque over Time
    axs[1, 0].plot(time_vals, torque_vals, label="Torque", color="tab:green")
    axs[1, 0].set_xlabel("Time [s]")
    axs[1, 0].set_ylabel("Torque [Nm]")
    axs[1, 0].yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    axs[1, 0].grid(True)
    axs[1, 0].legend(loc='upper right')
    axs[1, 0].set_title("Torque vs Time")

    # Plot 4: Velocity vs Position
    axs[1, 1].plot(position_vals, velocity_vals, label="Linear velocity", color="tab:purple")
    axs[1, 1].set_xlabel("Position [m]")
    axs[1, 1].set_ylabel("Velocity [m/s]")
    axs[1, 1].yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    axs[1, 1].grid(True)
    axs[1, 1].legend(loc='upper right')
    axs[1, 1].set_title("Velocity vs Position")

    # Plot 5: Voltage over Time
    axs[2, 0].plot(time_vals, voltage_vals, label="Voltage", color="tab:green")
    axs[2, 0].set_xlabel("Time [s]")
    axs[2, 0].set_ylabel("Voltage [V]")
    axs[2, 0].yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    axs[2, 0].grid(True)
    axs[2, 0].legend(loc='upper right')
    axs[2, 0].set_title("Voltage vs Time")
    axs[2, 0].set_ylim(12, 18)

    # Plot 6: Current over Time
    axs[2, 1].plot(time_vals, current_vals, label="Current", color="tab:green")
    axs[2, 1].set_xlabel("Time [s]")
    axs[2, 1].set_ylabel("Current [V]")
    axs[2, 1].yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    axs[2, 1].grid(True)
    axs[2, 1].legend(loc='upper right')
    axs[2, 1].set_title("Current vs Time")


    plt.suptitle(f"System Overview - {timestamp}", fontsize=16)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(csv_path.parent / f"plot_motor_data_{timestamp}.png", dpi=300)
    plt.close(fig)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python script.py <path_to_csv_file>")
    else:
        plot_data(sys.argv[1])