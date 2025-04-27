import csv
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
from pathlib import Path
import math

def plot_data(csv_path):
    # Read data from CSV
    time_vals = []
    velocity_vals = []
    position_vals = []
    torque_vals = []
    tracking_error = []

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            time_vals.append(float(row["run_time (s)"]))
            velocity_vals.append(float(row["linear_speed (m/s)"]))
            position_vals.append(float(row["linear_position (m)"]))
            torque_vals.append(float(row["torque (Nm)"]))
            tracking_error.append(float(row["tracking_error (m)"]))

    # Compute plane_position_vals
    plane_position_vals = []
    for pos, error in zip(position_vals, tracking_error):
        if math.isnan(error):
            plane_position_vals.append(float('nan'))
        else:
            plane_position_vals.append(pos + error)

    # Extract timestamp from filename
    timestamp = Path(csv_path).stem.replace("data_", "")

    # Plot 1: Time Series with 3 subplots
    fig1, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    ax1.plot(time_vals, position_vals, label="Cart position", color="tab:red")
    ax1.plot(time_vals, plane_position_vals, label="Plane position", color="tab:blue")
    ax1.set_ylabel("Position [m]")
    ax1.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    ax1.grid(True)
    ax1.legend(loc='upper right')

    ax2.plot(time_vals, velocity_vals, label="Linear velocity", color="tab:purple")
    ax2.set_ylabel("Linear velocity [m/s]")
    ax2.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    ax2.grid(True)
    ax2.legend(loc='upper right')

    ax3.plot(time_vals, torque_vals, label="Torque", color="tab:green")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Torque [Nm]")
    ax3.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    ax3.grid(True)
    ax3.legend(loc='upper right')

    plt.tight_layout()
    plt.savefig(csv_path.parent / f"data_{timestamp}_plot_time_series.png")

    # Plot 2: Velocity vs Position
    fig2 = plt.figure(figsize=(8, 6))
    ax = plt.gca()
    ax.plot(position_vals, velocity_vals, label="Linear velocity", color="tab:purple")
    ax.set_xlabel("Position [m]")
    ax.set_ylabel("Linear velocity [m/s]")
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax.grid(True)
    ax.legend(loc='upper right')

    plt.tight_layout()
    plt.savefig(csv_path.parent / f"data_{timestamp}_plot_velocity_position.png")

    # Plot 3: Tracking position error
    fig3 = plt.figure(figsize=(8, 6))
    ax = plt.gca()
    ax.plot(time_vals, tracking_error, label="Tracking error", color="tab:blue")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Tracking error [m]")
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    ax.grid(True)
    ax.legend(loc='upper right')

    plt.tight_layout()
    plt.savefig(csv_path.parent / f"data_{timestamp}_plot_tracking_error.png")