import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter

# Load CSV
file_name = "data_2025-04-22_17-29-33"
df = pd.read_csv(f"data/{file_name}.csv")

# Extract data
time_vals = df["run_time (s)"]
velocity_vals = df["linear_speed (m/s)"]
position_vals = df["linear_position (m)"]
torque_vals = df["torque (Nm)"]

# --- Plot 1: Time Series with 3 subplots ---
fig1, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

# Position vs Time
ax1.plot(time_vals, position_vals, label="Linear position", color="tab:red")
ax1.set_ylabel("Linear position [m]")
ax1.yaxis.set_major_formatter(FormatStrFormatter('%.1f')) 
ax1.xaxis.set_major_formatter(FormatStrFormatter('%.0f'))
ax1.grid(True)
ax1.legend(loc='upper right')

# Velocity vs Time
ax2.plot(time_vals, velocity_vals, label="Linear velocity", color="tab:purple")
ax2.set_ylabel("Linear velocity [m/s]")
ax2.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
ax2.xaxis.set_major_formatter(FormatStrFormatter('%.0f'))
ax2.grid(True)
ax2.legend(loc='upper right')

# Torque vs Time
ax3.plot(time_vals, torque_vals, label="Torque", color="tab:green")
ax3.set_xlabel("Time [s]")
ax3.set_ylabel("Torque [Nm]")
ax3.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
ax3.xaxis.set_major_formatter(FormatStrFormatter('%.0f'))
ax3.grid(True)
ax3.legend(loc='upper right')

# Save first figure
plt.tight_layout()
plt.savefig(f"data/{file_name}_plot_time_series.png")

# --- Plot 2: Velocity vs Position ---
fig2 = plt.figure(figsize=(8, 6))
ax = plt.gca()
ax.plot(position_vals, velocity_vals, label="Linear velocity", color="tab:purple")
ax.set_xlabel("Position [m]")
ax.set_ylabel("Linear velocity [m/s]")
ax.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
ax.xaxis.set_major_formatter(FormatStrFormatter('%.0f'))
ax.grid(True)
ax.legend(loc='upper right')

# Save second figure
plt.tight_layout()
plt.savefig(f"data/{file_name}_plot_velocity_position.png")