from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib as mpl
from scipy.spatial.transform import Rotation as R
from scipy.signal import savgol_filter


# DATA PATHS
CSV_DATA_PATH      = Path("data/run_2025-10-09_16-02-26/data_2025-10-09_16-02-32.csv")
CSV_WIND_DATA_PATH = Path("data/run_2025-10-09_16-02-26/wind_data_2025-10-09_16-02-26.csv")

# PAREMETERS FOR QUATERNION CLEANING
MAX_RATE_DEG_S = 200.0 # maximum allowed angular rate in degrees per second (empirically chosen)
SLACK_DEG      = 10.0  # additional slack in degrees to allow for temporary spikes in angular rate without classifying them as outliers (empirically chosen)
TAU_S          = 0.25  # time constant for quaternion smoothing (empirically chosen)

# PARAMETERS FOR WIND FILTERING
WINDOW_LENGTH = 15 # length of the filter window
ORDER_POLY    = 2  # order of the polynomial used to fit the samples

# RUN INDECES FOR PLOTTING
START_INDEX = 1340 # 0 for the full run 
END_INDEX   = 1440 # np.min([len(pd.read_csv(CSV_DATA_PATH)), len(pd.read_csv(CSV_WIND_DATA_PATH))]) for the full run


def read_all_data_and_interpolate() -> pd.DataFrame:
    """
    Read the system data and wind data CSV files, sort by timestamp, and interpolate the wind data onto the system data timestamps.
    """
    df_data = pd.read_csv(CSV_DATA_PATH)
    df_wind = pd.read_csv(CSV_WIND_DATA_PATH)

    # Sort by timestamp
    df_data = df_data.sort_values("Timestamp [s]").reset_index(drop=True)
    df_wind = df_wind.sort_values("Timestamp [s]").reset_index(drop=True)

    # Interpolate wind columns onto system timestamps
    ts_data = df_data["Timestamp [s]"].to_numpy()
    ts_wind = df_wind["Timestamp [s]"].to_numpy()

    df_all = df_data.copy()

    wind_cols = [col for col in df_wind.columns if col != "Timestamp [s]"]

    for col in wind_cols:
        df_all[col] = np.interp(
            ts_data,
            ts_wind,
            df_wind[col].to_numpy(),
        )
    
    print(len(df_all), len(df_data), len(df_wind))
    return df_all


def _normalize_quat(q: np.ndarray) -> np.ndarray:
    """
    Normalize a quaternion to unit length.
    """
    n = np.linalg.norm(q)
    if n <= 0:
        return q
    return q / n


def _quat_dot(q1: np.ndarray, q2: np.ndarray) -> float:
    """
    Compute the dot product of two quaternions.
    """
    return float(np.dot(q1, q2))


def _quat_angle_deg(q1: np.ndarray, q2: np.ndarray) -> float:
    """
    Compute the angle in degrees between two quaternions.
    """
    d = abs(_quat_dot(q1, q2))
    d = float(np.clip(d, -1.0, 1.0))
    return float(np.degrees(2.0 * np.arccos(d)))


def _slerp(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
    """
    Perform Spherical Linear Interpolation (SLERP) between two quaternions q0 and q1 to remove outliers (from ArUco anbiguity). 
    The parameter alpha is the interpolation factor in [0, 1], where 0 corresponds to q0 and 1 corresponds to q1.
    """
    q0 = _normalize_quat(q0)
    q1 = _normalize_quat(q1)

    dot = _quat_dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    dot = float(np.clip(dot, -1.0, 1.0))

    if dot > 0.9995:
        q = q0 + alpha * (q1 - q0)
        return _normalize_quat(q)

    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * alpha
    sin_theta = np.sin(theta)

    s0 = np.sin(theta_0 - theta) / sin_theta_0
    s1 = sin_theta / sin_theta_0
    q = s0 * q0 + s1 * q1
    return _normalize_quat(q)


def quat_xyzw_to_euler_zyx_deg(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    """
    Convert a quaternion (qx, qy, qz, qw) to Euler angles (roll, pitch, yaw) in degrees.
    """
    q = _normalize_quat(np.array([qx, qy, qz, qw], dtype=float))
    qx, qy, qz, qw = q.tolist()

    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = np.sign(sinp) * (np.pi / 2.0)
    else:
        pitch = np.arcsin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return (
        float(np.degrees(roll)),
        float(np.degrees(pitch)),
        float(np.degrees(yaw)),
    )


def clean_quaternion_series(df_all: pd.DataFrame, max_rate_deg_s: float, slack_deg: float, tau_s: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Clean a quaternion time series on the full timestamp grid.
    Missing samples (NaN quaternions) and detected outliers are both treated as invalid samples and filled by SLERP between neighboring valid samples.
    """
    t_data = df_all["Timestamp [s]"].to_numpy()
    q_raw = df_all[["qx", "qy", "qz", "qw"]].to_numpy()
    n = len(t_data)
    q = np.full((n, 4), np.nan, dtype=float)

    is_missing = np.isnan(q_raw).any(axis=1)
    is_valid_raw = ~is_missing

    # Normalize only valid raw quaternions
    for i in range(n):
        if is_valid_raw[i]:
            q[i] = _normalize_quat(q_raw[i])

    # Sign continuity over valid samples only
    valid_idx = np.where(is_valid_raw)[0]
    for k in range(1, len(valid_idx)):
        i_prev = valid_idx[k - 1]
        i_curr = valid_idx[k]
        if _quat_dot(q[i_prev], q[i_curr]) < 0.0:
            q[i_curr] = -q[i_curr]

    # Outlier detection on consecutive valid samples
    is_out = np.zeros(n, dtype=bool)
    for k in range(1, len(valid_idx)):
        i_prev = valid_idx[k - 1]
        i_curr = valid_idx[k]

        dt = float(t_data[i_curr] - t_data[i_prev])
        if dt <= 0.0:
            is_out[i_curr] = True
            continue

        ang = _quat_angle_deg(q[i_prev], q[i_curr])
        allowed = max_rate_deg_s * dt + slack_deg
        if ang > allowed:
            is_out[i_curr] = True

    # Invalid = missing or outlier
    is_invalid = is_missing | is_out
    is_valid = ~is_invalid

    q_filled = q.copy()

    valid_idx = np.where(is_valid)[0]
    if len(valid_idx) == 0:
        return q_filled, is_out, is_missing

    if len(valid_idx) == 1:
        q_filled[:] = q_filled[valid_idx[0]]
        return q_filled, is_out, is_missing

    first = int(valid_idx[0])
    last = int(valid_idx[-1])

    # Extend boundaries
    for i in range(0, first):
        q_filled[i] = q_filled[first]
    for i in range(last + 1, n):
        q_filled[i] = q_filled[last]

    # Fill interior invalid gaps with SLERP
    i = first
    while i <= last:
        if is_valid[i]:
            i += 1
            continue

        a = i - 1
        b = i
        while b <= last and not is_valid[b]:
            b += 1

        ta = float(t_data[a])
        tb = float(t_data[b])

        for k in range(a + 1, b):
            tk = float(t_data[k])
            frac = 0.0 if tb == ta else (tk - ta) / (tb - ta)
            q_filled[k] = _slerp(q_filled[a], q_filled[b], float(frac))

        i = b + 1

    # Low-pass smoothing
    q_smooth = q_filled.copy()
    q_smooth[0] = _normalize_quat(q_smooth[0])

    for i in range(1, n):
        dt = float(t_data[i] - t_data[i - 1])
        if dt <= 0.0:
            q_smooth[i] = q_smooth[i - 1]
            continue
        alpha = 1.0 - np.exp(-dt / tau_s)
        q_smooth[i] = _slerp(q_smooth[i - 1], q_filled[i], float(alpha))

    return q_smooth, is_out, is_missing


def _interp_1d_over_time(t: np.ndarray, x: np.ndarray,) -> np.ndarray:
    """
    Linearly interpolate missing values of a 1D signal over time.
    Missing values at the beginning or end are extended with the nearestvalid sample.
    """
    valid = np.isfinite(x) & np.isfinite(t)

    if valid.sum() == 0:
        return np.full_like(x, np.nan, dtype=float)

    if valid.sum() == 1:
        out = np.empty_like(x, dtype=float)
        out[:] = x[valid][0]
        return out

    return np.interp(t, t[valid], x[valid])


def compute_uav_velocity_in_wasp_frame(df_all: pd.DataFrame) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Compute UAV velocity in WASP frame by first interpolating missing ArUco
    positions, then differentiating with respect to time.
    """
    t = df_all["Timestamp [s]"].to_numpy(dtype=float)
    x = df_all["tvec_x [m]"].to_numpy(dtype=float)
    y = df_all["tvec_y [m]"].to_numpy(dtype=float)
    z = df_all["tvec_z [m]"].to_numpy(dtype=float)

    x_i = _interp_1d_over_time(t, x)
    y_i = _interp_1d_over_time(t, y)
    z_i = _interp_1d_over_time(t, z)

    vx = np.gradient(x_i, t)
    vy = np.gradient(y_i, t)
    vz = np.gradient(z_i, t)

    return vx, vy, vz


def compute_wind_at_uav(df_all: pd.DataFrame) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Compute the wind at the UAV by rotating the wind vector from the WASP frame to the UAV frame using the UAV attitude from ArUco, 
    and then subtracting the UAV velocity in the WASP frame to account for the UAV motion relative to the WASP.
    Filter the resulting wind at the UAV to smooth out high-frequency noise from the ArUco differentiation.
    """
    wind_u = - df_all["V Vector [m/s]"].to_numpy()    # minus sign because going in the negative x direction
    wind_v = df_all["U Vector [m/s]"].to_numpy()
    wind_w = df_all["W Vector [m/s]"].to_numpy()
    roll_uav = df_all["roll_uav [deg]"].to_numpy()
    pitch_uav = df_all["pitch_uav [deg]"].to_numpy()
    yaw_uav = df_all["yaw_uav [deg]"].to_numpy()
    vel_uav_x = df_all["aruco_x_vel [m/s]"].to_numpy()
    vel_uav_y = df_all["aruco_y_vel [m/s]"].to_numpy()
    vel_uav_z = df_all["aruco_z_vel [m/s]"].to_numpy()
    
    wind_at_wasp = np.empty((len(df_all), 3), dtype=float)
    wind_at_wasp[:, 0] = wind_u
    wind_at_wasp[:, 1] = wind_v
    wind_at_wasp[:, 2] = wind_w

    wind_at_uav = np.empty_like(wind_at_wasp)

    for i in range(len(df_all)):
        rot = R.from_euler("xyz", [np.deg2rad(roll_uav[i]), np.deg2rad(pitch_uav[i]), np.deg2rad(yaw_uav[i])])

        uav_motion = np.array([vel_uav_x[i], vel_uav_y[i], vel_uav_z[i]], dtype=float)

        wind_at_uav[i] = rot.apply(wind_at_wasp[i] - uav_motion)
    
    # Apply filter to smooth signals
    wind_at_uav[:, 0] = savgol_filter(wind_at_uav[:, 0], WINDOW_LENGTH, ORDER_POLY, mode="interp")
    wind_at_uav[:, 1] = savgol_filter(wind_at_uav[:, 1], WINDOW_LENGTH, ORDER_POLY, mode="interp")
    wind_at_uav[:, 2] = savgol_filter(wind_at_uav[:, 2], WINDOW_LENGTH, ORDER_POLY, mode="interp")

    # Flipping the x-wind because of the convention of the WASP frame where the wind is measured in the opposite direction of the UAV motion
    wind_at_uav[:, 0] = - wind_at_uav[:, 0]

    return wind_at_uav[:, 0], wind_at_uav[:, 1], wind_at_uav[:, 2]


def compute_aerodynamic_quantities(df_all: pd.DataFrame) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Compute aerodynamic quantities such as angle of attack, angle of sideslip and airspeed from the estimated wind at the UAV.
    """
    wind_uav_x = df_all["wind_uav_x [m/s]"].to_numpy()
    wind_uav_y = df_all["wind_uav_y [m/s]"].to_numpy()
    wind_uav_z = df_all["wind_uav_z [m/s]"].to_numpy()

    airspeed = np.sqrt(wind_uav_x**2 + wind_uav_y**2 + wind_uav_z**2)
    alpha = np.rad2deg(np.arctan2(wind_uav_z, wind_uav_x))
    beta = np.rad2deg(np.arcsin(wind_uav_y / airspeed))
    
    return airspeed, alpha, beta


def plot_all_data(df_all: pd.DataFrame):
    """
    Plot all the relevant data columns in df_all against time.
    """
    time = df_all["Timestamp [s]"].to_numpy()
    time = time - time[START_INDEX] # shift time to start at 0 for better visualization
    
    fig, axes = plt.subplots(5, 1, figsize=(7, 8), sharex=True)
    colormap_twi = mpl.colormaps.get_cmap("twilight")
    colormap_civ = mpl.colormaps.get_cmap("cividis")
    colormap_inf = mpl.colormaps.get_cmap("inferno")

    # POSITION PLOT --------------------------------------------------------------------------
    pos_wasp_x = df_all["Linear position [m]"].to_numpy()
    pos_wasp_x = np.max(pos_wasp_x) - pos_wasp_x # WASP moving in the negative x direction, so flip the sign to have it increasing in the positive x direction for better visualization
    pos_uav_vision_x = pos_wasp_x + df_all["tvec_x [m]"].to_numpy()
    est_pos_uav_x = np.max(pos_wasp_x) - df_all["Estimated plane position [m]"].to_numpy()

    axes[0].plot(time[START_INDEX:END_INDEX], pos_wasp_x[START_INDEX:END_INDEX],       color="black",           linewidth=1.5, label="WASP")
    axes[0].plot(time[START_INDEX:END_INDEX], est_pos_uav_x[START_INDEX:END_INDEX],    color=colormap_twi(0.2), linewidth=1.5, label="UAV predicted")
    axes[0].plot(time[START_INDEX:END_INDEX], pos_uav_vision_x[START_INDEX:END_INDEX], color=colormap_twi(0.7), linewidth=1.5, label="UAV through vision")
    axes[0].set_ylabel("Position [m]", fontsize=8)
    axes[0].set_title("Position of WASP and UAV (predicted and through vision)", fontsize=10)
    axes[0].legend(loc="upper left", fontsize=8)    

    # MEASURED WIND AND VELOCITY PLOT -------------------------------------------------------
    wind_u = - df_all["V Vector [m/s]"].to_numpy()    # minus sign because going in the negative x direction
    wind_v = df_all["U Vector [m/s]"].to_numpy()
    wind_w = df_all["W Vector [m/s]"].to_numpy()
    vel_x = - df_all["Linear speed [m/s]"].to_numpy() # minus sign because going in the negative x direction

    axes[1].plot(time[START_INDEX:END_INDEX], wind_u[START_INDEX:END_INDEX], color=colormap_civ(0.8), linewidth=1.5, label="Raw x-wind")
    axes[1].plot(time[START_INDEX:END_INDEX], wind_v[START_INDEX:END_INDEX], color=colormap_civ(0.5), linewidth=1.5, label="Raw y-wind")
    axes[1].plot(time[START_INDEX:END_INDEX], wind_w[START_INDEX:END_INDEX], color=colormap_civ(0.2), linewidth=1.5, label="Raw z-wind")
    axes[1].plot(time[START_INDEX:END_INDEX], vel_x[START_INDEX:END_INDEX],  color="black",           linewidth=1.5, label="WASP speed")
    axes[1].set_ylabel("Amplitude [m/s]", fontsize=8)
    axes[1].set_title("Measured wind and WASP speed", fontsize=10)
    axes[1].legend(loc="upper left", fontsize=8)   

    # UAV ATTITUDE PLOT ----------------------------------------------------------------------
    roll_uav = df_all["roll_uav [deg]"].to_numpy()
    pitch_uav = df_all["pitch_uav [deg]"].to_numpy()
    yaw_uav = df_all["yaw_uav [deg]"].to_numpy()

    axes[2].plot(time[START_INDEX:END_INDEX], roll_uav[START_INDEX:END_INDEX],  color=colormap_inf(0.8), linewidth=1.5, label="UAV roll")
    axes[2].plot(time[START_INDEX:END_INDEX], pitch_uav[START_INDEX:END_INDEX], color=colormap_inf(0.5), linewidth=1.5, label="UAV pitch")
    axes[2].plot(time[START_INDEX:END_INDEX], yaw_uav[START_INDEX:END_INDEX],   color=colormap_inf(0.2), linewidth=1.5, label="UAV yaw")
    axes[2].set_ylabel("Attitude [°]", fontsize=8)
    axes[2].set_title("Attitude of the UAV (from vision)", fontsize=10)
    axes[2].legend(loc="upper left", fontsize=8)

    # WIND AT UAV PLOT ----------------------------------------------------------------------
    wind_uav_x = df_all["wind_uav_x [m/s]"].to_numpy()
    wind_uav_y = df_all["wind_uav_y [m/s]"].to_numpy()
    wind_uav_z = df_all["wind_uav_z [m/s]"].to_numpy()

    axes[3].plot(time[START_INDEX:END_INDEX], wind_uav_x[START_INDEX:END_INDEX], color=colormap_civ(0.8), linewidth=1.5, label="x-wind at UAV")
    axes[3].plot(time[START_INDEX:END_INDEX], wind_uav_y[START_INDEX:END_INDEX], color=colormap_civ(0.5), linewidth=1.5, label="y-wind at UAV")
    axes[3].plot(time[START_INDEX:END_INDEX], wind_uav_z[START_INDEX:END_INDEX], color=colormap_civ(0.2), linewidth=1.5, label="z-wind at UAV")
    axes[3].set_ylabel("Amplitude [m/s]", fontsize=8)
    axes[3].set_title("Wind at the UAV (from vision and WASP wind)", fontsize=10)
    axes[3].legend(loc="upper left", fontsize=8)

    # AERODYNAMIC QUANTITIES PLOT -----------------------------------------------------------
    airspeed = df_all["airspeed [m/s]"].to_numpy()
    alpha = df_all["alpha [deg]"].to_numpy() 
    beta = df_all["beta [deg]"].to_numpy()

    ax_left = axes[4]
    ax_right = ax_left.twinx()
    ax_left.plot(time[START_INDEX:END_INDEX], airspeed[START_INDEX:END_INDEX], color="green", alpha=0.7, linewidth=1.5, label="Airspeed est.")
    ax_right.plot(time[START_INDEX:END_INDEX], alpha[START_INDEX:END_INDEX],   color="blue",  alpha=0.7, linewidth=1.5, label="Angle of attack est.")
    ax_right.plot(time[START_INDEX:END_INDEX], beta[START_INDEX:END_INDEX],    color="red",   alpha=0.7, linewidth=1.5, label="Angle of sideslip est.")
    ax_left.set_ylabel("Airspeed [m/s]", fontsize=8)
    ax_right.set_ylabel("Airflow angle [°]", fontsize=8)
    axes[4].set_title("Estimated aerodynamic quantities of UAV", fontsize=10)
    l1, lab1 = ax_left.get_legend_handles_labels()
    l2, lab2 = ax_right.get_legend_handles_labels()
    ax_left.legend(l1 + l2, lab1 + lab2, fontsize=8, ncol=1, loc="upper left")

    for ax in axes:
        ax.grid(True, linestyle="-.", color="#BBBBBB", linewidth=0.4)
    plt.xlabel("Time [s]", fontsize=8)
    plt.tight_layout()
    plt.show()


def main():
    # Read and interpolate data
    df_all = read_all_data_and_interpolate()

    # Clean the ArUco quaternions and detect outliers based on maximum allowed angular rate, fill outliers using SLERP interpolation, and apply low-pass smoothing in quaternion space    
    quat_uav_clean, is_out, is_missing = clean_quaternion_series(df_all, MAX_RATE_DEG_S, SLACK_DEG, TAU_S)

    # Compute the attitude of the UAV (Euler angles) from the ArUco quaternions
    roll_uav = np.zeros(len(df_all), dtype=float)
    pitch_uav = np.zeros(len(df_all), dtype=float)
    yaw_uav = np.zeros(len(df_all), dtype=float)

    for i, (qx, qy, qz, qw) in enumerate(quat_uav_clean):
        roll_uav[i], pitch_uav[i], yaw_uav[i] = (quat_xyzw_to_euler_zyx_deg(qx, qy, qz, qw))

    # Euler angles are returned in a wrapped principal range (typically [-180, 180]): unwrapping restores temporal continuity for visualization.
    # A fixed +180° roll offset is then applied to account for the ArUco/camera-to-UAV frame convention.
    roll_uav_unwrapped = np.rad2deg(np.unwrap(np.deg2rad(roll_uav)) + np.pi)

    df_all["roll_uav [deg]"] = roll_uav_unwrapped
    df_all["pitch_uav [deg]"] = pitch_uav
    df_all["yaw_uav [deg]"] = yaw_uav

    # Compute the UAV velocity in the WASP frame by differentiating the ArUco position
    aruco_x_vel, aruco_y_vel, aruco_z_vel = compute_uav_velocity_in_wasp_frame(df_all)
    df_all["aruco_x_vel [m/s]"] = aruco_x_vel
    df_all["aruco_y_vel [m/s]"] = aruco_y_vel
    df_all["aruco_z_vel [m/s]"] = aruco_z_vel

    # Compute wind at UAV
    wind_uav_x, wind_uav_y, wind_uav_z = compute_wind_at_uav(df_all)
    df_all["wind_uav_x [m/s]"] = wind_uav_x
    df_all["wind_uav_y [m/s]"] = wind_uav_y
    df_all["wind_uav_z [m/s]"] = wind_uav_z

    # Compute airspeed, angle of attack and angle of sideslip of the UAV
    airspeed, alpha, beta = compute_aerodynamic_quantities(df_all)
    df_all["airspeed [m/s]"] = airspeed
    df_all["alpha [deg]"] = alpha
    df_all["beta [deg]"] = beta

    # Plot all the data
    plot_all_data(df_all)


if __name__ == "__main__":
    main()