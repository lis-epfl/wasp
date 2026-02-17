import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.ticker import MultipleLocator, FormatStrFormatter
import re
import os
import numpy as np
import matplotlib.colors as mcolors
import csv


# Enable LaTeX rendering
mpl.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",  # or "sans-serif" or another family
    "font.serif": ["Computer Modern Roman"],  # the default LaTeX font
    "axes.unicode_minus": False  # ensure minus sign renders correctly with LaTeX
})


# Data
file_path_1ms_1m = "src/Figures/vision_figure/vision_data/run_2026-01-07_12-16-14_1m_1ms_no/data_2026-01-07_12-16-20.csv"
file_path_1ms_2m = "src/Figures/vision_figure/vision_data/run_2026-01-07_12-25-13_2m_1ms_no/data_2026-01-07_12-25-19.csv"
file_path_1ms_3m = "src/Figures/vision_figure/vision_data/run_2026-01-07_12-35-36_3m_1ms_no/data_2026-01-07_12-35-42.csv"
file_path_1ms_4m = "src/Figures/vision_figure/vision_data/run_2026-01-07_12-46-58_4m_1ms_no/data_2026-01-07_12-47-04.csv"

file_path_14ms_1m = "src/Figures/vision_figure/vision_data/run_2026-01-07_12-13-08_1m_14ms_no/data_2026-01-07_12-13-14.csv"
file_path_14ms_2m = "src/Figures/vision_figure/vision_data/run_2026-01-07_12-27-46_2m_14ms_no/data_2026-01-07_12-27-52.csv"
file_path_14ms_3m = "src/Figures/vision_figure/vision_data/run_2026-01-07_12-34-24_3m_14ms_no/data_2026-01-07_12-34-30.csv"
file_path_14ms_4m = "src/Figures/vision_figure/vision_data/run_2026-01-07_12-49-35_4m_14ms_no/data_2026-01-07_12-49-41.csv"


# Arrays
heights = [1, 2, 3, 4]

pos_1ms_1m, bool_1ms_1m, error_1ms_1m = [], [], []
pos_1ms_2m, bool_1ms_2m, error_1ms_2m = [], [], []
pos_1ms_3m, bool_1ms_3m, error_1ms_3m = [], [], []
pos_1ms_4m, bool_1ms_4m, error_1ms_4m = [], [], []
offsets_1ms = [27.0, 27.0, 30.0, 30.0]
max_index_1ms = [350, 500, 350, 450] # indices to only consider one way (not the return trip)

pos_14ms_1m, bool_14ms_1m, error_14ms_1m = [], [], []
pos_14ms_2m, bool_14ms_2m, error_14ms_2m = [], [], []
pos_14ms_3m, bool_14ms_3m, error_14ms_3m = [], [], []
pos_14ms_4m, bool_14ms_4m, error_14ms_4m = [], [], []
offsets_14ms = [28.0, 29.0, 29.0, 31.0]
max_index_14ms = [75, 75, 75, 75] # indices to only consider one way (not the return trip)


# Helper functions
def read_csv(file_path, pos_list, bool_list, error_list):
    with open(file_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            pos_list.append(float(row["Linear position [m]"]))
            
            if row["x ArUco [m]"] != "":
                bool_list.append(True)
                error_list.append(float(row["x ArUco [m]"]))
            else:
                bool_list.append(False)
                error_list.append(np.nan)

def only_first_run(pos_list, bool_list, error_list, max_index):
    # Only consider one way (not the return trip)
    pos_list[:] = pos_list[0:max_index]
    bool_list[:] = bool_list[0:max_index]
    error_list[:] = error_list[0:max_index]

def compute_error(pos_list, error_list):
    # Find minimum error and its index
    error = np.abs(np.array(error_list))
    mask = ~np.isnan(error)                
    min_val = np.min(error[mask])
    idx = np.where(error == min_val)[0][0]   

    # Compute true tripod position
    true_tripod_pos = pos_list[idx] - error_list[idx]

    # Recompute error with respect to true tripod position
    for i in range(len(pos_list)):
        if not np.isnan(error_list[i]):
            error_list[i] = np.abs((pos_list[i] - error_list[i]) - true_tripod_pos)
        else:
            error_list[i] = np.nan
    print(error_list)

def plot_row(ax, xs, oks, errs, y, norm, cmap="viridis"):
    xs = np.asarray(xs)
    oks = np.asarray(oks, dtype=bool)
    errs = np.asarray(errs, dtype=float)

    m_valid = oks & np.isfinite(errs)          # “black dots” with a real error
    m_nan = ~m_valid                           # keep as white/empty

    mappable = ax.scatter(
        xs[m_valid], np.full(m_valid.sum(), y),
        c=errs[m_valid], cmap=cmap, norm=norm,
        s=10, linewidths=0.15, edgecolors='face'
    )
    ax.scatter(
        xs[m_nan], np.full(m_nan.sum(), y),
        facecolors="white", edgecolors="black",
        s=10, linewidths=0.15
    )
    return mappable

# Read and process data
for i, (file_path, pos_list, bool_list, error_list) in enumerate([
    (file_path_1ms_1m,  pos_1ms_1m,  bool_1ms_1m,  error_1ms_1m),
    (file_path_1ms_2m,  pos_1ms_2m,  bool_1ms_2m,  error_1ms_2m),
    (file_path_1ms_3m,  pos_1ms_3m,  bool_1ms_3m,  error_1ms_3m),
    (file_path_1ms_4m,  pos_1ms_4m,  bool_1ms_4m,  error_1ms_4m),
    (file_path_14ms_1m, pos_14ms_1m, bool_14ms_1m, error_14ms_1m),
    (file_path_14ms_2m, pos_14ms_2m, bool_14ms_2m, error_14ms_2m),
    (file_path_14ms_3m, pos_14ms_3m, bool_14ms_3m, error_14ms_3m),
    (file_path_14ms_4m, pos_14ms_4m, bool_14ms_4m, error_14ms_4m),
]):
    read_csv(file_path, pos_list, bool_list, error_list)

    if i < 4:
        only_first_run(pos_list, bool_list, error_list, max_index_1ms[i])
    else:
        only_first_run(pos_list, bool_list, error_list, max_index_14ms[i - 4])

    compute_error(pos_list, error_list)

# !!!!!
# Taking the return trip for pos_14ms_3m becaues bad data in the first half of the run (probably due to a bad initial marker detection)
pos_14ms_3m, bool_14ms_3m, error_14ms_3m = [], [], [] # reset to read the full data again
read_csv(file_path_14ms_3m, pos_14ms_3m, bool_14ms_3m, error_14ms_3m)
pos_14ms_3m[:] = pos_14ms_3m[90:130] # only consider the return trip
bool_14ms_3m[:] = bool_14ms_3m[90:130]
error_14ms_3m[:] = error_14ms_3m[90:130]
compute_error(pos_14ms_3m, error_14ms_3m)
# !!!!!

datasets_1ms = [
    (pos_1ms_1m, bool_1ms_1m, error_1ms_1m),
    (pos_1ms_2m, bool_1ms_2m, error_1ms_2m),
    (pos_1ms_3m, bool_1ms_3m, error_1ms_3m),
    (pos_1ms_4m, bool_1ms_4m, error_1ms_4m),
]

datasets_14ms = [
    (pos_14ms_1m, bool_14ms_1m, error_14ms_1m),
    (pos_14ms_2m, bool_14ms_2m, error_14ms_2m),
    (pos_14ms_3m, bool_14ms_3m, error_14ms_3m),
    (pos_14ms_4m, bool_14ms_4m, error_14ms_4m),
]

# Create figure
fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1, figsize=(6, 4), sharex=True)

# One global normalization across both speeds/heights
all_errs = np.concatenate([
    np.asarray(error_1ms_1m, dtype=float),
    np.asarray(error_1ms_2m, dtype=float),
    np.asarray(error_1ms_3m, dtype=float),
    np.asarray(error_1ms_4m, dtype=float),
    np.asarray(error_14ms_1m, dtype=float),
    np.asarray(error_14ms_2m, dtype=float),
    np.asarray(error_14ms_3m, dtype=float),
    np.asarray(error_14ms_4m, dtype=float),
])
norm = mcolors.Normalize(vmin=0, vmax=np.nanmax(all_errs))

# Left subplot: 1 m/s
mappable = None
error_1ms_1m = [error_1ms_1m] # transform to list of lists for zip
for (xs, oks, errs), y, offset in zip(datasets_1ms, heights, offsets_1ms):
    xs_offset = np.asarray(xs) - offset
    mappable = plot_row(ax1, xs_offset, oks, errs, y, norm)

# Left subplot: 14 m/s
mappable = None
error_14ms_1m = [error_14ms_1m] # transform to list of lists for zip
for (xs, oks, errs), y, offset in zip(datasets_14ms, heights, offsets_14ms):
    xs_offset = np.asarray(xs) - offset
    mappable = plot_row(ax2, xs_offset, oks, errs, y, norm)

for ax in (ax1, ax2):
    ax.set_aspect('equal', adjustable='box')  # 1 unit in x == 1 unit in y
    ax.set_yticks(np.arange(0, 5, 1))
    ax.set_ylim(0, 5)
    ax.set_xlim(-5, 5)
    ax.invert_yaxis()
    ax.set_ylabel("Distance to the marker [m]", fontsize=10)
    ax.tick_params(axis='both', which='major', labelsize=8) 
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

ax2.set_xlabel("Position relative to the marker [m]", fontsize=10)

cax = fig.add_axes([0.77, 0.15, 0.02, 0.7])  # [left, bottom, width, height]
cbar = fig.colorbar(
    mappable,
    cax=cax,
    label="Vision based position error [m]",
    ticks=np.linspace(0, np.nanmax(all_errs), num=5),
)
cbar.ax.yaxis.set_major_formatter(mpl.ticker.FormatStrFormatter('%.1f'))
cbar.ax.tick_params(labelsize=8)        
cbar.set_label("Vision based position error [m]", fontsize=10)

plt.tight_layout()
plt.subplots_adjust(hspace=0.3) # add vertical space between the subplots
plt.savefig('src/Figures/vision_figure/vision_figure.png', dpi=500, bbox_inches='tight')
plt.show()
