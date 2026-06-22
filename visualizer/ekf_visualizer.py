"""
======================================================================
        AUV EKF Navigation — 6-Group Comparison Plots
    Structure  : plot_groups.py  (6 legend groups, left=raw, right=EKF)
    Style/helpers: ekf_visualizer.py  (colours, loaders, badges)

    Group 1 — Position     (col 1–3):  px_m, py_m, pz_m
    Group 2 — Velocity     (col 4–6):  vx_ms, vy_ms, vz_ms
    Group 3 — Attitude     (col 7–9):  roll_deg, pitch_deg, yaw_deg
    Group 4 — Derived      (col 10–13): speed_horiz_ms, speed_total_ms,
                                        distance_total_m, depth_m
    Group 5 — Uncertainty  (col 14–16): std_pos_m, std_vel_ms, std_att_deg

    Each group → one figure:
        LEFT panel  =  raw sensor data  (BEFORE EKF)
        RIGHT panel =  EKF output       (AFTER  EKF)

    Usage:
        python3 plot_groups.py                   # show windows
        python3 plot_groups.py --save            # save PNG files
        python3 plot_groups.py --save --out plots/
======================================================================
"""

import os
import sys
import argparse
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
import warnings
warnings.filterwarnings("ignore")

# ─────────────────────────────────────────────────────────────
# Global style  (from ekf_visualizer.py)
# ─────────────────────────────────────────────────────────────
BG    = "#0D1117"
PANEL = "#161B22"
BORDER= "#30363D"
GRID  = "#21262D"
TEXT  = "#E6EDF3"
MUTED = "#8B949E"

# Raw sensor colours
COL_IMU_AX = "#FF6B6B";  COL_IMU_AY = "#4ECDC4";  COL_IMU_AZ = "#45B7D1"
COL_GX     = "#FF9F43";  COL_GY     = "#A29BFE";  COL_GZ     = "#FD79A8"
COL_DVL_VX = "#00B4D8";  COL_DVL_VY = "#52B788";  COL_DVL_VZ = "#C77DFF"
COL_DEPTH  = "#48CAE4";  COL_GPS_X  = "#F9C74F";  COL_GPS_Y  = "#90BE6D"
COL_MAG    = "#F3722C";  COL_DROP   = "#FF4757"

# EKF output colours
COL_EKF_VX  = "#0077B6"; COL_EKF_VY  = "#2D6A4F"; COL_EKF_VZ  = "#7B2FBE"
COL_EKF_R   = "#D62839"; COL_EKF_P   = "#E9C46A"; COL_EKF_Y   = "#9B5DE5"
COL_EKF_DEP = "#023E8A"; COL_EKF_PX  = "#E76F51"; COL_EKF_PY  = "#2A9D8F"
COL_EKF_YAW = "#6A0572"; COL_SPD_T   = "#F72585"; COL_SPD_H   = "#7209B7"
COL_EKF_PATH= "#F72585"; COL_GPS_PATH= "#FFD166"

matplotlib.rcParams.update({
    # Figure
    "figure.facecolor": BG,
    "figure.dpi":       120,

    # Axes
    "axes.facecolor":  PANEL,
    "axes.edgecolor":  BORDER,
    "axes.labelcolor": TEXT,
    "axes.titlecolor": TEXT,
    "axes.labelsize":  9,
    "axes.titlesize":  11,

    # Ticks
    "xtick.color":     MUTED,
    "ytick.color":     MUTED,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,

    # Grid
    "grid.color":      GRID,
    "grid.linewidth":  0.5,

    # Text
    "text.color":      TEXT,
    "font.size":       9,

    # Legend
    "legend.facecolor": PANEL,
    "legend.edgecolor": BORDER,
    "legend.fontsize":  8,

    # Lines
    "lines.linewidth": 1.6,
})


# ─────────────────────────────────────────────────────────────
# CSV Loaders
# ─────────────────────────────────────────────────────────────

def load_csv(path, label):
    """
    Load a CSV file and normalize column names.
    """
    if not os.path.isfile(path):
        print(f"  [SKIP] {label}: file not found → {path}")
        return None

    df = pd.read_csv(path, comment="#")
    df.columns = df.columns.str.strip().str.lower()

    print(
        f"  [OK]   {label}: "
        f"{len(df):,} rows, cols={list(df.columns)}"
    )

    return df


def valid_rows(df, col_name="valid"):
    """
    Return only valid rows.

    """
    if df is None:
        return None

    if col_name in df.columns:
        return df[df[col_name] == 1].copy()

    return df.copy()


# ─────────────────────────────────────────────────────────────
# Plot helpers  (from ekf_visualizer.py)
# ─────────────────────────────────────────────────────────────

# ─────────────────────────────────────────────────────────────
# Figure Utilities
# ─────────────────────────────────────────────────────────────

def make_fig(title, rows=3, figsize=None):
    """Create a figure with project dark styling."""
    if figsize is None:
        figsize = (18, 4.2 * rows)

    fig = plt.figure(figsize=figsize, facecolor=BG)

    fig.suptitle(
        title,
        fontsize=14,
        fontweight="bold",
        color=TEXT,
        y=0.995
    )
    return fig

# ─────────────────────────────────────────────────────────────
# Axis Styling
# ─────────────────────────────────────────────────────────────

def style_ax(ax, title="", xlabel="", ylabel="", invert_y=False):
    """Apply consistent axis styling."""
    ax.set_title(title, fontsize=10, fontweight="bold", pad=6)

    if xlabel:
        ax.set_xlabel(xlabel, labelpad=4)

    if ylabel:
        ax.set_ylabel(ylabel, labelpad=4)

    ax.grid(True, alpha=0.4, linewidth=0.5)

    if invert_y:
        ax.invert_yaxis()


def std_band(ax, t, y, sigma, color, alpha=0.18):
    """Draw ±1σ uncertainty band around a line."""
    ax.fill_between(
        t,
        y - sigma,
        y + sigma,
        color=color,
        alpha=alpha,
        linewidth=0,
        label="±1σ"
    )


def dropout_markers(ax, drop_times, color=COL_DROP):
    """Draw thin vertical lines at DVL dropout timestamps."""
    for dt in drop_times:
        ax.axvline(
            dt,
            color=color,
            alpha=0.35,
            lw=0.7,
            zorder=1
        )

    if len(drop_times):
        ax.axvline(
            drop_times[0],
            color=color,
            alpha=0.8,
            lw=1.0,
            label=f"DVL Dropouts ({len(drop_times)})"
        )


def annotate_stats(ax, stats_dict, loc="upper left"):
    """Add a small statistics box to an axis."""
    txt = "\n".join(f"{k}: {v}" for k, v in stats_dict.items())

    props = dict(
        boxstyle="round,pad=0.4",
        facecolor=PANEL,
        edgecolor=BORDER,
        alpha=0.85
    )

    x = 0.02 if "left" in loc else 0.98
    ha = "left" if "left" in loc else "right"

    ax.text(
        x,
        0.97,
        txt,
        transform=ax.transAxes,
        fontsize=7.5,
        va="top",
        ha=ha,
        color=MUTED,
        bbox=props,
        zorder=10
    )


def RAW_tag(ax):
    """Add red BEFORE EKF badge."""
    ax.text(
        0.02,
        0.02,
        "BEFORE EKF",
        transform=ax.transAxes,
        fontsize=8,
        fontweight="bold",
        color="#FF6B6B",
        alpha=0.85,
        va="bottom",
        bbox=dict(
            facecolor=PANEL,
            edgecolor="#FF6B6B",
            pad=3,
            alpha=0.7
        )
    )


def EKF_tag(ax):
    """Add green AFTER EKF badge."""
    ax.text(
        0.02,
        0.02,
        "AFTER EKF",
        transform=ax.transAxes,
        fontsize=8,
        fontweight="bold",
        color="#2ECC71",
        alpha=0.85,
        va="bottom",
        bbox=dict(
            facecolor=PANEL,
            edgecolor="#2ECC71",
            pad=3,
            alpha=0.7
        )
    )


def save_fig(fig, out_dir, name, do_save):
    """Save figure to PNG or keep in memory for plt.show()."""
    fig.tight_layout(rect=[0, 0, 1, 0.96])

    if do_save:
        os.makedirs(out_dir, exist_ok=True)

        path = os.path.join(out_dir, name)

        fig.savefig(
            path,
            dpi=150,
            bbox_inches="tight",
            facecolor=BG
        )

        print(f"   [OK] Saved → {path}")

        plt.close(fig)


# ═══════════════════════════════════════════════════════════════
# GROUP 1 — POSITION  (px_m, py_m, pz_m)
# Raw:  GPS x_m / y_m,  depth sensor depth_m
# EKF:  px_m, py_m, pz_m  +  std_pos_m band
# ═══════════════════════════════════════════════════════════════

def group1_position(ekf, gps, dep, out_dir, do_save):
    """Compare raw position sensors against EKF fused position."""

    print("\n[1/6] Group 1 — Position (col 1–3)...")

    fig = make_fig(
        "Group 1 — POSITION  (col 1–3):  px_m  py_m  pz_m\n"
        "Left = Raw sensor data (GPS + Depth)  |  Right = EKF fused output",
        rows=3
    )

    gs = gridspec.GridSpec(
        3,
        2,
        figure=fig,
        hspace=0.52,
        wspace=0.30,
        left=0.07,
        right=0.97,
        top=0.92,
        bottom=0.06
    )

    # Time vectors
    ekf_t = ekf["timestamp"].to_numpy()

    # Valid sensor measurements
    gps_v = valid_rows(gps)
    dep_v = valid_rows(dep)

    # EKF position uncertainty
    sig = ekf["std_pos_m"].to_numpy()

        # ── px (North) ────────────────────────────────────────────

    ax = fig.add_subplot(gs[0, 0])

    if gps_v is not None:
        ax.scatter(
            gps_v["timestamp"].to_numpy(),
            gps_v["x_m"].to_numpy(),
            s=18,
            color=COL_GPS_X,
            alpha=0.75,
            label="GPS x_m (raw, 1 Hz)"
        )

        ax.plot(
            gps_v["timestamp"].to_numpy(),
            gps_v["x_m"].to_numpy(),
            color=COL_GPS_X,
            lw=0.6,
            alpha=0.3
        )

    style_ax(
        ax,
        "px — North position  |  raw GPS x_m (1 Hz)",
        "Time (s)",
        "px / x_m (m)"
    )

    ax.legend(loc="upper left")
    RAW_tag(ax)

    if gps_v is not None:
        annotate_stats(
            ax,
            {
                "rate": "1 Hz",
                "points": str(len(gps_v)),
                "noise σ": "1.5 m",
                "range": (
                    f"{gps_v['x_m'].min():.1f}"
                    f"→{gps_v['x_m'].max():.1f} m"
                )
            }
        )

        # EKF Fused Position
    ax = fig.add_subplot(gs[0, 1])

    ax.plot(
        ekf_t,
        ekf["px_m"].to_numpy(),
        color=COL_EKF_PX,
        lw=1.8,
        label="px_m (EKF)"
    )

    std_band(
        ax,
        ekf_t,
        ekf["px_m"].to_numpy(),
        sig,
        COL_EKF_PX
    )

    if gps_v is not None:
        ax.scatter(
            gps_v["timestamp"].to_numpy(),
            gps_v["x_m"].to_numpy(),
            s=22,
            color="#FFD166",
            zorder=5,
            alpha=0.8,
            label="GPS raw (overlaid)"
        )

    gps_upd = ekf["upd_gps"].to_numpy() == 1

    ax.scatter(
        ekf_t[gps_upd],
        ekf["px_m"].to_numpy()[gps_upd],
        s=14,
        color="#FFD166",
        alpha=0.55,
        zorder=6,
        label=f"GPS Update ({gps_upd.sum()})"
    )

    style_ax(
        ax,
        "px_m — EKF North position | 100 Hz fused",
        "Time (s)",
        "px_m (m)"
    )

    ax.legend(loc="upper left")
    EKF_tag(ax)

    annotate_stats(
        ax,
        {
            "rate": "100 Hz",
            "GPS updates": str(gps_upd.sum()),
            "range": (
                f"{ekf['px_m'].min():.1f}"
                f"→{ekf['px_m'].max():.1f} m"
            ),
            "σ_pos": f"{sig.mean():.3f} m"
        }
    )

        # ── py (East) ─────────────────────────────────────────────

    ax = fig.add_subplot(gs[1, 0])

    if gps_v is not None:
        ax.scatter(
            gps_v["timestamp"].to_numpy(),
            gps_v["y_m"].to_numpy(),
            s=18,
            color=COL_GPS_Y,
            alpha=0.75,
            label="GPS y_m (raw, 1 Hz)"
        )

        ax.plot(
            gps_v["timestamp"].to_numpy(),
            gps_v["y_m"].to_numpy(),
            color=COL_GPS_Y,
            lw=0.6,
            alpha=0.3
        )

    if gps is not None and "valid" in gps.columns:
        gps_inv = gps[gps["valid"] == 0]

        if len(gps_inv):
            ax.scatter(
                gps_inv["timestamp"].to_numpy(),
                gps_inv["y_m"].to_numpy(),
                color=COL_DROP,
                s=22,
                marker="x",
                alpha=0.5,
                label="Invalid GPS"
            )

    style_ax(
        ax,
        "py — East position  |  raw GPS y_m (1 Hz)",
        "Time (s)",
        "py / y_m (m)"
    )

    ax.legend(loc="upper left")
    RAW_tag(ax)

    ax = fig.add_subplot(gs[1, 1])

    ax.plot(
        ekf_t,
        ekf["py_m"].to_numpy(),
        color=COL_EKF_PY,
        lw=1.8,
        label="py_m (EKF)"
    )

    std_band(
        ax,
        ekf_t,
        ekf["py_m"].to_numpy(),
        sig,
        COL_EKF_PY
    )

    if gps_v is not None:
        ax.scatter(
            gps_v["timestamp"].to_numpy(),
            gps_v["y_m"].to_numpy(),
            s=22,
            color="#FFD166",
            zorder=5,
            alpha=0.8,
            label="GPS raw (overlaid)"
        )

    style_ax(
        ax,
        "py_m — EKF East position | 100 Hz fused",
        "Time (s)",
        "py_m (m)"
    )

    ax.legend(loc="upper left")
    EKF_tag(ax)

    annotate_stats(
        ax,
        {
            "range": (
                f"{ekf['py_m'].min():.1f}"
                f"→{ekf['py_m'].max():.1f} m"
            ),
            "σ_pos": f"{sig.mean():.3f} m"
        }
    )

    # ── pz (Depth) ────────────────────────────────────────────

    ax = fig.add_subplot(gs[2, 0])

    if dep_v is not None:
        ax.scatter(
            dep_v["timestamp"].to_numpy(),
            dep_v["depth_m"].to_numpy(),
            s=22,
            color=COL_DEPTH,
            alpha=0.75,
            label="Depth sensor depth_m (raw, 2 Hz)"
        )

        ax.plot(
            dep_v["timestamp"].to_numpy(),
            dep_v["depth_m"].to_numpy(),
            color=COL_DEPTH,
            lw=0.6,
            alpha=0.3
        )

    style_ax(
        ax,
        "pz — Depth | raw pressure sensor depth_m (2 Hz)",
        "Time (s)",
        "depth_m (m)",
        invert_y=True
    )

    ax.legend(loc="upper right")
    RAW_tag(ax)

    if dep_v is not None:
        annotate_stats(
            ax,
            {
                "rate": "2 Hz",
                "points": str(len(dep_v)),
                "range": (
                    f"{dep_v['depth_m'].min():.2f}"
                    f"→{dep_v['depth_m'].max():.2f} m"
                )
            }
        )

    ax = fig.add_subplot(gs[2, 1])

    ekf_dep_col = (
        "depth_m"
        if "depth_m" in ekf.columns
        else "pz_m"
    )

    ax.plot(
        ekf_t,
        ekf[ekf_dep_col].to_numpy(),
        color=COL_EKF_DEP,
        lw=1.8,
        label=f"{ekf_dep_col} (EKF, 100 Hz)"
    )

    std_band(
        ax,
        ekf_t,
        ekf[ekf_dep_col].to_numpy(),
        sig,
        COL_EKF_DEP
    )

    if dep_v is not None:
        ax.scatter(
            dep_v["timestamp"].to_numpy(),
            dep_v["depth_m"].to_numpy(),
            s=22,
            color="#FFD166",
            zorder=5,
            alpha=0.8,
            label="Depth sensor (overlaid)"
        )

    dep_upd = ekf["upd_depth"].to_numpy() == 1

    ax.scatter(
        ekf_t[dep_upd],
        ekf[ekf_dep_col].to_numpy()[dep_upd],
        s=12,
        color="#FFD166",
        zorder=6,
        alpha=0.55,
        label=f"Depth Update ({dep_upd.sum()})"
    )

    style_ax(
        ax,
        "pz_m — EKF Depth | 100 Hz fused (= depth_m)",
        "Time (s)",
        "pz_m (m)",
        invert_y=True
    )

    ax.legend(loc="upper right")
    EKF_tag(ax)

    annotate_stats(
        ax,
        {
            "depth updates": str(dep_upd.sum()),
            "max depth": f"{ekf[ekf_dep_col].max():.3f} m",
            "σ_pos": f"{sig.mean():.4f} m"
        }
    )

    save_fig(
        fig,
        out_dir,
        "group1_position.png",
        do_save
    )


# ═══════════════════════════════════════════════════════════════
# GROUP 2 — VELOCITY  (vx_ms, vy_ms, vz_ms)
# Raw:  DVL vx / vy / vz  (with dropout markers)
# EKF:  vx_ms, vy_ms, vz_ms  +  std_vel_ms band
# ═══════════════════════════════════════════════════════════════

def group2_velocity(ekf, dvl, out_dir, do_save):
    """Compare raw DVL velocity measurements against EKF velocity estimates."""

    print("[2/6] Group 2 — Velocity (col 4–6)...")

    fig = make_fig(
        "Group 2 — VELOCITY (col 4–6): vx_ms  vy_ms  vz_ms\n"
        "Left = Raw DVL sensor (10 Hz, with dropouts) | "
        "Right = EKF output (100 Hz)",
        rows=3
    )

    gs = gridspec.GridSpec(
        3,
        2,
        figure=fig,
        hspace=0.52,
        wspace=0.30,
        left=0.07,
        right=0.97,
        top=0.92,
        bottom=0.06
    )

    # Time base
    ekf_t = ekf["timestamp"].to_numpy()

    # Valid DVL measurements
    dvl_v = valid_rows(dvl)

    # DVL dropouts
    dvl_d = (
        dvl[dvl["valid"] == 0]
        if dvl is not None and "valid" in dvl.columns
        else pd.DataFrame()
    )

    drops = (
        dvl_d["timestamp"].to_numpy()
        if len(dvl_d)
        else np.array([])
    )

    # EKF uncertainty
    sig = ekf["std_vel_ms"].to_numpy()

    # DVL update flags
    upd = ekf["upd_dvl"].to_numpy() == 1

    # (Raw DVL column, raw color, raw title,
    #  EKF column, EKF color, EKF title)
    pairs = [
        (
            "vx",
            COL_DVL_VX,
            "vx — DVL North velocity (raw, 10 Hz)",
            "vx_ms",
            COL_EKF_VX,
            "vx_ms — EKF North velocity (100 Hz)"
        ),
        (
            "vy",
            COL_DVL_VY,
            "vy — DVL East velocity (raw, 10 Hz)",
            "vy_ms",
            COL_EKF_VY,
            "vy_ms — EKF East velocity (100 Hz)"
        ),
        (
            "vz",
            COL_DVL_VZ,
            "vz — DVL Down velocity (raw, 10 Hz)",
            "vz_ms",
            COL_EKF_VZ,
            "vz_ms — EKF Down velocity (100 Hz)"
        ),
    ]

    for row, (rc, rcol, rtitle, ec, ecol, etitle) in enumerate(pairs):

        # ── LEFT — Raw DVL Velocity ──────────────────────────

        ax = fig.add_subplot(gs[row, 0])

        if dvl_v is not None and rc in dvl_v.columns:
            ax.scatter(
                dvl_v["timestamp"].to_numpy(),
                dvl_v[rc].to_numpy(),
                color=rcol,
                s=8,
                alpha=0.65,
                label=f"DVL {rc}"
            )

        dropout_markers(ax, drops)

        ax.axhline(
            0,
            color=MUTED,
            lw=0.5,
            ls="--",
            alpha=0.5
        )

        style_ax(
            ax,
            rtitle,
            "Time (s)",
            f"{rc} (m/s)"
        )

        ax.legend(loc="upper right")
        RAW_tag(ax)

        if dvl_v is not None:
            annotate_stats(
                ax,
                {
                    "rate": "10 Hz",
                    "valid": str(len(dvl_v)),
                    "dropouts": str(len(drops)),
                    "range": (
                        f"{dvl_v[rc].min():.3f}"
                        f"→{dvl_v[rc].max():.3f} m/s"
                    )
                }
            )

        
        ax = fig.add_subplot(gs[row, 1])

        ekf_vel = ekf[ec].to_numpy()

        ax.plot(
            ekf_t,
            ekf_vel,
            color=ecol,
            lw=1.8,
            label=f"EKF {ec}"
        )

        std_band(
            ax,
            ekf_t,
            ekf_vel,
            sig,
            ecol
        )

        ax.scatter(
            ekf_t[upd],
            ekf_vel[upd],
            s=10,
            color=rcol,
            alpha=0.25,
            zorder=4,
            label="DVL update"
        )

        ax.axhline(
            0,
            color=MUTED,
            lw=0.5,
            ls="--",
            alpha=0.5
        )

        style_ax(
            ax,
            etitle,
            "Time (s)",
            f"{ec} (m/s)"
        )

        ax.legend(loc="upper right")
        EKF_tag(ax)

        annotate_stats(
            ax,
            {
                "rate": "100 Hz",
                "DVL updates": str(upd.sum()),
                "σ_vel": f"{sig.mean():.5f} m/s",
                "range": (
                    f"{ekf_vel.min():.3f}"
                    f"→{ekf_vel.max():.3f} m/s"
                )
            }
        )

    save_fig(
        fig,
        out_dir,
        "group2_velocity.png",
        do_save
    )

# ═══════════════════════════════════════════════════════════════
# GROUP 3 — ATTITUDE  (roll_deg, pitch_deg, yaw_deg)
# Raw:  IMU gyro gx/gy/gz + accel-derived roll/pitch + mag yaw
# EKF:  roll_deg, pitch_deg, yaw_deg  +  std_att_deg band
# ═══════════════════════════════════════════════════════════════

def group3_attitude(ekf, imu, mag, out_dir, do_save):
    print("[3/6] Group 3 — Attitude (col 7–9)...")

    fig = make_fig(
        "Group 3 — ATTITUDE (col 7–9): "
        "roll_deg  pitch_deg  yaw_deg\n"
        "Left = Raw IMU gyro / accel / mag  |  "
        "Right = EKF output (100 Hz)",
        rows=3
    )

    gs = gridspec.GridSpec(
        3, 2,
        figure=fig,
        hspace=0.52,
        wspace=0.30,
        left=0.07,
        right=0.97,
        top=0.92,
        bottom=0.06
    )

    # EKF data
    ekf_t = ekf["timestamp"].to_numpy()
    sig = ekf["std_att_deg"].to_numpy()

    # IMU timestamps (if available)
    imu_t = (
        imu["timestamp"].to_numpy()
        if imu is not None
        else np.array([])
    )

    # Magnetometer update flags
    mag_upd = (
        ekf["upd_mag"].to_numpy() == 1
    )

    # ── Roll ─────────────────────────────────────────────────
    ax = fig.add_subplot(gs[0, 0])
    if imu is not None:
        ax.plot(
            imu_t,
            imu["gx"].to_numpy(),
            color=COL_GX,
            lw=0.9,
            alpha=0.8,
            label="gx — roll rate (rad/s)"
        )
    ax.axhline(
        0,
        color=MUTED,
        lw=0.5,
        ls="--",
        alpha=0.5
    )

    style_ax(
        ax,
        "gx — Raw IMU Roll Rate (rad/s) | 100 Hz",
        "Time (s)",
        "gx (rad/s)"
    )

    ax.legend(loc="upper right")
    RAW_tag(ax)

    if imu is not None:
        annotate_stats(
            ax,
            {
                "mean": f"{imu['gx'].mean():.4f} rad/s",
                "std":  f"{imu['gx'].std():.4f}",
                "max":  f"{imu['gx'].abs().max():.4f}",
            }
        )

    # # ── EKF Roll ────────────────────────────────────────────────

    # ax = fig.add_subplot(gs[0, 1])

    # ax.plot(
    #     ekf_t,
    #     ekf["roll_deg"].to_numpy(),
    #     color=COL_EKF_R,
    #     lw=1.8,
    #     label="roll_deg (EKF)"
    # )

    # std_band(
    #     ax,
    #     ekf_t,
    #     ekf["roll_deg"].to_numpy(),
    #     sig,
    #     COL_EKF_R
    # )

    # ax.axhline(
    #     0,
    #     color=MUTED,
    #     lw=0.5,
    #     ls="--",
    #     alpha=0.5
    # )

    # style_ax(
    #     ax,
    #     "roll_deg — EKF Roll Angle (°) | Gyro Integrated",
    #     "Time (s)",
    #     "roll_deg (°)"
    # )

    # ax.legend(loc="upper right")
    # EKF_tag(ax)

    # annotate_stats(
    #     ax,
    #     {
    #         "mean":  f"{ekf['roll_deg'].mean():.2f}°",
    #         "range": (
    #             f"{ekf['roll_deg'].min():.1f}"
    #             f" → "
    #             f"{ekf['roll_deg'].max():.1f}°"
    #         ),
    #         "σ_att": f"{sig.mean():.3f}°",
    #     }
    # )
    
    # ── EKF Roll ─────────────────────────────────────────────

    ax = fig.add_subplot(gs[0, 1])

    roll_deg = ekf["roll_deg"].to_numpy()

    # Remove 180° offset for visualization
    roll_plot = np.where(
        roll_deg > 90,
        roll_deg - 180,
        roll_deg
    )

    roll_plot = np.where(
        roll_plot < -90,
        roll_plot + 180,
        roll_plot
    )

    ax.plot(
        ekf_t,
        roll_plot,
        color=COL_EKF_R,
        lw=1.8,
        label="roll_deg (EKF)"
    )

    std_band(
        ax,
        ekf_t,
        roll_plot,
        sig,
        COL_EKF_R
    )

    ax.axhline(
        0,
        color=MUTED,
        lw=0.5,
        ls="--",
        alpha=0.5
    )

    style_ax(
        ax,
        "roll_deg — EKF Roll Angle (°) | Gyro Integrated",
        "Time (s)",
        "roll_deg (°)"
    )

    ax.legend(loc="upper right")
    EKF_tag(ax)

    annotate_stats(
        ax,
        {
            "mean": f"{roll_plot.mean():.2f}°",
            "range": (
                f"{roll_plot.min():.1f}"
                f" → "
                f"{roll_plot.max():.1f}°"
            ),
            "σ_att": f"{sig.mean():.3f}°",
        }
    )

    # ── Pitch ────────────────────────────────────────────────

    ax = fig.add_subplot(gs[1, 0])

    if imu is not None:
        ax.plot(
            imu_t,
            imu["gy"].to_numpy(),
            color=COL_GY,
            lw=0.9,
            alpha=0.8,
            label="gy — pitch rate (rad/s)"
        )

    ax.axhline(
        0,
        color=MUTED,
        lw=0.5,
        ls="--",
        alpha=0.5
    )

    style_ax(
        ax,
        "gy — Raw IMU pitch rate (rad/s) | 100 Hz",
        "Time (s)",
        "gy (rad/s)"
    )

    ax.legend(loc="upper right")
    RAW_tag(ax)

    if imu is not None:
        annotate_stats(
            ax,
            {
                "mean": f"{imu['gy'].mean():.4f} rad/s",
                "std":  f"{imu['gy'].std():.4f}",
                "max":  f"{imu['gy'].abs().max():.4f}"
            }
        )

    ax = fig.add_subplot(gs[1, 1])

    ekf_pitch = ekf["pitch_deg"].to_numpy()

    ax.plot(
        ekf_t,
        ekf_pitch,
        color=COL_EKF_P,
        lw=1.8,
        label="pitch_deg (EKF)"
    )

    std_band(
        ax,
        ekf_t,
        ekf_pitch,
        sig,
        COL_EKF_P
    )

    ax.axhline(
        0,
        color=MUTED,
        lw=0.5,
        ls="--",
        alpha=0.5
    )

    style_ax(
        ax,
        "pitch_deg — EKF Pitch angle (°) | gyro + accel fused",
        "Time (s)",
        "pitch_deg (°)"
    )

    ax.legend(loc="upper right")
    EKF_tag(ax)

    annotate_stats(
        ax,
        {
            "mean": f"{ekf_pitch.mean():.2f}°",
            "range": (
                f"{ekf_pitch.min():.1f}"
                f"→{ekf_pitch.max():.1f}°"
            ),
            "σ_att": f"{sig.mean():.3f}°"
        }
    )

    # ── Yaw ─────────────────────────────────────────────────

    ax = fig.add_subplot(gs[2, 0])

    if imu is not None:
        ax.plot(
            imu_t,
            imu["gz"].to_numpy(),
            color=COL_GZ,
            lw=0.9,
            alpha=0.8,
            label="gz — yaw rate (rad/s)"
        )

    if mag is not None:
        mag_v = valid_rows(mag)

        if mag_v is not None and "yaw_rad" in mag_v.columns:
            mag_deg = np.degrees(
                mag_v["yaw_rad"].to_numpy()
            )

            ax_r2 = ax.twinx()

            ax_r2.scatter(
                mag_v["timestamp"].to_numpy(),
                mag_deg,
                s=12,
                color=COL_MAG,
                alpha=0.7,
                label="Mag yaw (deg)"
            )

            ax_r2.set_ylabel(
                "Mag yaw (°)",
                color=COL_MAG,
                labelpad=4
            )

            ax_r2.tick_params(
                axis="y",
                colors=COL_MAG
            )

            ax_r2.set_facecolor("none")

            ax_r2.legend(
                loc="lower right",
                fontsize=7
            )
        ax.axhline(
        0,
        color=MUTED,
        lw=0.5,
        ls="--",
        alpha=0.5
    )

    style_ax(
        ax,
        "gz — Raw yaw rate (rad/s) + Magnetometer yaw (deg)",
        "Time (s)",
        "gz (rad/s)"
    )

    ax.legend(loc="upper right")
    RAW_tag(ax)

    if imu is not None:
        annotate_stats(
            ax,
            {
                "mean": f"{imu['gz'].mean():.4f} rad/s",
                "mag rate": "5 Hz",
                "mag noise": "0.02 rad = 1.1°"
            }
        )

    # ── EKF Yaw Angle ────────────────────────────────────────

    ax = fig.add_subplot(gs[2, 1])

    ekf_yaw = ekf["yaw_deg"].to_numpy()

    ax.plot(
        ekf_t,
        ekf_yaw,
        color=COL_EKF_Y,
        lw=1.8,
        label="yaw_deg (EKF)"
    )

    std_band(
        ax,
        ekf_t,
        ekf_yaw,
        sig,
        COL_EKF_Y
    )

    if mag is not None:
        mag_v2 = valid_rows(mag)

        if mag_v2 is not None and "yaw_rad" in mag_v2.columns:
            mag_deg2 = np.degrees(
                mag_v2["yaw_rad"].to_numpy()
            )

            ax.scatter(
                mag_v2["timestamp"].to_numpy(),
                mag_deg2,
                s=16,
                color=COL_MAG,
                zorder=5,
                alpha=0.65,
                label="Mag raw (overlaid)"
            )
        mag_upd = ekf["upd_mag"].to_numpy() == 1

    ax.scatter(
        ekf_t[mag_upd],
        ekf_yaw[mag_upd],
        s=8,
        color="#FFD166",
        alpha=0.35,
        zorder=4,
        label=f"Mag Update ({mag_upd.sum()})"
    )

    style_ax(
        ax,
        "yaw_deg — EKF Heading (°) | gyro + mag fused, angle-wrapped",
        "Time (s)",
        "yaw_deg (°)"
    )

    ax.legend(loc="upper right")
    EKF_tag(ax)

    annotate_stats(
        ax,
        {
            "mag updates": str(mag_upd.sum()),
            "range": (
                f"{ekf_yaw.min():.1f}"
                f"→{ekf_yaw.max():.1f}°"
            ),
            "final yaw": f"{ekf_yaw[-1]:.2f}°",
            "σ_att": f"{sig.mean():.3f}°"
        }
    )

    save_fig(
        fig,
        out_dir,
        "group3_attitude.png",
        do_save
    )


# ═══════════════════════════════════════════════════════════════
# GROUP 4 — DERIVED  (speed_horiz_ms, speed_total_ms,
#                     distance_total_m, depth_m)
# Raw:  DVL-derived speed, depth sensor
# EKF:  all four derived columns
# ═══════════════════════════════════════════════════════════════

def group4_derived(ekf, dvl, dep, out_dir, do_save):
    """Compare raw-derived metrics against EKF-derived quantities."""

    print("[4/6] Group 4 — Derived (col 10–13)...")

    fig = make_fig(
        "Group 4 — DERIVED (col 10–13): "
        "speed_horiz_ms  speed_total_ms  distance_total_m  depth_m\n"
        "Left = Raw sensor derived values | Right = EKF output (100 Hz)",
        rows=4,
        figsize=(18, 16)
    )

    gs = gridspec.GridSpec(
        4,
        2,
        figure=fig,
        hspace=0.55,
        wspace=0.30,
        left=0.07,
        right=0.97,
        top=0.93,
        bottom=0.05
    )

    # Time base
    ekf_t = ekf["timestamp"].to_numpy()

    # Valid sensor measurements
    dvl_v = valid_rows(dvl)
    dep_v = valid_rows(dep)

    # DVL dropouts
    dvl_d = (
        dvl[dvl["valid"] == 0]
        if dvl is not None and "valid" in dvl.columns
        else pd.DataFrame()
    )

    drops = (
        dvl_d["timestamp"].to_numpy()
        if len(dvl_d)
        else np.array([])
    )

        # ── Horizontal Speed ─────────────────────────────────────

    # Raw DVL-Derived Horizontal Speed
    ax = fig.add_subplot(gs[0, 0])

    if dvl_v is not None:
        raw_h = np.sqrt(
            dvl_v["vx"]**2 +
            dvl_v["vy"]**2
        )

        ax.scatter(
            dvl_v["timestamp"].to_numpy(),
            raw_h.to_numpy(),
            color=COL_DVL_VX,
            s=10,
            alpha=0.7,
            label="√(vx² + vy²) from DVL"
        )

        ax.plot(
            dvl_v["timestamp"].to_numpy(),
            raw_h.to_numpy(),
            color=COL_DVL_VX,
            lw=0.5,
            alpha=0.3
        )

    dropout_markers(ax, drops)

    ax.set_ylim(bottom=0)

    style_ax(
        ax,
        "Horizontal speed (raw DVL) √(vx² + vy²) | 10 Hz, gaps at dropouts",
        "Time (s)",
        "speed_horiz (m/s)"
    )

    ax.legend(loc="upper left")
    RAW_tag(ax)

    # EKF Horizontal Speed
    ax = fig.add_subplot(gs[0, 1])

    ekf_hspeed = ekf["speed_horiz_ms"].to_numpy()

    ax.plot(
        ekf_t,
        ekf_hspeed,
        color=COL_SPD_H,
        lw=1.8,
        label="speed_horiz_ms (EKF)"
    )

    ax.fill_between(
        ekf_t,
        0,
        ekf_hspeed,
        color=COL_SPD_H,
        alpha=0.10
    )

    ax.set_ylim(bottom=0)

    style_ax(
        ax,
        "speed_horiz_ms — EKF √(vx² + vy²) | 100 Hz, no gaps",
        "Time (s)",
        "speed_horiz_ms (m/s)"
    )

    ax.legend(loc="upper left")
    EKF_tag(ax)

    annotate_stats(
        ax,
        {
            "max": f"{ekf_hspeed.max():.3f} m/s",
            "mean": f"{ekf_hspeed.mean():.3f} m/s"
        }
    )

    # ── Total Speed ──────────────────────────────────────────

    # Raw DVL-Derived Total Speed
    ax = fig.add_subplot(gs[1, 0])

    if dvl_v is not None:
        raw_t = np.sqrt(
            dvl_v["vx"]**2 +
            dvl_v["vy"]**2 +
            dvl_v["vz"]**2
        )

        ax.scatter(
            dvl_v["timestamp"].to_numpy(),
            raw_t.to_numpy(),
            color=COL_MAG,
            s=10,
            alpha=0.7,
            label="√(vx² + vy² + vz²) from DVL"
        )

        ax.plot(
            dvl_v["timestamp"].to_numpy(),
            raw_t.to_numpy(),
            color=COL_MAG,
            lw=0.5,
            alpha=0.3
        )

    dropout_markers(ax, drops)

    ax.set_ylim(bottom=0)

    style_ax(
        ax,
        "Total 3D speed (raw DVL) √(vx² + vy² + vz²) | 10 Hz, gaps at dropouts",
        "Time (s)",
        "speed_total (m/s)"
    )

    ax.legend(loc="upper left")
    RAW_tag(ax)

    # EKF Total Speed
    ax = fig.add_subplot(gs[1, 1])

    ekf_speed_total = ekf["speed_total_ms"].to_numpy()

    ax.plot(
        ekf_t,
        ekf_speed_total,
        color=COL_SPD_T,
        lw=1.8,
        label="speed_total_ms (EKF)"
    )

    ax.fill_between(
        ekf_t,
        0,
        ekf_speed_total,
        color=COL_SPD_T,
        alpha=0.08
    )

    std_band(
        ax,
        ekf_t,
        ekf_speed_total,
        ekf["std_vel_ms"].to_numpy(),
        COL_SPD_T
    )

    ax.set_ylim(bottom=0)

    style_ax(
        ax,
        "speed_total_ms — EKF √(vx² + vy² + vz²) | 100 Hz smooth",
        "Time (s)",
        "speed_total_ms (m/s)"
    )

    ax.legend(loc="upper left")
    EKF_tag(ax)

    annotate_stats(
        ax,
        {
            "max": f"{ekf_speed_total.max():.3f} m/s",
            "mean": f"{ekf_speed_total.mean():.3f} m/s"
        }
    )

    # ── Distance Total ───────────────────────────────────────

    # # Raw DVL Dead-Reckoning Distance
    # ax = fig.add_subplot(gs[2, 0])

    # if dvl_v is not None:
    #     raw_spd = np.sqrt(
    #         dvl_v["vx"]**2 +
    #         dvl_v["vy"]**2 +
    #         dvl_v["vz"]**2
    #     )

    #     dt_dvl = (
    #         dvl_v["timestamp"]
    #         .diff()
    #         .fillna(0.1)
    #         .to_numpy()
    #     )

    #     dist_dr = np.cumsum(
    #         raw_spd.to_numpy() * dt_dvl
    #     )

    #     ax.plot(
    #         dvl_v["timestamp"].to_numpy(),
    #         dist_dr,
    #         color=COL_DVL_VY,
    #         lw=1.4,
    #         label="DVL-only dead-reckoning distance"
    #     )

    # style_ax(
    #     ax,
    #     "Distance dead-reckoning from DVL only | no position correction",
    #     "Time (s)",
    #     "distance (m)"
    # )

    # ax.legend(loc="upper left")
    # RAW_tag(ax)
    
    # Raw DVL Dead-Reckoning Distance
    ax = fig.add_subplot(gs[2, 0])

    if dvl_v is not None:

        t = dvl_v["timestamp"].to_numpy()

        vx = dvl_v["vx"].to_numpy()
        vy = dvl_v["vy"].to_numpy()
        vz = dvl_v["vz"].to_numpy()

        # Time step between DVL samples
        dt = np.diff(t, prepend=t[0])

        # Dead-reckoning position from DVL velocities
        px_dr = np.cumsum(vx * dt)
        py_dr = np.cumsum(vy * dt)
        pz_dr = np.cumsum(vz * dt)

        # Distance travelled from position steps
        dx = np.diff(px_dr, prepend=px_dr[0])
        dy = np.diff(py_dr, prepend=py_dr[0])
        dz = np.diff(pz_dr, prepend=pz_dr[0])

        dist_dr = np.cumsum(
            np.sqrt(dx*dx + dy*dy + dz*dz)
        )

        ax.plot(
            t,
            dist_dr,
            color=COL_DVL_VY,
            lw=1.4,
            label="DVL Dead-Reckoning Distance"
        )

        ax.fill_between(
            t,
            0,
            dist_dr,
            color=COL_DVL_VY,
            alpha=0.08
        )

        annotate_stats(
            ax,
            {
                "final": f"{dist_dr[-1]:.1f} m",
                "samples": str(len(dist_dr)),
                "source": "DVL only"
            }
        )

    style_ax(
        ax,
        "Distance from DVL Dead-Reckoning Position | No GPS Correction",
        "Time (s)",
        "distance (m)"
    )

    ax.legend(loc="upper left")
    RAW_tag(ax)

    # EKF Odometer Distance
    ax = fig.add_subplot(gs[2, 1])

    ekf_dist = ekf["distance_total_m"].to_numpy()

    ax.plot(
        ekf_t,
        ekf_dist,
        color="#0891B2",
        lw=1.8,
        label="distance_total_m (EKF odometer)"
    )

    ax.fill_between(
        ekf_t,
        0,
        ekf_dist,
        color="#0891B2",
        alpha=0.08
    )

    ax.annotate(
        f"Final: {ekf_dist.max():.1f} m",
        xy=(ekf_t[-1], ekf_dist.max()),
        xytext=(ekf_t[-1] * 0.60, ekf_dist.max() * 0.8),
        arrowprops=dict(
            arrowstyle="->",
            color=TEXT
        ),
        color=TEXT,
        fontsize=9,
        fontweight="bold"
    )

    style_ax(
        ax,
        "distance_total_m — EKF path odometer | always increases",
        "Time (s)",
        "distance_total_m (m)"
    )

    ax.legend(loc="upper left")
    EKF_tag(ax)

    annotate_stats(
        ax,
        {
            "final": f"{ekf_dist.max():.1f} m",
            "avg step": (
                f"{ekf['distance_total_m'].diff().mean():.4f} "
                "m/row"
            )
        }
    )

    #     # ── Depth_m ───────────────────────────────────────────────

    # ax = fig.add_subplot(gs[3, 0])

    # if dep_v is not None:
    #     ax.scatter(
    #         dep_v["timestamp"].to_numpy(),
    #         dep_v["depth_m"].to_numpy(),
    #         s=18,
    #         color=COL_DEPTH,
    #         alpha=0.75,
    #         label="Depth sensor depth_m (raw, 2 Hz)"
    #     )

    #     ax.plot(
    #         dep_v["timestamp"].to_numpy(),
    #         dep_v["depth_m"].to_numpy(),
    #         color=COL_DEPTH,
    #         lw=0.6,
    #         alpha=0.3
    #     )

    # style_ax(
    #     ax,
    #     "depth_m  raw pressure sensor  |  2 Hz sparse",
    #     "Time (s)",
    #     "depth_m  (m)",
    #     invert_y=True
    # )

    # ax.legend(loc="upper right")
    # RAW_tag(ax)

    # ax = fig.add_subplot(gs[3, 1])

    # ekf_dep = "depth_m" if "depth_m" in ekf.columns else "pz_m"

    # ax.plot(
    #     ekf_t,
    #     ekf[ekf_dep].to_numpy(),
    #     color=COL_EKF_DEP,
    #     lw=1.8,
    #     label=f"{ekf_dep} (EKF, 100 Hz)"
    # )

    # ax.fill_between(
    #     ekf_t,
    #     0,
    #     ekf[ekf_dep].to_numpy(),
    #     where=ekf[ekf_dep].to_numpy() > 0,
    #     color=COL_EKF_DEP,
    #     alpha=0.10
    # )

    # if dep_v is not None:
    #     ax.scatter(
    #         dep_v["timestamp"].to_numpy(),
    #         dep_v["depth_m"].to_numpy(),
    #         s=22,
    #         color="#FFD166",
    #         zorder=5,
    #         alpha=0.75,
    #         label="Raw depth (overlaid)"
    #     )

    # dep_upd = ekf["upd_depth"].to_numpy() == 1

    # ax.scatter(
    #     ekf_t[dep_upd],
    #     ekf[ekf_dep].to_numpy()[dep_upd],
    #     s=12,
    #     color="#FFD166",
    #     zorder=6,
    #     alpha=0.55,
    #     label=f"Depth Update ({dep_upd.sum()})"
    # )

    # style_ax(
    #     ax,
    #     f"depth_m — EKF depth (= pz_m)  |  100 Hz fused",
    #     "Time (s)",
    #     "depth_m  (m)",
    #     invert_y=True
    # )

    # ax.legend(loc="upper right")
    # EKF_tag(ax)

    # annotate_stats(
    #     ax,
    #     {
    #         "= pz_m": "exactly (convenience copy)",
    #         "max": f"{ekf[ekf_dep].max():.3f} m"
    #     }
    # )

    # save_fig(
    #     fig,
    #     out_dir,
    #     "group4_derived.png",
    #     do_save
    # )
    
        # ── Depth_m ───────────────────────────────────────────────

    ax = fig.add_subplot(gs[3, 0])

    if dep_v is not None:
        dep_t = dep_v["timestamp"].to_numpy()
        dep_depth = dep_v["depth_m"].to_numpy()

        ax.scatter(
            dep_t,
            dep_depth,
            s=18,
            color=COL_DEPTH,
            alpha=0.75,
            label="Depth sensor depth_m (raw, 2 Hz)"
        )

        ax.plot(
            dep_t,
            dep_depth,
            color=COL_DEPTH,
            lw=0.6,
            alpha=0.3
        )

    style_ax(
        ax,
        "depth_m — Raw pressure sensor | 2 Hz sparse",
        "Time (s)",
        "depth_m (m)",
        invert_y=True
    )

    ax.legend(loc="upper right")
    RAW_tag(ax)

    # ── EKF Depth ─────────────────────────────────────────────

    ax = fig.add_subplot(gs[3, 1])

    if "depth_m" in ekf.columns:
        ekf_dep = "depth_m"
    elif "pz_m" in ekf.columns:
        ekf_dep = "pz_m"
    else:
        raise ValueError(
            "Neither 'depth_m' nor 'pz_m' found in EKF output."
        )

    ekf_depth = ekf[ekf_dep].to_numpy()

    ax.plot(
        ekf_t,
        ekf_depth,
        color=COL_EKF_DEP,
        lw=1.8,
        label=f"{ekf_dep} (EKF, 100 Hz)"
    )

    # Optional shading
    ax.fill_between(
        ekf_t,
        0,
        ekf_depth,
        where=(ekf_depth > 0),
        color=COL_EKF_DEP,
        alpha=0.10
    )

    if dep_v is not None:
        ax.scatter(
            dep_t,
            dep_depth,
            s=22,
            color="#FFD166",
            zorder=5,
            alpha=0.75,
            label="Raw depth (overlaid)"
        )

    dep_upd = ekf["upd_depth"].to_numpy() == 1

    ax.scatter(
        ekf_t[dep_upd],
        ekf_depth[dep_upd],
        s=12,
        color="#FFD166",
        zorder=6,
        alpha=0.55,
        label=f"Depth Update ({dep_upd.sum()})"
    )

    style_ax(
        ax,
        "depth_m — EKF depth (= pz_m) | 100 Hz fused",
        "Time (s)",
        "depth_m (m)",
        invert_y=True
    )

    ax.legend(loc="upper right")
    EKF_tag(ax)

    annotate_stats(
        ax,
        {
            "= pz_m": "exactly (convenience copy)",
            "max": f"{ekf_depth.max():.3f} m",
            "updates": str(dep_upd.sum())
        }
    )

    save_fig(
        fig,
        out_dir,
        "group4_derived.png",
        do_save
    )
    


# ═══════════════════════════════════════════════════════════════
# GROUP 5 — UNCERTAINTY  (std_pos_m, std_vel_ms, std_att_deg)
# Left:  simulated drift without EKF corrections
# Right: EKF covariance σ convergence with sensor event lines
# ═══════════════════════════════════════════════════════════════

def group5_uncertainty(ekf, out_dir, do_save):
    print("[5/6] Group 5 — Uncertainty (col 14–16)...")

    fig = make_fig(
        "Group 5 — UNCERTAINTY  (col 14–16):  std_pos_m  std_vel_ms  std_att_deg\n"
        "Left = Simulated drift WITHOUT EKF  |  Right = EKF σ convergence (WITH EKF)",
        rows=3
    )

    gs = gridspec.GridSpec(
        3,
        2,
        figure=fig,
        hspace=0.52,
        wspace=0.30,
        left=0.07,
        right=0.97,
        top=0.92,
        bottom=0.06
    )

    ekf_t = ekf["timestamp"].to_numpy()

    dvl_t = ekf_t[
        ekf["upd_dvl"].to_numpy() == 1
    ]

    dep_t = ekf_t[
        ekf["upd_depth"].to_numpy() == 1
    ]

    gps_t = ekf_t[
        ekf["upd_gps"].to_numpy() == 1
    ]

    mag_t = ekf_t[
        ekf["upd_mag"].to_numpy() == 1
    ]

    sensor_legend = [
        Patch(
            color="#00B4D8",
            alpha=0.6,
            label="DVL update"
        ),
        Patch(
            color="#48CAE4",
            alpha=0.6,
            label="Depth update"
        ),
        Patch(
            color="#F9C74F",
            alpha=0.7,
            label="GPS update"
        ),
        Patch(
            color="#F3722C",
            alpha=0.6,
            label="Mag update"
        ),
    ]

    triples = [
        (
            "std_pos_m",
            "#3B82F6",
            "#1E3A8A",
            "Position drift WITHOUT EKF\n(dead-reckoning: σ grows ~as t^1.5, unbounded)",
            "std_pos_m — EKF position σ (m)\n(drops after GPS + Depth updates)"
        ),
        (
            "std_vel_ms",
            "#10B981",
            "#064E3B",
            "Velocity drift WITHOUT EKF\n(IMU-only: bias accumulates linearly with time)",
            "std_vel_ms — EKF velocity σ (m/s)\n(drops sharply after each DVL update at 10 Hz)"
        ),
        (
            "std_att_deg",
            "#A855F7",
            "#581C87",
            "Attitude drift WITHOUT EKF\n(gyro drift: heading error grows linearly with time)",
            "std_att_deg — EKF attitude σ (°)\n(drops after each Magnetometer update at 5 Hz)"
        ),
    ]

    for row, (col_name, light, dark, ltitle, rtitle) in enumerate(triples):

        vals = ekf[col_name].to_numpy()

        # LEFT — simulated unbounded drift

        ax = fig.add_subplot(gs[row, 0])

        t_sim = np.linspace(
            0,
            ekf_t[-1],
            500
        )

        if row == 0:
            sim = vals[0] + 0.005 * t_sim ** 1.5
        elif row == 1:
            sim = vals[0] + 0.0008 * t_sim
        else:
            sim = vals[0] + 0.04 * t_sim

        ax.plot(
            t_sim,
            sim,
            color=light,
            lw=2.0,
            label="Without EKF — grows unbounded"
        )

        ax.fill_between(
            t_sim,
            0,
            sim,
            color=light,
            alpha=0.12
        )

        ax.axhline(
            vals[-1],
            color="#2ECC71",
            lw=1.0,
            ls="--",
            label=f"EKF final value: {vals[-1]:.4f}"
        )

        ax.set_ylim(bottom=0)

        style_ax(
            ax,
            ltitle,
            "Time (s)",
            col_name
        )

        ax.legend(
            loc="upper left",
            fontsize=8
        )

        RAW_tag(ax)

                # RIGHT — EKF convergence

        ax = fig.add_subplot(gs[row, 1])

        ax.plot(
            ekf_t,
            vals,
            color=dark,
            lw=1.8,
            zorder=4,
            label=col_name
        )

        ax.fill_between(
            ekf_t,
            0,
            vals,
            color=dark,
            alpha=0.12,
            zorder=3
        )

        # Sensor event lines
        for s_times, s_col in [
            (dvl_t, "#00B4D8"),
            (dep_t, "#48CAE4"),
            (gps_t, "#F9C74F"),
            (mag_t, "#F3722C")
        ]:
            step = max(1, len(s_times) // 50)

            for st in s_times[::step]:
                ax.axvline(
                    st,
                    color=s_col,
                    alpha=0.12,
                    lw=0.5,
                    zorder=1
                )

        ax.annotate(
            f"Start: {vals[0]:.4f}",
            xy=(ekf_t[0], vals[0]),
            xytext=(ekf_t[-1] * 0.05, vals[0] * 0.92),
            color=light,
            fontsize=8,
            fontweight="bold"
        )

        pct = (
            100 * (1 - vals[-1] / vals[0])
            if vals[0] > 1e-9
            else 0
        )

        ax.annotate(
            f"End: {vals[-1]:.4f}  (−{pct:.0f}%)",
            xy=(ekf_t[-1], vals[-1]),
            xytext=(
                ekf_t[-1] * 0.55,
                vals[-1] + (vals.max() - vals[-1]) * 0.25
            ),
            arrowprops=dict(
                arrowstyle="->",
                color="#2ECC71"
            ),
            color="#2ECC71",
            fontsize=9,
            fontweight="bold"
        )

        ax.set_ylim(bottom=0)

        style_ax(
            ax,
            rtitle,
            "Time (s)",
            col_name
        )

        EKF_tag(ax)

        h = [
            Line2D(
                [0],
                [0],
                color=dark,
                lw=1.8,
                label=col_name
            )
        ] + sensor_legend

        ax.legend(
            handles=h,
            loc="upper right",
            fontsize=7,
            ncol=2
        )

    save_fig(
        fig,
        out_dir,
        "group5_uncertainty.png",
        do_save
    )



# ═══════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="AUV EKF — 5-Group Comparison Plots "
                    "(Raw Sensor Data vs EKF Output)"
    )

    parser.add_argument(
        "--data",
        default="data",
        help="Folder containing sensor CSV files"
    )

    parser.add_argument(
        "--results",
        default="results",
        help="Folder containing nav_output.csv"
    )

    parser.add_argument(
        "--out",
        default="plots",
        help="Output folder for generated PNGs"
    )

    parser.add_argument(
        "--save",
        action="store_true",
        help="Save figures instead of displaying them"
    )

    args = parser.parse_args()

    print("\n" + "=" * 70)
    print("     AUV EKF Navigation — 5-Group Comparison Plots")
    print("     Raw Sensor Data  vs  EKF nav_output.csv")
    print("=" * 70)

    print("\nLoading CSV files ...")

    ekf = load_csv(
        os.path.join(args.results, "nav_output.csv"),
        "EKF Output"
    )

    imu = load_csv(
        os.path.join(args.data, "imu.csv"),
        "IMU"
    )

    dvl = load_csv(
        os.path.join(args.data, "dvl.csv"),
        "DVL"
    )

    depth = load_csv(
        os.path.join(args.data, "depth.csv"),
        "Depth"
    )

    gps = load_csv(
        os.path.join(args.data, "gps.csv"),
        "GPS"
    )

    mag = load_csv(
        os.path.join(args.data, "mag.csv"),
        "Magnetometer"
    )

    if ekf is None:
        sys.exit(
            "\n[ERROR] nav_output.csv not found. "
            "Run ./auv_nav2 first."
        )

    print("\nGenerating 5 group figures ...")
    print("  LEFT  panel = Raw Sensor Data (BEFORE EKF)")
    print("  RIGHT panel = EKF Output      (AFTER EKF)\n")

    try:
        group1_position(
            ekf, gps, depth,
            args.out, args.save
        )

        group2_velocity(
            ekf, dvl,
            args.out, args.save
        )

        group3_attitude(
            ekf, imu, mag,
            args.out, args.save
        )

        group4_derived(
            ekf, dvl, depth,
            args.out, args.save
        )

        group5_uncertainty(
            ekf,
            args.out, args.save
        )

        # group6_flags(
        #     ekf, imu, dvl,
        #     depth, gps, mag,
        #     args.out, args.save
        # )

    except Exception as e:
        import traceback

        print(f"\n[ERROR] {e}")
        traceback.print_exc()
        sys.exit(1)

    if args.save:
        print(
            f"\n[OK] All 6 plots saved to:\n"
            f"     {os.path.abspath(args.out)}/"
        )
    else:
        print(
            "\n[OK] 5 figures generated successfully."
        )
        print(
            "Close all matplotlib windows to exit."
        )
        plt.show()

    print("=" * 70)


if __name__ == "__main__":
    main()