#!/usr/bin/env python3
"""
Straight-Line PID Ki/Kd Sweep — Find optimal gains for heading hold.

Uses the physics model from sim.py with intentional L/R motor asymmetry
(extracted from run #90 data) to simulate 1m straight drives across a grid
of Ki and Kd values. Reports yaw drift, PWM differential, lateral drift,
and overall tracking quality.

Usage:
    python3 tools/sweep_straight_pid.py                    # run sweep
    python3 tools/sweep_straight_pid.py --csv logs/runlog_90.csv  # use specific log
    python3 tools/sweep_straight_pid.py --no-plot          # text only

Requires: pip install numpy matplotlib
"""

import argparse
import math
import sys
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Tuple

import numpy as np

# Add parent so we can import from tools/sim.py if needed
sys.path.insert(0, str(Path(__file__).resolve().parent))

from sim import (
    parse_csv, segment_run, extract_motor_model,
    MotorModel, DrivePIDParams, SimState, TRACK_WIDTH_M,
    _wrap_deg, find_latest_csv,
)


# ---------------------------------------------------------------------------
# Straight-line DrivePID (matches firmware exactly)
# ---------------------------------------------------------------------------

class StraightDrivePID:
    """Firmware-identical steering PID for straight test."""

    def __init__(self, kp: float, ki: float, kd: float,
                 i_leak_tau: float = 1.0, i_max_corr: float = 25.0,
                 d_filter_tau: float = 0.030, corr_clamp: float = 60.0,
                 drive_pwm: float = 130.0, motor_bias_pwm: int = 1,
                 decel_boost_kp: float = 1.5, decel_zone_m: float = 0.30,
                 speed_decel_zone_m: float = 0.20, speed_decel_floor: float = 0.30):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_leak_tau = i_leak_tau
        self.i_max_corr = i_max_corr
        self.d_filter_tau = d_filter_tau
        self.corr_clamp = corr_clamp
        self.drive_pwm = drive_pwm
        self.motor_bias_pwm = motor_bias_pwm
        self.decel_boost_kp = decel_boost_kp
        self.decel_zone_m = decel_zone_m
        self.speed_decel_zone_m = speed_decel_zone_m
        self.speed_decel_floor = speed_decel_floor

        self.integral = 0.0
        self.prev_err = 0.0
        self.d_filt = 0.0
        self.first = True

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0
        self.d_filt = 0.0
        self.first = True

    def update(self, yaw_err: float, driven_m: float, target_m: float,
               dt: float) -> Tuple[float, float, float, float]:
        """Returns (left_pwm, right_pwm, steer_corr, base_pwm)."""
        if self.first:
            self.first = False
            self.prev_err = yaw_err
            dt = 0.010

        # Leaky integral
        self.integral -= self.integral * (dt / self.i_leak_tau)
        if abs(yaw_err) > 0.1:
            self.integral += yaw_err * dt

        # Anti-windup
        i_lim = self.i_max_corr / self.ki if self.ki > 0 else 100
        self.integral = max(-i_lim, min(i_lim, self.integral))

        # Filtered derivative
        raw_deriv = (yaw_err - self.prev_err) / dt if dt > 0 else 0
        self.prev_err = yaw_err
        alpha = dt / (self.d_filter_tau + dt)
        self.d_filt += alpha * (raw_deriv - self.d_filt)

        # Effective Kp (boost in decel zone)
        remaining = target_m - driven_m
        kp = self.kp
        if remaining < self.decel_zone_m:
            kp *= self.decel_boost_kp

        steer_corr = kp * yaw_err + self.ki * self.integral + self.kd * self.d_filt
        steer_corr = max(-self.corr_clamp, min(self.corr_clamp, steer_corr))

        # Speed factor (decel near end)
        speed_factor = 1.0
        if remaining < self.speed_decel_zone_m:
            speed_factor = remaining / self.speed_decel_zone_m
            speed_factor = max(self.speed_decel_floor, speed_factor)

        base_pwm = self.drive_pwm * speed_factor
        base_pwm = max(110.0, base_pwm)

        # Mixer: positive steerCorr → slow left, speed right
        left_pwm = base_pwm - steer_corr + self.motor_bias_pwm
        right_pwm = base_pwm + steer_corr - self.motor_bias_pwm

        left_pwm = max(0, min(255, left_pwm))
        right_pwm = max(0, min(255, right_pwm))
        if 0 < left_pwm < 100:
            left_pwm = 100
        if 0 < right_pwm < 110:
            right_pwm = 110

        return left_pwm, right_pwm, steer_corr, base_pwm


# ---------------------------------------------------------------------------
# Straight-line simulation
# ---------------------------------------------------------------------------

@dataclass
class StraightRecord:
    t: float
    x: float
    y: float
    yaw: float
    yaw_err: float
    l_pwm: float
    r_pwm: float
    steer_corr: float
    steer_i: float
    driven_m: float
    dist_l: float
    dist_r: float
    base_pwm: float


def simulate_straight(motor: MotorModel, kp: float, ki: float, kd: float,
                      target_m: float = 1.0, drive_pwm: float = 130.0,
                      dt: float = 0.010, motor_bias_pwm: int = 1,
                      initial_yaw_offset: float = 0.0) -> List[StraightRecord]:
    """Simulate a straight drive with given PID gains and motor model."""
    pid = StraightDrivePID(kp=kp, ki=ki, kd=kd,
                           drive_pwm=drive_pwm,
                           motor_bias_pwm=motor_bias_pwm)
    pid.reset()

    state = SimState(yaw=initial_yaw_offset)
    records = []
    driven = 0.0
    enc_start_l = 0.0
    enc_start_r = 0.0
    target_yaw = 0.0
    max_time = 30.0  # safety

    while driven < target_m - 0.08 and state.t < max_time:
        yaw_err = _wrap_deg(target_yaw - state.yaw)
        left_pwm, right_pwm, steer_corr, base_pwm = pid.update(
            yaw_err, driven, target_m, dt)

        # Motor model: PWM → wheel speed
        vl = motor.gain_l * max(0, left_pwm - motor.deadband_l) if left_pwm > motor.deadband_l else 0
        vr = motor.gain_r * max(0, right_pwm - motor.deadband_r) if right_pwm > motor.deadband_r else 0

        # Integrate position
        v_avg = (vl + vr) / 2.0
        omega = (vr - vl) / TRACK_WIDTH_M  # rad/s
        heading_rad = math.radians(state.yaw)
        state.x += v_avg * math.cos(heading_rad) * dt
        state.y += v_avg * math.sin(heading_rad) * dt
        state.yaw = _wrap_deg(state.yaw + math.degrees(omega) * dt)

        state.dist_l += vl * dt
        state.dist_r += vr * dt
        driven = ((state.dist_l - enc_start_l) + (state.dist_r - enc_start_r)) / 2.0

        state.t += dt

        records.append(StraightRecord(
            t=state.t, x=state.x, y=state.y,
            yaw=state.yaw, yaw_err=yaw_err,
            l_pwm=left_pwm, r_pwm=right_pwm,
            steer_corr=steer_corr, steer_i=pid.integral,
            driven_m=driven, dist_l=state.dist_l, dist_r=state.dist_r,
            base_pwm=base_pwm,
        ))

    return records


# ---------------------------------------------------------------------------
# Metrics for a straight run
# ---------------------------------------------------------------------------

@dataclass
class StraightMetrics:
    label: str
    kp: float
    ki: float
    kd: float
    yaw_rms_deg: float = 0.0
    yaw_max_deg: float = 0.0
    yaw_final_deg: float = 0.0
    lateral_drift_m: float = 0.0
    pwm_diff_rms: float = 0.0      # RMS of |lPwm - rPwm|
    pwm_diff_max: float = 0.0
    enc_diff_mm: float = 0.0       # |distL - distR| at end
    total_time_s: float = 0.0
    # Composite score (lower = better): yaw_rms + lateral + pwm_oscillation
    score: float = 0.0


def compute_straight_metrics(records: List[StraightRecord],
                             label: str, kp: float, ki: float, kd: float) -> StraightMetrics:
    m = StraightMetrics(label=label, kp=kp, ki=ki, kd=kd)
    if not records:
        m.score = 999.0
        return m

    # Skip first 10 samples (startup transient)
    steady = records[10:] if len(records) > 15 else records

    yaw_errs = [abs(r.yaw_err) for r in steady]
    m.yaw_rms_deg = float(np.sqrt(np.mean(np.array(yaw_errs) ** 2)))
    m.yaw_max_deg = max(yaw_errs)
    m.yaw_final_deg = abs(records[-1].yaw)

    m.lateral_drift_m = abs(records[-1].y)

    pwm_diffs = [abs(r.l_pwm - r.r_pwm) for r in steady]
    m.pwm_diff_rms = float(np.sqrt(np.mean(np.array(pwm_diffs) ** 2)))
    m.pwm_diff_max = max(pwm_diffs)

    m.enc_diff_mm = abs(records[-1].dist_l - records[-1].dist_r) * 1000

    m.total_time_s = records[-1].t

    # Composite score: weighted combination
    # - yaw_rms: heading accuracy (weight 3)
    # - lateral_drift: physical accuracy (weight 200 to convert m→cm-like scale)
    # - pwm_diff_rms: motor stress/smoothness (weight 0.1)
    m.score = 3.0 * m.yaw_rms_deg + 200.0 * m.lateral_drift_m + 0.1 * m.pwm_diff_rms

    return m


# ---------------------------------------------------------------------------
# Sweep
# ---------------------------------------------------------------------------

def run_sweep(motor: MotorModel, target_m: float = 1.0,
              drive_pwm: float = 130.0) -> List[StraightMetrics]:
    """Sweep Ki and Kd values with fixed Kp=2.0."""
    kp = 2.0  # fixed (firmware default)

    ki_values = [0.5, 0.8, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0]
    kd_values = [0.02, 0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.50]

    results = []
    print(f"\n{'='*90}")
    print(f"  STRAIGHT-LINE PID SWEEP  (Kp={kp}, target={target_m}m, basePWM={drive_pwm})")
    print(f"  Motor asymmetry: gain_L={motor.gain_l:.5f}  gain_R={motor.gain_r:.5f}  "
          f"(ratio={motor.gain_l/motor.gain_r:.3f})" if motor.gain_r > 0 else "")
    print(f"{'='*90}")
    print(f"  {'Ki':>5s}  {'Kd':>5s}  | {'yawRMS':>7s}  {'yawMax':>7s}  {'latDrift':>9s}  "
          f"{'pwmDiffRMS':>10s}  {'pwmDiffMax':>10s}  {'encDiff':>8s}  {'time':>5s}  {'SCORE':>7s}")
    print(f"  {'-'*5}  {'-'*5}  | {'-'*7}  {'-'*7}  {'-'*9}  {'-'*10}  {'-'*10}  {'-'*8}  {'-'*5}  {'-'*7}")

    for ki in ki_values:
        for kd in kd_values:
            label = f"Ki={ki:.1f} Kd={kd:.2f}"
            recs = simulate_straight(motor, kp=kp, ki=ki, kd=kd,
                                     target_m=target_m, drive_pwm=drive_pwm)
            m = compute_straight_metrics(recs, label, kp, ki, kd)
            results.append(m)

            print(f"  {ki:5.1f}  {kd:5.2f}  | {m.yaw_rms_deg:7.2f}°  {m.yaw_max_deg:7.2f}°  "
                  f"{m.lateral_drift_m*100:8.2f}cm  {m.pwm_diff_rms:10.1f}  {m.pwm_diff_max:10.1f}  "
                  f"{m.enc_diff_mm:7.1f}mm  {m.total_time_s:5.2f}s  {m.score:7.2f}")

    # Sort by score and show top 10
    results.sort(key=lambda x: x.score)
    print(f"\n{'='*90}")
    print("  TOP 10 BEST CONFIGURATIONS (lowest score = best tracking)")
    print(f"{'='*90}")
    for i, m in enumerate(results[:10]):
        marker = " <-- CURRENT" if abs(m.ki - 0.8) < 0.01 and abs(m.kd - 0.05) < 0.01 else ""
        marker = " <-- BACKUP" if abs(m.ki - 4.0) < 0.01 and abs(m.kd - 0.20) < 0.01 else marker
        print(f"  #{i+1:2d}  Ki={m.ki:4.1f}  Kd={m.kd:5.2f}  "
              f"yawRMS={m.yaw_rms_deg:.2f}°  lateral={m.lateral_drift_m*100:.2f}cm  "
              f"pwmDiff={m.pwm_diff_rms:.1f}  score={m.score:.2f}{marker}")

    # Show where current and backup fall in ranking
    print(f"\n  --- Specific configurations ---")
    for m in results:
        if abs(m.ki - 0.8) < 0.01 and abs(m.kd - 0.05) < 0.01:
            rank = results.index(m) + 1
            print(f"  CURRENT (Ki=0.8 Kd=0.05): rank #{rank}/{len(results)}  score={m.score:.2f}  "
                  f"yawRMS={m.yaw_rms_deg:.2f}°  lateral={m.lateral_drift_m*100:.2f}cm")
        if abs(m.ki - 4.0) < 0.01 and abs(m.kd - 0.20) < 0.01:
            rank = results.index(m) + 1
            print(f"  BACKUP  (Ki=4.0 Kd=0.20): rank #{rank}/{len(results)}  score={m.score:.2f}  "
                  f"yawRMS={m.yaw_rms_deg:.2f}°  lateral={m.lateral_drift_m*100:.2f}cm")

    return results


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_sweep_heatmap(results: List[StraightMetrics], title: str = ""):
    import matplotlib.pyplot as plt

    ki_vals = sorted(set(m.ki for m in results))
    kd_vals = sorted(set(m.kd for m in results))

    # Build matrices
    score_mat = np.full((len(ki_vals), len(kd_vals)), np.nan)
    yaw_mat = np.full((len(ki_vals), len(kd_vals)), np.nan)
    lat_mat = np.full((len(ki_vals), len(kd_vals)), np.nan)
    pwm_mat = np.full((len(ki_vals), len(kd_vals)), np.nan)

    for m in results:
        i = ki_vals.index(m.ki)
        j = kd_vals.index(m.kd)
        score_mat[i, j] = m.score
        yaw_mat[i, j] = m.yaw_rms_deg
        lat_mat[i, j] = m.lateral_drift_m * 100  # cm
        pwm_mat[i, j] = m.pwm_diff_rms

    fig, axes = plt.subplots(2, 2, figsize=(14, 11))
    fig.suptitle(title or "Straight-Line PID Ki/Kd Sweep", fontsize=14, fontweight="bold")

    kd_labels = [f"{v:.2f}" for v in kd_vals]
    ki_labels = [f"{v:.1f}" for v in ki_vals]

    for ax, mat, name, cmap in [
        (axes[0, 0], score_mat, "Composite Score (lower=better)", "RdYlGn_r"),
        (axes[0, 1], yaw_mat, "Yaw RMS (deg)", "Reds"),
        (axes[1, 0], lat_mat, "Lateral Drift (cm)", "Oranges"),
        (axes[1, 1], pwm_mat, "PWM Diff RMS", "Blues"),
    ]:
        im = ax.imshow(mat, aspect="auto", cmap=cmap, origin="lower")
        ax.set_xticks(range(len(kd_vals)))
        ax.set_xticklabels(kd_labels, fontsize=8)
        ax.set_yticks(range(len(ki_vals)))
        ax.set_yticklabels(ki_labels, fontsize=8)
        ax.set_xlabel("Kd")
        ax.set_ylabel("Ki")
        ax.set_title(name)
        fig.colorbar(im, ax=ax, shrink=0.8)

        # Annotate cells
        for i in range(len(ki_vals)):
            for j in range(len(kd_vals)):
                val = mat[i, j]
                if not np.isnan(val):
                    ax.text(j, i, f"{val:.1f}", ha="center", va="center",
                            fontsize=6, color="white" if val > np.nanmedian(mat) else "black")

        # Mark current and backup
        for m in results:
            if abs(m.ki - 0.8) < 0.01 and abs(m.kd - 0.05) < 0.01:
                i = ki_vals.index(m.ki)
                j = kd_vals.index(m.kd)
                ax.plot(j, i, "kx", markersize=12, markeredgewidth=2)
            if abs(m.ki - 4.0) < 0.01 and abs(m.kd - 0.20) < 0.01:
                i = ki_vals.index(m.ki)
                j = kd_vals.index(m.kd)
                ax.plot(j, i, "k*", markersize=14, markeredgewidth=1)

    # Legend
    fig.text(0.02, 0.02, "X = Current (Ki=0.8, Kd=0.05)   ★ = Backup (Ki=4.0, Kd=0.20)",
             fontsize=10, fontstyle="italic")

    fig.tight_layout(rect=[0, 0.04, 1, 0.96])
    return fig


def plot_top_runs(motor: MotorModel, top_results: List[StraightMetrics],
                  target_m: float = 1.0, title: str = ""):
    """Plot time-series for the top N configurations + current + backup."""
    import matplotlib.pyplot as plt

    # Always include current and backup for comparison
    configs = []
    # Add top 3
    for m in top_results[:3]:
        configs.append((f"#{top_results.index(m)+1} Ki={m.ki:.1f} Kd={m.kd:.2f} (score={m.score:.1f})",
                        m.ki, m.kd, "solid"))
    # Add current if not in top 3
    if not any(abs(m.ki - 0.8) < 0.01 and abs(m.kd - 0.05) < 0.01 for m in top_results[:3]):
        configs.append(("CURRENT Ki=0.8 Kd=0.05", 0.8, 0.05, "dashed"))
    # Add backup if not in top 3
    if not any(abs(m.ki - 4.0) < 0.01 and abs(m.kd - 0.20) < 0.01 for m in top_results[:3]):
        configs.append(("BACKUP Ki=4.0 Kd=0.20", 4.0, 0.20, "dashdot"))

    fig, axes = plt.subplots(4, 1, figsize=(14, 14))
    fig.suptitle(title or "Top Configurations — Straight Line 1m", fontsize=14, fontweight="bold")
    colors = plt.cm.tab10.colors

    sims = []
    for label, ki, kd, ls in configs:
        recs = simulate_straight(motor, kp=2.0, ki=ki, kd=kd, target_m=target_m)
        sims.append((label, recs, ls))

    # Yaw
    ax = axes[0]
    ax.set_title("Heading (Yaw)")
    ax.set_ylabel("Yaw (deg)")
    ax.axhline(0, color="gray", linewidth=0.5)
    ax.grid(True, alpha=0.3)
    for idx, (label, recs, ls) in enumerate(sims):
        t = [r.t for r in recs]
        y = [r.yaw for r in recs]
        ax.plot(t, y, linestyle=ls, color=colors[idx % len(colors)], linewidth=1.2, label=label)
    ax.legend(fontsize=7)

    # PWM L/R
    ax = axes[1]
    ax.set_title("Motor PWM (L=solid, R=dashed)")
    ax.set_ylabel("PWM")
    ax.grid(True, alpha=0.3)
    for idx, (label, recs, ls) in enumerate(sims):
        t = [r.t for r in recs]
        color = colors[idx % len(colors)]
        ax.plot(t, [r.l_pwm for r in recs], "-", color=color, linewidth=0.8, alpha=0.8,
                label=f"{label} L")
        ax.plot(t, [r.r_pwm for r in recs], "--", color=color, linewidth=0.8, alpha=0.6)
    ax.legend(fontsize=6, ncol=2)

    # Steering correction
    ax = axes[2]
    ax.set_title("Steering Correction")
    ax.set_ylabel("corrOut")
    ax.axhline(0, color="gray", linewidth=0.5)
    ax.grid(True, alpha=0.3)
    for idx, (label, recs, ls) in enumerate(sims):
        t = [r.t for r in recs]
        ax.plot(t, [r.steer_corr for r in recs], linestyle=ls,
                color=colors[idx % len(colors)], linewidth=1, label=label)
    ax.legend(fontsize=7)

    # Lateral drift (Y position)
    ax = axes[3]
    ax.set_title("Lateral Drift (Y position)")
    ax.set_ylabel("Y (m)")
    ax.set_xlabel("Time (s)")
    ax.axhline(0, color="gray", linewidth=0.5)
    ax.grid(True, alpha=0.3)
    for idx, (label, recs, ls) in enumerate(sims):
        t = [r.t for r in recs]
        ax.plot(t, [r.y for r in recs], linestyle=ls,
                color=colors[idx % len(colors)], linewidth=1.2, label=label)
    ax.legend(fontsize=7)

    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Straight-Line PID Ki/Kd Sweep")
    parser.add_argument("--csv", help="CSV log to extract motor model from")
    parser.add_argument("--no-plot", action="store_true", help="Text only, no plots")
    parser.add_argument("--save", help="Save plots to file prefix (e.g., 'sweep' → sweep_heatmap.png)")
    parser.add_argument("--target", type=float, default=1.0, help="Target distance (m)")
    parser.add_argument("--asymmetry", type=float, default=None,
                        help="Override L/R gain ratio (e.g., 1.08 means L is 8%% faster)")
    args = parser.parse_args()

    # Load motor model from CSV or use defaults based on run #90
    motor = MotorModel()
    if args.csv and os.path.exists(args.csv):
        print(f"Loading motor model from: {args.csv}")
        header, samples = parse_csv(args.csv)
        segments = segment_run(samples)
        motor = extract_motor_model(segments, samples)
        print(f"  Extracted: gain_L={motor.gain_l:.5f}  gain_R={motor.gain_r:.5f}")
    else:
        csv_path = find_latest_csv()
        loaded = False
        if csv_path and os.path.exists(csv_path):
            print(f"Loading motor model from: {csv_path}")
            header, samples = parse_csv(csv_path)
            segments = segment_run(samples)
            motor = extract_motor_model(segments, samples)
            if motor.gain_l > 0 and motor.gain_r > 0:
                print(f"  Extracted: gain_L={motor.gain_l:.5f}  gain_R={motor.gain_r:.5f}")
                loaded = True

        if not loaded:
            # Fallback: model from run #90 data
            # Left traveled 0.9048m, right 0.8377m at ~130 PWM over same time
            # → left is ~8% faster at same PWM
            print("Using default motor model (based on run #90 asymmetry)")
            motor.gain_l = 0.0100   # m/s per PWM above deadband
            motor.gain_r = 0.0092   # ~8% slower right motor
            motor.deadband_l = 95.0
            motor.deadband_r = 105.0

    # Fallback safety
    if motor.gain_l <= 0:
        motor.gain_l = 0.0100
    if motor.gain_r <= 0:
        motor.gain_r = 0.0092

    # Override asymmetry if requested
    if args.asymmetry is not None:
        base_gain = (motor.gain_l + motor.gain_r) / 2
        motor.gain_l = base_gain * math.sqrt(args.asymmetry)
        motor.gain_r = base_gain / math.sqrt(args.asymmetry)
        print(f"  Overridden: gain_L={motor.gain_l:.5f}  gain_R={motor.gain_r:.5f}  "
              f"(ratio={args.asymmetry:.3f})")

    ratio = motor.gain_l / motor.gain_r if motor.gain_r > 0 else 0
    print(f"\n  Motor model for sweep:")
    print(f"    gain_L = {motor.gain_l:.5f} m/s/PWM   deadband_L = {motor.deadband_l:.0f}")
    print(f"    gain_R = {motor.gain_r:.5f} m/s/PWM   deadband_R = {motor.deadband_r:.0f}")
    print(f"    L/R ratio = {ratio:.4f}  ({(ratio-1)*100:+.1f}% asymmetry)")

    # Run sweep
    results = run_sweep(motor, target_m=args.target)

    if not args.no_plot:
        import matplotlib.pyplot as plt

        # Heatmap
        fig1 = plot_sweep_heatmap(results, f"Ki/Kd Sweep — L/R asymmetry {(ratio-1)*100:+.1f}%")
        if args.save:
            fig1.savefig(f"{args.save}_heatmap.png", dpi=150, bbox_inches="tight")
            print(f"\nSaved: {args.save}_heatmap.png")

        # Top runs time-series
        results_sorted = sorted(results, key=lambda x: x.score)
        fig2 = plot_top_runs(motor, results_sorted, target_m=args.target,
                             title=f"Top Configs vs Current/Backup — {(ratio-1)*100:+.1f}% motor asymmetry")
        if args.save:
            fig2.savefig(f"{args.save}_timeseries.png", dpi=150, bbox_inches="tight")
            print(f"Saved: {args.save}_timeseries.png")

        if not args.save:
            plt.show()


if __name__ == "__main__":
    main()
