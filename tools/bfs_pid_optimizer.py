#!/usr/bin/env python3
"""
BFS Drive PID Optimizer — Physics Simulation

Extracts a yaw-rate plant model (PWM differential → yaw rate) from BFS drive
log data, then replays drive segments with candidate PID gains to find optimal
KP, KI, KD that minimize yaw tracking error.

Usage:
    python3 tools/bfs_pid_optimizer.py                        # latest log
    python3 tools/bfs_pid_optimizer.py logs/..._107.csv       # specific log
    python3 tools/bfs_pid_optimizer.py --no-plot               # text only
    python3 tools/bfs_pid_optimizer.py --all-runs              # use runs 107-109

Requires: pip install numpy scipy matplotlib
"""

import argparse
import csv
import glob
import math
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Tuple, Optional, Dict
import numpy as np

try:
    from scipy.optimize import minimize
except ImportError:
    minimize = None

# ─── Firmware constants (must match src/main.cpp) ───────────────────────────
MOTOR_BIAS_PWM    = 1       # from CSV header mtr_bias=1 (prefs override)
MIN_PWM_L         = 100
MIN_PWM_R         = 110
CORR_CLAMP        = 60.0
STEER_RAMP_MS     = 150     # BFS_DRIVE_STEER_RAMP_MS
DECEL_ZONE_M      = 0.30
DECEL_BOOST_KP    = 1.5
I_LEAK_TAU        = 1.0
I_MAX_CORR        = 25.0
D_FILTER_TAU      = 0.030
ARRIVE_TOL_M      = 0.08

# Current firmware PID gains
CURRENT_KP = 2.0
CURRENT_KI = 1.0
CURRENT_KD = 0.06


# ─── CSV parsing ────────────────────────────────────────────────────────────

@dataclass
class Sample:
    t_ms: float
    meters: float
    currentYaw: float
    targetYaw: float
    yawErrDeg: float
    gz_dps: float
    basePWM: float
    corrOut: float
    lPwm: float
    rPwm: float
    remainingM: float
    steerI: float
    distL: float
    distR: float


def parse_log(path: str) -> Tuple[Dict, List[Sample]]:
    """Parse a BFS drive log CSV file."""
    header_info = {}
    samples = []
    with open(path, newline="") as f:
        lines = f.readlines()

    data_lines = []
    for line in lines:
        line = line.strip()
        if not line:
            continue
        if line.startswith("# TEST_BEGIN"):
            for tok in line.split():
                if "=" in tok:
                    k, v = tok.split("=", 1)
                    header_info[k] = v
            continue
        if line.startswith("#"):
            continue
        data_lines.append(line)

    if len(data_lines) < 2:
        return header_info, samples

    reader = csv.DictReader(data_lines)
    for row in reader:
        try:
            s = Sample(
                t_ms=float(row["t_ms"]),
                meters=float(row["meters"]),
                currentYaw=float(row["currentYaw"]),
                targetYaw=float(row["targetYaw"]),
                yawErrDeg=float(row["yawErrDeg"]),
                gz_dps=float(row["gz_dps"]),
                basePWM=float(row["basePWM"]),
                corrOut=float(row["corrOut"]),
                lPwm=float(row["lPwm"]),
                rPwm=float(row["rPwm"]),
                remainingM=float(row["remainingM"]),
                steerI=float(row["steerI"]),
                distL=float(row["distL"]),
                distR=float(row["distR"]),
            )
            samples.append(s)
        except (ValueError, KeyError):
            continue
    return header_info, samples


def segment_by_gaps(samples: List[Sample], gap_ms: float = 400) -> List[List[Sample]]:
    """Split into drive segments by time gaps (turns appear as gaps)."""
    if not samples:
        return []
    segs = []
    start = 0
    for i in range(1, len(samples)):
        if samples[i].t_ms - samples[i - 1].t_ms > gap_ms:
            segs.append(samples[start:i])
            start = i
    segs.append(samples[start:])
    return [s for s in segs if len(s) >= 5]


def wrap_deg(d: float) -> float:
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return d


# ─── Plant model identification ─────────────────────────────────────────────

@dataclass
class PlantModel:
    """First-order yaw-rate plant: τ·dω/dt + ω = K·Δpwm + bias"""
    K_yaw: float = 0.0       # deg/s per unit PWM differential
    drift_bias: float = 0.0  # inherent yaw rate bias (deg/s) when Δpwm=0
    tau_motor: float = 0.08  # motor response time constant (seconds)


def identify_plant(segments: List[List[Sample]]) -> PlantModel:
    """Fit plant parameters by forward simulation matching.

    Uses physically constrained parameter ranges:
    - K_yaw: 0.05-0.5 dps per PWM-diff (from motor/track geometry)
    - drift_bias: -8 to 0 dps (inherent rightward drift seen in all runs)
    - tau_motor: 0.03-0.50 s (motor+inertia response lag)
    """
    current_gains = PIDGains(CURRENT_KP, CURRENT_KI, CURRENT_KD)

    def sim_error(params):
        K_yaw, drift_bias, tau_motor = params
        # Hard bounds
        if K_yaw < 0.01 or K_yaw > 1.0:
            return 1e6
        if drift_bias < -12 or drift_bias > 5:
            return 1e6
        if tau_motor < 0.02 or tau_motor > 1.0:
            return 1e6
        plant = PlantModel(K_yaw=K_yaw, drift_bias=drift_bias, tau_motor=tau_motor)
        total_err = 0.0
        n_pts = 0
        for seg in segments:
            results = simulate_segment(seg, plant, current_gains)
            if len(results) != len(seg):
                continue
            for s, r in zip(seg, results):
                yaw_diff = wrap_deg(s.currentYaw - r.yaw)
                total_err += yaw_diff ** 2
                n_pts += 1
        return total_err / max(n_pts, 1)

    # Grid search with physically reasonable ranges
    best_err = 1e9
    best_params = [0.15, -4.0, 0.10]
    print("  Identifying plant model (constrained grid search)...")
    for K in np.arange(0.05, 0.51, 0.025):
        for bias in np.arange(-8.0, 0.5, 0.5):
            for tau in [0.03, 0.06, 0.10, 0.15, 0.20, 0.30, 0.40, 0.50]:
                err = sim_error([K, bias, tau])
                if err < best_err:
                    best_err = err
                    best_params = [K, bias, tau]

    print(f"  Grid best: K={best_params[0]:.4f} bias={best_params[1]:.1f} "
          f"tau={best_params[2]:.3f} err={best_err:.2f}")

    # Refine with bounded Nelder-Mead
    if minimize is not None:
        # Use penalty to enforce bounds
        def bounded_sim_error(params):
            K, b, t = params
            penalty = 0.0
            if K < 0.03: penalty += (0.03 - K) * 1000
            if K > 0.8: penalty += (K - 0.8) * 1000
            if b < -10: penalty += (-10 - b) * 100
            if b > 3: penalty += (b - 3) * 100
            if t < 0.02: penalty += (0.02 - t) * 1000
            if t > 0.8: penalty += (t - 0.8) * 1000
            return sim_error(params) + penalty

        result = minimize(bounded_sim_error, best_params, method="Nelder-Mead",
                          options={"xatol": 0.001, "fatol": 0.01, "maxiter": 500})
        if result.fun < best_err + 1:  # allow small penalty
            refined = list(result.x)
            refined[0] = np.clip(refined[0], 0.03, 0.8)
            refined[1] = np.clip(refined[1], -10, 3)
            refined[2] = np.clip(refined[2], 0.02, 0.8)
            ref_err = sim_error(refined)
            if ref_err < best_err:
                best_params = refined
                best_err = ref_err
        print(f"  Refined:   K={best_params[0]:.4f} bias={best_params[1]:.2f} "
              f"tau={best_params[2]:.4f} err={best_err:.2f}")

    model = PlantModel(
        K_yaw=best_params[0],
        drift_bias=best_params[1],
        tau_motor=best_params[2],
    )
    return model


# ─── PID + Plant simulation ─────────────────────────────────────────────────

@dataclass
class PIDGains:
    kp: float = 2.0
    ki: float = 1.0
    kd: float = 0.06


@dataclass
class SimResult:
    """Per-timestep simulation output."""
    t_ms: float
    yaw: float
    target_yaw: float
    yaw_err: float
    gz_dps: float
    steer_corr: float
    steer_i: float
    l_pwm: float
    r_pwm: float
    base_pwm: float


def simulate_segment(seg: List[Sample], plant: PlantModel, gains: PIDGains,
                     motor_bias: int = MOTOR_BIAS_PWM) -> List[SimResult]:
    """
    Replay a drive segment with given PID gains and plant model.

    Uses actual logged basePWM (interpolated by time) but simulates yaw
    trajectory from scratch.
    """
    if len(seg) < 3:
        return []

    results = []
    target_yaw = seg[0].targetYaw
    segment_dist = seg[0].meters + seg[0].remainingM  # total segment distance

    # Initial conditions from the real segment start
    yaw = seg[0].currentYaw
    gz_dps = seg[0].gz_dps
    integral = 0.0
    prev_err = wrap_deg(target_yaw - yaw)
    d_filt = 0.0
    first_step = True

    # Build time-indexed basePWM lookup from real data
    times = np.array([s.t_ms for s in seg])
    base_pwms = np.array([s.basePWM for s in seg])
    distances = np.array([s.meters for s in seg])

    # Simulate at ~10ms steps using the actual time grid
    seg_start_t = seg[0].t_ms

    for i in range(len(seg)):
        t = seg[i].t_ms
        elapsed_ms = t - seg_start_t

        # Use actual basePWM and distance from log (these depend on speed scaling,
        # not PID gains)
        basePwm = seg[i].basePWM
        driven_m = seg[i].meters - seg[0].meters
        remaining = segment_dist - seg[0].meters - driven_m
        if remaining < 0:
            remaining = 0

        yaw_err = wrap_deg(target_yaw - yaw)

        # Compute dt
        if i == 0:
            dt = 0.010
        else:
            dt = (seg[i].t_ms - seg[i - 1].t_ms) / 1000.0
            if dt < 0.001:
                dt = 0.001
            if dt > 0.5:
                dt = 0.5

        # PID: initialize prev_err on first step to avoid derivative kick
        if first_step:
            prev_err = yaw_err
            first_step = False
            dt = 0.010

        # Leaky integral
        integral -= integral * (dt / I_LEAK_TAU)
        if abs(yaw_err) > 0.1:
            integral += yaw_err * dt
        i_lim = I_MAX_CORR / gains.ki if gains.ki > 0 else 100
        integral = max(-i_lim, min(i_lim, integral))

        # Filtered derivative
        raw_deriv = (yaw_err - prev_err) / dt if dt > 0 else 0
        prev_err = yaw_err
        alpha = dt / (D_FILTER_TAU + dt)
        d_filt += alpha * (raw_deriv - d_filt)

        # Effective Kp (boost in decel zone)
        effective_kp = gains.kp
        if remaining < DECEL_ZONE_M:
            effective_kp = gains.kp * DECEL_BOOST_KP

        steer_corr = (effective_kp * yaw_err
                      + gains.ki * integral
                      + gains.kd * d_filt)
        steer_corr = max(-CORR_CLAMP, min(CORR_CLAMP, steer_corr))

        # Steer ramp (first 150ms of segment)
        if elapsed_ms < STEER_RAMP_MS:
            ramp = elapsed_ms / STEER_RAMP_MS
            steer_corr *= ramp

        # Motor PWM
        l_pwm = basePwm - steer_corr + motor_bias
        r_pwm = basePwm + steer_corr - motor_bias

        # Clamp
        l_pwm = max(0, min(255, l_pwm))
        r_pwm = max(0, min(255, r_pwm))
        if 0 < l_pwm < MIN_PWM_L:
            l_pwm = MIN_PWM_L
        if 0 < r_pwm < MIN_PWM_R:
            r_pwm = MIN_PWM_R

        # Plant model: first-order response
        pwm_diff = r_pwm - l_pwm
        target_gz = plant.K_yaw * pwm_diff + plant.drift_bias
        alpha_m = dt / (plant.tau_motor + dt)
        gz_dps += alpha_m * (target_gz - gz_dps)

        # Integrate heading
        yaw = wrap_deg(yaw + gz_dps * dt)

        results.append(SimResult(
            t_ms=t, yaw=yaw, target_yaw=target_yaw,
            yaw_err=wrap_deg(target_yaw - yaw),
            gz_dps=gz_dps, steer_corr=steer_corr, steer_i=integral,
            l_pwm=l_pwm, r_pwm=r_pwm, base_pwm=basePwm,
        ))

    return results


# ─── Scoring ─────────────────────────────────────────────────────────────────

@dataclass
class SegmentScore:
    seg_idx: int
    n_samples: int
    rms_err: float
    max_err: float
    mean_abs_err: float
    floor_clamp_pct: float   # % of samples where leftPwm hit floor
    final_err: float


def score_segment(results: List[SimResult]) -> Optional[SegmentScore]:
    if len(results) < 3:
        return None
    errs = [abs(r.yaw_err) for r in results]
    floor_count = sum(1 for r in results if r.l_pwm <= MIN_PWM_L + 0.5)
    return SegmentScore(
        seg_idx=0,
        n_samples=len(results),
        rms_err=float(np.sqrt(np.mean(np.array(errs) ** 2))),
        max_err=float(np.max(errs)),
        mean_abs_err=float(np.mean(errs)),
        floor_clamp_pct=100.0 * floor_count / len(results),
        final_err=results[-1].yaw_err,
    )


def total_cost(segments: List[List[Sample]], plant: PlantModel,
               gains: PIDGains, motor_bias: int = MOTOR_BIAS_PWM) -> float:
    """Compute total cost (lower = better) across all segments."""
    costs = []
    for seg in segments:
        results = simulate_segment(seg, plant, gains, motor_bias)
        sc = score_segment(results)
        if sc is None:
            continue
        # Weighted cost: RMS error is primary, max error penalty, integral of error
        cost = sc.rms_err + 0.3 * sc.max_err + 0.2 * abs(sc.final_err)
        costs.append(cost)
    return float(np.mean(costs)) if costs else 999.0


# ─── Optimization ────────────────────────────────────────────────────────────

def grid_search(segments: List[List[Sample]], plant: PlantModel,
                motor_bias: int = MOTOR_BIAS_PWM) -> Tuple[PIDGains, float]:
    """Coarse grid search over KP, KI, KD with realistic bounds."""
    kp_range = np.arange(1.0, 8.1, 0.5)
    ki_range = np.arange(0.0, 6.1, 0.5)
    kd_range = np.arange(0.0, 0.55, 0.05)

    best_cost = 999.0
    best_gains = PIDGains()
    total = len(kp_range) * len(ki_range) * len(kd_range)
    count = 0

    print(f"\n  Grid search: {len(kp_range)}×{len(ki_range)}×{len(kd_range)} = {total} combinations")

    for kp in kp_range:
        for ki in ki_range:
            for kd in kd_range:
                count += 1
                gains = PIDGains(kp=kp, ki=ki, kd=kd)
                cost = total_cost(segments, plant, gains, motor_bias)
                if cost < best_cost:
                    best_cost = cost
                    best_gains = PIDGains(kp=kp, ki=ki, kd=kd)
        # Progress
        pct = 100.0 * count / total
        if count % (len(ki_range) * len(kd_range)) == 0:
            print(f"    {pct:.0f}% done ... best so far: KP={best_gains.kp:.1f} "
                  f"KI={best_gains.ki:.1f} KD={best_gains.kd:.2f} cost={best_cost:.3f}")

    return best_gains, best_cost


def refine_search(segments: List[List[Sample]], plant: PlantModel,
                  initial: PIDGains,
                  motor_bias: int = MOTOR_BIAS_PWM) -> Tuple[PIDGains, float]:
    """Fine-tune with Nelder-Mead optimization starting from grid search result."""
    if minimize is None:
        print("  scipy not available, skipping refinement")
        return initial, total_cost(segments, plant, initial, motor_bias)

    def objective(x):
        kp, ki, kd = x
        if kp < 0.5 or ki < 0 or kd < 0:
            return 999.0
        if kp > 10 or ki > 8 or kd > 0.8:
            return 999.0
        return total_cost(segments, plant, PIDGains(kp, ki, kd), motor_bias)

    x0 = [initial.kp, initial.ki, initial.kd]
    result = minimize(objective, x0, method="Nelder-Mead",
                      options={"xatol": 0.01, "fatol": 0.001,
                               "maxiter": 500, "adaptive": True})

    opt = PIDGains(kp=max(0.1, result.x[0]),
                   ki=max(0.0, result.x[1]),
                   kd=max(0.0, result.x[2]))
    return opt, result.fun


# ─── Visualization ───────────────────────────────────────────────────────────

def plot_segment_comparison(seg: List[Sample], results_map: Dict[str, List[SimResult]],
                            seg_idx: int, ax_err, ax_corr, ax_pwm):
    """Plot real vs simulated data for one segment on shared axes."""
    colors = {"Real": "black", "Current": "tab:blue", "Optimal": "tab:red",
              "Optimal+bias": "tab:green"}

    # Real data
    t0 = seg[0].t_ms
    rt = [(s.t_ms - t0) / 1000 for s in seg]
    ax_err.plot(rt, [s.yawErrDeg for s in seg], "k-", lw=1.5, alpha=0.7,
                label=f"Real (seg {seg_idx})")
    ax_corr.plot(rt, [s.corrOut for s in seg], "k-", lw=1.5, alpha=0.7)
    ax_pwm.plot(rt, [s.lPwm for s in seg], "k-", lw=0.8, alpha=0.5, label="Real L")
    ax_pwm.plot(rt, [s.rPwm for s in seg], "k--", lw=0.8, alpha=0.5, label="Real R")

    for label, results in results_map.items():
        if not results:
            continue
        color = colors.get(label, "tab:orange")
        st = [(r.t_ms - t0) / 1000 for r in results]
        ax_err.plot(st, [r.yaw_err for r in results], color=color, lw=1, label=label)
        ax_corr.plot(st, [r.steer_corr for r in results], color=color, lw=1, label=label)
        ax_pwm.plot(st, [r.l_pwm for r in results], color=color, lw=0.6, alpha=0.7)


def make_comparison_plot(segments: List[List[Sample]], plant: PlantModel,
                         current: PIDGains, optimal: PIDGains,
                         optimal_bias: Optional[Tuple[PIDGains, int]] = None):
    """Create multi-panel comparison plot."""
    import matplotlib.pyplot as plt

    n_segs = len(segments)
    fig, axes = plt.subplots(n_segs, 3, figsize=(18, 4 * n_segs), squeeze=False)
    fig.suptitle("BFS Drive PID: Real vs Simulated (Current vs Optimal)", fontsize=14, y=1.01)

    for i, seg in enumerate(segments):
        results_map = {}

        # Current gains
        res_curr = simulate_segment(seg, plant, current)
        results_map["Current"] = res_curr

        # Optimal gains
        res_opt = simulate_segment(seg, plant, optimal)
        results_map["Optimal"] = res_opt

        # Optimal + different bias
        if optimal_bias is not None:
            gains_b, bias_b = optimal_bias
            res_ob = simulate_segment(seg, plant, gains_b, motor_bias=bias_b)
            results_map["Optimal+bias"] = res_ob

        plot_segment_comparison(seg, results_map, i,
                                axes[i][0], axes[i][1], axes[i][2])

        axes[i][0].set_ylabel(f"Seg {i}\nYaw Err (°)")
        axes[i][0].axhline(0, color="gray", lw=0.5)
        axes[i][0].grid(True, alpha=0.3)
        axes[i][0].legend(fontsize=7)

        axes[i][1].set_ylabel("corrOut")
        axes[i][1].axhline(0, color="gray", lw=0.5)
        axes[i][1].grid(True, alpha=0.3)

        axes[i][2].set_ylabel("Motor PWM")
        axes[i][2].axhline(MIN_PWM_L, color="red", lw=0.5, ls=":", label=f"L floor={MIN_PWM_L}")
        axes[i][2].grid(True, alpha=0.3)
        axes[i][2].legend(fontsize=6)

    for j in range(3):
        axes[-1][j].set_xlabel("Time in segment (s)")

    fig.tight_layout()
    return fig


def make_sweep_heatmap(segments: List[List[Sample]], plant: PlantModel,
                       best_kd: float):
    """2D heatmap of cost vs KP and KI at fixed optimal KD."""
    import matplotlib.pyplot as plt

    kp_range = np.arange(0.5, 10.1, 0.25)
    ki_range = np.arange(0.0, 6.1, 0.25)
    cost_grid = np.full((len(ki_range), len(kp_range)), np.nan)

    print(f"\n  Generating KP×KI heatmap at KD={best_kd:.3f} ...")
    for j, kp in enumerate(kp_range):
        for k, ki in enumerate(ki_range):
            cost_grid[k, j] = total_cost(segments, plant, PIDGains(kp, ki, best_kd))

    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    im = ax.imshow(cost_grid, aspect="auto", origin="lower",
                   extent=[kp_range[0], kp_range[-1], ki_range[0], ki_range[-1]],
                   cmap="viridis_r")
    ax.set_xlabel("KP")
    ax.set_ylabel("KI")
    ax.set_title(f"Cost landscape (KD={best_kd:.3f}) — lower = better")
    fig.colorbar(im, ax=ax, label="Cost (RMS yaw err + penalties)")

    # Mark minimum
    min_idx = np.unravel_index(np.nanargmin(cost_grid), cost_grid.shape)
    ax.plot(kp_range[min_idx[1]], ki_range[min_idx[0]], "r*", markersize=15, label="Best")
    ax.legend()
    fig.tight_layout()
    return fig


# ─── CLI ─────────────────────────────────────────────────────────────────────

def find_bfs_logs() -> List[str]:
    """Find BFS run log CSVs (runs 107+)."""
    logs_dir = Path(__file__).resolve().parent.parent / "logs"
    csvs = sorted(glob.glob(str(logs_dir / "*runlog_10[789]*.csv")))
    if not csvs:
        csvs = sorted(glob.glob(str(logs_dir / "*runlog*.csv")))
    return csvs


def main():
    parser = argparse.ArgumentParser(description="BFS Drive PID Optimizer")
    parser.add_argument("csv", nargs="*", help="CSV log file(s)")
    parser.add_argument("--all-runs", action="store_true",
                        help="Use all available BFS run logs")
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--save", help="Save plots to file prefix")
    parser.add_argument("--skip-heatmap", action="store_true",
                        help="Skip the slow heatmap generation")
    args = parser.parse_args()

    # Gather CSV files
    if args.csv:
        csv_paths = args.csv
    elif args.all_runs:
        csv_paths = find_bfs_logs()
    else:
        # Default: latest
        csv_paths = find_bfs_logs()[-1:] if find_bfs_logs() else []

    if not csv_paths:
        print("No CSV files found. Place BFS logs in logs/ or provide paths.")
        sys.exit(1)

    # Parse all logs
    all_segments = []
    for path in csv_paths:
        if not os.path.exists(path):
            print(f"  File not found: {path}")
            continue
        header, samples = parse_log(path)
        run_id = header.get("id", "?")
        print(f"\nLoaded run {run_id}: {path}")
        print(f"  {len(samples)} samples, dist={header.get('dist','?')}m, "
              f"time={header.get('time','?')}s, mtr_bias={header.get('mtr_bias','?')}")

        segs = segment_by_gaps(samples)
        print(f"  {len(segs)} drive segments detected:")
        for i, seg in enumerate(segs):
            dt_s = (seg[-1].t_ms - seg[0].t_ms) / 1000
            dist = seg[-1].meters - seg[0].meters
            errs = [abs(s.yawErrDeg) for s in seg]
            rms = float(np.sqrt(np.mean(np.array(errs) ** 2)))
            clamp_pct = 100.0 * sum(1 for s in seg if s.lPwm <= MIN_PWM_L + 0.5) / len(seg)
            print(f"    Seg {i}: {len(seg)} pts, {dt_s:.1f}s, {dist:.3f}m, "
                  f"RMS err={rms:.1f}°, floor-clamp={clamp_pct:.0f}%")
            all_segments.append(seg)

    if not all_segments:
        print("No valid drive segments found!")
        sys.exit(1)

    # ─── Step 1: Plant identification ────────────────────────────────────
    print("\n" + "=" * 60)
    print("  PLANT MODEL IDENTIFICATION")
    print("=" * 60)
    plant = identify_plant(all_segments)
    print(f"  K_yaw      = {plant.K_yaw:.4f} °/s per PWM-diff unit")
    print(f"  drift_bias = {plant.drift_bias:.2f} °/s (inherent yaw rate at Δpwm=0)")
    print(f"  tau_motor  = {plant.tau_motor:.3f} s (motor response lag)")

    # Validate plant model against real data
    print("\n  Plant model validation (simulating current gains vs real data):")
    current = PIDGains(CURRENT_KP, CURRENT_KI, CURRENT_KD)
    for i, seg in enumerate(all_segments):
        real_errs = [abs(s.yawErrDeg) for s in seg]
        real_rms = float(np.sqrt(np.mean(np.array(real_errs) ** 2)))

        sim_results = simulate_segment(seg, plant, current)
        sim_sc = score_segment(sim_results)
        sim_rms = sim_sc.rms_err if sim_sc else 0

        print(f"    Seg {i}: Real RMS={real_rms:.2f}°  Sim RMS={sim_rms:.2f}°  "
              f"(match: {'GOOD' if abs(real_rms - sim_rms) < 1.5 else 'POOR'})")

    # ─── Step 2: Grid search ─────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("  GRID SEARCH — Coarse PID Optimization")
    print("=" * 60)

    current_cost = total_cost(all_segments, plant, current)
    print(f"\n  Current gains: KP={current.kp:.2f} KI={current.ki:.2f} KD={current.kd:.3f}"
          f"  →  cost={current_cost:.3f}")

    grid_best, grid_cost = grid_search(all_segments, plant)
    print(f"\n  Grid best: KP={grid_best.kp:.2f} KI={grid_best.ki:.2f} "
          f"KD={grid_best.kd:.3f}  →  cost={grid_cost:.3f}")

    # ─── Step 3: Refine with Nelder-Mead ─────────────────────────────────
    print("\n" + "=" * 60)
    print("  REFINEMENT — Nelder-Mead Optimization")
    print("=" * 60)

    optimal, opt_cost = refine_search(all_segments, plant, grid_best)
    print(f"\n  Optimal gains: KP={optimal.kp:.3f} KI={optimal.ki:.3f} "
          f"KD={optimal.kd:.4f}  →  cost={opt_cost:.3f}")
    print(f"  Improvement over current: {(1.0 - opt_cost / current_cost) * 100:.1f}%")

    # ─── Step 4: Also optimize motor_bias ────────────────────────────────
    print("\n" + "=" * 60)
    print("  MOTOR BIAS SWEEP (with optimal PID gains)")
    print("=" * 60)

    best_bias = MOTOR_BIAS_PWM
    best_bias_cost = opt_cost
    best_bias_gains = optimal
    for bias in range(-15, 16):
        # Quick: just test the optimal gains with different bias
        c = total_cost(all_segments, plant, optimal, motor_bias=bias)
        if c < best_bias_cost:
            best_bias_cost = c
            best_bias = bias

    # Also optimize PID for the best bias
    if best_bias != MOTOR_BIAS_PWM:
        print(f"  Best motor_bias = {best_bias} (was {MOTOR_BIAS_PWM}), cost={best_bias_cost:.3f}")
        grid_b, _ = grid_search(all_segments, plant, motor_bias=best_bias)
        best_bias_gains, best_bias_cost = refine_search(
            all_segments, plant, grid_b, motor_bias=best_bias)
        print(f"  Re-optimized: KP={best_bias_gains.kp:.3f} KI={best_bias_gains.ki:.3f} "
              f"KD={best_bias_gains.kd:.4f} bias={best_bias}  →  cost={best_bias_cost:.3f}")
    else:
        print(f"  Current motor_bias={MOTOR_BIAS_PWM} is already optimal")

    # ─── Summary ─────────────────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("  FINAL RECOMMENDATIONS")
    print("=" * 60)

    # Per-segment breakdown with optimal gains
    print(f"\n  Current:  KP={CURRENT_KP:.2f}  KI={CURRENT_KI:.2f}  KD={CURRENT_KD:.3f}  "
          f"bias={MOTOR_BIAS_PWM}  cost={current_cost:.3f}")
    for i, seg in enumerate(all_segments):
        sc = score_segment(simulate_segment(seg, plant, current))
        if sc:
            print(f"    Seg {i}: RMS={sc.rms_err:.2f}°  max={sc.max_err:.1f}°  "
                  f"final={sc.final_err:+.1f}°  floor%={sc.floor_clamp_pct:.0f}%")

    print(f"\n  Optimal:  KP={optimal.kp:.3f}  KI={optimal.ki:.3f}  KD={optimal.kd:.4f}  "
          f"bias={MOTOR_BIAS_PWM}  cost={opt_cost:.3f}")
    for i, seg in enumerate(all_segments):
        sc = score_segment(simulate_segment(seg, plant, optimal))
        if sc:
            print(f"    Seg {i}: RMS={sc.rms_err:.2f}°  max={sc.max_err:.1f}°  "
                  f"final={sc.final_err:+.1f}°  floor%={sc.floor_clamp_pct:.0f}%")

    if best_bias != MOTOR_BIAS_PWM:
        print(f"\n  Opt+bias: KP={best_bias_gains.kp:.3f}  KI={best_bias_gains.ki:.3f}  "
              f"KD={best_bias_gains.kd:.4f}  bias={best_bias}  cost={best_bias_cost:.3f}")
        for i, seg in enumerate(all_segments):
            sc = score_segment(simulate_segment(seg, plant, best_bias_gains, best_bias))
            if sc:
                print(f"    Seg {i}: RMS={sc.rms_err:.2f}°  max={sc.max_err:.1f}°  "
                      f"final={sc.final_err:+.1f}°  floor%={sc.floor_clamp_pct:.0f}%")

    # Firmware defines
    print("\n  ─── Firmware defines (copy to src/main.cpp) ───")
    print(f"  #define BFS_DRIVE_STEER_KP {optimal.kp:.3f}f")
    print(f"  #define BFS_DRIVE_STEER_KI {optimal.ki:.3f}f")
    print(f"  #define BFS_DRIVE_STEER_KD {optimal.kd:.4f}f")
    if best_bias != MOTOR_BIAS_PWM:
        print(f"\n  Also consider: motor_bias_pwm = {best_bias}  (currently {MOTOR_BIAS_PWM})")
        print(f"  With bias={best_bias}:")
        print(f"  #define BFS_DRIVE_STEER_KP {best_bias_gains.kp:.3f}f")
        print(f"  #define BFS_DRIVE_STEER_KI {best_bias_gains.ki:.3f}f")
        print(f"  #define BFS_DRIVE_STEER_KD {best_bias_gains.kd:.4f}f")

    # ─── Plots ───────────────────────────────────────────────────────────
    if not args.no_plot:
        import matplotlib
        matplotlib.use("Agg")  # non-interactive backend
        import matplotlib.pyplot as plt

        bias_opt = (best_bias_gains, best_bias) if best_bias != MOTOR_BIAS_PWM else None
        fig1 = make_comparison_plot(all_segments, plant, current, optimal, bias_opt)
        save1 = (args.save + "_comparison.png") if args.save else "bfs_pid_comparison.png"
        fig1.savefig(save1, dpi=150, bbox_inches="tight")
        print(f"\n  Saved comparison plot: {save1}")

        if not args.skip_heatmap:
            fig2 = make_sweep_heatmap(all_segments, plant, optimal.kd)
            save2 = (args.save + "_heatmap.png") if args.save else "bfs_pid_heatmap.png"
            fig2.savefig(save2, dpi=150, bbox_inches="tight")
            print(f"  Saved heatmap: {save2}")

        plt.close("all")


if __name__ == "__main__":
    main()
