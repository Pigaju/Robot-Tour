#!/usr/bin/env python3
"""Physics simulation and analysis of robot square-test CSV logs.
Diagnoses wobble during driving and turning."""

import csv
import math
import sys
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

# ─── Configuration ───────────────────────────────────────────────────
TRACK_WIDTH_M = 0.10  # approximate wheel-to-wheel distance (meters)
PPM_L = 922.9
PPM_R = 889.3
PPM  = 906.1

LOG_DIR = os.path.dirname(os.path.abspath(__file__))
FILES = [
    os.path.join(LOG_DIR, "20260328_121316_runlog_63.csv"),
    os.path.join(LOG_DIR, "20260328_121316_runlog_64.csv"),
]

def parse_csv(path):
    """Parse a run-log CSV, returning list of dicts with numeric values."""
    rows = []
    headers = None
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line.startswith('#') or not line:
                continue
            if headers is None:
                headers = line.split(',')
                continue
            vals = line.split(',')
            row = {}
            for h, v in zip(headers, vals):
                try:
                    row[h] = float(v)
                except ValueError:
                    row[h] = v
            rows.append(row)
    return rows


def segment_runs(rows):
    """Split rows into drive segments (gaps > 500ms indicate turn phases)."""
    segments = []
    seg = [rows[0]]
    for i in range(1, len(rows)):
        dt = rows[i]['t_ms'] - rows[i-1]['t_ms']
        if dt > 500:  # big gap = turn/brake/settle
            segments.append(seg)
            seg = []
        seg.append(rows[i])
    if seg:
        segments.append(seg)
    return segments


def analyze_segment(seg, seg_idx):
    """Compute stats for a single drive segment."""
    t = [r['t_ms'] for r in seg]
    yaw_err = [r['yawErrDeg'] for r in seg]
    corr_out = [r['corrOut'] for r in seg]
    steer_i = [r['steerI'] for r in seg]
    dist_l = [r['distL'] for r in seg]
    dist_r = [r['distR'] for r in seg]
    l_pwm = [r['lPwm'] for r in seg]
    r_pwm = [r['rPwm'] for r in seg]
    gz = [r['gz_dps'] for r in seg]
    base_pwm = [r['basePWM'] for r in seg]

    duration_s = (t[-1] - t[0]) / 1000.0
    target_yaw = seg[0]['targetYaw']
    
    # Encoder divergence
    dl_start, dr_start = dist_l[0], dist_r[0]
    dl_end, dr_end = dist_l[-1], dist_r[-1]
    delta_l = dl_end - dl_start
    delta_r = dr_end - dr_start
    divergence = delta_l - delta_r

    # Yaw error stats
    yaw_err_abs = [abs(e) for e in yaw_err]
    
    # Oscillation: count zero-crossings in yaw error
    crossings = 0
    for i in range(1, len(yaw_err)):
        if yaw_err[i-1] * yaw_err[i] < 0:
            crossings += 1
    osc_freq_hz = crossings / (2 * duration_s) if duration_s > 0 else 0

    # corrOut saturation
    sat_count = sum(1 for c in corr_out if abs(c) >= 59.0)

    return {
        'seg_idx': seg_idx,
        'target_yaw': target_yaw,
        't_start': t[0],
        't_end': t[-1],
        'duration_s': duration_s,
        'samples': len(seg),
        'yaw_err_mean': np.mean(yaw_err),
        'yaw_err_std': np.std(yaw_err),
        'yaw_err_max': max(yaw_err_abs),
        'yaw_err_rms': np.sqrt(np.mean(np.array(yaw_err)**2)),
        'osc_freq_hz': osc_freq_hz,
        'zero_crossings': crossings,
        'corr_out_max': max(abs(c) for c in corr_out),
        'corr_sat_pct': 100.0 * sat_count / len(seg),
        'steer_i_final': steer_i[-1],
        'steer_i_max': max(abs(si) for si in steer_i),
        'enc_delta_l': delta_l,
        'enc_delta_r': delta_r,
        'enc_divergence_m': divergence,
        'pwm_l_mean': np.mean(l_pwm),
        'pwm_r_mean': np.mean(r_pwm),
        'gz_std': np.std(gz),
    }


def simulate_2d_trajectory(rows):
    """Dead-reckoning 2D position from encoder + IMU heading."""
    x, y = 0.0, 0.0
    xs, ys = [0.0], [0.0]
    headings = []
    
    for i, r in enumerate(rows):
        heading_deg = r['currentYaw']
        heading_rad = math.radians(heading_deg)
        headings.append(heading_deg)
        
        if i > 0:
            # Average distance from both encoders
            dl = r['distL'] - rows[i-1]['distL']
            dr = r['distR'] - rows[i-1]['distR']
            ds = (dl + dr) / 2.0
            
            # For heading, use average of current and previous
            prev_heading_rad = math.radians(rows[i-1]['currentYaw'])
            avg_heading = (prev_heading_rad + heading_rad) / 2.0
            
            x += ds * math.cos(avg_heading)
            y += ds * math.sin(avg_heading)
        
        xs.append(x)
        ys.append(y)
    
    return xs, ys, headings


def plot_full_analysis(rows, segments, seg_stats, run_id, outdir):
    """Generate comprehensive multi-panel analysis plot."""
    
    t_all = np.array([r['t_ms'] for r in rows]) / 1000.0  # seconds
    t0 = t_all[0]
    t_all -= t0
    
    fig = plt.figure(figsize=(20, 28))
    gs = gridspec.GridSpec(7, 2, hspace=0.35, wspace=0.3)
    fig.suptitle(f'Robot Square Test Physics Analysis — Run {run_id}', fontsize=16, y=0.98)

    # ── Panel 1: 2D Trajectory ──
    ax1 = fig.add_subplot(gs[0, 0])
    xs, ys, headings = simulate_2d_trajectory(rows)
    ax1.plot(xs, ys, 'b-', linewidth=1.5, label='Robot path')
    ax1.plot(xs[0], ys[0], 'go', markersize=10, label='Start')
    ax1.plot(xs[-1], ys[-1], 'rs', markersize=10, label='End')
    # Mark segment boundaries
    idx = 0
    for seg in segments:
        idx += len(seg)
        if idx < len(xs):
            ax1.plot(xs[idx], ys[idx], 'k^', markersize=8)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('2D Trajectory (dead-reckoning)')
    ax1.legend(fontsize=8)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)

    # ── Panel 2: Yaw Error over time ──
    ax2 = fig.add_subplot(gs[0, 1])
    yaw_err = [r['yawErrDeg'] for r in rows]
    ax2.plot(t_all, yaw_err, 'r-', linewidth=0.8)
    ax2.axhline(0, color='k', linewidth=0.5, linestyle='--')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Yaw Error (°)')
    ax2.set_title('Yaw Error vs Time')
    ax2.grid(True, alpha=0.3)
    # Shade drive segments
    for seg in segments:
        t_s = (seg[0]['t_ms'] - t0*1000) / 1000
        t_e = (seg[-1]['t_ms'] - t0*1000) / 1000
        ax2.axvspan(t_s, t_e, alpha=0.1, color='green')

    # ── Panel 3: PWM Commands ──
    ax3 = fig.add_subplot(gs[1, 0])
    l_pwm = [r['lPwm'] for r in rows]
    r_pwm = [r['rPwm'] for r in rows]
    ax3.plot(t_all, l_pwm, 'b-', linewidth=0.8, label='Left PWM')
    ax3.plot(t_all, r_pwm, 'r-', linewidth=0.8, label='Right PWM')
    ax3.axhline(120, color='gray', linewidth=0.5, linestyle='--', label='Base PWM')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('PWM')
    ax3.set_title('Motor PWM Commands')
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)

    # ── Panel 4: corrOut (Steering Correction) ──
    ax4 = fig.add_subplot(gs[1, 1])
    corr_out = [r['corrOut'] for r in rows]
    ax4.plot(t_all, corr_out, 'm-', linewidth=0.8)
    ax4.axhline(60, color='r', linewidth=0.5, linestyle='--', alpha=0.5, label='±60 sat')
    ax4.axhline(-60, color='r', linewidth=0.5, linestyle='--', alpha=0.5)
    ax4.axhline(0, color='k', linewidth=0.5, linestyle='--')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('corrOut (PWM)')
    ax4.set_title('Steering Correction Output')
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)

    # ── Panel 5: Encoder Distances ──
    ax5 = fig.add_subplot(gs[2, 0])
    dist_l = [r['distL'] for r in rows]
    dist_r = [r['distR'] for r in rows]
    ax5.plot(t_all, dist_l, 'b-', linewidth=1, label='distL')
    ax5.plot(t_all, dist_r, 'r-', linewidth=1, label='distR')
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Distance (m)')
    ax5.set_title('Encoder Distances (L vs R)')
    ax5.legend(fontsize=8)
    ax5.grid(True, alpha=0.3)

    # ── Panel 6: Encoder Divergence ──
    ax6 = fig.add_subplot(gs[2, 1])
    divergence = [dl - dr for dl, dr in zip(dist_l, dist_r)]
    ax6.plot(t_all, divergence, 'k-', linewidth=1)
    ax6.axhline(0, color='gray', linewidth=0.5, linestyle='--')
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('distL - distR (m)')
    ax6.set_title('Encoder Divergence (L-R)')
    ax6.grid(True, alpha=0.3)
    ax6.fill_between(t_all, divergence, alpha=0.3, color='orange')

    # ── Panel 7: Gyro Rate ──
    ax7 = fig.add_subplot(gs[3, 0])
    gz = [r['gz_dps'] for r in rows]
    ax7.plot(t_all, gz, 'g-', linewidth=0.8)
    ax7.axhline(0, color='k', linewidth=0.5, linestyle='--')
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('Gyro Z (°/s)')
    ax7.set_title('Gyroscope Rate')
    ax7.grid(True, alpha=0.3)

    # ── Panel 8: Steering Integral ──
    ax8 = fig.add_subplot(gs[3, 1])
    steer_i = [r['steerI'] for r in rows]
    ax8.plot(t_all, steer_i, 'c-', linewidth=1)
    ax8.axhline(0, color='k', linewidth=0.5, linestyle='--')
    ax8.set_xlabel('Time (s)')
    ax8.set_ylabel('steerI')
    ax8.set_title('Steering Integral (I-term accumulation)')
    ax8.grid(True, alpha=0.3)

    # ── Panel 9: Per-Segment Yaw Error Histograms ──
    ax9 = fig.add_subplot(gs[4, 0])
    colors = ['blue', 'red', 'green', 'orange', 'purple']
    for i, seg in enumerate(segments):
        errs = [r['yawErrDeg'] for r in seg]
        # Only plot drive segments (skip turns where errors are huge)
        if max(abs(e) for e in errs) < 30:
            ax9.hist(errs, bins=15, alpha=0.5, color=colors[i % len(colors)],
                     label=f'Seg {i} (target={seg[0]["targetYaw"]:.0f}°)')
    ax9.set_xlabel('Yaw Error (°)')
    ax9.set_ylabel('Count')
    ax9.set_title('Yaw Error Distribution (Drive Segments)')
    ax9.legend(fontsize=7)
    ax9.grid(True, alpha=0.3)

    # ── Panel 10: PWM Differential ──
    ax10 = fig.add_subplot(gs[4, 1])
    pwm_diff = [l - r for l, r in zip(l_pwm, r_pwm)]
    ax10.plot(t_all, pwm_diff, 'k-', linewidth=0.8)
    ax10.axhline(0, color='gray', linewidth=0.5, linestyle='--')
    ax10.set_xlabel('Time (s)')
    ax10.set_ylabel('lPwm - rPwm')
    ax10.set_title('PWM Differential (L-R, positive = turning right)')
    ax10.grid(True, alpha=0.3)

    # ── Panel 11: Phase portrait (yaw error vs gyro rate) ──
    ax11 = fig.add_subplot(gs[5, 0])
    for i, seg in enumerate(segments):
        errs = [r['yawErrDeg'] for r in seg]
        gzs = [r['gz_dps'] for r in seg]
        if max(abs(e) for e in errs) < 30:
            ax11.plot(errs, gzs, '-', linewidth=0.5, alpha=0.7, color=colors[i % len(colors)],
                      label=f'Seg {i}')
            ax11.plot(errs[0], gzs[0], 'o', markersize=5, color=colors[i % len(colors)])
    ax11.set_xlabel('Yaw Error (°)')
    ax11.set_ylabel('Gyro Rate (°/s)')
    ax11.set_title('Phase Portrait: Yaw Error vs Rotation Rate')
    ax11.legend(fontsize=7)
    ax11.grid(True, alpha=0.3)

    # ── Panel 12: basePWM & dynamic scaling ──
    ax12 = fig.add_subplot(gs[5, 1])
    base = [r['basePWM'] for r in rows]
    avg_pwm = [(l + r) / 2 for l, r in zip(l_pwm, r_pwm)]
    ax12.plot(t_all, base, 'k--', linewidth=1, label='basePWM')
    ax12.plot(t_all, avg_pwm, 'b-', linewidth=0.8, alpha=0.7, label='avg(lPwm,rPwm)')
    ax12.set_xlabel('Time (s)')
    ax12.set_ylabel('PWM')
    ax12.set_title('Base PWM vs Actual Average PWM')
    ax12.legend(fontsize=8)
    ax12.grid(True, alpha=0.3)

    # ── Panel 13: Stats Table ──
    ax13 = fig.add_subplot(gs[6, :])
    ax13.axis('off')
    
    # Build table data for drive segments only
    table_data = []
    col_labels = ['Seg', 'Target°', 'Duration', 'Yaw RMS°', 'Yaw Max°', 
                  'Osc Hz', 'corrSat%', 'steerI_max', 'ΔencL', 'ΔencR', 'L-R div(m)']
    for s in seg_stats:
        # Skip turn segments (huge yaw errors)
        if s['yaw_err_max'] > 30:
            continue
        table_data.append([
            f"{s['seg_idx']}",
            f"{s['target_yaw']:.1f}",
            f"{s['duration_s']:.2f}s",
            f"{s['yaw_err_rms']:.2f}",
            f"{s['yaw_err_max']:.2f}",
            f"{s['osc_freq_hz']:.1f}",
            f"{s['corr_sat_pct']:.0f}%",
            f"{s['steer_i_max']:.2f}",
            f"{s['enc_delta_l']:.3f}",
            f"{s['enc_delta_r']:.3f}",
            f"{s['enc_divergence_m']:+.3f}",
        ])
    
    if table_data:
        table = ax13.table(cellText=table_data, colLabels=col_labels,
                          loc='center', cellLoc='center')
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.scale(1, 1.4)
        # Color cells with high divergence
        for i, row in enumerate(table_data):
            div = float(row[-1])
            if abs(div) > 0.05:
                table[i+1, -1].set_facecolor('#ffcccc')
            rms = float(row[3])
            if rms > 2.0:
                table[i+1, 3].set_facecolor('#ffcccc')
    
    ax13.set_title('Drive Segment Statistics', fontsize=12, pad=20)

    outpath = os.path.join(outdir, f'physics_analysis_run{run_id}.png')
    fig.savefig(outpath, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved: {outpath}")
    return outpath


def print_diagnosis(all_stats):
    """Print a textual diagnosis of the wobble issues."""
    print("\n" + "="*70)
    print("WOBBLE DIAGNOSIS REPORT")
    print("="*70)
    
    for run_stats in all_stats:
        drive_segs = [s for s in run_stats if s['yaw_err_max'] < 30]
        turn_segs = [s for s in run_stats if s['yaw_err_max'] >= 30]
        
        print(f"\n--- Drive Segments ---")
        for s in drive_segs:
            print(f"\nSegment {s['seg_idx']}: target={s['target_yaw']:.1f}°, "
                  f"duration={s['duration_s']:.2f}s")
            print(f"  Yaw Error:  mean={s['yaw_err_mean']:+.2f}°, RMS={s['yaw_err_rms']:.2f}°, "
                  f"max={s['yaw_err_max']:.2f}°")
            print(f"  Oscillation: {s['osc_freq_hz']:.1f} Hz ({s['zero_crossings']} crossings)")
            print(f"  corrOut max={s['corr_out_max']:.1f}, saturation={s['corr_sat_pct']:.0f}%")
            print(f"  steerI: max={s['steer_i_max']:.2f}, final={s['steer_i_final']:.2f}")
            print(f"  Encoders: ΔL={s['enc_delta_l']:.4f}m, ΔR={s['enc_delta_r']:.4f}m, "
                  f"DIVERGENCE={s['enc_divergence_m']:+.4f}m")
            print(f"  Mean PWM: L={s['pwm_l_mean']:.1f}, R={s['pwm_r_mean']:.1f}")
        
        print(f"\n--- Turn Segments ---")
        for s in turn_segs:
            print(f"\nSegment {s['seg_idx']}: target={s['target_yaw']:.1f}°, "
                  f"duration={s['duration_s']:.2f}s")
            print(f"  Peak yaw error: {s['yaw_err_max']:.1f}° (OVERSHOOT)")
            print(f"  Encoder divergence: {s['enc_divergence_m']:+.4f}m")
    
    print("\n" + "="*70)
    print("KEY FINDINGS:")
    print("="*70)
    
    # Aggregate across all drive segments
    all_drive = []
    for run_stats in all_stats:
        all_drive.extend([s for s in run_stats if s['yaw_err_max'] < 30])
    
    if all_drive:
        avg_rms = np.mean([s['yaw_err_rms'] for s in all_drive])
        avg_div = np.mean([abs(s['enc_divergence_m']) for s in all_drive])
        max_div = max(abs(s['enc_divergence_m']) for s in all_drive)
        avg_osc = np.mean([s['osc_freq_hz'] for s in all_drive])
        avg_steer = np.mean([s['steer_i_max'] for s in all_drive])
        max_steer = max(s['steer_i_max'] for s in all_drive)
        
        print(f"\n1. YAW OSCILLATION:")
        print(f"   Average RMS yaw error: {avg_rms:.2f}°")
        print(f"   Average oscillation frequency: {avg_osc:.1f} Hz")
        if avg_rms > 2.0:
            print(f"   ⚠  HIGH — PID is overcorrecting. Yaw swings ±{avg_rms:.1f}° around target.")
            print(f"      Suggest: Lower Kp further (currently 2.0 → try 1.5)")
            print(f"      Suggest: Increase Kd slightly for more damping (0.06 → 0.08)")
        
        print(f"\n2. ENCODER DIVERGENCE (LEFT vs RIGHT):")
        print(f"   Average |divergence|: {avg_div:.4f}m per segment")
        print(f"   Max divergence: {max_div:.4f}m")
        if max_div > 0.05:
            print(f"   ⚠  SEVERE — Left wheel consistently travels further than right.")
            print(f"      This indicates a MECHANICAL bias (wheel diameter, friction, alignment)")
            print(f"      OR the motor_bias_pwm=1 is insufficient.")
            print(f"      The PID fights this bias → steerI builds up → oscillation.")
        
        print(f"\n3. STEERING INTEGRAL WINDUP:")
        print(f"   Average max |steerI|: {avg_steer:.2f}")
        print(f"   Peak steerI: {max_steer:.2f}")
        if max_steer > 2.0:
            print(f"   ⚠  steerI growing large — persistent heading bias not corrected by P-term alone.")
            print(f"      This confirms a steady-state steering offset (mechanical or calibration).")
        
        print(f"\n4. TURN OVERSHOOT:")
        all_turns = []
        for run_stats in all_stats:
            all_turns.extend([s for s in run_stats if s['yaw_err_max'] >= 30])
        if all_turns:
            max_overshoot = max(s['yaw_err_max'] for s in all_turns)
            print(f"   Max turn overshoot: {max_overshoot:.1f}°")
            if max_overshoot > 15:
                print(f"   ⚠  Turns overshoot significantly. The robot swings past target then fights back.")
                print(f"      During recovery, corrOut saturates at ±60 for multiple cycles.")
        
        print(f"\n5. DYNAMIC PWM EFFECT:")
        # Check if lPwm/rPwm exceed basePWM significantly
        high_pwm_segs = [s for s in all_drive if s['pwm_l_mean'] > 125 or s['pwm_r_mean'] > 125]
        if high_pwm_segs:
            print(f"   {len(high_pwm_segs)}/{len(all_drive)} segments had mean PWM > 125")
            print(f"   Dynamic scaling pushes PWM higher, amplifying any steering imbalance.")
        
        print(f"\n" + "="*70)
        print(f"RECOMMENDATIONS:")
        print(f"="*70)
        print(f"  1. INCREASE motor_bias_pwm from 1 to ~3-5 to compensate for L>R wheel travel")
        print(f"  2. Lower drive steer Kp: 2.0 → 1.2 (reduce overcorrection amplitude)")
        print(f"  3. Raise drive steer Kd: 0.06 → 0.10 (add damping to reduce oscillation)")
        print(f"  4. Lower drive steer Ki: 1.0 → 0.5 (reduce integral windup)")
        print(f"  5. Consider lowering corrOut clamp from ±60 to ±30 during straight driving")
        print(f"  6. Check wheel alignment / tire inflation for mechanical L-R asymmetry")
        print()


def main():
    all_stats = []
    
    for fpath in FILES:
        fname = os.path.basename(fpath)
        print(f"\n{'='*60}")
        print(f"Analyzing: {fname}")
        print(f"{'='*60}")
        
        rows = parse_csv(fpath)
        print(f"  Parsed {len(rows)} samples")
        print(f"  Time range: {rows[0]['t_ms']:.0f} – {rows[-1]['t_ms']:.0f} ms "
              f"({(rows[-1]['t_ms'] - rows[0]['t_ms'])/1000:.1f}s)")
        
        segments = segment_runs(rows)
        print(f"  Found {len(segments)} segments (drive/turn phases)")
        
        seg_stats = []
        for i, seg in enumerate(segments):
            stats = analyze_segment(seg, i)
            seg_stats.append(stats)
        
        all_stats.append(seg_stats)
        
        # Extract run ID from header
        run_id = fname.split('_')[-1].replace('.csv', '')
        plot_full_analysis(rows, segments, seg_stats, run_id, LOG_DIR)
    
    print_diagnosis(all_stats)


if __name__ == '__main__':
    main()
