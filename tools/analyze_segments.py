#!/usr/bin/env python3
"""Analyze BFS log segments: measure drive time, overhead per segment, and speed."""
import csv, sys, os, glob

def analyze_log(filepath):
    rows = []
    header_info = {}
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if line.startswith("# TEST_BEGIN"):
                for part in line.split():
                    if '=' in part:
                        k, v = part.split('=', 1)
                        header_info[k] = v
                continue
            if line.startswith("#") or line.startswith("t_ms,"):
                continue
            vals = line.split(',')
            if len(vals) < 15:
                continue
            rows.append({
                't_ms': int(vals[0]),
                'meters': float(vals[1]),
                'forwardM': float(vals[2]),
                'currentYaw': float(vals[3]),
                'targetYaw': float(vals[4]),
                'basePWM': float(vals[14]),
                'remainingM': float(vals[31]) if len(vals) > 31 else 0.0,
            })

    if not rows:
        print(f"  No data rows in {filepath}")
        return

    run_id = header_info.get('id', '?')
    dist = header_info.get('dist', '?')
    time_goal = header_info.get('time', '?')
    print(f"\n{'='*70}")
    print(f"RUN {run_id}: dist={dist}m, time_goal={time_goal}s")
    print(f"{'='*70}")

    # Detect segment boundaries: targetYaw changes or remainingM jumps up
    segments = []
    seg_start = 0
    for i in range(1, len(rows)):
        target_changed = abs(rows[i]['targetYaw'] - rows[i-1]['targetYaw']) > 5.0
        remaining_jumped = rows[i]['remainingM'] > rows[i-1]['remainingM'] + 0.05
        if target_changed or remaining_jumped:
            segments.append((seg_start, i - 1))
            seg_start = i
    segments.append((seg_start, len(rows) - 1))

    total_start_ms = rows[0]['t_ms']
    total_end_ms = rows[-1]['t_ms']
    total_drive_ms = 0

    print(f"\n{'Seg':>3} {'Start_ms':>10} {'End_ms':>10} {'Drive_s':>8} {'Gap_s':>7} "
          f"{'TargetYaw':>10} {'Dist_m':>7} {'AvgPWM':>7} {'Speed_m/s':>10}")
    print("-" * 95)

    prev_end_ms = None
    for idx, (s, e) in enumerate(segments):
        t0 = rows[s]['t_ms']
        t1 = rows[e]['t_ms']
        drive_s = (t1 - t0) / 1000.0
        total_drive_ms += (t1 - t0)

        gap_s = (t0 - prev_end_ms) / 1000.0 if prev_end_ms is not None else 0.0
        target_yaw = rows[s]['targetYaw']

        # Distance driven in this segment
        dist_m = rows[e]['forwardM'] - rows[s]['forwardM']
        if dist_m < 0:
            dist_m = rows[e]['meters'] - rows[s]['meters']

        # Avg basePWM
        pwm_vals = [rows[i]['basePWM'] for i in range(s, e + 1)]
        avg_pwm = sum(pwm_vals) / len(pwm_vals) if pwm_vals else 0

        speed = dist_m / drive_s if drive_s > 0.1 else 0.0

        print(f"{idx+1:>3} {t0:>10} {t1:>10} {drive_s:>8.2f} {gap_s:>7.2f} "
              f"{target_yaw:>10.1f} {dist_m:>7.3f} {avg_pwm:>7.1f} {speed:>10.3f}")

        prev_end_ms = t1

    total_s = (total_end_ms - total_start_ms) / 1000.0
    drive_s = total_drive_ms / 1000.0
    overhead_s = total_s - drive_s
    num_gaps = len(segments) - 1

    print(f"\n--- Summary ---")
    print(f"  Total wall time:   {total_s:.1f}s")
    print(f"  Total drive time:  {drive_s:.1f}s")
    print(f"  Total overhead:    {overhead_s:.1f}s")
    print(f"  Segments:          {len(segments)}")
    if num_gaps > 0:
        # Sum all gaps
        gaps = []
        prev_end = None
        for s, e in segments:
            t0 = rows[s]['t_ms']
            if prev_end is not None:
                gaps.append((t0 - prev_end) / 1000.0)
            prev_end = rows[e]['t_ms']
        total_gap = sum(gaps)
        print(f"  Sum of inter-segment gaps: {total_gap:.1f}s")
        print(f"  Avg gap per transition:    {total_gap/len(gaps):.1f}s")
        print(f"  Min/Max gap:               {min(gaps):.1f}s / {max(gaps):.1f}s")
        for i, g in enumerate(gaps):
            print(f"    Gap {i+1}: {g:.2f}s")

if __name__ == "__main__":
    log_dir = os.path.join(os.path.dirname(__file__), "..", "logs")
    # Find the most recent BFS logs (dist > 1m)
    csvs = sorted(glob.glob(os.path.join(log_dir, "*runlog_10[0-9].csv")) +
                  glob.glob(os.path.join(log_dir, "*runlog_1[0-9][0-9].csv")))
    for f in csvs:
        # Check if it's a BFS log (dist > 1m)
        with open(f) as fh:
            first = fh.readline()
            if 'dist=' in first:
                dist_str = first.split('dist=')[1].split()[0]
                if float(dist_str) > 1.0:
                    analyze_log(f)
