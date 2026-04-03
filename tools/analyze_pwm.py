#!/usr/bin/env python3
"""Analyze recent BFS run logs to determine optimal PWM for time goals."""
import csv, re, glob, os

LOGS_DIR = os.path.join(os.path.dirname(__file__), '..', 'logs')

# Find recent CSV logs
csvs = sorted(glob.glob(os.path.join(LOGS_DIR, '*_runlog_*.csv')))
# Skip _last files
csvs = [c for c in csvs if '_last' not in c]

for run_file in csvs[-5:]:
    fname = os.path.basename(run_file)
    print(f'=== {fname} ===')
    
    rows = []
    header_line = ''
    with open(run_file) as f:
        for line in f:
            if line.startswith('# TEST_BEGIN'):
                header_line = line.strip()
            elif not line.startswith('#') and line.strip():
                rows.append(line.strip())
    
    dist_m = re.search(r'dist=([0-9.]+)', header_line)
    time_s = re.search(r'time=([0-9.]+)', header_line)
    if dist_m and time_s:
        print(f'  Header: dist={dist_m.group(1)}m  time={time_s.group(1)}s')
    
    if len(rows) < 2:
        print('  No data rows\n')
        continue
    
    reader = list(csv.DictReader(rows))
    if not reader:
        print('  No parsed rows\n')
        continue
    
    t_first = float(reader[0]['t_ms'])
    t_last = float(reader[-1]['t_ms'])
    duration_s = (t_last - t_first) / 1000.0
    print(f'  Samples={len(reader)}  Duration={duration_s:.1f}s')
    
    # Drive phase stats (basePWM >= 110 means driving)
    base_vals = []
    lpwm_vals = []
    rpwm_vals = []
    drive_time_ms = 0
    drive_dist = 0
    prev_t = None
    prev_m = None
    
    for r in reader:
        try:
            bp = float(r.get('basePWM', 0))
            lp = float(r.get('lPwm', 0))
            rp = float(r.get('rPwm', 0))
            t = float(r.get('t_ms', 0))
            m = float(r.get('meters', 0))
            
            if bp >= 110 and lp > 0:
                base_vals.append(bp)
                lpwm_vals.append(lp)
                rpwm_vals.append(rp)
                if prev_t is not None and (t - prev_t) < 500:
                    drive_time_ms += (t - prev_t)
                if prev_m is not None:
                    dm = m - prev_m
                    if dm > 0:
                        drive_dist += dm
            prev_t = t
            prev_m = m
        except (ValueError, TypeError):
            pass
    
    total_dist = 0
    for r in reader:
        try:
            total_dist = max(total_dist, float(r.get('meters', 0)))
        except:
            pass
    
    if base_vals:
        print(f'  Drive basePWM: min={min(base_vals):.0f} avg={sum(base_vals)/len(base_vals):.0f} max={max(base_vals):.0f}')
        print(f'  Drive lPwm:    min={min(lpwm_vals):.0f} avg={sum(lpwm_vals)/len(lpwm_vals):.0f} max={max(lpwm_vals):.0f}')
        print(f'  Drive rPwm:    min={min(rpwm_vals):.0f} avg={sum(rpwm_vals)/len(rpwm_vals):.0f} max={max(rpwm_vals):.0f}')
        print(f'  Drive time:    {drive_time_ms/1000:.1f}s  Distance driven: {total_dist:.3f}m')
        if drive_time_ms > 0:
            speed = total_dist / (drive_time_ms / 1000.0)
            print(f'  Drive speed:   {speed:.3f} m/s')
    
    # Overall effective speed
    if duration_s > 0 and total_dist > 0:
        print(f'  Overall speed: {total_dist / duration_s:.3f} m/s (incl turns/settle)')
    
    # Estimate needed PWM for time goal
    if time_s and dist_m:
        goal_t = float(time_s.group(1))
        goal_d = float(dist_m.group(1))
        if goal_t > 0 and duration_s > 0:
            ratio = duration_s / goal_t
            print(f'  Time ratio:    {ratio:.2f}x (actual/goal)')
            if base_vals:
                avg_pwm = sum(base_vals) / len(base_vals)
                needed_pwm = avg_pwm * ratio
                print(f'  Needed PWM:    ~{needed_pwm:.0f} (to meet time goal at {goal_t:.0f}s)')
    
    print()
