#!/usr/bin/env python3
"""Analyze BFS run 106 log."""
import sys

rows = []
with open('logs/20260402_232301_runlog_106.csv') as f:
    for line in f:
        line = line.strip()
        if line.startswith('#') or line.startswith('t_ms,'):
            continue
        vals = line.split(',')
        if len(vals) < 53:
            continue
        rows.append({
            't_ms': int(vals[0]),
            'meters': float(vals[1]),
            'yaw': float(vals[3]),
            'target': float(vals[4]),
            'err': float(vals[5]),
            'gz': float(vals[7]),
            'basePWM': float(vals[14]),
            'corrOut': float(vals[17]),
            'lPwm': int(vals[24]),
            'rPwm': int(vals[25]),
            'remaining': float(vals[31]),
            'steerI': float(vals[37]),
            'distL': float(vals[51]),
            'distR': float(vals[52]),
        })

# Detect segment boundaries
segs = []
seg_start = 0
for i in range(1, len(rows)):
    target_changed = abs(rows[i]['target'] - rows[i-1]['target']) > 5.0
    remaining_jumped = rows[i]['remaining'] > rows[i-1]['remaining'] + 0.05
    if target_changed or remaining_jumped:
        segs.append((seg_start, i - 1))
        seg_start = i
segs.append((seg_start, len(rows) - 1))

print(f'Run 106: {len(rows)} samples, {len(segs)} drive segments')
print(f'Header: dist=6.250m, time=63.0s')
print()

for si, (s, e) in enumerate(segs):
    r0, re = rows[s], rows[e]
    t0, te = r0['t_ms'], re['t_ms']
    t0_disp = 'UNDERFLOW' if t0 > 4000000000 else str(t0)

    dist_driven = re['meters'] - r0['meters']
    seg_dist = r0['remaining']

    errs = [abs(rows[i]['err']) for i in range(s, e + 1)]
    max_err = max(errs)
    avg_err = sum(errs) / len(errs)
    min_basePWM = min(rows[i]['basePWM'] for i in range(s, e + 1))
    max_basePWM = max(rows[i]['basePWM'] for i in range(s, e + 1))
    min_lPwm = min(rows[i]['lPwm'] for i in range(s, e + 1))
    max_rPwm = max(rows[i]['rPwm'] for i in range(s, e + 1))
    max_gz = max(abs(rows[i]['gz']) for i in range(s, e + 1))

    print(f'--- Segment {si+1} ({e-s+1} samples) ---')
    print(f'  Time: t={t0_disp} to {te}ms  target_yaw={r0["target"]:.1f}')
    print(f'  Distance: {dist_driven:.3f}m driven of {seg_dist:.3f}m segment')
    print(f'  Yaw error: avg={avg_err:.1f} max={max_err:.1f} start={r0["err"]:.1f} end={re["err"]:.1f}')
    print(f'  basePWM: {min_basePWM:.0f}-{max_basePWM:.0f}  lPwm_min={min_lPwm} rPwm_max={max_rPwm}')
    print(f'  Max gyro rate: {max_gz:.1f} dps')
    print(f'  Encoder: dL={re["distL"]-r0["distL"]:.4f} dR={re["distR"]-r0["distR"]:.4f}')

    # Count floor-clamped samples (lPwm stuck at 100)
    floor_count = sum(1 for i in range(s, e + 1) if rows[i]['lPwm'] == 100)
    if floor_count > 0:
        print(f'  LEFT MOTOR FLOOR-CLAMPED: {floor_count}/{e-s+1} samples ({floor_count*100//(e-s+1)}%)')

    # Stall detection
    enc_total = abs(re['distL'] - r0['distL']) + abs(re['distR'] - r0['distR'])
    dur_s = (te - t0) / 1000.0 if te > t0 and t0 < 4000000000 else 0
    if dur_s > 0.5 and enc_total < 0.02:
        print(f'  ** STALLED ** {enc_total:.4f}m encoder in {dur_s:.1f}s')

    if si < len(segs) - 1:
        next_s = segs[si + 1][0]
        gap_ms = rows[next_s]['t_ms'] - re['t_ms']
        if 0 < gap_ms < 4000000000:
            print(f'  Gap to next: {gap_ms}ms ({gap_ms/1000:.1f}s)')
    print()

print(f'Total driven: {rows[-1]["meters"]:.3f}m of 6.250m ({rows[-1]["meters"]/6.250*100:.0f}%)')
te = rows[-1]['t_ms']
if te < 4000000000:
    print(f'Total elapsed: {te/1000:.1f}s of 63.0s')
