#!/usr/bin/env python3
"""Analyze Run 131 in detail."""
import csv

fname = 'logs/20260403_150102_runlog_131.csv'
with open(fname) as f:
    lines = [l for l in f if not l.startswith('#')]
reader = csv.DictReader(lines)
rows = list(reader)

print(f"=== RUN 131 ({len(rows)} samples) ===\n")

# Segment transitions
print("--- Segment transitions ---")
segments = []
seg_start = 0
for i in range(1, len(rows)):
    prev_rm = float(rows[i-1].get('remainingM', 0))
    curr_rm = float(rows[i].get('remainingM', 0))
    if curr_rm > prev_rm + 0.1:
        segments.append((seg_start, i-1))
        seg_start = i
        t = rows[i]['t_ms']
        fm = rows[i]['forwardM']
        print(f"  t={t}ms: remaining {prev_rm:.3f} -> {curr_rm:.3f}, forwardM={fm}")
segments.append((seg_start, len(rows)-1))

# Per-segment analysis
print(f"\n--- Per-segment stats ({len(segments)} segments) ---")
for si, (start, end) in enumerate(segments):
    seg_rows = rows[start:end+1]
    t0 = int(seg_rows[0]['t_ms'])
    t1 = int(seg_rows[-1]['t_ms'])
    
    bps = [float(r['basePWM']) for r in seg_rows]
    yes = [float(r['yawErrDeg']) for r in seg_rows]
    cos = [float(r['corrOut']) for r in seg_rows]
    lps = [int(r['lPwm']) for r in seg_rows]
    rps = [int(r['rPwm']) for r in seg_rows]
    
    m0 = float(seg_rows[0]['meters'])
    m1 = float(seg_rows[-1]['meters'])
    rm0 = float(seg_rows[0]['remainingM'])
    rm1 = float(seg_rows[-1]['remainingM'])
    
    ty0 = float(seg_rows[0]['targetYaw'])
    ty1 = float(seg_rows[-1]['targetYaw'])
    cy0 = float(seg_rows[0]['currentYaw'])
    cy1 = float(seg_rows[-1]['currentYaw'])
    
    # Check for stalls (distance not changing >500ms)
    stall_count = 0
    prev_m = float(seg_rows[0]['meters'])
    stall_start_t = None
    for r in seg_rows[1:]:
        m = float(r['meters'])
        t = int(r['t_ms'])
        if abs(m - prev_m) < 0.001:
            if stall_start_t is None:
                stall_start_t = t
        else:
            if stall_start_t is not None and (t - stall_start_t) > 500:
                stall_count += 1
            stall_start_t = None
            prev_m = m
    
    print(f"\n  Seg {si+1}: t={t0}-{t1}ms ({t1-t0}ms)")
    print(f"    distance: {m0:.3f} -> {m1:.3f} (drove {m1-m0:.3f}m, remaining {rm1:.3f}m)")
    print(f"    targetYaw: {ty0:.1f} -> {ty1:.1f} (delta={ty1-ty0:.1f})")
    print(f"    currentYaw: {cy0:.1f} -> {cy1:.1f} (drift={cy1-cy0-(ty1-ty0):.1f})")
    print(f"    basePWM: min={min(bps):.0f} max={max(bps):.0f} avg={sum(bps)/len(bps):.0f}")
    print(f"    yawErr: min={min(yes):.1f} max={max(yes):.1f} avg={sum(yes)/len(yes):.1f}")
    print(f"    corrOut: min={min(cos):.1f} max={max(cos):.1f} avg={sum(cos)/len(cos):.1f}")
    print(f"    lPwm: min={min(lps)} max={max(lps)} | rPwm: min={min(rps)} max={max(rps)}")
    if stall_count > 0:
        print(f"    *** {stall_count} stall(s) detected ***")

    # PWM jumps > 25
    jumps = 0
    for j in range(start+1, end+1):
        dl = abs(int(rows[j]['lPwm']) - int(rows[j-1]['lPwm']))
        dr = abs(int(rows[j]['rPwm']) - int(rows[j-1]['rPwm']))
        if dl > 25 or dr > 25:
            jumps += 1
    print(f"    PWM jumps (>25): {jumps}")

# Overall
print(f"\n--- Run summary ---")
fm_final = float(rows[-1]['forwardM'])
t_final = int(rows[-1]['t_ms'])
print(f"  Total distance: {fm_final:.3f}m / 6.750m target")
print(f"  Total time: {t_final/1000:.1f}s / 63.0s budget")
print(f"  Final yaw: {rows[-1]['currentYaw']} target: {rows[-1]['targetYaw']}")
print(f"  Final remaining: {rows[-1]['remainingM']}m")

# Encoder asymmetry
print(f"\n--- Encoder asymmetry per segment ---")
for si, (start, end) in enumerate(segments):
    dl = float(rows[end]['distL'])
    dr = float(rows[end]['distR'])
    if dl > 0.01:
        ratio = dr / dl
        print(f"  Seg {si+1}: distL={dl:.4f} distR={dr:.4f} ratio={ratio:.3f} (1.0=perfect)")
