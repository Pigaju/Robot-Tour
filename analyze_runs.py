#!/usr/bin/env python3
"""Analyze runs 127-129 for PWM inconsistency issues."""
import csv, sys

# ---- Stall analysis for Run 129 ----
print("\n========== STALL ANALYSIS (Run 129) ==========")
fname = 'logs/20260403_084633_runlog_129.csv'
with open(fname) as f:
    lines = [l for l in f if not l.startswith('#')]
reader = csv.DictReader(lines)
rows129 = list(reader)

prev_m = None
stall_start = None
stall_start_m = 0
for i, row in enumerate(rows129):
    t = int(row['t_ms'])
    m = float(row['meters'])
    dl = float(row['distL'])
    dr = float(row['distR'])
    bp = float(row['basePWM'])
    lp = int(row['lPwm'])
    rp = int(row['rPwm'])
    ye = float(row['yawErrDeg'])
    
    if prev_m is not None and abs(m - prev_m) < 0.001:
        if stall_start is None:
            stall_start = t
            stall_start_m = m
    else:
        if stall_start is not None:
            duration = t - stall_start
            if duration > 300:
                print(f'  STALL: t={stall_start}-{t}ms ({duration}ms) at m={stall_start_m:.4f}')
            stall_start = None
    prev_m = m

r = rows129[-1]
print(f'\nFinal: t={r["t_ms"]}ms meters={r["meters"]} remaining={r["remainingM"]}')
print(f'       basePWM={r["basePWM"]} lPwm={r["lPwm"]} rPwm={r["rPwm"]} yawErr={r["yawErrDeg"]}')

# ---- Speed scaling trace for Run 129 ----
print("\n========== SPEED SCALING ANALYSIS ==========")
print("Checking where basePWM < 120 (too low for reliable steering):")
for row in rows129:
    bp = float(row['basePWM'])
    if bp < 120:
        t = row['t_ms']
        lp = row['lPwm']
        rp = row['rPwm']
        co = row['corrOut']
        si = row['steerI']
        print(f'  t={t}ms basePWM={bp:.1f} lPwm={lp} rPwm={rp} corrOut={co} steerI={si}')

# Count samples at different basePWM ranges
bps = [float(r['basePWM']) for r in rows129]
print(f'\nbasePWM distribution:')
for lo, hi in [(90,100),(100,110),(110,120),(120,130),(130,145),(145,165)]:
    cnt = sum(1 for b in bps if lo <= b < hi)
    print(f'  [{lo}-{hi}): {cnt}/{len(bps)} samples ({100*cnt/len(bps):.0f}%)')

# ---- Main analysis ----
print("\n\n========== PER-RUN ANALYSIS ==========")
for run in ['127', '128', '129']:
    fname = f'logs/20260403_084633_runlog_{run}.csv'
    print(f'\n=== RUN {run} ===')
    with open(fname) as f:
        lines = [l for l in f if not l.startswith('#')]
    reader = csv.DictReader(lines)
    rows = list(reader)
    print(f'Total rows: {len(rows)}')

    hdr = f"{'t_ms':>8} {'meters':>8} {'basePWM':>8} {'lPwm':>6} {'rPwm':>6} {'corrOut':>8} {'yawErr':>8} {'steerI':>8} {'distL':>8} {'distR':>8} {'remaining':>10}"
    print(hdr)
    for i, row in enumerate(rows):
        if i % 5 == 0 or i < 3 or i >= len(rows) - 3:
            t = row.get('t_ms', '')
            m = row.get('meters', '')
            bp = row.get('basePWM', '')
            lp = row.get('lPwm', '')
            rp = row.get('rPwm', '')
            co = row.get('corrOut', '')
            ye = row.get('yawErrDeg', '')
            si = row.get('steerI', '')
            dl = row.get('distL', '')
            dr = row.get('distR', '')
            rm = row.get('remainingM', '')
            print(f'{t:>8} {m:>8} {bp:>8} {lp:>6} {rp:>6} {co:>8} {ye:>8} {si:>8} {dl:>8} {dr:>8} {rm:>10}')

    # Detect large PWM jumps
    print(f'\n--- Large PWM jumps (>30 in lPwm or rPwm between consecutive samples) ---')
    for i in range(1, len(rows)):
        try:
            prev_l = float(rows[i-1].get('lPwm', 0))
            curr_l = float(rows[i].get('lPwm', 0))
            prev_r = float(rows[i-1].get('rPwm', 0))
            curr_r = float(rows[i].get('rPwm', 0))
            dl = abs(curr_l - prev_l)
            dr = abs(curr_r - prev_r)
            if dl > 30 or dr > 30:
                t = rows[i].get('t_ms', '')
                ye = rows[i].get('yawErrDeg', '')
                co = rows[i].get('corrOut', '')
                bp = rows[i].get('basePWM', '')
                print(f'  t={t}ms: lPwm {prev_l:.0f}->{curr_l:.0f} (d={dl:.0f}), rPwm {prev_r:.0f}->{curr_r:.0f} (d={dr:.0f}), basePWM={bp}, yawErr={ye}, corrOut={co}')
        except (ValueError, TypeError):
            pass

    # basePWM range analysis
    try:
        bps = [float(r['basePWM']) for r in rows if r.get('basePWM')]
        print(f'\n--- basePWM stats: min={min(bps):.1f} max={max(bps):.1f} mean={sum(bps)/len(bps):.1f} ---')
    except:
        pass

    # corrOut range analysis
    try:
        cos = [float(r['corrOut']) for r in rows if r.get('corrOut')]
        print(f'--- corrOut stats: min={min(cos):.1f} max={max(cos):.1f} mean={sum(cos)/len(cos):.1f} ---')
    except:
        pass

    # Check yaw error pattern
    try:
        yes = [float(r['yawErrDeg']) for r in rows if r.get('yawErrDeg')]
        print(f'--- yawErr stats: min={min(yes):.1f} max={max(yes):.1f} mean={sum(yes)/len(yes):.1f} ---')
    except:
        pass

    # Detect segment transitions (when remaining jumps up)
    print(f'\n--- Segment transitions (remainingM jumps) ---')
    for i in range(1, len(rows)):
        try:
            prev_rm = float(rows[i-1].get('remainingM', 0))
            curr_rm = float(rows[i].get('remainingM', 0))
            if curr_rm > prev_rm + 0.1:
                t = rows[i].get('t_ms', '')
                m = rows[i].get('meters', '')
                fm = rows[i].get('forwardM', '')
                print(f'  t={t}ms: remaining {prev_rm:.3f} -> {curr_rm:.3f} (new seg), meters={m}, forwardM={fm}')
        except (ValueError, TypeError):
            pass
