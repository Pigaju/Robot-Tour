#!/usr/bin/env python3
"""Analyze BFS runs 107-109: per-segment breakdown."""
import sys, os, glob

def parse_log(filepath):
    rows = []
    header = {}
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if line.startswith('# TEST_BEGIN'):
                for part in line.split():
                    if '=' in part:
                        k, v = part.split('=', 1)
                        header[k] = v
                continue
            if line.startswith('# TEST_END'):
                for part in line.split():
                    if '=' in part:
                        k, v = part.split('=', 1)
                        header['end_' + k] = v
                continue
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
    return header, rows

def analyze(filepath):
    header, rows = parse_log(filepath)
    run_id = header.get('id', '?')
    dist = header.get('dist', '?')
    time_goal = header.get('time', '?')
    stop = header.get('end_stop', '?')
    samples = header.get('end_samples', '?')
    
    print(f'\n{"="*70}')
    print(f'RUN {run_id}: dist={dist}m, time={time_goal}s, stop={stop}, samples={samples}')
    print(f'{"="*70}')
    
    if not rows:
        print('  No data rows!')
        return header, rows
    
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
    
    print(f'{len(rows)} samples, {len(segs)} drive segments')
    
    total_drive_ms = 0
    total_overhead_ms = 0
    
    for si, (s, e) in enumerate(segs):
        r0, re = rows[s], rows[e]
        t0, te = r0['t_ms'], re['t_ms']
        dur_ms = te - t0
        total_drive_ms += dur_ms
        
        dist_driven = re['meters'] - r0['meters']
        seg_total = r0['remaining'] + dist_driven  # approx total segment dist
        
        errs = [rows[i]['err'] for i in range(s, e + 1)]
        abs_errs = [abs(x) for x in errs]
        max_err = max(abs_errs)
        avg_err = sum(abs_errs) / len(abs_errs)
        rms_err = (sum(x*x for x in errs) / len(errs)) ** 0.5
        
        pwms = [(rows[i]['lPwm'], rows[i]['rPwm']) for i in range(s, e + 1)]
        basePWMs = [rows[i]['basePWM'] for i in range(s, e + 1)]
        floor_l = sum(1 for l, r in pwms if l == 100)
        floor_r = sum(1 for l, r in pwms if r <= 110)
        
        corrs = [rows[i]['corrOut'] for i in range(s, e + 1)]
        avg_corr = sum(corrs) / len(corrs)
        max_corr = max(abs(c) for c in corrs)
        
        # Encoder asymmetry
        dL = re['distL'] - r0['distL']
        dR = re['distR'] - r0['distR']
        asym_pct = 0
        if dR > 0.01:
            asym_pct = ((dL - dR) / dR) * 100
        
        speed_mps = dist_driven / (dur_ms / 1000.0) if dur_ms > 0 else 0
        
        print(f'\n  Seg {si+1}: t={t0}-{te}ms ({dur_ms/1000:.1f}s)')
        print(f'    Driven: {dist_driven:.3f}m  remaining_start={r0["remaining"]:.3f}m  speed={speed_mps:.2f} m/s')
        print(f'    Yaw err: avg={avg_err:.1f}° rms={rms_err:.1f}° max={max_err:.1f}° (start={r0["err"]:.1f}° end={re["err"]:.1f}°)')
        print(f'    Correction: avg={avg_corr:.1f} max={max_corr:.1f}  steerI_end={re["steerI"]:.2f}')
        print(f'    basePWM: {min(basePWMs):.0f}-{max(basePWMs):.0f}  lPwm={min(l for l,r in pwms)}-{max(l for l,r in pwms)}  rPwm={min(r for l,r in pwms)}-{max(r for l,r in pwms)}')
        print(f'    L floor-clamped: {floor_l}/{len(pwms)} ({floor_l*100//len(pwms) if len(pwms) else 0}%)')
        print(f'    Encoder: dL={dL:.4f} dR={dR:.4f} asym={asym_pct:+.1f}%')
        
        # Stall check
        enc_total = abs(dL) + abs(dR)
        if dur_ms > 500 and enc_total < 0.02:
            print(f'    ** STALLED ** {enc_total:.4f}m in {dur_ms/1000:.1f}s')
        
        # Gap to next
        if si < len(segs) - 1:
            next_t = rows[segs[si + 1][0]]['t_ms']
            gap = next_t - te
            total_overhead_ms += gap
            print(f'    Gap to next: {gap}ms ({gap/1000:.1f}s)')
    
    print(f'\n  SUMMARY:')
    print(f'    Total driven: {rows[-1]["meters"]:.3f}m / {dist}m ({rows[-1]["meters"]/float(dist)*100:.0f}%)')
    print(f'    Total time: {rows[-1]["t_ms"]/1000:.1f}s / {time_goal}s')
    print(f'    Drive time: {total_drive_ms/1000:.1f}s  Overhead: {total_overhead_ms/1000:.1f}s')
    if len(segs) > 1:
        print(f'    Avg gap: {total_overhead_ms/(len(segs)-1)/1000:.1f}s')
    
    return header, rows

if __name__ == '__main__':
    logdir = 'logs'
    for rid in [107, 108, 109]:
        matches = glob.glob(f'{logdir}/*runlog_{rid}.csv')
        if matches:
            analyze(matches[0])
        else:
            print(f'Run {rid}: not found')
