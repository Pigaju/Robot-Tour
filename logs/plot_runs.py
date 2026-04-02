#!/usr/bin/env python3
"""Plot PWM, encoder rate, and yaw for runs 87 and 85."""
import csv
import matplotlib.pyplot as plt
import numpy as np
import sys

def parse_log(path):
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith('#') or line.startswith('t_ms,'):
                continue
            parts = line.strip().split(',')
            if len(parts) < 50:
                continue
            rows.append({
                't_ms':    float(parts[0]),
                'meters':  float(parts[1]),
                'currentYaw': float(parts[3]),
                'targetYaw':  float(parts[4]),
                'yawErrDeg':  float(parts[5]),
                'corrOut': float(parts[17]),
                'lPwm':    float(parts[24]),
                'rPwm':    float(parts[25]),
                'distL':   float(parts[51]),
                'distR':   float(parts[52]),
                'steerI':  float(parts[37]),
            })
    return rows

def compute_enc_rate(rows, key):
    """Compute encoder rate (m/s) using finite differences."""
    rates = [0.0]
    for i in range(1, len(rows)):
        dt = (rows[i]['t_ms'] - rows[i-1]['t_ms']) / 1000.0
        dd = rows[i][key] - rows[i-1][key]
        rates.append(dd / dt if dt > 0 else 0.0)
    return rates

def segment_rows(rows):
    """Split rows into segments by detecting large time gaps (>500ms)."""
    if not rows:
        return []
    segments = [[rows[0]]]
    for i in range(1, len(rows)):
        if rows[i]['t_ms'] - rows[i-1]['t_ms'] > 500:
            segments.append([])
        segments[-1].append(rows[i])
    return segments

def plot_run(rows, title, axes_row):
    """Plot a single run across 3 subplots."""
    segments = segment_rows(rows)

    ax_pwm, ax_enc, ax_yaw = axes_row
    t0 = rows[0]['t_ms'] if rows else 0

    seg_colors_l = ['#1f77b4', '#17becf', '#9467bd']
    seg_colors_r = ['#d62728', '#ff7f0e', '#e377c2']

    for si, seg in enumerate(segments):
        t = [(r['t_ms'] - t0) / 1000.0 for r in seg]
        lPwm = [r['lPwm'] for r in seg]
        rPwm = [r['rPwm'] for r in seg]
        yawErr = [r['yawErrDeg'] for r in seg]
        corrOut = [r['corrOut'] for r in seg]

        rateL = compute_enc_rate(seg, 'distL')
        rateR = compute_enc_rate(seg, 'distR')

        cl = seg_colors_l[si % len(seg_colors_l)]
        cr = seg_colors_r[si % len(seg_colors_r)]
        label_sfx = f" seg{si}" if len(segments) > 1 else ""

        # PWM plot
        ax_pwm.plot(t, lPwm, '-o', color=cl, markersize=3,
                    label=f'lPwm{label_sfx}')
        ax_pwm.plot(t, rPwm, '-s', color=cr, markersize=3,
                    label=f'rPwm{label_sfx}')
        ax_pwm.fill_between(t, lPwm, rPwm, alpha=0.15, color='purple')

        # Encoder rate plot (m/s)
        ax_enc.plot(t, rateL, '-o', color=cl, markersize=3,
                    label=f'L rate{label_sfx}')
        ax_enc.plot(t, rateR, '-s', color=cr, markersize=3,
                    label=f'R rate{label_sfx}')

        # Yaw error plot
        ax_yaw.plot(t, yawErr, '-^', color='orange', markersize=3,
                    label=f'yawErr{label_sfx}' if si == 0 else None)
        ax_yaw.plot(t, corrOut, '--', color='green', markersize=2,
                    label=f'corrOut{label_sfx}' if si == 0 else None)

    ax_pwm.set_title(f'{title} — PWM Values')
    ax_pwm.set_ylabel('PWM')
    ax_pwm.legend(fontsize=7, ncol=2)
    ax_pwm.grid(True, alpha=0.3)

    ax_enc.set_title(f'{title} — Encoder Rate (m/s)')
    ax_enc.set_ylabel('Rate (m/s)')
    ax_enc.legend(fontsize=7, ncol=2)
    ax_enc.grid(True, alpha=0.3)

    ax_yaw.set_title(f'{title} — Yaw Error (°) & Correction (PWM)')
    ax_yaw.set_ylabel('Value')
    ax_yaw.legend(fontsize=7, loc='upper left')
    ax_yaw.grid(True, alpha=0.3)

    for ax in axes_row:
        ax.set_xlabel('Time from run start (s)')

def main():
    log87 = parse_log('logs/20260401_181352_runlog_87.csv')
    log85 = parse_log('logs/20260401_181352_runlog_85.csv')

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle('Run Comparison: Straight Test #87 vs BFS Nav #85', fontsize=14, fontweight='bold')

    plot_run(log87, 'Run #87 (Straight Test, 1m)', axes[0])
    plot_run(log85, 'Run #85 (BFS Nav, 2m)', axes[1])

    plt.tight_layout()
    plt.savefig('logs/run_analysis_87_85.png', dpi=150, bbox_inches='tight')
    print("Saved logs/run_analysis_87_85.png")
    # plt.show()  # skip interactive window

if __name__ == '__main__':
    main()
