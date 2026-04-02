#!/usr/bin/env python3
"""Plot run #88 straight test with annotated sign-chain analysis."""
import csv
import matplotlib.pyplot as plt
import numpy as np

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
                't_ms':       float(parts[0]),
                'meters':     float(parts[1]),
                'currentYaw': float(parts[3]),
                'targetYaw':  float(parts[4]),
                'yawErrDeg':  float(parts[5]),
                'yawI':       float(parts[6]),
                'corrOut':    float(parts[17]),
                'lPwm':       float(parts[24]),
                'rPwm':       float(parts[25]),
                'distL':      float(parts[51]),
                'distR':      float(parts[52]),
                'steerI':     float(parts[37]),
            })
    return rows

def compute_enc_rate(rows, key):
    rates = [0.0]
    for i in range(1, len(rows)):
        dt = (rows[i]['t_ms'] - rows[i-1]['t_ms']) / 1000.0
        dd = rows[i][key] - rows[i-1][key]
        rates.append(dd / dt if dt > 0 else 0.0)
    return rates

def main():
    log = parse_log('logs/20260401_183902_runlog_88.csv')
    if not log:
        print("No data found in run 88 log")
        return

    t0 = log[0]['t_ms']
    t = [(r['t_ms'] - t0) / 1000.0 for r in log]
    lPwm = [r['lPwm'] for r in log]
    rPwm = [r['rPwm'] for r in log]
    yawErr = [r['yawErrDeg'] for r in log]
    corrOut = [r['corrOut'] for r in log]
    currentYaw = [r['currentYaw'] for r in log]
    steerI = [r['steerI'] for r in log]

    rateL = compute_enc_rate(log, 'distL')
    rateR = compute_enc_rate(log, 'distR')

    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    fig.suptitle('Run #88 — Straight Test 1m (BROKEN: positive feedback from motor address swap)',
                 fontsize=13, fontweight='bold', color='red')

    # 1: PWM
    ax = axes[0, 0]
    ax.plot(t, lPwm, '-o', color='#1f77b4', markersize=3, label='lPwm (→ addr 0x20)')
    ax.plot(t, rPwm, '-s', color='#d62728', markersize=3, label='rPwm (→ addr 0x21)')
    ax.fill_between(t, lPwm, rPwm, alpha=0.15, color='purple')
    ax.axhline(130, ls='--', color='gray', alpha=0.5, label='basePwm')
    ax.set_title('PWM Commands')
    ax.set_ylabel('PWM')
    ax.set_xlabel('Time (s)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.annotate('lPwm hits floor (100)\nwhile rPwm runs away',
                xy=(t[-5], lPwm[-5]), fontsize=8, color='red',
                xytext=(0.5, 110), textcoords=('data', 'data'),
                arrowprops=dict(arrowstyle='->', color='red'))

    # 2: Yaw & correction
    ax = axes[0, 1]
    ax.plot(t, currentYaw, '-^', color='blue', markersize=3, label='currentYaw (°)')
    ax.plot(t, yawErr, '-v', color='orange', markersize=3, label='yawErrDeg (°)')
    ax.plot(t, corrOut, '--', color='green', linewidth=2, label='corrOut (PWM)')
    ax.axhline(0, ls='-', color='black', alpha=0.3)
    ax.set_title('Yaw Tracking & Correction')
    ax.set_ylabel('Value')
    ax.set_xlabel('Time (s)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.annotate('Yaw goes negative → yawErr positive\n→ positive corrOut → WRONG motor sped up\n→ more negative yaw (POSITIVE FEEDBACK)',
                xy=(t[len(t)//2], yawErr[len(t)//2]), fontsize=8, color='red',
                xytext=(0.2, max(corrOut)*0.7),
                arrowprops=dict(arrowstyle='->', color='red'))

    # 3: Encoder rates
    ax = axes[1, 0]
    ax.plot(t, rateL, '-o', color='#1f77b4', markersize=3, label='L encoder rate (m/s)')
    ax.plot(t, rateR, '-s', color='#d62728', markersize=3, label='R encoder rate (m/s)')
    ax.set_title('Encoder Rates')
    ax.set_ylabel('Rate (m/s)')
    ax.set_xlabel('Time (s)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 4: Sign-chain explanation text
    ax = axes[1, 1]
    ax.axis('off')
    explanation = """
ROOT CAUSE: Motor Address Swap Inverted PID Feedback

ORIGINAL CODE (worked):                CURRENT CODE (broken):
  MOTOR_L_ADDR = 0x21                    MOTOR_L_ADDR = 0x20  ← SWAPPED
  MOTOR_R_ADDR = 0x20                    MOTOR_R_ADDR = 0x21  ← SWAPPED

MIXER (same in both):
  leftPwm  = basePwm − steerCorr   →  sent to MOTOR_L_ADDR
  rightPwm = basePwm + steerCorr   →  sent to MOTOR_R_ADDR

SIGN CHAIN (current, broken):
  1. Robot drifts LEFT  →  currentYaw < 0
  2. yaw_err = target(0) − currentYaw  →  POSITIVE
  3. steerCorr = Kp × yaw_err  →  POSITIVE
  4. leftPwm = base − (positive)  →  DECREASES  →  sent to 0x20
  5. rightPwm = base + (positive) →  INCREASES  →  sent to 0x21
  6. Physical effect: 0x21 faster, 0x20 slower
     → robot turns MORE LEFT (same direction as drift!)
     → POSITIVE FEEDBACK → runaway

FIX: Revert MOTOR_L_ADDR to 0x21, MOTOR_R_ADDR to 0x20
     (and revert pivot direction swap)
"""
    ax.text(0.05, 0.95, explanation, transform=ax.transAxes,
            fontsize=8.5, verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='lightyellow', edgecolor='orange'))

    plt.tight_layout()
    outpath = 'logs/run88_analysis.png'
    plt.savefig(outpath, dpi=150, bbox_inches='tight')
    print(f"Saved {outpath}")

if __name__ == '__main__':
    main()
