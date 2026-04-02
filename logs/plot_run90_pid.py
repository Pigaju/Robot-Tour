#!/usr/bin/env python3
"""
Comprehensive PID loop analysis for Run #90.
Shows every stage of the feedback loop:
  1. Measured yaw vs target → yaw error
  2. PID terms (P, I, D) → steerCorr
  3. Mixer: basePwm ± steerCorr → commanded PWM
  4. RUN_MIN_PWM clamp: what actually reaches the motors
  5. Encoder rates: the physical result
  6. Path: lateral drift accumulation
"""
import csv
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

RUN_MIN_PWM = 170.0  # from code: #define RUN_MIN_PWM 170.0f
BFS_DRIVE_STEER_KP = 2.0
STR_STEER_KI = 0.8
STR_STEER_KD = 0.05
STEER_CLAMP = 60.0
STALL_LEFT = 100.0
STALL_RIGHT = 110.0

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
                'gz_dps':     float(parts[7]),
                'corrOut':    float(parts[17]),
                'basePWM':    float(parts[14]),
                'lPwm':       float(parts[24]),
                'rPwm':       float(parts[25]),
                'lateralM':   float(parts[30]),
                'remainingM': float(parts[31]),
                'steerI':     float(parts[37]),
                'distL':      float(parts[51]),
                'distR':      float(parts[52]),
            })
    return rows

def compute_rate(rows, key):
    rates = [0.0]
    for i in range(1, len(rows)):
        dt = (rows[i]['t_ms'] - rows[i-1]['t_ms']) / 1000.0
        dd = rows[i][key] - rows[i-1][key]
        rates.append(dd / dt if dt > 0 else 0.0)
    return rates

def main():
    log = parse_log('logs/20260401_185646_runlog_90.csv')
    if not log:
        print("No data"); return

    t0 = log[0]['t_ms']
    t = np.array([(r['t_ms'] - t0) / 1000.0 for r in log])
    yaw = np.array([r['currentYaw'] for r in log])
    target = np.array([r['targetYaw'] for r in log])
    yawErr = np.array([r['yawErrDeg'] for r in log])
    corrOut = np.array([r['corrOut'] for r in log])
    basePwm = np.array([r['basePWM'] for r in log])
    lPwm_cmd = np.array([r['lPwm'] for r in log])
    rPwm_cmd = np.array([r['rPwm'] for r in log])
    steerI = np.array([r['steerI'] for r in log])
    lateralM = np.array([r['lateralM'] for r in log])
    remainM = np.array([r['remainingM'] for r in log])
    gz = np.array([r['gz_dps'] for r in log])
    distL = np.array([r['distL'] for r in log])
    distR = np.array([r['distR'] for r in log])

    rateL = np.array(compute_rate(log, 'distL'))
    rateR = np.array(compute_rate(log, 'distR'))

    # Reconstruct what driveMotor() actually sends (RUN_MIN_PWM clamp)
    actual_lPwm = np.where(lPwm_cmd == 0, 0, np.maximum(lPwm_cmd, RUN_MIN_PWM))
    actual_rPwm = np.where(rPwm_cmd == 0, 0, np.maximum(rPwm_cmd, RUN_MIN_PWM))
    pwm_diff_cmd = rPwm_cmd - lPwm_cmd
    pwm_diff_actual = actual_rPwm - actual_lPwm

    # Decompose PID terms from corrOut
    # corrOut = Kp*yawErr + Ki*steerI + Kd*dFilt  (we have yawErr and steerI)
    p_term = BFS_DRIVE_STEER_KP * yawErr
    i_term = STR_STEER_KI * steerI
    d_term = corrOut - p_term - i_term  # residual = D term

    fig, axes = plt.subplots(4, 2, figsize=(18, 16))
    fig.suptitle('Run #90 — Full PID Loop Dissection (Straight Test, 1m)\n'
                 'ROOT CAUSE: RUN_MIN_PWM=170 nullifies all PID corrections below 170 PWM',
                 fontsize=13, fontweight='bold', color='red')

    # ── Panel 1: Measured yaw vs target ─────────────────
    ax = axes[0, 0]
    ax.plot(t, target, '--', color='green', linewidth=2, label='targetYaw (0°)')
    ax.plot(t, yaw, '-o', color='blue', markersize=3, label='currentYaw (°)')
    ax.fill_between(t, target, yaw, alpha=0.2, color='red')
    ax.set_title('① Measured vs Target Yaw')
    ax.set_ylabel('Degrees')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.annotate(f'Final yaw: {yaw[-1]:.1f}°\n(should be 0°)',
                xy=(t[-1], yaw[-1]), fontsize=9, color='red',
                xytext=(t[-1]-0.8, yaw[-1]+5),
                arrowprops=dict(arrowstyle='->', color='red'))

    # ── Panel 2: PID terms breakdown ─────────────────
    ax = axes[0, 1]
    ax.plot(t, p_term, '-', color='#1f77b4', linewidth=2, label=f'P = {BFS_DRIVE_STEER_KP}×yawErr')
    ax.plot(t, i_term, '-', color='#ff7f0e', linewidth=2, label=f'I = {STR_STEER_KI}×steerI')
    ax.plot(t, d_term, '-', color='#2ca02c', linewidth=1.5, label=f'D (residual)')
    ax.plot(t, corrOut, '-', color='red', linewidth=2.5, label='corrOut (total, clamped ±60)')
    ax.axhline(60, ls=':', color='black', alpha=0.5, label='clamp ±60')
    ax.axhline(-60, ls=':', color='black', alpha=0.5)
    ax.set_title('② PID Terms → steerCorr')
    ax.set_ylabel('PWM correction')
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)
    # Mark where corrOut hits clamp
    sat_idx = np.where(np.abs(corrOut) >= 59.9)[0]
    if len(sat_idx):
        ax.axvline(t[sat_idx[0]], ls='--', color='red', alpha=0.5)
        ax.annotate(f'SATURATED at t={t[sat_idx[0]]:.1f}s\ncorrOut=±60, no more authority',
                    xy=(t[sat_idx[0]], 60), fontsize=8, color='red',
                    xytext=(t[sat_idx[0]]-0.5, 45),
                    arrowprops=dict(arrowstyle='->', color='red'))

    # ── Panel 3: Commanded PWM (what PID wants) ─────────────────
    ax = axes[1, 0]
    ax.plot(t, lPwm_cmd, '-o', color='#1f77b4', markersize=3, label='lPwm (PID output)')
    ax.plot(t, rPwm_cmd, '-s', color='#d62728', markersize=3, label='rPwm (PID output)')
    ax.plot(t, basePwm, '--', color='gray', label='basePwm')
    ax.axhline(RUN_MIN_PWM, ls='-', color='black', linewidth=2.5, label=f'RUN_MIN_PWM={int(RUN_MIN_PWM)}')
    ax.fill_between(t, 0, RUN_MIN_PWM, alpha=0.1, color='red')
    ax.set_title('③ PID Commanded PWM (before driveMotor clamp)')
    ax.set_ylabel('PWM')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.annotate('ENTIRE left motor command\nis below RUN_MIN_PWM!\nPID correction is erased.',
                xy=(t[len(t)//2], lPwm_cmd[len(t)//2]), fontsize=9, color='red', fontweight='bold',
                xytext=(0.5, 200),
                arrowprops=dict(arrowstyle='->', color='red', lw=2))

    # ── Panel 4: ACTUAL PWM reaching motors ─────────────────
    ax = axes[1, 1]
    ax.plot(t, actual_lPwm, '-o', color='#1f77b4', markersize=3, label='lPwm ACTUAL (after clamp)')
    ax.plot(t, actual_rPwm, '-s', color='#d62728', markersize=3, label='rPwm ACTUAL (after clamp)')
    ax.axhline(RUN_MIN_PWM, ls='--', color='black', alpha=0.5)
    ax.fill_between(t, actual_lPwm, actual_rPwm, alpha=0.2, color='purple')
    ax.set_title('④ ACTUAL PWM at Motors (after RUN_MIN_PWM clamp)')
    ax.set_ylabel('PWM')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax2 = ax.twinx()
    ax2.plot(t, pwm_diff_cmd, '--', color='orange', alpha=0.7, label='Δ commanded')
    ax2.plot(t, pwm_diff_actual, '-', color='red', linewidth=2, label='Δ ACTUAL')
    ax2.set_ylabel('R−L diff (PWM)', color='red')
    ax2.legend(fontsize=7, loc='upper left')

    # ── Panel 5: Integral windup ─────────────────
    ax = axes[2, 0]
    ax.plot(t, steerI, '-', color='#ff7f0e', linewidth=2, label='steerI (integral state)')
    ax.plot(t, yawErr, '-', color='blue', linewidth=1.5, label='yawErr (°)')
    ax.axhline(25.0/STR_STEER_KI, ls=':', color='red', alpha=0.5, label=f'I anti-windup limit (±{25.0/STR_STEER_KI:.1f})')
    ax.axhline(-25.0/STR_STEER_KI, ls=':', color='red', alpha=0.5)
    ax.set_title('⑤ Integral Windup: error persists → I grows → saturates')
    ax.set_ylabel('Value')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.annotate('I accumulates because\nerror never decreases\n(correction is nullified)',
                xy=(t[-1], steerI[-1]), fontsize=8, color='#ff7f0e',
                xytext=(t[-1]-1.5, steerI[-1]*0.5),
                arrowprops=dict(arrowstyle='->', color='#ff7f0e'))

    # ── Panel 6: Encoder rates (physical result) ─────────────────
    ax = axes[2, 1]
    ax.plot(t, rateL, '-o', color='#1f77b4', markersize=3, label='L encoder rate (m/s)')
    ax.plot(t, rateR, '-s', color='#d62728', markersize=3, label='R encoder rate (m/s)')
    ax.set_title('⑥ Encoder Rates (physical motor speeds)')
    ax.set_ylabel('Rate (m/s)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.annotate('Both wheels run at same speed\nbecause both get clamped to 170+',
                xy=(t[len(t)//2], rateL[len(t)//2]), fontsize=9, color='red',
                xytext=(0.5, max(rateL)*0.3),
                arrowprops=dict(arrowstyle='->', color='red'))

    # ── Panel 7: Lateral drift & path ─────────────────
    ax = axes[3, 0]
    ax.plot(t, lateralM * 100, '-', color='purple', linewidth=2, label='lateral drift (cm)')
    ax.set_title('⑦ Lateral Path Drift')
    ax.set_ylabel('cm (lateral)')
    ax.set_xlabel('Time (s)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.annotate(f'Final lateral: {lateralM[-1]*100:.1f} cm off-center',
                xy=(t[-1], lateralM[-1]*100), fontsize=9, color='purple',
                xytext=(t[-1]-1.0, lateralM[-1]*100 + 5),
                arrowprops=dict(arrowstyle='->', color='purple'))

    # ── Panel 8: Root cause summary ─────────────────
    ax = axes[3, 1]
    ax.axis('off')
    summary = """
╔══════════════════════════════════════════════════════════════╗
║  ROOT CAUSE: RUN_MIN_PWM = 170 DESTROYS PID AUTHORITY      ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  driveMotor() clamps ALL PWM values to min 170:             ║
║    pwm = (speed < 170) ? 170 : speed;                       ║
║                                                              ║
║  PID loop commands: lPwm=100, rPwm=169                      ║
║  Actual at motors:  lPwm=170, rPwm=170  ← NO DIFFERENCE!   ║
║                                                              ║
║  The PID's full ±60 PWM correction range (100-190) is       ║
║  compressed to just 0-19 PWM effective differential.        ║
║                                                              ║
║  Timeline:                                                   ║
║   0.1s: yaw_err=2.4° → corrOut=5.8 → cmd 125/135           ║
║         ACTUAL: 170/170 ← zero differential!                ║
║   0.5s: yaw_err=8.2° → corrOut=18.8 → cmd 112/148          ║
║         ACTUAL: 170/170 ← still zero!                       ║
║   1.0s: yaw_err=13.7° → corrOut=32 → cmd 100/161           ║
║         ACTUAL: 170/170 ← PID maxed, no effect              ║
║   2.0s: yaw_err=22.5° → corrOut=60 (SATURATED) → cmd 100/189║
║         ACTUAL: 170/189 ← finally 19 PWM diff (too late!)   ║
║                                                              ║
║  FIX: Revert RUN_MIN_PWM to 95 (original value) or apply   ║
║       min-PWM clamp ONLY during softstart, not in PID loop  ║
╚══════════════════════════════════════════════════════════════╝
"""
    ax.text(0.02, 0.98, summary, transform=ax.transAxes,
            fontsize=8, verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='lightyellow', edgecolor='red', linewidth=2))

    for row in axes:
        for a in row:
            if a.axison:
                a.set_xlabel('Time (s)')

    plt.tight_layout()
    outpath = 'logs/run90_pid_analysis.png'
    plt.savefig(outpath, dpi=150, bbox_inches='tight')
    print(f"Saved {outpath}")

if __name__ == '__main__':
    main()
