#!/usr/bin/env python3
"""Quick analysis of stacked precision improvements."""
import sys, math
sys.path.insert(0, 'tools')
from sweep_straight_pid import StraightDrivePID, SimState, StraightRecord, compute_straight_metrics, _wrap_deg
from sim import MotorModel

motor = MotorModel(gain_l=0.02528, gain_r=0.00647, deadband_l=95.0, deadband_r=105.0)
TARGET = 1.0
DT = 0.010

def run_sim(ki, kd, clamp, i_max, bias, label):
    pid = StraightDrivePID(kp=2.0, ki=ki, kd=kd, corr_clamp=clamp, motor_bias_pwm=bias)
    pid.i_max_corr = i_max
    pid.reset()
    state = SimState()
    records = []
    driven = 0.0
    while driven < TARGET - 0.08 and state.t < 30:
        yaw_err = _wrap_deg(0 - state.yaw)
        lp, rp, sc, bp = pid.update(yaw_err, driven, TARGET, DT)
        vl = motor.gain_l * max(0, lp - motor.deadband_l) if lp > motor.deadband_l else 0
        vr = motor.gain_r * max(0, rp - motor.deadband_r) if rp > motor.deadband_r else 0
        v_avg = (vl + vr) / 2.0
        omega = (vr - vl) / 0.148
        heading_rad = math.radians(state.yaw)
        state.x += v_avg * math.cos(heading_rad) * DT
        state.y += v_avg * math.sin(heading_rad) * DT
        state.yaw = _wrap_deg(state.yaw + math.degrees(omega) * DT)
        state.dist_l += vl * DT
        state.dist_r += vr * DT
        driven = (state.dist_l + state.dist_r) / 2.0
        state.t += DT
        records.append(StraightRecord(
            t=state.t, x=state.x, y=state.y, yaw=state.yaw, yaw_err=yaw_err,
            l_pwm=lp, r_pwm=rp, steer_corr=sc, steer_i=pid.integral,
            driven_m=driven, dist_l=state.dist_l, dist_r=state.dist_r, base_pwm=bp))
    m = compute_straight_metrics(records, label, 2.0, ki, kd)
    # Count clamped samples
    corrs = [abs(r.steer_corr) for r in records[10:]]
    clamped = sum(1 for c in corrs if c >= clamp - 0.5)
    pct = 100 * clamped / len(corrs) if corrs else 0
    return m, pct

configs = [
    # (ki, kd, clamp, i_max, bias, label)
    (6.0, 0.10, 60, 25, 1,  "A. Current firmware (Ki=6 clamp=60 iMax=25 bias=1)"),
    (6.0, 0.10, 90, 25, 1,  "B. + Raise steerCorr clamp to 90"),
    (6.0, 0.10, 90, 50, 1,  "C. + Raise anti-windup iMax to 50"),
    (8.0, 0.10, 90, 50, 1,  "D. + Raise Ki to 8.0"),
    (8.0, 0.10, 90, 50, 10, "E. + motor_bias_pwm = 10 (feedforward)"),
    (8.0, 0.10, 90, 50, 15, "F. + motor_bias_pwm = 15"),
    (8.0, 0.10, 90, 50, 20, "G. + motor_bias_pwm = 20"),
    (6.0, 0.10, 60, 50, 1,  "H. Just raise iMax to 50 (keep clamp=60)"),
    (6.0, 0.10, 60, 50, 10, "I. iMax=50 + bias=10 (keep clamp=60)"),
]

print(f"{'='*100}")
print(f"  STACKED IMPROVEMENT ANALYSIS — Motor asymmetry L/R = {motor.gain_l/motor.gain_r:.2f}x")
print(f"{'='*100}")
print(f"  {'Config':<55s}  {'yawRMS':>7s}  {'latDrift':>9s}  {'pwmDiff':>8s}  {'%clamp':>7s}  {'score':>7s}")
print(f"  {'-'*55}  {'-'*7}  {'-'*9}  {'-'*8}  {'-'*7}  {'-'*7}")

for ki, kd, clamp, imax, bias, label in configs:
    m, pct = run_sim(ki, kd, clamp, imax, bias, label)
    print(f"  {label:<55s}  {m.yaw_rms_deg:6.2f}°  {m.lateral_drift_m*100:8.2f}cm  {m.pwm_diff_rms:7.1f}  {pct:6.0f}%  {m.score:7.2f}")

print()
print("  KEY: %clamp = percentage of time steerCorr is saturated at the clamp limit")
print("  LOWER score = BETTER tracking (weighted: 3×yawRMS + 200×lateral + 0.1×pwmDiff)")
