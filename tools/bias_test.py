#!/usr/bin/env python3
"""Test motor bias direction sweep."""
import sys, math
sys.path.insert(0, 'tools')
from sweep_straight_pid import StraightDrivePID, SimState, StraightRecord, compute_straight_metrics, _wrap_deg
from sim import MotorModel

motor = MotorModel(gain_l=0.02528, gain_r=0.00647, deadband_l=95.0, deadband_r=105.0)

def run(ki, kd, clamp, imax, bias):
    pid = StraightDrivePID(kp=2.0, ki=ki, kd=kd, corr_clamp=clamp, motor_bias_pwm=bias)
    pid.i_max_corr = imax
    pid.reset()
    state = SimState()
    records = []
    driven = 0.0
    while driven < 0.92 and state.t < 30:
        ye = _wrap_deg(0 - state.yaw)
        lp, rp, sc, bp = pid.update(ye, driven, 1.0, 0.01)
        vl = motor.gain_l * max(0, lp - motor.deadband_l) if lp > motor.deadband_l else 0
        vr = motor.gain_r * max(0, rp - motor.deadband_r) if rp > motor.deadband_r else 0
        omega = (vr - vl) / 0.148
        h = math.radians(state.yaw)
        state.x += (vl + vr) / 2 * math.cos(h) * 0.01
        state.y += (vl + vr) / 2 * math.sin(h) * 0.01
        state.yaw = _wrap_deg(state.yaw + math.degrees(omega) * 0.01)
        state.dist_l += vl * 0.01
        state.dist_r += vr * 0.01
        driven = (state.dist_l + state.dist_r) / 2
        state.t += 0.01
        records.append(StraightRecord(
            t=state.t, x=state.x, y=state.y, yaw=state.yaw, yaw_err=ye,
            l_pwm=lp, r_pwm=rp, steer_corr=sc, steer_i=pid.integral,
            driven_m=driven, dist_l=state.dist_l, dist_r=state.dist_r, base_pwm=bp))
    return compute_straight_metrics(records, "", 2.0, ki, kd)

print("=== MOTOR BIAS DIRECTION TEST (Ki=8, clamp=60) ===")
print("Motor: L is 3.9x faster than R. Positive bias = +L, -R")
print()
print(f"  {'bias':>5s}  {'yawRMS':>7s}  {'latDrift':>9s}  {'pwmDiffRMS':>10s}  {'score':>7s}")
for b in [-20, -15, -10, -5, -3, -1, 0, 1, 3, 5, 10, 15, 20]:
    m = run(8.0, 0.10, 60, 25, b)
    print(f"  {b:+5d}  {m.yaw_rms_deg:6.2f}d  {m.lateral_drift_m*100:8.2f}cm  {m.pwm_diff_rms:9.1f}  {m.score:7.2f}")

print()
print("=== BEST COMBO: Ki=8 + optimal bias ===")
for b in [-20, -15, -10, -5, 0]:
    m = run(8.0, 0.10, 90, 50, b)
    print(f"  bias={b:+3d} Ki=8 clamp=90 iMax=50:  yaw={m.yaw_rms_deg:.2f}d  lat={m.lateral_drift_m*100:.2f}cm  score={m.score:.2f}")
