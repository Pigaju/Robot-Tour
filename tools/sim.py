#!/usr/bin/env python3
"""
Robot Tour — Physics Simulation & Virtual Testing

Extracts a physics model from CSV square-test logs and replays runs with
adjustable PID gains, turn tolerances, and motor parameters. Useful for
testing firmware changes without re-flashing the robot.

Usage:
    python3 tools/sim.py                         # sim latest log with current + proposed gains
    python3 tools/sim.py logs/some_runlog.csv    # sim a specific log
    python3 tools/sim.py --sweep                 # sweep PID parameters
    python3 tools/sim.py --compare               # compare old vs new tolerance

Requires: pip install numpy matplotlib
"""

import argparse
import csv
import glob
import math
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Constants matching firmware
# ---------------------------------------------------------------------------
TRACK_WIDTH_M = 0.148
PPM = 906.1
PPM_L = 922.9
PPM_R = 889.3
MOTOR_BIAS_PWM = 1
MIN_PWM_L = 100
MIN_PWM_R = 110

# ---------------------------------------------------------------------------
# CSV parsing
# ---------------------------------------------------------------------------

@dataclass
class LogHeader:
    run_id: int = 0
    mode: str = "IMU"
    dist: float = 2.0
    ppm: float = 906.1
    ppm_l: float = 922.9
    ppm_r: float = 889.3
    mtr_bias: int = 1

@dataclass
class LogSample:
    t_ms: float
    meters: float
    forward_m: float
    current_yaw: float
    target_yaw: float
    yaw_err_deg: float
    gz_dps: float
    base_pwm: float
    corr_out: float
    l_pwm: float
    r_pwm: float
    remaining_m: float
    steer_i: float
    dist_l: float
    dist_r: float

def parse_csv(path: str) -> Tuple[LogHeader, List[LogSample]]:
    """Parse a robot CSV log file, return header + samples."""
    header = LogHeader()
    samples = []
    with open(path, newline="") as fh:
        lines = fh.readlines()

    data_lines = []
    for line in lines:
        line = line.strip()
        if not line:
            continue
        if line.startswith("# TEST_BEGIN"):
            for tok in line.split():
                if "=" in tok:
                    k, v = tok.split("=", 1)
                    if k == "id":
                        header.run_id = int(v)
                    elif k == "mode":
                        header.mode = v
                    elif k == "dist":
                        header.dist = float(v)
                    elif k == "ppm":
                        header.ppm = float(v)
                    elif k == "ppmL":
                        header.ppm_l = float(v)
                    elif k == "ppmR":
                        header.ppm_r = float(v)
                    elif k == "mtr_bias":
                        header.mtr_bias = int(v)
            continue
        if line.startswith("#"):
            continue
        data_lines.append(line)

    if len(data_lines) < 2:
        return header, samples

    reader = csv.DictReader(data_lines)
    for row in reader:
        try:
            s = LogSample(
                t_ms=float(row.get("t_ms", 0)),
                meters=float(row.get("meters", 0)),
                forward_m=float(row.get("forwardM", 0)),
                current_yaw=float(row.get("currentYaw", 0)),
                target_yaw=float(row.get("targetYaw", 0)),
                yaw_err_deg=float(row.get("yawErrDeg", 0)),
                gz_dps=float(row.get("gz_dps", 0)),
                base_pwm=float(row.get("basePWM", 0)),
                corr_out=float(row.get("corrOut", 0)),
                l_pwm=float(row.get("lPwm", 0)),
                r_pwm=float(row.get("rPwm", 0)),
                remaining_m=float(row.get("remainingM", 0)),
                steer_i=float(row.get("steerI", 0)),
                dist_l=float(row.get("distL", 0)),
                dist_r=float(row.get("distR", 0)),
            )
            samples.append(s)
        except (ValueError, TypeError):
            continue
    return header, samples


# ---------------------------------------------------------------------------
# Segment detection — split drive and turn phases
# ---------------------------------------------------------------------------

@dataclass
class Segment:
    kind: str  # "drive" or "turn"
    start_idx: int
    end_idx: int
    t_start_ms: float
    t_end_ms: float
    start_yaw: float
    end_yaw: float
    target_yaw: float
    dist_driven: float  # meters advanced during this segment
    samples: List[LogSample] = field(default_factory=list)

def segment_run(samples: List[LogSample], gap_ms: float = 500) -> List[Segment]:
    """Split a square-test run into drive/turn segments by time gaps.
    
    The square test CSV only logs during drive phases, so turns appear as
    time gaps with large yaw changes. We infer implicit turn segments from
    these gaps.
    """
    if not samples:
        return []

    # First pass: find drive segments (contiguous logged data)
    drive_groups = []
    seg_start = 0
    for i in range(1, len(samples)):
        dt = samples[i].t_ms - samples[i - 1].t_ms
        if dt > gap_ms:
            drive_groups.append((seg_start, i - 1))
            seg_start = i
    drive_groups.append((seg_start, len(samples) - 1))

    # Build segment list interleaving implicit turns with drive segments
    segments = []
    for g_idx, (i0, i1) in enumerate(drive_groups):
        # Insert implicit turn between consecutive drive groups
        if g_idx > 0:
            prev_end = drive_groups[g_idx - 1][1]
            s_prev = samples[prev_end]
            s_cur = samples[i0]
            yaw_change = _wrap_deg(s_cur.current_yaw - s_prev.current_yaw)
            if abs(yaw_change) > 10.0:
                segments.append(Segment(
                    kind="turn",
                    start_idx=prev_end, end_idx=i0,
                    t_start_ms=s_prev.t_ms, t_end_ms=s_cur.t_ms,
                    start_yaw=s_prev.current_yaw, end_yaw=s_cur.current_yaw,
                    target_yaw=s_cur.target_yaw,
                    dist_driven=0.0,
                    samples=[],  # no logged samples during turns
                ))

        # Add drive segment
        if i1 > i0:
            s0 = samples[i0]
            s1 = samples[i1]
            segments.append(Segment(
                kind="drive",
                start_idx=i0, end_idx=i1,
                t_start_ms=s0.t_ms, t_end_ms=s1.t_ms,
                start_yaw=s0.current_yaw, end_yaw=s1.current_yaw,
                target_yaw=s0.target_yaw,
                dist_driven=s1.meters - s0.meters,
                samples=samples[i0:i1 + 1],
            ))

    return segments


# ---------------------------------------------------------------------------
# Physics model (extracted from logs)
# ---------------------------------------------------------------------------

@dataclass
class MotorModel:
    """Simple first-order motor model: PWM → wheel speed (m/s)."""
    # Linear: speed = gain * (pwm - deadband)  for pwm > deadband, else 0
    gain_l: float = 0.0   # m/s per PWM unit
    gain_r: float = 0.0
    deadband_l: float = 95.0  # PWM below which wheel doesn't move
    deadband_r: float = 105.0
    # Rotational: pivot_rate = pivot_gain * (pwm - pivot_deadband)  (deg/s)
    # pivot_deadband uses min(deadband_l, deadband_r) since both motors contribute
    # to rotation during pivots — even if only one wheel exceeds its deadband
    pivot_deadband: float = 95.0  # min of individual deadbands
    pivot_gain: float = 0.0       # deg/s per effective PWM above pivot_deadband
    # Friction model for pivots: coulomb friction offset (deg/s)
    pivot_friction_dps: float = 0.0
    # Rotational inertia: how quickly angular velocity decays when motors stop
    pivot_decay_tau: float = 0.15  # seconds (time constant for angular velocity decay)

def extract_motor_model(segments: List[Segment], samples: List[LogSample]) -> MotorModel:
    """Extract motor model parameters from drive and turn segments."""
    model = MotorModel()

    # Extract speed-per-PWM from drive segments
    speeds_l = []
    speeds_r = []
    for seg in segments:
        if seg.kind != "drive" or len(seg.samples) < 4:
            continue
        # Use middle portion (skip first/last 20% for accel/decel)
        n = len(seg.samples)
        i0 = max(1, n // 5)
        i1 = min(n - 1, n * 4 // 5)
        for i in range(i0, i1):
            s0 = seg.samples[i - 1]
            s1 = seg.samples[i]
            dt = (s1.t_ms - s0.t_ms) / 1000.0
            if dt < 0.01:
                continue
            dl = s1.dist_l - s0.dist_l
            dr = s1.dist_r - s0.dist_r
            vl = dl / dt
            vr = dr / dt
            pwm_l = s0.l_pwm
            pwm_r = s0.r_pwm
            if pwm_l > 100:
                speeds_l.append((pwm_l, vl))
            if pwm_r > 110:
                speeds_r.append((pwm_r, vr))

    if speeds_l:
        pwms, vels = zip(*speeds_l)
        pwms, vels = np.array(pwms), np.array(vels)
        model.deadband_l = 95.0
        active = pwms - model.deadband_l
        active = np.maximum(active, 1.0)
        model.gain_l = float(np.mean(vels / active))

    if speeds_r:
        pwms, vels = zip(*speeds_r)
        pwms, vels = np.array(pwms), np.array(vels)
        model.deadband_r = 105.0
        active = pwms - model.deadband_r
        active = np.maximum(active, 1.0)
        model.gain_r = float(np.mean(vels / active))

    # Set pivot deadband = min of individual deadbands (both motors contribute to rotation)
    model.pivot_deadband = min(model.deadband_l, model.deadband_r)

    # Extract pivot dynamics from implicit turn segments
    # The gap between drive segments includes the full turn cycle:
    #   brake/stop + turn PID active + settle
    # Average rate = yaw_change / gap_time estimates overall dynamics.
    # We calibrate pivot_gain against pivot_deadband using an estimated
    # average effective PWM across the fast/medium/crawl turn phases.
    turn_rates = []
    for seg in segments:
        if seg.kind == "turn":
            dt_turn = (seg.t_end_ms - seg.t_start_ms) / 1000.0
            yaw_change = abs(_wrap_deg(seg.end_yaw - seg.start_yaw))
            if dt_turn > 0.1 and yaw_change > 20:
                turn_rates.append((yaw_change, dt_turn))

    if turn_rates:
        # Average turn rate across all detected turns
        total_yaw = sum(r[0] for r in turn_rates)
        total_time = sum(r[1] for r in turn_rates)
        avg_rate = total_yaw / total_time
        # Estimate average effective PWM above pivot_deadband during turn:
        # Turn PWM profile: 130 (>45°), 110-130 (15-45°), 100-110 (5-15°), ~100 (<5°)
        # Weighted average across a 90° turn ≈ 118 PWM
        avg_turn_pwm = 118.0
        avg_eff_pwm = avg_turn_pwm - model.pivot_deadband
        if avg_eff_pwm < 1:
            avg_eff_pwm = 1
        model.pivot_gain = avg_rate / avg_eff_pwm

    return model


# ---------------------------------------------------------------------------
# PID controllers (matching firmware exactly)
# ---------------------------------------------------------------------------

def _wrap_deg(d: float) -> float:
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return d

@dataclass
class TurnPIDParams:
    """Parameters for the turn (pivot) PID controller."""
    ki: float = 1.0
    kd: float = 0.03
    i_leak_tau: float = 1.0
    i_max_pwm: float = 12.0
    d_filter_tau: float = 0.030
    tolerance_deg: float = 2.0     # was 5.0
    settle_ms: float = 300.0
    gyro_still_dps: float = 3.0    # was 8.0
    turn_pwm: float = 130.0
    timeout_ms: float = 5000.0

@dataclass
class DrivePIDParams:
    """Parameters for the drive (straight) steering PID controller."""
    kp: float = 2.0
    ki: float = 1.0    # square test uses 4.0
    kd: float = 0.06   # square test uses 0.20
    i_leak_tau: float = 1.0
    i_max_corr: float = 25.0
    d_filter_tau: float = 0.030
    corr_clamp: float = 60.0
    drive_pwm: float = 130.0
    decel_boost_kp: float = 1.5
    decel_zone_m: float = 0.30
    speed_decel_zone_m: float = 0.20
    speed_decel_floor: float = 0.30
    arrive_tolerance_m: float = 0.08
    motor_bias_pwm: int = 1
    init_prev_err: bool = True  # fix derivative kick

@dataclass
class SqDrivePIDParams(DrivePIDParams):
    """Square test uses more aggressive drive PID gains."""
    ki: float = 4.0
    kd: float = 0.20

class TurnPID:
    """Simulates the firmware turn PID controller."""
    def __init__(self, params: TurnPIDParams):
        self.p = params
        self.integral = 0.0
        self.prev_err = 0.0
        self.d_filt = 0.0
        self.first = True
        self.in_tol_t = None  # time when first entered tolerance

    def reset(self, initial_err: float = 0.0):
        self.integral = 0.0
        self.prev_err = initial_err
        self.d_filt = 0.0
        self.first = True
        self.in_tol_t = None

    def update(self, yaw_err: float, gz_dps: float, dt: float, t: float) -> Tuple[Optional[float], bool]:
        """
        Returns (pwm_command, turn_done).
        pwm_command is None when in tolerance (motors stopped).
        Positive pwm → pivot in positive yaw direction.
        """
        if self.first:
            self.prev_err = yaw_err
            self.first = False

        abs_err = abs(yaw_err)

        # Check tolerance
        if abs_err < self.p.tolerance_deg:
            gyro_still = abs(gz_dps) < self.p.gyro_still_dps
            if self.in_tol_t is None and gyro_still:
                self.in_tol_t = t
            if not gyro_still:
                self.in_tol_t = None
            if self.in_tol_t is not None and (t - self.in_tol_t) >= self.p.settle_ms / 1000.0:
                return None, True  # turn complete
            return None, False  # in tolerance, waiting

        # Out of tolerance
        self.in_tol_t = None

        # Leaky integral
        self.integral -= self.integral * (dt / self.p.i_leak_tau)
        if abs_err > 0.5:
            self.integral += yaw_err * dt
        i_lim = self.p.i_max_pwm / self.p.ki if self.p.ki > 0 else 100
        self.integral = max(-i_lim, min(i_lim, self.integral))

        # Filtered derivative
        raw_deriv = (yaw_err - self.prev_err) / dt if dt > 0 else 0
        self.prev_err = yaw_err
        alpha = dt / (self.p.d_filter_tau + dt)
        self.d_filt += alpha * (raw_deriv - self.d_filt)

        # Base PWM from crawl zone
        if abs_err > 45:
            base_pwm = self.p.turn_pwm
        elif abs_err > 15:
            base_pwm = 110 + (abs_err - 15) * (self.p.turn_pwm - 110) / 30
        elif abs_err > 5:
            t_f = (abs_err - 5) / 10
            base_pwm = 100 + t_f * 10
        else:
            base_pwm = 100

        err_sign = 1.0 if yaw_err > 0 else -1.0
        correction = (self.p.ki * self.integral + self.p.kd * self.d_filt) * err_sign
        pwm = base_pwm + correction
        pwm = max(100, min(200, pwm))

        sign = 1 if yaw_err > 0 else -1
        return sign * pwm, False

class DrivePID:
    """Simulates the firmware drive steering PID controller."""
    def __init__(self, params: DrivePIDParams):
        self.p = params
        self.integral = 0.0
        self.prev_err = 0.0
        self.d_filt = 0.0
        self.first = True

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0
        self.d_filt = 0.0
        self.first = True

    def update(self, yaw_err: float, driven_m: float, segment_dist_m: float,
               dt: float) -> Tuple[float, float, float]:
        """
        Returns (left_pwm, right_pwm, steer_corr).
        """
        if self.first:
            self.first = False
            if self.p.init_prev_err:
                self.prev_err = yaw_err
            dt = 0.010

        # Leaky integral
        self.integral -= self.integral * (dt / self.p.i_leak_tau)
        if abs(yaw_err) > 0.1:
            self.integral += yaw_err * dt
        i_lim = self.p.i_max_corr / self.p.ki if self.p.ki > 0 else 100
        self.integral = max(-i_lim, min(i_lim, self.integral))

        # Filtered derivative
        raw_deriv = (yaw_err - self.prev_err) / dt if dt > 0 else 0
        self.prev_err = yaw_err
        alpha = dt / (self.p.d_filter_tau + dt)
        self.d_filt += alpha * (raw_deriv - self.d_filt)

        # Effective Kp (boost in decel zone)
        remaining = segment_dist_m - driven_m
        kp = self.p.kp
        if remaining < self.p.decel_zone_m:
            kp *= self.p.decel_boost_kp

        steer_corr = kp * yaw_err + self.p.ki * self.integral + self.p.kd * self.d_filt
        steer_corr = max(-self.p.corr_clamp, min(self.p.corr_clamp, steer_corr))

        # Speed factor
        speed_factor = 1.0
        if remaining < self.p.speed_decel_zone_m:
            speed_factor = remaining / self.p.speed_decel_zone_m
            speed_factor = max(self.p.speed_decel_floor, speed_factor)

        base_pwm = self.p.drive_pwm * speed_factor
        base_pwm = max(110.0, base_pwm)

        left_pwm = base_pwm - steer_corr + self.p.motor_bias_pwm
        right_pwm = base_pwm + steer_corr - self.p.motor_bias_pwm

        left_pwm = max(0, min(255, left_pwm))
        right_pwm = max(0, min(255, right_pwm))
        if 0 < left_pwm < 100:
            left_pwm = 100
        if 0 < right_pwm < 110:
            right_pwm = 110

        return left_pwm, right_pwm, steer_corr


# ---------------------------------------------------------------------------
# Robot physics simulation
# ---------------------------------------------------------------------------

@dataclass
class SimState:
    t: float = 0.0        # seconds
    x: float = 0.0        # meters (world frame)
    y: float = 0.0        # meters (world frame)
    yaw: float = 0.0      # degrees (IMU convention)
    gz_dps: float = 0.0   # angular rate
    dist_l: float = 0.0   # cumulative left wheel distance
    dist_r: float = 0.0   # cumulative right wheel distance

@dataclass
class SimRecord:
    """One recorded time step of the simulation."""
    t: float
    x: float
    y: float
    yaw: float
    target_yaw: float
    yaw_err: float
    gz_dps: float
    l_pwm: float
    r_pwm: float
    corr_out: float
    steer_i: float
    driven_m: float
    remaining_m: float
    phase: str  # "turn" / "drive" / "brake" / "settle"
    segment: int
    dist_l: float
    dist_r: float

class RobotSimulator:
    """
    Full physics simulation of the square-test robot.
    
    The motor model converts PWM → wheel speed, the physics model integrates
    wheel speeds into position/heading, and the PID controllers mirror firmware.
    """

    def __init__(self, motor: MotorModel,
                 turn_pid_params: TurnPIDParams,
                 drive_pid_params: DrivePIDParams):
        self.motor = motor
        self.turn_params = turn_pid_params
        self.drive_params = drive_pid_params
        self.dt = 0.010  # 10ms simulation step (matches firmware ~100Hz)

    def simulate_square(self, side_len_m: float = 0.5, start_yaw: float = 0.0,
                        initial_yaw_offset: float = 0.0) -> List[SimRecord]:
        """
        Simulate a full 4-side square test.
        Returns list of SimRecord for every time step.
        """
        records = []
        state = SimState(yaw=start_yaw + initial_yaw_offset)

        # Square test: drive side 0, then (turn + drive) for sides 1-3
        target_yaw = start_yaw
        segment_idx = 0

        for side in range(4):
            if side > 0:
                # TURN phase: pivot -90° (right turn)
                target_yaw = _wrap_deg(target_yaw - 90.0)
                records += self._sim_turn(state, target_yaw, segment_idx)
                segment_idx += 1
                # Brief settle
                records += self._sim_settle(state, target_yaw, segment_idx, 0.15)

            # DRIVE phase
            records += self._sim_drive(state, target_yaw, side_len_m, segment_idx)
            segment_idx += 1
            # Brake + settle
            records += self._sim_brake(state, target_yaw, segment_idx, 0.40)
            records += self._sim_settle(state, target_yaw, segment_idx, 0.30)

        return records

    def simulate_waypoints(self, waypoints: List[Tuple[float, float]],
                           start_yaw: float = 0.0) -> List[SimRecord]:
        """
        Simulate navigating a list of (x, y) waypoints.
        Returns list of SimRecord.
        """
        records = []
        state = SimState(yaw=start_yaw)
        segment_idx = 0

        for i in range(1, len(waypoints)):
            dx = waypoints[i][0] - waypoints[i - 1][0]
            dy = waypoints[i][1] - waypoints[i - 1][1]
            dist = math.sqrt(dx * dx + dy * dy)
            heading = math.degrees(math.atan2(dy, dx))
            target_yaw = _wrap_deg(heading)

            # Turn to face waypoint
            records += self._sim_turn(state, target_yaw, segment_idx)
            segment_idx += 1
            records += self._sim_settle(state, target_yaw, segment_idx, 0.15)

            # Drive to waypoint
            records += self._sim_drive(state, target_yaw, dist, segment_idx)
            segment_idx += 1
            records += self._sim_brake(state, target_yaw, segment_idx, 0.40)
            records += self._sim_settle(state, target_yaw, segment_idx, 0.30)

        return records

    def _sim_turn(self, state: SimState, target_yaw: float, seg_idx: int,
                  max_time: float = 5.0) -> List[SimRecord]:
        records = []
        pid = TurnPID(self.turn_params)
        initial_err = _wrap_deg(target_yaw - state.yaw)
        pid.reset(initial_err)
        t0 = state.t

        while (state.t - t0) < max_time:
            yaw_err = _wrap_deg(target_yaw - state.yaw)
            pwm_cmd, done = pid.update(yaw_err, state.gz_dps, self.dt, state.t)

            if done:
                records.append(self._record(state, target_yaw, yaw_err, 0, 0, 0,
                                            pid.integral, 0, 0, "turn", seg_idx))
                break

            if pwm_cmd is None:
                # In tolerance, motors stopped — decay angular velocity
                state.gz_dps *= math.exp(-self.dt / self.motor.pivot_decay_tau)
            else:
                # Apply pivot: convert PWM to angular rate
                pwm_mag = abs(pwm_cmd)
                sign = 1 if pwm_cmd > 0 else -1
                # Model: dps = pivot_gain * (pwm - pivot_deadband) with friction
                effective_pwm = pwm_mag - self.motor.pivot_deadband
                if effective_pwm < 0:
                    effective_pwm = 0
                target_rate = sign * self.motor.pivot_gain * effective_pwm
                # First-order response toward target rate
                rate_tau = 0.08  # motor response time constant
                alpha_r = self.dt / (rate_tau + self.dt)
                state.gz_dps += alpha_r * (target_rate - state.gz_dps)

            # Integrate heading
            state.yaw = _wrap_deg(state.yaw + state.gz_dps * self.dt)
            state.t += self.dt

            l_pwm = abs(pwm_cmd) if pwm_cmd else 0
            r_pwm = abs(pwm_cmd) if pwm_cmd else 0
            records.append(self._record(state, target_yaw, yaw_err, l_pwm, r_pwm, 0,
                                        pid.integral, 0, 0, "turn", seg_idx))

        return records

    def _sim_drive(self, state: SimState, target_yaw: float, dist_m: float,
                   seg_idx: int) -> List[SimRecord]:
        records = []
        pid = DrivePID(self.drive_params)
        pid.reset()
        driven = 0.0
        t0 = state.t
        enc_start_l = state.dist_l
        enc_start_r = state.dist_r

        while driven < dist_m - self.drive_params.arrive_tolerance_m:
            if (state.t - t0) > 20.0:
                break  # safety timeout

            yaw_err = _wrap_deg(target_yaw - state.yaw)
            left_pwm, right_pwm, steer_corr = pid.update(yaw_err, driven, dist_m, self.dt)

            # Convert PWM to wheel speeds
            vl = self._pwm_to_speed_l(left_pwm)
            vr = self._pwm_to_speed_r(right_pwm)

            # Integrate position
            v_avg = (vl + vr) / 2.0
            omega = (vr - vl) / TRACK_WIDTH_M  # rad/s
            heading_rad = math.radians(state.yaw)
            state.x += v_avg * math.cos(heading_rad) * self.dt
            state.y += v_avg * math.sin(heading_rad) * self.dt
            state.yaw = _wrap_deg(state.yaw + math.degrees(omega) * self.dt)
            state.gz_dps = math.degrees(omega)

            state.dist_l += vl * self.dt
            state.dist_r += vr * self.dt
            driven = ((state.dist_l - enc_start_l) + (state.dist_r - enc_start_r)) / 2.0

            state.t += self.dt

            remaining = dist_m - driven
            records.append(self._record(state, target_yaw, yaw_err, left_pwm, right_pwm,
                                        steer_corr, pid.integral, driven, remaining,
                                        "drive", seg_idx))

        return records

    def _sim_brake(self, state: SimState, target_yaw: float, seg_idx: int,
                   duration: float = 0.40) -> List[SimRecord]:
        """Simulate brakeToStop (motors reversed briefly)."""
        records = []
        t0 = state.t
        while (state.t - t0) < duration:
            # Decelerate: assume linear speed decay
            state.gz_dps *= math.exp(-self.dt / 0.05)
            state.t += self.dt
            yaw_err = _wrap_deg(target_yaw - state.yaw)
            state.yaw = _wrap_deg(state.yaw + state.gz_dps * self.dt)
            records.append(self._record(state, target_yaw, yaw_err, 0, 0, 0, 0, 0, 0,
                                        "brake", seg_idx))
        return records

    def _sim_settle(self, state: SimState, target_yaw: float, seg_idx: int,
                    duration: float = 0.30) -> List[SimRecord]:
        """Simulate settle phase (motors off, waiting)."""
        records = []
        t0 = state.t
        while (state.t - t0) < duration:
            state.gz_dps *= math.exp(-self.dt / 0.05)
            state.yaw = _wrap_deg(state.yaw + state.gz_dps * self.dt)
            state.t += self.dt
            yaw_err = _wrap_deg(target_yaw - state.yaw)
            records.append(self._record(state, target_yaw, yaw_err, 0, 0, 0, 0, 0, 0,
                                        "settle", seg_idx))
        return records

    def _pwm_to_speed_l(self, pwm: float) -> float:
        if pwm <= self.motor.deadband_l:
            return 0.0
        return self.motor.gain_l * (pwm - self.motor.deadband_l)

    def _pwm_to_speed_r(self, pwm: float) -> float:
        if pwm <= self.motor.deadband_r:
            return 0.0
        return self.motor.gain_r * (pwm - self.motor.deadband_r)

    def _record(self, state, target_yaw, yaw_err, l_pwm, r_pwm, corr,
                steer_i, driven, remaining, phase, seg_idx) -> SimRecord:
        return SimRecord(
            t=state.t, x=state.x, y=state.y,
            yaw=state.yaw, target_yaw=target_yaw,
            yaw_err=_wrap_deg(target_yaw - state.yaw),
            gz_dps=state.gz_dps,
            l_pwm=l_pwm, r_pwm=r_pwm,
            corr_out=corr, steer_i=steer_i,
            driven_m=driven, remaining_m=remaining,
            phase=phase, segment=seg_idx,
            dist_l=state.dist_l, dist_r=state.dist_r,
        )


# ---------------------------------------------------------------------------
# Replay: re-simulate a real log with different PID params
# ---------------------------------------------------------------------------

def replay_with_params(samples: List[LogSample], segments: List[Segment],
                       motor: MotorModel,
                       turn_params: TurnPIDParams,
                       drive_params: DrivePIDParams) -> List[SimRecord]:
    """
    Re-run a recorded square test with different PID parameters.
    Uses real motor model but simulated PID decisions.
    """
    # Extract the square test structure from segments
    side_len = 0.5  # default
    for seg in segments:
        if seg.kind == "drive" and seg.dist_driven > 0.1:
            side_len = seg.dist_driven
            break

    start_yaw = samples[0].current_yaw if samples else 0.0
    sim = RobotSimulator(motor, turn_params, drive_params)
    return sim.simulate_square(side_len, start_yaw)


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------

@dataclass
class RunMetrics:
    label: str
    turn_residuals_deg: List[float] = field(default_factory=list)
    drive_yaw_rms_deg: List[float] = field(default_factory=list)
    drive_corr_rms: List[float] = field(default_factory=list)
    max_yaw_err_deg: float = 0.0
    final_position_err_m: float = 0.0
    total_time_s: float = 0.0

def compute_metrics(records: List[SimRecord], label: str = "") -> RunMetrics:
    m = RunMetrics(label=label)
    if not records:
        return m

    # Group by segment
    segs = {}
    for r in records:
        segs.setdefault(r.segment, []).append(r)

    for seg_id, recs in sorted(segs.items()):
        phase = recs[-1].phase
        if phase in ("turn", "settle"):
            # Turn residual: yaw error at end
            last_turn = [r for r in recs if r.phase == "turn"]
            if last_turn:
                m.turn_residuals_deg.append(last_turn[-1].yaw_err)
        elif phase == "drive":
            errs = [abs(r.yaw_err) for r in recs if r.phase == "drive"]
            if errs:
                m.drive_yaw_rms_deg.append(float(np.sqrt(np.mean(np.array(errs) ** 2))))
            corrs = [abs(r.corr_out) for r in recs if r.phase == "drive"]
            if corrs:
                m.drive_corr_rms.append(float(np.sqrt(np.mean(np.array(corrs) ** 2))))

    all_err = [abs(r.yaw_err) for r in records]
    m.max_yaw_err_deg = max(all_err) if all_err else 0

    last = records[-1]
    m.final_position_err_m = math.sqrt(last.x ** 2 + last.y ** 2)
    m.total_time_s = last.t - records[0].t

    return m


def print_metrics(m: RunMetrics):
    print(f"\n{'='*60}")
    print(f"  {m.label}")
    print(f"{'='*60}")
    if m.turn_residuals_deg:
        print(f"  Turn residuals (deg):  {['%.2f' % r for r in m.turn_residuals_deg]}")
        print(f"  Mean |turn residual|:  {np.mean(np.abs(m.turn_residuals_deg)):.2f}°")
    if m.drive_yaw_rms_deg:
        print(f"  Drive yaw RMS (deg):   {['%.2f' % r for r in m.drive_yaw_rms_deg]}")
        print(f"  Mean drive yaw RMS:    {np.mean(m.drive_yaw_rms_deg):.2f}°")
    if m.drive_corr_rms:
        print(f"  Drive corrOut RMS:     {['%.1f' % r for r in m.drive_corr_rms]}")
    print(f"  Max |yaw error|:       {m.max_yaw_err_deg:.2f}°")
    print(f"  Final position error:  {m.final_position_err_m * 100:.1f} cm")
    print(f"  Total time:            {m.total_time_s:.2f} s")


# ---------------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------------

def plot_comparison(real_samples: List[LogSample],
                    sims: List[Tuple[str, List[SimRecord]]],
                    title: str = ""):
    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyArrowPatch

    n_plots = 5
    fig, axes = plt.subplots(1 + n_plots, 1, figsize=(16, 4 * (1 + n_plots)))
    fig.suptitle(title or "Robot Simulation Comparison", fontsize=14, fontweight="bold")

    colors = plt.cm.tab10.colors

    # --- Plot 0: 2D Trajectory ---
    ax = axes[0]
    ax.set_title("2D Trajectory")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    # Real trajectory from encoder dead-reckoning
    if real_samples:
        rx, ry = [0.0], [0.0]
        for i in range(1, len(real_samples)):
            s0 = real_samples[i - 1]
            s1 = real_samples[i]
            dt = (s1.t_ms - s0.t_ms) / 1000
            if dt <= 0 or dt > 1.0:
                rx.append(rx[-1])
                ry.append(ry[-1])
                continue
            dl = s1.dist_l - s0.dist_l
            dr = s1.dist_r - s0.dist_r
            d_avg = (dl + dr) / 2
            heading_rad = math.radians(s0.current_yaw)
            rx.append(rx[-1] + d_avg * math.cos(heading_rad))
            ry.append(ry[-1] + d_avg * math.sin(heading_rad))
        ax.plot(rx, ry, "k-", linewidth=2, label="Real", alpha=0.7)
        ax.plot(rx[0], ry[0], "go", markersize=8, label="Start")
        ax.plot(rx[-1], ry[-1], "rs", markersize=8, label="End (real)")

    for idx, (label, recs) in enumerate(sims):
        sx = [r.x for r in recs]
        sy = [r.y for r in recs]
        color = colors[idx % len(colors)]
        ax.plot(sx, sy, "-", color=color, linewidth=1.5, label=label, alpha=0.8)
        if recs:
            ax.plot(recs[-1].x, recs[-1].y, "x", color=color, markersize=8)
    ax.legend(fontsize=8)

    # --- Plot 1: Yaw over time ---
    ax = axes[1]
    ax.set_title("Heading (Yaw)")
    ax.set_ylabel("Yaw (deg)")
    ax.grid(True, alpha=0.3)
    if real_samples:
        rt = [s.t_ms / 1000 for s in real_samples]
        ry_yaw = [s.current_yaw for s in real_samples]
        rt_tgt = [s.target_yaw for s in real_samples]
        ax.plot(rt, ry_yaw, "k-", linewidth=1.5, label="Real yaw", alpha=0.7)
        ax.plot(rt, rt_tgt, "k--", linewidth=0.8, label="Real target", alpha=0.5)
    for idx, (label, recs) in enumerate(sims):
        st = [r.t for r in recs]
        sy_yaw = [r.yaw for r in recs]
        st_tgt = [r.target_yaw for r in recs]
        color = colors[idx % len(colors)]
        ax.plot(st, sy_yaw, "-", color=color, linewidth=1, label=f"{label} yaw")
        ax.plot(st, st_tgt, "--", color=color, linewidth=0.6, alpha=0.5)
    ax.legend(fontsize=7, ncol=3)

    # --- Plot 2: Yaw error ---
    ax = axes[2]
    ax.set_title("Yaw Error")
    ax.set_ylabel("Error (deg)")
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="gray", linewidth=0.5)
    if real_samples:
        rt = [s.t_ms / 1000 for s in real_samples]
        re = [s.yaw_err_deg for s in real_samples]
        ax.plot(rt, re, "k-", linewidth=1.5, label="Real", alpha=0.7)
    for idx, (label, recs) in enumerate(sims):
        st = [r.t for r in recs]
        se = [r.yaw_err for r in recs]
        color = colors[idx % len(colors)]
        ax.plot(st, se, "-", color=color, linewidth=1, label=label)
    ax.legend(fontsize=7, ncol=3)

    # --- Plot 3: PWM commands ---
    ax = axes[3]
    ax.set_title("Motor PWM")
    ax.set_ylabel("PWM")
    ax.grid(True, alpha=0.3)
    if real_samples:
        rt = [s.t_ms / 1000 for s in real_samples]
        ax.plot(rt, [s.l_pwm for s in real_samples], "b-", linewidth=0.8, alpha=0.5, label="Real L")
        ax.plot(rt, [s.r_pwm for s in real_samples], "r-", linewidth=0.8, alpha=0.5, label="Real R")
    for idx, (label, recs) in enumerate(sims):
        st = [r.t for r in recs]
        color = colors[idx % len(colors)]
        ax.plot(st, [r.l_pwm for r in recs], "-", color=color, linewidth=0.6, alpha=0.7,
                label=f"{label} L")
    ax.legend(fontsize=7, ncol=4)

    # --- Plot 4: Steering correction ---
    ax = axes[4]
    ax.set_title("Steering Correction (corrOut)")
    ax.set_ylabel("corrOut")
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="gray", linewidth=0.5)
    if real_samples:
        rt = [s.t_ms / 1000 for s in real_samples]
        ax.plot(rt, [s.corr_out for s in real_samples], "k-", linewidth=1.5, label="Real", alpha=0.7)
    for idx, (label, recs) in enumerate(sims):
        st = [r.t for r in recs]
        color = colors[idx % len(colors)]
        ax.plot(st, [r.corr_out for r in recs], "-", color=color, linewidth=1, label=label)
    ax.legend(fontsize=7, ncol=3)

    # --- Plot 5: Steer integral ---
    ax = axes[5]
    ax.set_title("Steering Integral (steerI)")
    ax.set_ylabel("steerI")
    ax.set_xlabel("Time (s)")
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="gray", linewidth=0.5)
    if real_samples:
        rt = [s.t_ms / 1000 for s in real_samples]
        ax.plot(rt, [s.steer_i for s in real_samples], "k-", linewidth=1.5, label="Real", alpha=0.7)
    for idx, (label, recs) in enumerate(sims):
        st = [r.t for r in recs]
        color = colors[idx % len(colors)]
        ax.plot(st, [r.steer_i for r in recs], "-", color=color, linewidth=1, label=label)
    ax.legend(fontsize=7, ncol=3)

    fig.tight_layout()
    return fig


def plot_sweep(sweep_results: List[Tuple[str, RunMetrics]], title: str = "PID Sweep"):
    """Bar chart comparing metrics across parameter sweeps."""
    import matplotlib.pyplot as plt

    labels = [r[0] for r in sweep_results]
    mean_turn_res = [np.mean(np.abs(r[1].turn_residuals_deg)) if r[1].turn_residuals_deg else 0
                     for r in sweep_results]
    mean_drive_rms = [np.mean(r[1].drive_yaw_rms_deg) if r[1].drive_yaw_rms_deg else 0
                      for r in sweep_results]
    pos_err = [r[1].final_position_err_m * 100 for r in sweep_results]

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle(title, fontsize=14, fontweight="bold")

    x = np.arange(len(labels))
    w = 0.6

    axes[0].bar(x, mean_turn_res, w, color="steelblue")
    axes[0].set_title("Mean |Turn Residual| (deg)")
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(labels, rotation=30, ha="right", fontsize=8)

    axes[1].bar(x, mean_drive_rms, w, color="coral")
    axes[1].set_title("Mean Drive Yaw RMS (deg)")
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(labels, rotation=30, ha="right", fontsize=8)

    axes[2].bar(x, pos_err, w, color="seagreen")
    axes[2].set_title("Final Position Error (cm)")
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(labels, rotation=30, ha="right", fontsize=8)

    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def find_latest_csv():
    logs_dir = Path(__file__).resolve().parent.parent / "logs"
    csvs = sorted(glob.glob(str(logs_dir / "*runlog*.csv")))
    return csvs[-1] if csvs else None

def main():
    parser = argparse.ArgumentParser(description="Robot Tour Physics Simulation")
    parser.add_argument("csv", nargs="?", help="CSV log file to analyze (default: latest)")
    parser.add_argument("--sweep", action="store_true", help="Sweep PID parameters")
    parser.add_argument("--compare", action="store_true", help="Compare old vs new tolerances")
    parser.add_argument("--waypoints", action="store_true", help="Simulate custom waypoints")
    parser.add_argument("--no-plot", action="store_true", help="Print metrics only, no plots")
    parser.add_argument("--save", help="Save plot to file instead of displaying")
    args = parser.parse_args()

    csv_path = args.csv or find_latest_csv()
    if not csv_path or not os.path.exists(csv_path):
        print(f"No CSV file found. Provide a path or place logs in logs/")
        sys.exit(1)

    print(f"Loading: {csv_path}")
    header, samples = parse_csv(csv_path)
    print(f"  Run {header.run_id}: {len(samples)} samples, mode={header.mode}, "
          f"dist={header.dist}m, ppm={header.ppm}")

    segments = segment_run(samples)
    print(f"  Segments: {len(segments)} "
          f"({sum(1 for s in segments if s.kind == 'drive')} drive, "
          f"{sum(1 for s in segments if s.kind == 'turn')} turn)")

    for seg in segments:
        yaw_change = _wrap_deg(seg.end_yaw - seg.start_yaw)
        print(f"    {seg.kind:5s}  t={seg.t_start_ms:.0f}-{seg.t_end_ms:.0f}ms  "
              f"dist={seg.dist_driven:.3f}m  dyaw={yaw_change:.1f}°")

    # Extract motor model
    motor = extract_motor_model(segments, samples)
    print(f"\n  Motor model:")
    print(f"    gain_L={motor.gain_l:.5f} m/s/PWM  gain_R={motor.gain_r:.5f} m/s/PWM")
    print(f"    deadband_L={motor.deadband_l:.0f}  deadband_R={motor.deadband_r:.0f}")
    print(f"    pivot_gain={motor.pivot_gain:.4f} dps/PWM")

    # Fallback: if extraction gave poor results
    if motor.gain_l <= 0:
        motor.gain_l = 0.010  # ~0.35 m/s at PWM 130 with deadband 95
    if motor.gain_r <= 0:
        motor.gain_r = 0.010
    if motor.pivot_gain <= 0:
        # Estimate from known turn dynamics: 90° in ~1.9s at avg ~118 PWM
        # avg rate ≈ 47 dps, effective PWM above pivot_deadband(95) ≈ 23
        motor.pivot_gain = 2.1  # dps per effective PWM unit

    # Determine test type
    side_len = 0.5
    for seg in segments:
        if seg.kind == "drive" and seg.dist_driven > 0.1:
            side_len = seg.dist_driven
            break

    start_yaw = samples[0].current_yaw if samples else 0.0

    if args.sweep:
        _run_sweep(motor, side_len, start_yaw, samples, args)
        return

    # --- Default: compare OLD firmware vs NEW firmware (current fixes) ---
    sims = []

    # OLD: tolerance=5°, gyro_still=8, no derivative fix
    old_turn = TurnPIDParams(tolerance_deg=5.0, gyro_still_dps=8.0)
    old_drive = SqDrivePIDParams(init_prev_err=False)
    sim_old = RobotSimulator(motor, old_turn, old_drive)
    recs_old = sim_old.simulate_square(side_len, start_yaw)
    sims.append(("OLD (tol=5° gyro<8)", recs_old))

    # NEW: tolerance=2°, gyro_still=3, derivative fix
    new_turn = TurnPIDParams(tolerance_deg=2.0, gyro_still_dps=3.0)
    new_drive = SqDrivePIDParams(init_prev_err=True)
    sim_new = RobotSimulator(motor, new_turn, new_drive)
    recs_new = sim_new.simulate_square(side_len, start_yaw)
    sims.append(("NEW (tol=2° gyro<3 D-fix)", recs_new))

    # Print metrics
    for label, recs in sims:
        m = compute_metrics(recs, label)
        print_metrics(m)

    # Also show real data metrics
    print(f"\n{'='*60}")
    print(f"  REAL DATA (from CSV)")
    print(f"{'='*60}")
    for seg in segments:
        if seg.kind == "turn":
            residual = _wrap_deg(seg.end_yaw - seg.target_yaw)
            print(f"  Turn residual: {residual:+.2f}°  (target={seg.target_yaw:.1f}° actual={seg.end_yaw:.1f}°)")
    drive_errs = []
    for seg in segments:
        if seg.kind == "drive":
            errs = [abs(s.yaw_err_deg) for s in seg.samples]
            rms = float(np.sqrt(np.mean(np.array(errs) ** 2))) if errs else 0
            drive_errs.append(rms)
            print(f"  Drive yaw RMS: {rms:.2f}°  dist={seg.dist_driven:.3f}m")
    if drive_errs:
        print(f"  Mean drive yaw RMS: {np.mean(drive_errs):.2f}°")

    if not args.no_plot:
        import matplotlib.pyplot as plt
        fig = plot_comparison(samples, sims, f"Run {header.run_id}: Old vs New Firmware Fixes")
        if args.save:
            fig.savefig(args.save, dpi=150, bbox_inches="tight")
            print(f"\nSaved plot to {args.save}")
        else:
            plt.show()


def _run_sweep(motor, side_len, start_yaw, samples, args):
    """Sweep through PID parameters and compare."""
    import matplotlib.pyplot as plt

    sweep_results = []

    # Sweep turn tolerance
    for tol in [1.0, 2.0, 3.0, 5.0, 8.0]:
        for gyro in [2.0, 3.0, 5.0, 8.0]:
            label = f"tol={tol}° gyro<{gyro}"
            turn_p = TurnPIDParams(tolerance_deg=tol, gyro_still_dps=gyro)
            drive_p = SqDrivePIDParams(init_prev_err=True)
            sim = RobotSimulator(motor, turn_p, drive_p)
            recs = sim.simulate_square(side_len, start_yaw)
            m = compute_metrics(recs, label)
            sweep_results.append((label, m))
            print(f"  {label:30s}  turn_res={np.mean(np.abs(m.turn_residuals_deg)):.2f}°  "
                  f"drive_rms={np.mean(m.drive_yaw_rms_deg):.2f}°  "
                  f"pos_err={m.final_position_err_m * 100:.1f}cm  "
                  f"time={m.total_time_s:.1f}s")

    if not args.no_plot:
        fig = plot_sweep(sweep_results, "PID Parameter Sweep: Turn Tolerance × Gyro Threshold")
        if args.save:
            fig.savefig(args.save, dpi=150, bbox_inches="tight")
        else:
            plt.show()


if __name__ == "__main__":
    main()
