#!/usr/bin/env python3
"""
Analyze encoder rates and motor asymmetry in straight test
"""
import csv
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

# Load the latest log file
log_file = Path("logs/20260328_151726_runlog_last.csv")
print(f"Analyzing: {log_file}")

# Parse CSV
data = {}
with open(log_file) as f:
    lines = f.readlines()
    data_lines = [l for l in lines if not l.startswith('#')]
    
    reader = csv.DictReader(data_lines)
    for row in reader:
        for key, val in row.items():
            if key not in data:
                data[key] = []
            try:
                data[key].append(float(val))
            except:
                data[key].append(val)

# Convert to numpy arrays
t = np.array(data['t_ms']) / 1000.0  # Convert to seconds
lPwm = np.array(data['lPwm'])
rPwm = np.array(data['rPwm'])
distL = np.array(data['distL'])
distR = np.array(data['distR'])
currentYaw = np.array(data['currentYaw'])
yawErrDeg = np.array(data['yawErrDeg'])
steerI = np.array(data['steerI'])
corrOut = np.array(data['corrOut'])

# Calculate wheel speeds from encoder distance deltas
dt = np.diff(t)
dt = np.append(dt, dt[-1])  # Extend last element

# Avoid division by zero
dt = np.where(dt < 0.001, 0.001, dt)

# Calculate rates (meters per second)
encRateL = np.gradient(distL, t)
encRateR = np.gradient(distR, t)

# Create figure
fig, axes = plt.subplots(3, 2, figsize=(16, 12))
fig.suptitle(f'Motor Asymmetry & Encoder Analysis: {log_file.name}', fontsize=16, fontweight='bold')

# 1. Motor PWM commands
ax = axes[0, 0]
ax.plot(t, lPwm, 'b-', label='Left PWM', linewidth=2)
ax.plot(t, rPwm, 'r-', label='Right PWM', linewidth=2)
ax.set_ylabel('PWM (0-255)')
ax.set_title('Motor PWM Commands (Steering Differential)')
ax.legend()
ax.grid(True, alpha=0.3)

# 2. PWM difference showing steering
ax = axes[0, 1]
pwm_diff = lPwm - rPwm
ax.plot(t, pwm_diff, 'purple', linewidth=2)
ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
ax.set_ylabel('PWM Diff (L - R)')
ax.set_title('Steering Commands (Negative = Turn Right)')
ax.grid(True, alpha=0.3)

# 3. Encoder rates
ax = axes[1, 0]
ax.plot(t, encRateL, 'b-', label='Left Encoder Rate', linewidth=2)
ax.plot(t, encRateR, 'r-', label='Right Encoder Rate', linewidth=2)
ax.set_ylabel('Rate (m/s)')
ax.set_title('Calculated Wheel Speeds from Encoder')
ax.legend()
ax.grid(True, alpha=0.3)

# 4. Encoder rate difference (showing actual asymmetry)
ax = axes[1, 1]
rate_diff = encRateL - encRateR
ax.plot(t, rate_diff * 100, 'darkgreen', linewidth=2)  # Convert to cm/s
ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
ax.set_ylabel('Rate Difference (cm/s)')
ax.set_title('Encoder Asymmetry (How fast L vs R)')
ax.grid(True, alpha=0.3)

# 5. Yaw error and integral
ax = axes[2, 0]
ax2 = ax.twinx()
ax.plot(t, yawErrDeg, 'b-', label='Yaw Error', linewidth=2)
ax2.plot(t, steerI, 'r-', label='Integral Term', linewidth=2)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Yaw Error (deg)', color='b')
ax2.set_ylabel('I Term', color='r')
ax.set_title('Yaw PID Components')
ax.grid(True, alpha=0.3)

# 6. Steering correction effect
ax = axes[2, 1]
ax.plot(t, corrOut, 'g-', label='Correction Output', linewidth=2)
ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Correction (PWM units)')
ax.set_title('PID Steering Correction (Applied to Motors)')
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('motor_encoder_analysis.png', dpi=150, bbox_inches='tight')
print(f"\n✓ Saved plot: motor_encoder_analysis.png")

# Print analysis
print(f"\n=== MOTOR & ENCODER ANALYSIS ===")
print(f"Duration: {t[-1]:.2f}s")
print(f"\nMotor PWM:")
print(f"  Left avg: {np.mean(lPwm):.1f}")
print(f"  Right avg: {np.mean(rPwm):.1f}")
print(f"  Difference: {np.mean(lPwm - rPwm):.1f} PWM")

print(f"\nEncoder Rates (calculated from distance):")
print(f"  Left avg: {np.mean(encRateL):.4f} m/s")
print(f"  Right avg: {np.mean(encRateR):.4f} m/s")
print(f"  Ratio (L/R): {np.mean(encRateL) / (np.mean(encRateR) + 0.0001):.4f}")
print(f"  Asymmetry: {np.mean(encRateL - encRateR)*100:.2f} cm/s")

print(f"\nYaw Control:")
print(f"  Final yaw error: {yawErrDeg[-1]:.2f}°")
print(f"  Max yaw error: {np.max(np.abs(yawErrDeg)):.2f}°")
print(f"  Integral accumulation: {steerI[-1]:.4f}")

print(f"\nSteering Analysis:")
print(f"  Max correction: {np.max(np.abs(corrOut)):.2f} PWM")
print(f"  Correction variance: {np.std(corrOut):.2f} PWM")

# Find the problem
enc_asym = np.mean(encRateL - encRateR)
pwm_asym = np.mean(lPwm - rPwm)

print(f"\n⚠️ DIAGNOSTICS:")
if abs(enc_asym) > 0.05:
    print(f"  • Encoder asymmetry {abs(enc_asym)*100:.1f} cm/s suggests mechanical issue:")
    if enc_asym > 0:
        print(f"    → LEFT wheel faster: check right wheel drag/diameter")
    else:
        print(f"    → RIGHT wheel faster: check left wheel drag/diameter")
        
print(f"  • PWM difference: {pwm_asym:.1f} (steering is {'ON' if abs(pwm_asym) > 1 else 'OFF'})")
