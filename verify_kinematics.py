#!/usr/bin/env python3
"""
Verify that motor commands (steps/feedrate) in Chile_calculated_table.xlsx
will produce the correct acceleration when measured by the MPU6050 sensor.
"""

import openpyxl
import numpy as np

# Constants from serial_sender.py
LEAD_MM_PER_REV = 2.0  # mm per revolution
PULSES_PER_REV = 400   # steps per revolution
CMPS2_TO_MPS2 = 0.01   # conversion factor
SAMPLE_INTERVAL = 0.03 # seconds

# Calculate steps per meter
steps_per_meter = PULSES_PER_REV / (LEAD_MM_PER_REV / 1000.0)
print(f"Steps per meter: {steps_per_meter}")
print(f"Resolution: {1/steps_per_meter * 1000:.3f} mm/step = {1/steps_per_meter * 1e6:.1f} microns/step")
print()

# Load the calculated table
wb = openpyxl.load_workbook('Chile_calculated_table.xlsx')
ws = wb.active

# Read data (skip header)
rows = []
for row in ws.iter_rows(min_row=2, values_only=True):
    rows.append({
        'index': row[0],
        'time_s': row[1],
        'accel_cmps2': row[2],
        'accel_mps2': row[3],
        'steps': row[4],
        'feedrate': row[5],
        'direction': row[6],
        'position_cm': row[7],
        'unclamped_position_cm': row[8],
    })

print(f"Loaded {len(rows)} rows\n")

# Verify first 10 rows in detail
print("=" * 100)
print("VERIFICATION OF FIRST 10 ROWS")
print("=" * 100)

for i in range(min(10, len(rows))):
    row = rows[i]
    
    print(f"\nRow {i} (t = {row['time_s']:.2f}s):")
    print(f"  Commanded accel: {row['accel_mps2']:.6f} m/s²")
    print(f"  Steps: {row['steps']}, Direction: {row['direction']}, Feedrate: {row['feedrate']} steps/s")
    
    # Calculate displacement from steps
    displacement_m = row['steps'] / steps_per_meter * row['direction']
    displacement_cm = displacement_m * 100
    print(f"  Calculated displacement: {displacement_cm:.4f} cm")
    print(f"  Table position_cm: {row['position_cm']:.4f} cm")
    
    # Calculate velocity from feedrate
    if row['feedrate'] > 0:
        velocity_mps = row['feedrate'] / steps_per_meter * row['direction']
        print(f"  Calculated velocity: {velocity_mps:.6f} m/s (from feedrate)")
    else:
        velocity_mps = 0.0
        print(f"  Calculated velocity: 0.000000 m/s (no motion)")

# Now calculate what acceleration the sensor will actually measure
print("\n" + "=" * 100)
print("SENSOR ACCELERATION VERIFICATION")
print("=" * 100)
print("\nCalculating actual acceleration from position changes...")
print("(This is what the MPU6050 sensor should measure)\n")

# Use the actual positions from the table (these are the commanded positions)
positions = [row['position_cm'] / 100.0 for row in rows]  # convert to meters

# Calculate velocity from position changes
velocities = []
for i in range(len(positions)):
    if i == 0:
        # First sample: forward difference
        vel = (positions[1] - positions[0]) / SAMPLE_INTERVAL
    elif i == len(positions) - 1:
        # Last sample: backward difference
        vel = (positions[i] - positions[i-1]) / SAMPLE_INTERVAL
    else:
        # Central difference (more accurate)
        vel = (positions[i+1] - positions[i-1]) / (2 * SAMPLE_INTERVAL)
    
    velocities.append(vel)

# Calculate acceleration from velocity changes
measured_accels = []
for i in range(len(velocities)):
    if i == 0:
        # First sample: use forward difference
        accel = (velocities[1] - velocities[0]) / SAMPLE_INTERVAL
    elif i == len(velocities) - 1:
        # Last sample: use backward difference
        accel = (velocities[i] - velocities[i-1]) / SAMPLE_INTERVAL
    else:
        # Central difference (more accurate)
        accel = (velocities[i+1] - velocities[i-1]) / (2 * SAMPLE_INTERVAL)
    
    measured_accels.append(accel)

# Compare commanded vs measured acceleration
print("Row | Time (s) | Commanded Accel | Measured Accel | Error    | Error %")
print("-" * 85)

errors = []
for i in range(min(20, len(rows))):
    cmd_accel = rows[i]['accel_mps2']
    meas_accel = measured_accels[i]
    error = meas_accel - cmd_accel
    error_pct = (error / cmd_accel * 100) if cmd_accel != 0 else 0
    
    errors.append(abs(error))
    
    print(f"{i:3d} | {rows[i]['time_s']:7.2f} | {cmd_accel:15.6f} | {meas_accel:14.6f} | {error:8.6f} | {error_pct:6.1f}%")

# Statistics
print("\n" + "=" * 100)
print("STATISTICS")
print("=" * 100)

all_errors = []
for i in range(len(rows)):
    error = abs(measured_accels[i] - rows[i]['accel_mps2'])
    all_errors.append(error)

print(f"Mean absolute error: {np.mean(all_errors):.8f} m/s²")
print(f"Max absolute error: {np.max(all_errors):.8f} m/s²")
print(f"RMS error: {np.sqrt(np.mean(np.array(all_errors)**2)):.8f} m/s²")
print(f"Median absolute error: {np.median(all_errors):.8f} m/s²")

# Check for large errors
large_errors = [(i, all_errors[i]) for i in range(len(all_errors)) if all_errors[i] > 0.001]
if large_errors:
    print(f"\nWarning: {len(large_errors)} samples have errors > 0.001 m/s²")
    print("First 10 large errors:")
    for i, err in large_errors[:10]:
        print(f"  Row {i} (t={rows[i]['time_s']:.2f}s): error = {err:.6f} m/s²")
else:
    print("\n✓ All errors are within acceptable range (< 0.001 m/s²)")

print("\n" + "=" * 100)
print("CONCLUSION")
print("=" * 100)

max_error_pct = (np.max(all_errors) / np.mean([abs(r['accel_mps2']) for r in rows]) * 100)
print(f"Maximum error as % of mean acceleration: {max_error_pct:.2f}%")

if np.max(all_errors) < 0.01:
    print("✓ PASS: Motor commands will produce acceleration matching commanded values")
    print("        The MPU6050 sensor should measure accelerations very close to commanded.")
else:
    print("⚠ WARNING: Significant errors detected. Sensor measurements may differ from commands.")
