#!/usr/bin/env python3
"""
Visualize the difference between commanded acceleration vs what sensor will measure
"""

import openpyxl
import numpy as np
import matplotlib.pyplot as plt

# Constants
SAMPLE_INTERVAL = 0.03  # seconds

# Load data
wb = openpyxl.load_workbook('Chile_calculated_table.xlsx')
ws = wb.active

rows = []
for row in ws.iter_rows(min_row=2, max_row=1002, values_only=True):  # First 1000 samples only
    rows.append({
        'time_s': row[1],
        'accel_mps2': row[3],
        'position_cm': row[7],
    })

# Extract data
times = [r['time_s'] for r in rows]
cmd_accel = [r['accel_mps2'] for r in rows]
positions = [r['position_cm'] / 100.0 for r in rows]  # convert to meters

# Calculate measured acceleration from position (second derivative)
velocities = np.gradient(positions, SAMPLE_INTERVAL)
measured_accel = np.gradient(velocities, SAMPLE_INTERVAL)

# Create figure
fig, axes = plt.subplots(3, 1, figsize=(12, 10))

# Plot 1: Position
axes[0].plot(times, np.array(positions) * 100, 'b-', linewidth=1.5, label='Commanded Position')
axes[0].set_ylabel('Position (cm)', fontsize=12)
axes[0].set_title('Position Profile (Quantized to 5-micron steps)', fontsize=14, fontweight='bold')
axes[0].grid(True, alpha=0.3)
axes[0].legend()

# Plot 2: Commanded Acceleration
axes[1].plot(times, cmd_accel, 'g-', linewidth=1.5, alpha=0.7, label='Commanded Accel (from FFT)')
axes[1].set_ylabel('Acceleration (m/s²)', fontsize=12)
axes[1].set_title('Commanded Acceleration (Smooth)', fontsize=14, fontweight='bold')
axes[1].grid(True, alpha=0.3)
axes[1].legend()

# Plot 3: Measured vs Commanded
axes[2].plot(times, cmd_accel, 'g-', linewidth=1.5, alpha=0.7, label='Commanded Accel')
axes[2].plot(times, measured_accel, 'r-', linewidth=1, alpha=0.8, label='Measured Accel (from position derivative)')
axes[2].set_xlabel('Time (s)', fontsize=12)
axes[2].set_ylabel('Acceleration (m/s²)', fontsize=12)
axes[2].set_title('Commanded vs Measured Acceleration', fontsize=14, fontweight='bold')
axes[2].grid(True, alpha=0.3)
axes[2].legend()

plt.tight_layout()
plt.savefig('acceleration_comparison.png', dpi=150, bbox_inches='tight')
print("✓ Saved plot to: acceleration_comparison.png")

# Calculate statistics
error = measured_accel - cmd_accel
print(f"\nStatistics (first 1000 samples):")
print(f"  Mean absolute error: {np.mean(np.abs(error)):.6f} m/s²")
print(f"  RMS error: {np.sqrt(np.mean(error**2)):.6f} m/s²")
print(f"  Max error: {np.max(np.abs(error)):.6f} m/s²")
print(f"  Correlation: {np.corrcoef(cmd_accel, measured_accel)[0,1]:.4f}")
