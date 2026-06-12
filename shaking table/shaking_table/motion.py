import math

import numpy as np

from .config import (
    CMPS2_TO_MPS2,
    MAX_FEEDRATE_STEPS_S,
    SAFE_ACCEL_LIMIT_MPS2,
    STEPS_PER_METER,
    STROKE_LIMIT_M,
)


def remove_mean(values):
    if not values:
        return []
    mean = sum(values) / len(values)
    return [v - mean for v in values]


def highpass_filter(values, dt, cutoff_hz=0.05):
    if len(values) < 2 or cutoff_hz <= 0 or dt <= 0:
        return list(values)
    rc = 1.0 / (2.0 * math.pi * cutoff_hz)
    alpha = rc / (rc + dt)
    filtered = [values[0]]
    for i in range(1, len(values)):
        filtered.append(alpha * (filtered[-1] + values[i] - values[i - 1]))
    return filtered


def accel_to_displacement_fft(accel_mps2, dt):
    if len(accel_mps2) < 2 or dt <= 0:
        return [0.0] * len(accel_mps2)

    accel = np.asarray(accel_mps2, dtype=float)
    accel = accel - np.mean(accel)
    n = len(accel)
    freqs = np.fft.rfftfreq(n, dt)
    spectrum = np.fft.rfft(accel)

    omega = 2.0 * np.pi * freqs
    displacement_spectrum = np.zeros_like(spectrum, dtype=complex)
    valid = omega > 1e-9
    displacement_spectrum[valid] = -spectrum[valid] / (omega[valid] ** 2)
    displacement_spectrum[~valid] = 0.0

    displacement = np.fft.irfft(displacement_spectrum, n=n)
    displacement -= displacement[0]
    return displacement.tolist()


def prepare_accel(accel_mps2, dt, highpass_enabled=True, cutoff_hz=0.05):
    values = remove_mean(accel_mps2)
    if highpass_enabled:
        values = highpass_filter(values, dt, cutoff_hz=cutoff_hz)
    return values


def scale_displacement_to_stroke(displacement_m, stroke_limit_m=STROKE_LIMIT_M):
    if not displacement_m:
        return [], 1.0
    max_abs = max(abs(v) for v in displacement_m)
    if max_abs <= stroke_limit_m or max_abs <= 1e-12:
        return list(displacement_m), 1.0
    scale = (stroke_limit_m * 0.95) / max_abs
    return [v * scale for v in displacement_m], scale


def generate_rows_from_accel(accel_mps2, dt):
    displacement = accel_to_displacement_fft(accel_mps2, dt)
    displacement, _ = scale_displacement_to_stroke(displacement)
    return generate_rows_from_displacement(displacement, dt, accel_mps2=accel_mps2)


def generate_rows_from_displacement(displacement_m, dt, accel_mps2=None):
    rows = []
    current_position_m = 0.0
    step_residual = 0.0
    accel_mps2 = accel_mps2 or [0.0] * len(displacement_m)

    for index, target_m in enumerate(displacement_m):
        clamped_m = max(-STROKE_LIMIT_M, min(STROKE_LIMIT_M, target_m))
        delta_m = clamped_m - current_position_m
        step_float = delta_m * STEPS_PER_METER + step_residual
        signed_steps = int(round(step_float))
        step_residual = step_float - signed_steps

        if signed_steps > 0:
            direction = 1
            steps = signed_steps
        elif signed_steps < 0:
            direction = -1
            steps = -signed_steps
        else:
            direction = 0
            steps = 0

        if steps:
            feedrate = max(1, min(MAX_FEEDRATE_STEPS_S, int(round(steps / dt))))
            current_position_m += signed_steps / STEPS_PER_METER
            current_position_m = max(-STROKE_LIMIT_M, min(STROKE_LIMIT_M, current_position_m))
        else:
            feedrate = 0

        accel_value = accel_mps2[index] if index < len(accel_mps2) else 0.0
        accel_limited = max(-SAFE_ACCEL_LIMIT_MPS2, min(SAFE_ACCEL_LIMIT_MPS2, accel_value))
        rows.append({
            "index": index,
            "time_s": index * dt,
            "target_accel_mps2": accel_value,
            "accel_cmps2": accel_limited / CMPS2_TO_MPS2,
            "accel_mps2": accel_limited,
            "steps": steps,
            "feedrate": feedrate,
            "direction": direction,
            "position_cm": current_position_m * 100.0,
            "unclamped_position_cm": target_m * 100.0,
        })
    return rows


def merge_rows(rows):
    merged = []
    for row in rows:
        if row["steps"] <= 0:
            continue
        if merged:
            last = merged[-1]
            if row["direction"] == last["direction"] and row["feedrate"] == last["feedrate"] and row["index"] == last["row_end"] + 1:
                last["steps"] += row["steps"]
                last["row_end"] = row["index"]
                continue
        merged.append({
            "steps": row["steps"],
            "direction": row["direction"],
            "feedrate": row["feedrate"],
            "row_start": row["index"],
            "row_end": row["index"],
        })
    return merged
