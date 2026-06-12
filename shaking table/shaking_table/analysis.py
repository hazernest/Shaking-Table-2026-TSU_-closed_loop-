import math


def resample_nearest(values, target_len):
    if target_len <= 0:
        return []
    if not values:
        return [0.0] * target_len
    if len(values) == 1:
        return [values[0]] * target_len
    if target_len == 1:
        return [values[0]]
    last = len(values) - 1
    return [values[int(round(i * last / (target_len - 1)))] for i in range(target_len)]


def rmse(expected, actual):
    n = min(len(expected), len(actual))
    if n == 0:
        return None
    return math.sqrt(sum((expected[i] - actual[i]) ** 2 for i in range(n)) / n)


def correlation(a, b):
    n = min(len(a), len(b))
    if n < 2:
        return None
    x = a[:n]
    y = b[:n]
    mx = sum(x) / n
    my = sum(y) / n
    sx = math.sqrt(sum((v - mx) ** 2 for v in x))
    sy = math.sqrt(sum((v - my) ** 2 for v in y))
    if sx <= 1e-12 or sy <= 1e-12:
        return None
    return sum((x[i] - mx) * (y[i] - my) for i in range(n)) / (sx * sy)


def finite_difference_accel_from_displacement(displacement_m, dt):
    n = len(displacement_m)
    if n < 3 or dt <= 0:
        return [0.0] * n
    accel = [0.0] * n
    inv_dt2 = 1.0 / (dt * dt)
    for i in range(1, n - 1):
        accel[i] = (displacement_m[i + 1] - 2.0 * displacement_m[i] + displacement_m[i - 1]) * inv_dt2
    accel[0] = accel[1]
    accel[-1] = accel[-2]
    return accel
