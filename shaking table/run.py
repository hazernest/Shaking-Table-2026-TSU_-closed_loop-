import csv
import importlib
import math
import queue
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import numpy as np
import serial
from serial.tools import list_ports


DEFAULT_BAUDRATE = 115200
PREFERRED_MOTOR_PORT = "COM9"
PREFERRED_ACCEL_PORT = "COM10"

LEAD_MM_PER_REV = 2.0
PULSES_PER_REV = 400
STEPS_PER_METER = PULSES_PER_REV / (LEAD_MM_PER_REV / 1000.0)

STROKE_LIMIT_M = 0.10
SAFE_ACCEL_LIMIT_MPS2 = 4.0
MAX_FEEDRATE_STEPS_S = 4000
MACHINE_MAX_SWEEP_HZ = 5.0
DEFAULT_SAMPLE_INTERVAL_S = 0.03

CM_TO_M = 0.01
CMPS2_TO_MPS2 = 0.01
GRAVITY_MS2 = 9.80665

GRAPH_BG = "#f8fafc"
GRAPH_GRID = "#e2e8f0"
GRAPH_AXIS = "#64748b"
ACCEL_COLOR = "#2563eb"
DISP_COLOR = "#059669"
ACTUAL_COLOR = "#dc2626"
ERROR_COLOR = "#7c3aed"


def to_float(value):
    if isinstance(value, (int, float)):
        return float(value)
    text = str(value).strip()
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def load_first_numeric_column(path, scale=1.0):
    suffix = Path(path).suffix.lower()
    if suffix in {".xlsx", ".xlsm"}:
        return _load_excel(path, scale)
    if suffix == ".csv":
        return _load_csv(path, scale)
    raise ValueError("Unsupported file type. Use .xlsx, .xlsm, or .csv")


def _load_excel(path, scale):
    try:
        openpyxl = importlib.import_module("openpyxl")
    except Exception as exc:
        raise RuntimeError("openpyxl is required. Install dependencies from requirements.txt") from exc

    workbook = openpyxl.load_workbook(path, read_only=True, data_only=True)
    worksheet = workbook.active
    values = []
    try:
        for row in worksheet.iter_rows(min_col=1, max_col=1, values_only=True):
            if not row:
                continue
            number = to_float(row[0])
            if number is not None:
                values.append(number * scale)
    finally:
        workbook.close()
    return values


def _load_csv(path, scale):
    values = []
    with open(path, "r", newline="", encoding="utf-8-sig") as file_obj:
        for row in csv.reader(file_obj):
            if not row:
                continue
            number = to_float(row[0])
            if number is not None:
                values.append(number * scale)
    return values


def available_ports():
    return [f"{p.device} - {p.description}" for p in list_ports.comports()]


def port_device(label):
    return label.split(" - ", 1)[0].strip()


class SerialClient:
    def __init__(self, name, baudrate=DEFAULT_BAUDRATE):
        self.name = name
        self.baudrate = baudrate
        self.serial = None
        self.rx_queue = queue.Queue()
        self.log_queue = queue.Queue()
        self._stop = threading.Event()
        self._thread = None
        self._ok_event = threading.Event()

    def connect(self, port_label):
        self.disconnect()
        device = port_device(port_label)
        self.serial = serial.Serial(device, self.baudrate, timeout=0.05, write_timeout=1.0)
        time.sleep(1.8)
        self._stop.clear()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        self.log_queue.put(f"{self.name}: connected to {device}")

    def disconnect(self):
        self._stop.set()
        if self.serial:
            try:
                self.serial.close()
            except Exception:
                pass
        self.serial = None

    def is_connected(self):
        return bool(self.serial and self.serial.is_open)

    def send_line(self, line):
        if not self.is_connected():
            raise RuntimeError(f"{self.name} is not connected")
        self.serial.write((line.rstrip() + "\n").encode("ascii"))
        self.serial.flush()

    def send_motion_wait_ok(self, steps, direction, feedrate, timeout_s=10.0):
        self._ok_event.clear()
        self.send_line(f"{int(steps)},{int(direction)},{int(feedrate)}")
        if not self._ok_event.wait(timeout_s):
            raise TimeoutError(f"{self.name}: no ok after command")

    def _read_loop(self):
        while not self._stop.is_set() and self.serial and self.serial.is_open:
            try:
                raw = self.serial.readline()
            except Exception as exc:
                self.log_queue.put(f"{self.name}: read error: {exc}")
                break
            if not raw:
                continue
            text = raw.decode("utf-8", errors="replace").strip()
            if not text:
                continue
            if text == "ok":
                self._ok_event.set()
            self.rx_queue.put(text)


def remove_mean(values):
    if not values:
        return []
    mean = sum(values) / len(values)
    return [value - mean for value in values]


def highpass_filter(values, dt, cutoff_hz=0.05):
    if len(values) < 2 or cutoff_hz <= 0 or dt <= 0:
        return list(values)
    rc = 1.0 / (2.0 * math.pi * cutoff_hz)
    alpha = rc / (rc + dt)
    filtered = [values[0]]
    for index in range(1, len(values)):
        filtered.append(alpha * (filtered[-1] + values[index] - values[index - 1]))
    return filtered


def accel_to_displacement_fft(accel_mps2, dt):
    if len(accel_mps2) < 2 or dt <= 0:
        return [0.0] * len(accel_mps2)

    accel = np.asarray(accel_mps2, dtype=float)
    accel = accel - np.mean(accel)
    sample_count = len(accel)
    freqs = np.fft.rfftfreq(sample_count, dt)
    spectrum = np.fft.rfft(accel)

    omega = 2.0 * np.pi * freqs
    displacement_spectrum = np.zeros_like(spectrum, dtype=complex)
    valid = omega > 1e-9
    displacement_spectrum[valid] = -spectrum[valid] / (omega[valid] ** 2)
    displacement_spectrum[~valid] = 0.0

    displacement = np.fft.irfft(displacement_spectrum, n=sample_count)
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
    max_abs = max(abs(value) for value in displacement_m)
    if max_abs <= stroke_limit_m or max_abs <= 1e-12:
        return list(displacement_m), 1.0
    scale = (stroke_limit_m * 0.95) / max_abs
    return [value * scale for value in displacement_m], scale


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
            same_direction = row["direction"] == last["direction"]
            same_feedrate = row["feedrate"] == last["feedrate"]
            adjacent_row = row["index"] == last["row_end"] + 1
            if same_direction and same_feedrate and adjacent_row:
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
    return [values[int(round(index * last / (target_len - 1)))] for index in range(target_len)]


def rmse(expected, actual):
    count = min(len(expected), len(actual))
    if count == 0:
        return None
    return math.sqrt(sum((expected[index] - actual[index]) ** 2 for index in range(count)) / count)


def correlation(a_values, b_values):
    count = min(len(a_values), len(b_values))
    if count < 2:
        return None
    x_values = a_values[:count]
    y_values = b_values[:count]
    x_mean = sum(x_values) / count
    y_mean = sum(y_values) / count
    x_scale = math.sqrt(sum((value - x_mean) ** 2 for value in x_values))
    y_scale = math.sqrt(sum((value - y_mean) ** 2 for value in y_values))
    if x_scale <= 1e-12 or y_scale <= 1e-12:
        return None
    return sum((x_values[index] - x_mean) * (y_values[index] - y_mean) for index in range(count)) / (x_scale * y_scale)


def finite_difference_accel_from_displacement(displacement_m, dt):
    sample_count = len(displacement_m)
    if sample_count < 3 or dt <= 0:
        return [0.0] * sample_count
    accel = [0.0] * sample_count
    inv_dt2 = 1.0 / (dt * dt)
    for index in range(1, sample_count - 1):
        accel[index] = (displacement_m[index + 1] - 2.0 * displacement_m[index] + displacement_m[index - 1]) * inv_dt2
    accel[0] = accel[1]
    accel[-1] = accel[-2]
    return accel


class ShakingTableApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Shaking Table Controller v2")
        self.root.geometry("1180x780")

        self.motor = SerialClient("Motor", DEFAULT_BAUDRATE)
        self.accel = SerialClient("Accel", DEFAULT_BAUDRATE)

        self.port_var = tk.StringVar()
        self.accel_port_var = tk.StringVar()
        self.connection_var = tk.StringVar(value="Motor: disconnected")
        self.accel_connection_var = tk.StringVar(value="Accel: disconnected")
        self.status_var = tk.StringVar(value="Ready.")
        self.send_status_var = tk.StringVar(value="No motion loaded.")
        self.progress_var = tk.DoubleVar(value=0.0)
        self.dt_var = tk.DoubleVar(value=DEFAULT_SAMPLE_INTERVAL_S)
        self.highpass_enabled_var = tk.BooleanVar(value=True)
        self.highpass_cutoff_var = tk.DoubleVar(value=0.05)
        self.bias_var = tk.DoubleVar(value=0.0)
        self.noise_var = tk.StringVar(value="Bias: not calibrated")
        self.imu_var = tk.StringVar(value="IMU accel X: -- m/s2")
        self.mode_var = tk.StringVar(value="No import")

        self.rows = []
        self.merged = []
        self.target_accel = []
        self.input_displacement_m = []
        self.input_values = []
        self.import_mode = None
        self.rt_accel = []
        self.recording = False
        self.send_stop = threading.Event()
        self.send_running = False
        self.bias_samples = []
        self.bias_collecting_until = 0.0
        self.sweep_start_hz_var = tk.DoubleVar(value=1.0)
        self.sweep_stop_hz_var = tk.DoubleVar(value=MACHINE_MAX_SWEEP_HZ)
        self.sweep_step_hz_var = tk.DoubleVar(value=1.0)
        self.sweep_amplitude_cm_var = tk.DoubleVar(value=0.2)
        self.sweep_cycles_var = tk.IntVar(value=5)
        self.sweep_dt_var = tk.DoubleVar(value=0.01)
        self.sweep_status_var = tk.StringVar(value="Sweep not started.")
        self.sweep_progress_var = tk.DoubleVar(value=0.0)
        self.sweep_results = []
        self.sweep_running = False
        self.sweep_stop = threading.Event()
        self.validate_status_var = tk.StringVar(value="Import a reference, calibrate, then run validation.")
        self.validate_progress_var = tk.DoubleVar(value=0.0)
        self.validate_running = False
        self._after_ids = []

        self._build_ui()
        self._refresh_ports()
        self._schedule(self._drain_serial, 50)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.notebook = ttk.Notebook(self.root)
        self.notebook.grid(row=0, column=0, sticky="nsew")

        self.tab_monitor = ttk.Frame(self.notebook, padding=10)
        self.tab_graphs = ttk.Frame(self.notebook, padding=10)
        self.tab_table = ttk.Frame(self.notebook, padding=10)
        self.tab_correction = ttk.Frame(self.notebook, padding=10)
        self.tab_calibration = ttk.Frame(self.notebook, padding=10)
        self.tab_validate = ttk.Frame(self.notebook, padding=10)

        self.notebook.add(self.tab_monitor, text="Monitor")
        self.notebook.add(self.tab_graphs, text="Earthquake Graphs")
        self.notebook.add(self.tab_table, text="Data Table")
        self.notebook.add(self.tab_correction, text="Correction Analysis")
        self.notebook.add(self.tab_calibration, text="Calibration")
        self.notebook.add(self.tab_validate, text="Validate")

        self._build_monitor_tab()
        self._build_graphs_tab()
        self._build_table_tab()
        self._build_correction_tab()
        self._build_calibration_tab()
        self._build_validate_tab()

    def _build_monitor_tab(self):
        tab = self.tab_monitor
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(3, weight=1)

        conn = ttk.LabelFrame(tab, text="Connections", padding=8)
        conn.grid(row=0, column=0, sticky="ew")
        conn.columnconfigure(1, weight=1)
        conn.columnconfigure(4, weight=1)

        ttk.Label(conn, text="Motor MCU:").grid(row=0, column=0, sticky="w")
        self.motor_combo = ttk.Combobox(conn, textvariable=self.port_var, width=34)
        self.motor_combo.grid(row=0, column=1, sticky="ew", padx=6)
        ttk.Button(conn, text="Connect", command=self._connect_motor).grid(row=0, column=2, padx=(0, 8))
        ttk.Label(conn, textvariable=self.connection_var).grid(row=0, column=3, sticky="w")

        ttk.Label(conn, text="Accel MCU:").grid(row=1, column=0, sticky="w", pady=(6, 0))
        self.accel_combo = ttk.Combobox(conn, textvariable=self.accel_port_var, width=34)
        self.accel_combo.grid(row=1, column=1, sticky="ew", padx=6, pady=(6, 0))
        ttk.Button(conn, text="Connect", command=self._connect_accel).grid(row=1, column=2, padx=(0, 8), pady=(6, 0))
        ttk.Label(conn, textvariable=self.accel_connection_var).grid(row=1, column=3, sticky="w", pady=(6, 0))
        ttk.Button(conn, text="Refresh Ports", command=self._refresh_ports).grid(row=0, column=4, sticky="e")

        imports = ttk.LabelFrame(tab, text="Import Motion Reference", padding=8)
        imports.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        imports.columnconfigure(6, weight=1)
        ttk.Label(imports, text="dt (s):").grid(row=0, column=0, sticky="w")
        ttk.Entry(imports, textvariable=self.dt_var, width=7).grid(row=0, column=1, sticky="w", padx=(4, 12))
        ttk.Checkbutton(imports, text="High-pass accel", variable=self.highpass_enabled_var).grid(row=0, column=2, sticky="w")
        ttk.Label(imports, text="cutoff:").grid(row=0, column=3, sticky="w", padx=(10, 2))
        ttk.Entry(imports, textvariable=self.highpass_cutoff_var, width=7).grid(row=0, column=4, sticky="w")
        ttk.Button(imports, text="Import Accel Data", command=self._import_accel).grid(row=1, column=0, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Button(imports, text="Import Displacement Data", command=self._import_displacement).grid(row=1, column=2, columnspan=3, sticky="w", padx=(8, 0), pady=(8, 0))
        ttk.Label(imports, textvariable=self.mode_var, foreground="#374151").grid(row=1, column=5, sticky="w", padx=(12, 0), pady=(8, 0))

        send = ttk.LabelFrame(tab, text="Send", padding=8)
        send.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        send.columnconfigure(4, weight=1)
        ttk.Button(send, text="Send to Arduino", command=self._start_send).grid(row=0, column=0, sticky="w")
        ttk.Button(send, text="Stop", command=self._stop_send).grid(row=0, column=1, sticky="w", padx=(6, 0))
        ttk.Progressbar(send, variable=self.progress_var, maximum=100.0, length=260).grid(row=0, column=2, padx=10)
        ttk.Label(send, textvariable=self.send_status_var).grid(row=0, column=3, sticky="w")

        live = ttk.LabelFrame(tab, text="Live IMU", padding=8)
        live.grid(row=3, column=0, sticky="nsew", pady=(8, 0))
        live.columnconfigure(0, weight=1)
        live.rowconfigure(1, weight=1)
        ttk.Label(live, textvariable=self.imu_var).grid(row=0, column=0, sticky="w")
        ttk.Label(live, textvariable=self.noise_var).grid(row=0, column=1, sticky="e")
        self.live_canvas = tk.Canvas(live, bg=GRAPH_BG, height=280, highlightthickness=0)
        self.live_canvas.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(6, 0))
        ttk.Button(live, text="Clear Graph", command=self._clear_live).grid(row=2, column=0, sticky="w", pady=(6, 0))
        ttk.Button(live, text="Export CSV", command=self._export_recorded_accel).grid(row=2, column=1, sticky="e", pady=(6, 0))

    def _build_graphs_tab(self):
        tab = self.tab_graphs
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(0, weight=1)
        tab.rowconfigure(1, weight=1)
        tab.rowconfigure(2, weight=1)

        self.input_graph_frame = ttk.LabelFrame(tab, text="Imported Input", padding=4)
        self.input_graph_frame.grid(row=0, column=0, sticky="nsew")
        self.input_graph_frame.columnconfigure(0, weight=1)
        self.input_graph_frame.rowconfigure(0, weight=1)
        self.input_canvas = tk.Canvas(self.input_graph_frame, bg=GRAPH_BG, height=210, highlightthickness=0)
        self.input_canvas.grid(row=0, column=0, sticky="nsew")

        rt_frame = ttk.LabelFrame(tab, text="Measured Acceleration from IMU", padding=4)
        rt_frame.grid(row=1, column=0, sticky="nsew", pady=(8, 0))
        rt_frame.columnconfigure(0, weight=1)
        rt_frame.rowconfigure(0, weight=1)
        self.rt_canvas = tk.Canvas(rt_frame, bg=GRAPH_BG, height=210, highlightthickness=0)
        self.rt_canvas.grid(row=0, column=0, sticky="nsew")

        pos_frame = ttk.LabelFrame(tab, text="Commanded Position", padding=4)
        pos_frame.grid(row=2, column=0, sticky="nsew", pady=(8, 0))
        pos_frame.columnconfigure(0, weight=1)
        pos_frame.rowconfigure(0, weight=1)
        self.pos_canvas = tk.Canvas(pos_frame, bg=GRAPH_BG, height=210, highlightthickness=0)
        self.pos_canvas.grid(row=0, column=0, sticky="nsew")

    def _build_table_tab(self):
        tab = self.tab_table
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(0, weight=1)
        columns = ("index", "time", "accel", "steps", "feedrate", "direction", "position")
        self.table = ttk.Treeview(tab, columns=columns, show="headings")
        headings = {
            "index": "#",
            "time": "Time (s)",
            "accel": "Accel (m/s2)",
            "steps": "Steps",
            "feedrate": "Feedrate",
            "direction": "Dir",
            "position": "Position (cm)",
        }
        for column, text in headings.items():
            self.table.heading(column, text=text)
            self.table.column(column, width=110, anchor="center")
        self.table.grid(row=0, column=0, sticky="nsew")
        ybar = ttk.Scrollbar(tab, orient="vertical", command=self.table.yview)
        ybar.grid(row=0, column=1, sticky="ns")
        self.table.configure(yscrollcommand=ybar.set)
        footer = ttk.Frame(tab)
        footer.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        ttk.Button(footer, text="Export Table CSV", command=self._export_table).grid(row=0, column=0, sticky="w")

    def _build_correction_tab(self):
        tab = self.tab_correction
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(1, weight=1)
        tab.rowconfigure(2, weight=1)
        self.correction_summary_var = tk.StringVar(value="Run a send to calculate accuracy.")
        ttk.Label(tab, textvariable=self.correction_summary_var, font=("TkDefaultFont", 10, "bold")).grid(row=0, column=0, sticky="w")

        overlay = ttk.LabelFrame(tab, text="Target vs Measured Acceleration", padding=4)
        overlay.grid(row=1, column=0, sticky="nsew", pady=(8, 0))
        overlay.columnconfigure(0, weight=1)
        overlay.rowconfigure(0, weight=1)
        self.overlay_canvas = tk.Canvas(overlay, bg=GRAPH_BG, height=260, highlightthickness=0)
        self.overlay_canvas.grid(row=0, column=0, sticky="nsew")

        error = ttk.LabelFrame(tab, text="Acceleration Error", padding=4)
        error.grid(row=2, column=0, sticky="nsew", pady=(8, 0))
        error.columnconfigure(0, weight=1)
        error.rowconfigure(0, weight=1)
        self.error_canvas = tk.Canvas(error, bg=GRAPH_BG, height=260, highlightthickness=0)
        self.error_canvas.grid(row=0, column=0, sticky="nsew")

    def _build_calibration_tab(self):
        tab = self.tab_calibration
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(2, weight=1)
        bias = ttk.LabelFrame(tab, text="IMU Bias Calibration", padding=8)
        bias.grid(row=0, column=0, sticky="ew")
        ttk.Label(bias, text="Keep the table stationary, then collect bias samples.").grid(row=0, column=0, columnspan=3, sticky="w")
        ttk.Button(bias, text="Calibrate 30 seconds", command=lambda: self._start_bias_calibration(30.0)).grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Button(bias, text="Calibrate 10 seconds", command=lambda: self._start_bias_calibration(10.0)).grid(row=1, column=1, sticky="w", padx=(8, 0), pady=(8, 0))
        ttk.Label(bias, textvariable=self.noise_var, foreground="#374151").grid(row=1, column=2, sticky="w", padx=(12, 0), pady=(8, 0))

        sweep = ttk.LabelFrame(tab, text="Frequency Sweep", padding=8)
        sweep.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        for column in (1, 3, 5, 7):
            sweep.columnconfigure(column, weight=1)

        ttk.Label(sweep, text="Start Hz:").grid(row=0, column=0, sticky="w")
        ttk.Entry(sweep, textvariable=self.sweep_start_hz_var, width=8).grid(row=0, column=1, sticky="w", padx=(4, 12))
        ttk.Label(sweep, text="Stop Hz:").grid(row=0, column=2, sticky="w")
        ttk.Entry(sweep, textvariable=self.sweep_stop_hz_var, width=8).grid(row=0, column=3, sticky="w", padx=(4, 12))
        ttk.Label(sweep, text="Step Hz:").grid(row=0, column=4, sticky="w")
        ttk.Entry(sweep, textvariable=self.sweep_step_hz_var, width=8).grid(row=0, column=5, sticky="w", padx=(4, 12))
        ttk.Label(sweep, text="Amplitude cm:").grid(row=0, column=6, sticky="w")
        ttk.Entry(sweep, textvariable=self.sweep_amplitude_cm_var, width=8).grid(row=0, column=7, sticky="w", padx=(4, 0))

        ttk.Label(sweep, text="Cycles/frequency:").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(sweep, textvariable=self.sweep_cycles_var, width=8).grid(row=1, column=1, sticky="w", padx=(4, 12), pady=(6, 0))
        ttk.Label(sweep, text="Command dt s:").grid(row=1, column=2, sticky="w", pady=(6, 0))
        ttk.Entry(sweep, textvariable=self.sweep_dt_var, width=8).grid(row=1, column=3, sticky="w", padx=(4, 12), pady=(6, 0))
        ttk.Button(sweep, text="Start Sweep", command=self._start_frequency_sweep).grid(row=1, column=4, sticky="w", padx=(0, 6), pady=(6, 0))
        ttk.Button(sweep, text="Stop", command=self._stop_frequency_sweep).grid(row=1, column=5, sticky="w", pady=(6, 0))
        ttk.Button(sweep, text="Export Sweep CSV", command=self._export_sweep_csv).grid(row=1, column=6, columnspan=2, sticky="e", pady=(6, 0))

        ttk.Progressbar(sweep, variable=self.sweep_progress_var, maximum=100.0).grid(row=2, column=0, columnspan=8, sticky="ew", pady=(8, 0))
        ttk.Label(sweep, textvariable=self.sweep_status_var, foreground="#374151").grid(row=3, column=0, columnspan=8, sticky="w", pady=(4, 0))

        response = ttk.LabelFrame(tab, text="Frequency Response", padding=4)
        response.grid(row=2, column=0, sticky="nsew", pady=(8, 0))
        response.columnconfigure(0, weight=1)
        response.rowconfigure(0, weight=1)
        self.sweep_canvas = tk.Canvas(response, bg=GRAPH_BG, height=280, highlightthickness=0)
        self.sweep_canvas.grid(row=0, column=0, sticky="nsew")

    def _build_validate_tab(self):
        tab = self.tab_validate
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(1, weight=1)
        tab.rowconfigure(2, weight=1)

        controls = ttk.LabelFrame(tab, text="Validate Calibrated Response", padding=8)
        controls.grid(row=0, column=0, sticky="ew")
        controls.columnconfigure(4, weight=1)
        ttk.Button(controls, text="Run Validation", command=self._start_validate).grid(row=0, column=0, sticky="w")
        ttk.Button(controls, text="Stop", command=self._stop_validate).grid(row=0, column=1, sticky="w", padx=(6, 0))
        ttk.Progressbar(controls, variable=self.validate_progress_var, maximum=100.0, length=260).grid(row=0, column=2, sticky="w", padx=(10, 0))
        ttk.Label(controls, textvariable=self.validate_status_var, foreground="#374151").grid(row=0, column=3, columnspan=2, sticky="w", padx=(12, 0))

        desired = ttk.LabelFrame(tab, text="Desired Acceleration", padding=4)
        desired.grid(row=1, column=0, sticky="nsew", pady=(8, 0))
        desired.columnconfigure(0, weight=1)
        desired.rowconfigure(0, weight=1)
        self.validate_desired_canvas = tk.Canvas(desired, bg=GRAPH_BG, height=260, highlightthickness=0)
        self.validate_desired_canvas.grid(row=0, column=0, sticky="nsew")

        measured = ttk.LabelFrame(tab, text="Real-Time Measured Acceleration", padding=4)
        measured.grid(row=2, column=0, sticky="nsew", pady=(8, 0))
        measured.columnconfigure(0, weight=1)
        measured.rowconfigure(0, weight=1)
        self.validate_measured_canvas = tk.Canvas(measured, bg=GRAPH_BG, height=260, highlightthickness=0)
        self.validate_measured_canvas.grid(row=0, column=0, sticky="nsew")

    def _refresh_ports(self):
        ports = available_ports()
        self.motor_combo["values"] = ports
        self.accel_combo["values"] = ports
        if ports:
            if not self.port_var.get():
                self.port_var.set(self._preferred_port(ports, PREFERRED_MOTOR_PORT))
            if not self.accel_port_var.get():
                self.accel_port_var.set(self._preferred_port(ports, PREFERRED_ACCEL_PORT))

    def _preferred_port(self, ports, preferred):
        for port in ports:
            if port.startswith(preferred):
                return port
        return ports[0]

    def _connect_motor(self):
        try:
            self.motor.connect(self.port_var.get())
            self.connection_var.set("Motor: connected")
        except Exception as exc:
            messagebox.showerror("Motor connection", str(exc))

    def _connect_accel(self):
        try:
            self.accel.connect(self.accel_port_var.get())
            self.accel_connection_var.set("Accel: connected")
        except Exception as exc:
            messagebox.showerror("Accel connection", str(exc))

    def _import_accel(self):
        path = self._ask_import_file("Open acceleration file (first column in cm/s2)")
        if not path:
            return
        try:
            dt = self._dt()
            raw = load_first_numeric_column(path, scale=CMPS2_TO_MPS2)
            if len(raw) < 2:
                raise ValueError("Need at least 2 numeric rows")
            accel = prepare_accel(raw, dt, self.highpass_enabled_var.get(), self.highpass_cutoff_var.get())
            rows = generate_rows_from_accel(accel, dt)
            self._set_import(path, "accel", raw, rows, accel, [row["unclamped_position_cm"] / 100.0 for row in rows])
        except Exception as exc:
            messagebox.showerror("Import acceleration", str(exc))

    def _import_displacement(self):
        path = self._ask_import_file("Open displacement file (first column in cm)")
        if not path:
            return
        try:
            dt = self._dt()
            displacement = load_first_numeric_column(path, scale=CM_TO_M)
            if len(displacement) < 2:
                raise ValueError("Need at least 2 numeric rows")
            start = displacement[0]
            displacement = [value - start for value in displacement]
            target_accel = finite_difference_accel_from_displacement(displacement, dt)
            rows = generate_rows_from_displacement(displacement, dt, accel_mps2=target_accel)
            self._set_import(path, "displacement", displacement, rows, target_accel, displacement)
        except Exception as exc:
            messagebox.showerror("Import displacement", str(exc))

    def _ask_import_file(self, title):
        return filedialog.askopenfilename(
            title=title,
            filetypes=[("Excel files", "*.xlsx *.xlsm"), ("CSV files", "*.csv"), ("All files", "*.*")],
        )

    def _set_import(self, path, mode, input_values, rows, target_accel, displacement_m):
        self.import_mode = mode
        self.input_values = list(input_values)
        self.rows = rows
        self.merged = merge_rows(rows)
        self.target_accel = list(target_accel)
        self.input_displacement_m = list(displacement_m)
        self.rt_accel = []
        self.progress_var.set(0.0)
        self.mode_var.set(f"{mode.title()} import: {Path(path).name}")
        nonzero = sum(1 for row in rows if row["steps"] > 0)
        self.send_status_var.set(f"Rows: {len(rows)} | nonzero: {nonzero} | merged: {len(self.merged)}")
        self._refresh_table()
        self._draw_all_graphs()
        self._draw_validate_graphs()
        self.notebook.select(self.tab_graphs)

    def _dt(self):
        value = float(self.dt_var.get())
        if value <= 0:
            raise ValueError("dt must be greater than 0")
        return value

    def _start_send(self):
        if self.send_running:
            return
        if not self.rows or not self.merged:
            messagebox.showwarning("Send", "Import data first.")
            return
        if not self.motor.is_connected():
            messagebox.showwarning("Send", "Connect Motor MCU first.")
            return
        self.send_stop.clear()
        self.send_running = True
        self.recording = True
        self.rt_accel = []
        self.progress_var.set(0.0)
        threading.Thread(target=self._send_worker, daemon=True).start()

    def _stop_send(self):
        self.send_stop.set()
        self.send_status_var.set("Stopping after current command...")

    def _send_worker(self):
        try:
            total = len(self.merged)
            for index, command in enumerate(self.merged, start=1):
                if self.send_stop.is_set():
                    break
                self.root.after(0, lambda i=index, t=total: self._send_progress(i, t))
                self.motor.send_motion_wait_ok(command["steps"], command["direction"], command["feedrate"])
            self.root.after(0, self._send_done)
        except Exception as exc:
            self.root.after(0, lambda: self._send_failed(str(exc)))

    def _send_progress(self, index, total):
        self.progress_var.set(index * 100.0 / max(total, 1))
        self.send_status_var.set(f"Sending command {index}/{total}")

    def _send_done(self):
        self.send_running = False
        self.recording = False
        self.progress_var.set(100.0)
        self.send_status_var.set(f"Done. Recorded {len(self.rt_accel)} IMU samples.")
        self._draw_all_graphs()
        self._update_correction()
        self.notebook.select(self.tab_correction)

    def _send_failed(self, error):
        self.send_running = False
        self.recording = False
        self.send_status_var.set(f"Send failed: {error}")
        messagebox.showerror("Send failed", error)

    def _drain_serial(self):
        self._drain_client_logs(self.motor)
        self._drain_client_logs(self.accel)
        self._drain_accel_rx()
        if self.validate_running:
            self._draw_validate_graphs()
        else:
            self._draw_live_graph()
        self._schedule(self._drain_serial, 50)

    def _drain_client_logs(self, client):
        while True:
            try:
                self.status_var.set(client.log_queue.get_nowait())
            except queue.Empty:
                break

    def _drain_accel_rx(self):
        latest = None
        while True:
            try:
                line = self.accel.rx_queue.get_nowait()
            except queue.Empty:
                break
            value = self._parse_accel_x(line)
            if value is None:
                continue
            corrected = value - self.bias_var.get()
            latest = corrected
            if self.recording:
                self.rt_accel.append(corrected)
            if time.time() < self.bias_collecting_until:
                self.bias_samples.append(value)
        if latest is not None:
            self.imu_var.set(f"IMU accel X: {latest:.4f} m/s2")

    def _parse_accel_x(self, line):
        parts = [part.strip() for part in line.split(",")]
        if len(parts) == 10 and parts[0] == "IMU":
            try:
                return float(parts[4]) * GRAVITY_MS2
            except ValueError:
                return None

        numeric_values = []
        for part in parts:
            try:
                numeric_values.append(float(part))
            except ValueError:
                continue
        if not numeric_values:
            return None
        return numeric_values[0]

    def _start_bias_calibration(self, seconds):
        if not self.accel.is_connected():
            messagebox.showwarning("Bias calibration", "Connect Accel MCU first.")
            return
        self.bias_samples = []
        self.bias_collecting_until = time.time() + seconds
        self.noise_var.set(f"Collecting bias for {seconds:.0f} seconds...")
        self._schedule(lambda: self._finish_bias_calibration(seconds), int(seconds * 1000) + 50)

    def _finish_bias_calibration(self, seconds):
        if not self.bias_samples:
            self.noise_var.set("Bias calibration failed: no IMU samples")
            return
        mean = sum(self.bias_samples) / len(self.bias_samples)
        variance = sum((value - mean) ** 2 for value in self.bias_samples) / len(self.bias_samples)
        std = math.sqrt(variance)
        self.bias_var.set(mean)
        self.noise_var.set(f"Bias: {mean:.5f} m/s2 | noise std: {std:.5f} | samples: {len(self.bias_samples)}")

    def _start_frequency_sweep(self):
        if self.sweep_running:
            return
        if self.send_running:
            messagebox.showwarning("Frequency sweep", "A send is already running.")
            return
        if not self.motor.is_connected():
            messagebox.showwarning("Frequency sweep", "Connect Motor MCU first.")
            return
        if not self.accel.is_connected():
            messagebox.showwarning("Frequency sweep", "Connect Accel MCU first.")
            return

        try:
            start_hz = float(self.sweep_start_hz_var.get())
            stop_hz = float(self.sweep_stop_hz_var.get())
            step_hz = float(self.sweep_step_hz_var.get())
            amplitude_cm = float(self.sweep_amplitude_cm_var.get())
            cycles = int(self.sweep_cycles_var.get())
            dt = float(self.sweep_dt_var.get())
            if start_hz <= 0 or stop_hz < start_hz or step_hz <= 0:
                raise ValueError("Use positive frequencies with stop >= start.")
            if amplitude_cm <= 0 or amplitude_cm > STROKE_LIMIT_M * 100.0:
                raise ValueError(f"Amplitude must be > 0 and <= {STROKE_LIMIT_M * 100:.1f} cm.")
            if start_hz > MACHINE_MAX_SWEEP_HZ:
                raise ValueError(f"start frequency must be <= {MACHINE_MAX_SWEEP_HZ:.1f} Hz")
            if stop_hz > MACHINE_MAX_SWEEP_HZ:
                stop_hz = MACHINE_MAX_SWEEP_HZ
                self.sweep_stop_hz_var.set(MACHINE_MAX_SWEEP_HZ)
            if cycles < 1:
                raise ValueError("Cycles must be at least 1.")
            if dt <= 0:
                raise ValueError("Command dt must be greater than 0.")
        except (ValueError, tk.TclError) as exc:
            messagebox.showerror("Frequency sweep", str(exc))
            return

        frequencies = []
        value = start_hz
        while value <= stop_hz + 1e-9:
            frequencies.append(round(value, 6))
            value += step_hz
        if not frequencies:
            messagebox.showerror("Frequency sweep", "No frequencies generated.")
            return

        self.sweep_results = []
        self.sweep_stop.clear()
        self.sweep_running = True
        self.sweep_progress_var.set(0.0)
        self.sweep_status_var.set(f"Starting sweep: {len(frequencies)} frequencies")
        self._draw_sweep_response()

        threading.Thread(
            target=self._frequency_sweep_worker,
            args=(frequencies, amplitude_cm, cycles, dt),
            daemon=True,
        ).start()

    def _stop_frequency_sweep(self):
        self.sweep_stop.set()
        self.sweep_status_var.set("Stopping sweep after current command...")

    def _frequency_sweep_worker(self, frequencies, amplitude_cm, cycles, dt):
        try:
            total = len(frequencies)
            amplitude_m = amplitude_cm / 100.0
            for index, frequency_hz in enumerate(frequencies, start=1):
                if self.sweep_stop.is_set():
                    break

                self.root.after(0, lambda f=frequency_hz, i=index, t=total: self._sweep_frequency_started(f, i, t))
                rows = self._make_sine_sweep_rows(frequency_hz, amplitude_m, cycles, dt)
                commands = merge_rows(rows)
                if not commands:
                    self.root.after(0, lambda f=frequency_hz: self._sweep_status_var_safe(f"{f:.2f} Hz produced no motion commands."))
                    continue

                self.rt_accel = []
                self.recording = True
                for command in commands:
                    if self.sweep_stop.is_set():
                        break
                    self.motor.send_motion_wait_ok(command["steps"], command["direction"], command["feedrate"])
                time.sleep(0.25)
                self.recording = False

                measured = list(self.rt_accel)
                result = self._calculate_sweep_result(frequency_hz, amplitude_m, measured)
                self.sweep_results.append(result)
                self.root.after(0, lambda r=result, i=index, t=total: self._sweep_frequency_finished(r, i, t))

            self.root.after(0, self._frequency_sweep_done)
        except Exception as exc:
            self.root.after(0, lambda: self._frequency_sweep_failed(str(exc)))

    def _sweep_status_var_safe(self, text):
        self.sweep_status_var.set(text)

    def _sweep_frequency_started(self, frequency_hz, index, total):
        self.sweep_progress_var.set((index - 1) * 100.0 / max(total, 1))
        self.sweep_status_var.set(f"Sweeping {frequency_hz:.2f} Hz ({index}/{total})...")

    def _sweep_frequency_finished(self, result, index, total):
        self.sweep_progress_var.set(index * 100.0 / max(total, 1))
        self.sweep_status_var.set(
            f"{result['frequency_hz']:.2f} Hz: measured {result['measured_peak_mps2']:.4f} m/s2, "
            f"gain {result['gain']:.3f}, samples {result['sample_count']}"
        )
        self._draw_sweep_response()

    def _frequency_sweep_done(self):
        self.sweep_running = False
        self.recording = False
        self.sweep_progress_var.set(100.0 if self.sweep_results else 0.0)
        if self.sweep_stop.is_set():
            self.sweep_status_var.set(f"Sweep stopped. Completed {len(self.sweep_results)} frequency points.")
        else:
            self.sweep_status_var.set(f"Sweep complete. Measured {len(self.sweep_results)} frequency points.")
        self._draw_sweep_response()

    def _frequency_sweep_failed(self, error):
        self.sweep_running = False
        self.recording = False
        self.sweep_status_var.set(f"Sweep failed: {error}")
        messagebox.showerror("Frequency sweep", error)

    def _make_sine_sweep_rows(self, frequency_hz, amplitude_m, cycles, dt):
        duration_s = cycles / frequency_hz
        sample_count = max(3, int(math.ceil(duration_s / dt)) + 1)
        displacement = []
        for index in range(sample_count):
            t = index * dt
            displacement.append(amplitude_m * math.sin(2.0 * math.pi * frequency_hz * t))
        displacement[-1] = 0.0
        target_accel = finite_difference_accel_from_displacement(displacement, dt)
        return generate_rows_from_displacement(displacement, dt, accel_mps2=target_accel)

    def _calculate_sweep_result(self, frequency_hz, amplitude_m, measured):
        commanded_peak = (2.0 * math.pi * frequency_hz) ** 2 * amplitude_m
        usable = measured[int(len(measured) * 0.15):] if len(measured) >= 10 else measured
        if usable:
            measured_peak = (max(usable) - min(usable)) / 2.0
            measured_rms = math.sqrt(sum(value * value for value in usable) / len(usable))
        else:
            measured_peak = 0.0
            measured_rms = 0.0
        gain = measured_peak / commanded_peak if commanded_peak > 1e-12 else 0.0
        return {
            "frequency_hz": frequency_hz,
            "amplitude_cm": amplitude_m * 100.0,
            "commanded_peak_mps2": commanded_peak,
            "measured_peak_mps2": measured_peak,
            "measured_rms_mps2": measured_rms,
            "gain": gain,
            "sample_count": len(measured),
        }

    def _draw_sweep_response(self):
        canvas = getattr(self, "sweep_canvas", None)
        if canvas is None or not canvas.winfo_exists():
            return
        canvas.delete("all")
        width = max(canvas.winfo_width(), 700)
        height = max(canvas.winfo_height(), 240)
        margin = 42
        canvas.create_rectangle(0, 0, width, height, fill=GRAPH_BG, outline="")
        canvas.create_text(margin, 14, anchor="w", text="Frequency response: gain = measured accel peak / commanded accel peak", fill="#0f172a", font=("TkDefaultFont", 9, "bold"))

        if len(self.sweep_results) < 1:
            canvas.create_text(width / 2, height / 2, text="Run a sweep to build the response curve", fill=GRAPH_AXIS)
            return

        frequencies = [row["frequency_hz"] for row in self.sweep_results]
        gains = [row["gain"] for row in self.sweep_results]
        x_min = min(frequencies)
        x_max = max(frequencies)
        y_min = min(0.0, min(gains))
        y_max = max(1.0, max(gains))
        if abs(x_max - x_min) < 1e-12:
            x_max += 1.0
            x_min -= 1.0
        if abs(y_max - y_min) < 1e-12:
            y_max += 1.0

        usable_w = width - 2 * margin
        usable_h = height - 2 * margin
        for index in range(5):
            y = margin + index * usable_h / 4
            canvas.create_line(margin, y, width - margin, y, fill=GRAPH_GRID)
            value = y_max - index * (y_max - y_min) / 4
            canvas.create_text(margin - 8, y, anchor="e", text=f"{value:.2f}", fill=GRAPH_AXIS)
        canvas.create_line(margin, margin, margin, height - margin, fill=GRAPH_AXIS)
        canvas.create_line(margin, height - margin, width - margin, height - margin, fill=GRAPH_AXIS)

        points = []
        for frequency_hz, gain in zip(frequencies, gains):
            x = margin + (frequency_hz - x_min) * usable_w / (x_max - x_min)
            y = margin + (y_max - gain) * usable_h / (y_max - y_min)
            points.extend((x, y))
            canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill=ACCEL_COLOR, outline="")
        if len(points) >= 4:
            canvas.create_line(*points, fill=ACCEL_COLOR, width=2)

        canvas.create_text(width / 2, height - 10, text="Frequency (Hz)", fill=GRAPH_AXIS)
        canvas.create_text(12, height / 2, text="Gain", angle=90, fill=GRAPH_AXIS)
        canvas.create_text(margin, height - margin + 16, anchor="w", text=f"{x_min:.1f} Hz", fill=GRAPH_AXIS)
        canvas.create_text(width - margin, height - margin + 16, anchor="e", text=f"{x_max:.1f} Hz", fill=GRAPH_AXIS)

    def _start_frequency_sweep(self):
        if self.sweep_running:
            return
        if self.send_running:
            messagebox.showwarning("Frequency sweep", "A send is already running.")
            return
        if not self.motor.is_connected():
            messagebox.showwarning("Frequency sweep", "Connect Motor MCU first.")
            return
        if not self.accel.is_connected():
            messagebox.showwarning("Frequency sweep", "Connect Accel MCU first.")
            return

        try:
            start_hz = float(self.sweep_start_hz_var.get())
            stop_hz = float(self.sweep_stop_hz_var.get())
            step_hz = float(self.sweep_step_hz_var.get())
            amplitude_cm = float(self.sweep_amplitude_cm_var.get())
            cycles = int(self.sweep_cycles_var.get())
            dt = float(self.sweep_dt_var.get())
            if start_hz <= 0 or stop_hz < start_hz or step_hz <= 0:
                raise ValueError("frequency range")
            if start_hz > MACHINE_MAX_SWEEP_HZ:
                raise ValueError(f"start frequency must be <= {MACHINE_MAX_SWEEP_HZ:.1f} Hz")
            if stop_hz > MACHINE_MAX_SWEEP_HZ:
                stop_hz = MACHINE_MAX_SWEEP_HZ
                self.sweep_stop_hz_var.set(MACHINE_MAX_SWEEP_HZ)
            if amplitude_cm <= 0 or amplitude_cm > STROKE_LIMIT_M * 100.0:
                raise ValueError("amplitude")
            if cycles < 1:
                raise ValueError("cycles")
            if dt <= 0 or dt > 0.05:
                raise ValueError("command dt")
        except (ValueError, tk.TclError) as exc:
            messagebox.showerror("Frequency sweep", f"Invalid sweep parameter: {exc}")
            return

        frequencies = []
        current = start_hz
        while current <= stop_hz + step_hz * 0.001:
            frequencies.append(round(current, 6))
            current += step_hz
        if not frequencies:
            messagebox.showerror("Frequency sweep", "No frequencies to sweep.")
            return

        self.sweep_results = []
        self.sweep_stop.clear()
        self.sweep_running = True
        self.sweep_progress_var.set(0.0)
        self.sweep_status_var.set(f"Starting sweep: {len(frequencies)} frequencies")
        self.send_status_var.set("Frequency sweep running...")
        self._draw_sweep_response()

        threading.Thread(
            target=self._frequency_sweep_worker,
            args=(frequencies, amplitude_cm, cycles, dt),
            daemon=True,
        ).start()

    def _stop_frequency_sweep(self):
        self.sweep_stop.set()
        if self.sweep_running:
            self.sweep_status_var.set("Stopping sweep after current command...")

    def _frequency_sweep_worker(self, frequencies, amplitude_cm, cycles, dt):
        try:
            total = len(frequencies)
            for index, frequency_hz in enumerate(frequencies, start=1):
                if self.sweep_stop.is_set():
                    break

                rows = self._generate_sine_sweep_rows(frequency_hz, amplitude_cm, cycles, dt)
                commands = merge_rows(rows)
                if not commands:
                    continue

                target_accel_amp = (2.0 * math.pi * frequency_hz) ** 2 * (amplitude_cm / 100.0)
                self.root.after(0, lambda i=index, t=total, f=frequency_hz: self._sweep_progress(i, t, f))

                self.rt_accel = []
                self.recording = True
                for command in commands:
                    if self.sweep_stop.is_set():
                        break
                    self.motor.send_motion_wait_ok(command["steps"], command["direction"], command["feedrate"])

                time.sleep(0.25)
                self.recording = False
                samples = list(self.rt_accel)
                result = self._analyze_sweep_frequency(frequency_hz, amplitude_cm, target_accel_amp, samples)
                self.sweep_results.append(result)
                self.root.after(0, self._draw_sweep_response)

                self._send_return_to_zero(rows[-1]["position_cm"] / 100.0, dt)
                time.sleep(0.15)

            self.root.after(0, self._frequency_sweep_done)
        except Exception as exc:
            self.root.after(0, lambda: self._frequency_sweep_failed(str(exc)))

    def _generate_sine_sweep_rows(self, frequency_hz, amplitude_cm, cycles, dt):
        duration = cycles / frequency_hz
        sample_count = max(8, int(math.ceil(duration / dt)) + 1)
        amplitude_m = amplitude_cm / 100.0
        displacement = []
        accel = []
        omega = 2.0 * math.pi * frequency_hz
        for index in range(sample_count):
            t = min(index * dt, duration)
            displacement.append(amplitude_m * math.sin(omega * t))
            accel.append(-amplitude_m * omega * omega * math.sin(omega * t))
        displacement[-1] = 0.0
        accel[-1] = 0.0
        return generate_rows_from_displacement(displacement, dt, accel_mps2=accel)

    def _send_return_to_zero(self, current_position_m, dt):
        if abs(current_position_m) < 1e-5 or not self.motor.is_connected():
            return
        rows = generate_rows_from_displacement([current_position_m, 0.0], dt, accel_mps2=[0.0, 0.0])
        for command in merge_rows(rows):
            self.motor.send_motion_wait_ok(command["steps"], command["direction"], command["feedrate"])

    def _analyze_sweep_frequency(self, frequency_hz, amplitude_cm, target_accel_amp, samples):
        usable = samples[int(len(samples) * 0.15):] if len(samples) > 10 else samples
        if usable:
            measured_peak = max(abs(value) for value in usable)
            measured_pp_amp = (max(usable) - min(usable)) / 2.0
            measured_rms = math.sqrt(sum(value * value for value in usable) / len(usable))
        else:
            measured_peak = 0.0
            measured_pp_amp = 0.0
            measured_rms = 0.0
        measured_amp = max(measured_peak, abs(measured_pp_amp))
        gain = measured_amp / target_accel_amp if target_accel_amp > 1e-12 else 0.0
        return {
            "frequency_hz": frequency_hz,
            "amplitude_cm": amplitude_cm,
            "target_accel_amp_mps2": target_accel_amp,
            "measured_accel_amp_mps2": measured_amp,
            "measured_accel_rms_mps2": measured_rms,
            "gain": gain,
            "samples": len(samples),
        }

    def _sweep_progress(self, index, total, frequency_hz):
        self.sweep_progress_var.set((index - 1) * 100.0 / max(total, 1))
        self.sweep_status_var.set(f"Sweeping {frequency_hz:.2f} Hz ({index}/{total})")

    def _frequency_sweep_done(self):
        self.sweep_running = False
        self.recording = False
        self.sweep_progress_var.set(100.0)
        if self.sweep_stop.is_set():
            self.sweep_status_var.set(f"Sweep stopped. Completed {len(self.sweep_results)} frequencies.")
        else:
            self.sweep_status_var.set(f"Sweep complete. Completed {len(self.sweep_results)} frequencies.")
        self.send_status_var.set("Frequency sweep complete.")
        self._draw_sweep_response()

    def _frequency_sweep_failed(self, error):
        self.sweep_running = False
        self.recording = False
        self.sweep_status_var.set(f"Sweep failed: {error}")
        self.send_status_var.set("Frequency sweep failed.")
        messagebox.showerror("Frequency sweep failed", error)

    def _draw_sweep_response(self):
        canvas = getattr(self, "sweep_canvas", None)
        if canvas is None or not canvas.winfo_exists():
            return
        canvas.delete("all")
        width = max(canvas.winfo_width(), 700)
        height = max(canvas.winfo_height(), 260)
        margin = 42
        canvas.create_rectangle(0, 0, width, height, fill=GRAPH_BG, outline="")
        canvas.create_text(margin, 14, anchor="w", text="Frequency response: measured acceleration gain", fill="#0f172a", font=("TkDefaultFont", 9, "bold"))
        if len(self.sweep_results) < 1:
            canvas.create_text(width / 2, height / 2, text="Run a sweep to build the response curve", fill=GRAPH_AXIS)
            return

        freqs = [row["frequency_hz"] for row in self.sweep_results]
        gains = [row["gain"] for row in self.sweep_results]
        f_min = min(freqs)
        f_max = max(freqs)
        g_min = min(0.0, min(gains))
        g_max = max(1.0, max(gains) * 1.1)
        if abs(f_max - f_min) < 1e-12:
            f_max = f_min + 1.0
        if abs(g_max - g_min) < 1e-12:
            g_max = g_min + 1.0

        usable_w = width - 2 * margin
        usable_h = height - 2 * margin
        for i in range(5):
            y = margin + i * usable_h / 4
            canvas.create_line(margin, y, width - margin, y, fill=GRAPH_GRID)
            value = g_max - i * (g_max - g_min) / 4
            canvas.create_text(margin - 6, y, anchor="e", text=f"{value:.2f}", fill=GRAPH_AXIS, font=("TkDefaultFont", 8))
        canvas.create_line(margin, margin, margin, height - margin, fill=GRAPH_AXIS)
        canvas.create_line(margin, height - margin, width - margin, height - margin, fill=GRAPH_AXIS)
        canvas.create_text(width / 2, height - 10, text="Frequency (Hz)", fill=GRAPH_AXIS, font=("TkDefaultFont", 8))
        canvas.create_text(12, height / 2, text="Gain", fill=GRAPH_AXIS, font=("TkDefaultFont", 8), angle=90)

        points = []
        for frequency_hz, gain in zip(freqs, gains):
            x = margin + (frequency_hz - f_min) * usable_w / (f_max - f_min)
            y = margin + (g_max - gain) * usable_h / (g_max - g_min)
            points.extend((x, y))
            canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill=DISP_COLOR, outline="")
        if len(points) >= 4:
            canvas.create_line(*points, fill=DISP_COLOR, width=2)

        last = self.sweep_results[-1]
        canvas.create_text(
            width - margin,
            margin,
            anchor="ne",
            text=(
                f"last {last['frequency_hz']:.2f} Hz\n"
                f"measured {last['measured_accel_amp_mps2']:.3f} m/s2\n"
                f"target {last['target_accel_amp_mps2']:.3f} m/s2\n"
                f"gain {last['gain']:.3f}"
            ),
            fill=GRAPH_AXIS,
        )

    def _export_sweep_csv(self):
        if not self.sweep_results:
            messagebox.showwarning("Export sweep", "No sweep results to export.")
            return
        columns = {
            "frequency_hz": [row["frequency_hz"] for row in self.sweep_results],
            "amplitude_cm": [row["amplitude_cm"] for row in self.sweep_results],
            "target_accel_amp_mps2": [row["target_accel_amp_mps2"] for row in self.sweep_results],
            "measured_accel_amp_mps2": [row["measured_accel_amp_mps2"] for row in self.sweep_results],
            "measured_accel_rms_mps2": [row["measured_accel_rms_mps2"] for row in self.sweep_results],
            "gain": [row["gain"] for row in self.sweep_results],
            "samples": [row["samples"] for row in self.sweep_results],
        }
        self._export_csv(columns, "frequency_sweep_response")

    def _refresh_table(self):
        for item in self.table.get_children():
            self.table.delete(item)
        for row in self.rows[:5000]:
            self.table.insert("", "end", values=(
                row["index"],
                f"{row['time_s']:.3f}",
                f"{row['accel_mps2']:.4f}",
                row["steps"],
                row["feedrate"],
                row["direction"],
                f"{row['position_cm']:.4f}",
            ))

    def _draw_all_graphs(self):
        if self.import_mode == "displacement":
            input_series = [value * 100.0 for value in self.input_displacement_m]
            input_label = "Displacement input (cm)"
            input_color = DISP_COLOR
        elif self.import_mode == "accel":
            input_series = self.target_accel
            input_label = "Acceleration input (m/s2)"
            input_color = ACCEL_COLOR
        else:
            input_series = []
            input_label = "Imported input"
            input_color = ACCEL_COLOR
        self._draw_series(self.input_canvas, input_series, input_label, input_color)
        self._draw_series(self.rt_canvas, self.rt_accel, "Measured acceleration (m/s2)", ACTUAL_COLOR)
        self._draw_series(self.pos_canvas, [row["position_cm"] for row in self.rows], "Commanded position (cm)", DISP_COLOR)

    def _draw_live_graph(self):
        self._draw_series(self.live_canvas, self.rt_accel[-1000:], "Live recorded acceleration (m/s2)", ACTUAL_COLOR)

    def _clear_live(self):
        self.rt_accel = []
        self._draw_all_graphs()

    def _draw_series(self, canvas, values, label, color):
        if canvas is None or not canvas.winfo_exists():
            return
        canvas.delete("all")
        width = max(canvas.winfo_width(), 600)
        height = max(canvas.winfo_height(), 180)
        margin = 34
        canvas.create_rectangle(0, 0, width, height, fill=GRAPH_BG, outline="")
        canvas.create_text(margin, 12, anchor="w", text=label, fill="#0f172a", font=("TkDefaultFont", 9, "bold"))
        if len(values) < 2:
            canvas.create_text(width / 2, height / 2, text="No data", fill=GRAPH_AXIS)
            return
        usable_w = width - 2 * margin
        usable_h = height - 2 * margin
        vmin = min(values)
        vmax = max(values)
        if abs(vmax - vmin) < 1e-12:
            vmax += 1.0
            vmin -= 1.0
        for index in range(5):
            y = margin + index * usable_h / 4
            canvas.create_line(margin, y, width - margin, y, fill=GRAPH_GRID)
        canvas.create_line(margin, margin, margin, height - margin, fill=GRAPH_AXIS)
        canvas.create_line(margin, height - margin, width - margin, height - margin, fill=GRAPH_AXIS)
        points = []
        sample_count = len(values)
        for index, value in enumerate(values):
            x = margin + index * usable_w / (sample_count - 1)
            y = margin + (vmax - value) * usable_h / (vmax - vmin)
            points.extend((x, y))
        canvas.create_line(*points, fill=color, width=2)
        canvas.create_text(width - margin, margin, anchor="ne", text=f"max {vmax:.3f}\nmin {vmin:.3f}", fill=GRAPH_AXIS)

    def _update_correction(self):
        if not self.target_accel or not self.rt_accel:
            self.correction_summary_var.set("Need target and measured acceleration.")
            return
        actual = resample_nearest(self.rt_accel, len(self.target_accel))
        value_rmse = rmse(self.target_accel, actual)
        value_corr = correlation(self.target_accel, actual)
        corr_text = "--" if value_corr is None else f"{value_corr:.4f}"
        rmse_text = "--" if value_rmse is None else f"{value_rmse:.5f} m/s2"
        self.correction_summary_var.set(
            f"RMSE: {rmse_text} | correlation: {corr_text} | samples: target {len(self.target_accel)}, measured {len(self.rt_accel)}"
        )
        self._draw_overlay(self.overlay_canvas, self.target_accel, actual)
        errors = [self.target_accel[index] - actual[index] for index in range(min(len(self.target_accel), len(actual)))]
        self._draw_series(self.error_canvas, errors, "target - measured (m/s2)", ERROR_COLOR)

    def _draw_overlay(self, canvas, expected, actual):
        if canvas is None or not canvas.winfo_exists():
            return
        width = max(canvas.winfo_width(), 600)
        height = max(canvas.winfo_height(), 180)
        margin = 34

        canvas.delete("all")
        canvas.create_rectangle(0, 0, width, height, fill=GRAPH_BG, outline="")
        canvas.create_text(
            margin,
            12,
            anchor="w",
            text="Blue target, red measured (shared scale)",
            fill="#0f172a",
            font=("TkDefaultFont", 9, "bold"),
        )

        if len(expected) < 2 and len(actual) < 2:
            canvas.create_text(width / 2, height / 2, text="No data", fill=GRAPH_AXIS)
            return

        combined = expected + actual
        vmin = min(combined)
        vmax = max(combined)
        if abs(vmax - vmin) < 1e-12:
            vmax += 1.0
            vmin -= 1.0
        usable_w = width - 2 * margin
        usable_h = height - 2 * margin

        for index in range(5):
            y = margin + index * usable_h / 4
            canvas.create_line(margin, y, width - margin, y, fill=GRAPH_GRID)
        canvas.create_line(margin, margin, margin, height - margin, fill=GRAPH_AXIS)
        canvas.create_line(margin, height - margin, width - margin, height - margin, fill=GRAPH_AXIS)

        def draw_curve(values, color):
            if len(values) < 2:
                return
            points = []
            sample_count = len(values)
            for index, value in enumerate(values):
                x = margin + index * usable_w / (sample_count - 1)
                y = margin + (vmax - value) * usable_h / (vmax - vmin)
                points.extend((x, y))
            canvas.create_line(*points, fill=color, width=2)

        draw_curve(expected, ACCEL_COLOR)
        draw_curve(actual, ACTUAL_COLOR)
        canvas.create_text(width - margin, margin, anchor="ne", text=f"max {vmax:.3f}\nmin {vmin:.3f}", fill=GRAPH_AXIS)

    def _start_validate(self):
        if self.validate_running or self.send_running or self.sweep_running:
            return
        if not self.rows or not self.merged or not self.target_accel:
            messagebox.showwarning("Validate", "Import acceleration or displacement reference data first.")
            return
        if not self.motor.is_connected():
            messagebox.showwarning("Validate", "Connect Motor MCU first.")
            return
        if not self.accel.is_connected():
            messagebox.showwarning("Validate", "Connect Accel MCU first.")
            return

        self.send_stop.clear()
        self.validate_running = True
        self.send_running = True
        self.recording = True
        self.rt_accel = []
        self.validate_progress_var.set(0.0)
        self.validate_status_var.set("Validation running...")
        self._draw_validate_graphs()
        self.notebook.select(self.tab_validate)
        threading.Thread(target=self._validate_worker, daemon=True).start()

    def _stop_validate(self):
        self.send_stop.set()
        if self.validate_running:
            self.validate_status_var.set("Stopping validation after current command...")

    def _validate_worker(self):
        try:
            total = len(self.merged)
            for index, command in enumerate(self.merged, start=1):
                if self.send_stop.is_set():
                    break
                self.root.after(0, lambda i=index, t=total: self._validate_progress(i, t))
                self.motor.send_motion_wait_ok(command["steps"], command["direction"], command["feedrate"])
            self.root.after(0, self._validate_done)
        except Exception as exc:
            self.root.after(0, lambda: self._validate_failed(str(exc)))

    def _validate_progress(self, index, total):
        self.validate_progress_var.set(index * 100.0 / max(total, 1))
        self.validate_status_var.set(f"Validation sending command {index}/{total}")
        self._draw_validate_graphs()

    def _validate_done(self):
        self.validate_running = False
        self.send_running = False
        self.recording = False
        self.validate_progress_var.set(100.0)
        actual = resample_nearest(self.rt_accel, len(self.target_accel)) if self.rt_accel else []
        value_rmse = rmse(self.target_accel, actual) if actual else None
        value_corr = correlation(self.target_accel, actual) if actual else None
        rmse_text = "--" if value_rmse is None else f"{value_rmse:.5f} m/s2"
        corr_text = "--" if value_corr is None else f"{value_corr:.4f}"
        self.validate_status_var.set(
            f"Validation complete. Samples: {len(self.rt_accel)} | RMSE: {rmse_text} | corr: {corr_text}"
        )
        self._draw_validate_graphs()
        self._update_correction()
        self.notebook.select(self.tab_validate)

    def _validate_failed(self, error):
        self.validate_running = False
        self.send_running = False
        self.recording = False
        self.validate_status_var.set(f"Validation failed: {error}")
        messagebox.showerror("Validate", error)

    def _draw_validate_graphs(self):
        desired_canvas = getattr(self, "validate_desired_canvas", None)
        measured_canvas = getattr(self, "validate_measured_canvas", None)
        if desired_canvas is not None and desired_canvas.winfo_exists():
            self._draw_series(desired_canvas, self.target_accel, "Desired acceleration (m/s2)", ACCEL_COLOR)
        if measured_canvas is not None and measured_canvas.winfo_exists():
            self._draw_series(measured_canvas, self.rt_accel, "Measured acceleration from IMU (m/s2)", ACTUAL_COLOR)

    def _export_recorded_accel(self):
        self._export_csv({"accel_x_mps2": self.rt_accel}, "recorded_accel")

    def _export_table(self):
        if not self.rows:
            messagebox.showwarning("Export", "No table to export.")
            return
        columns = ["index", "time_s", "accel_mps2", "steps", "feedrate", "direction", "position_cm", "unclamped_position_cm"]
        data = {key: [row[key] for row in self.rows] for key in columns}
        self._export_csv(data, "motion_table")

    def _export_csv(self, columns, default_name):
        if not columns:
            return
        path = filedialog.asksaveasfilename(
            title="Export CSV",
            defaultextension=".csv",
            initialfile=f"{default_name}.csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return
        headers = list(columns.keys())
        values = list(columns.values())
        count = max((len(value) for value in values), default=0)
        with open(path, "w", newline="", encoding="utf-8") as file_obj:
            writer = csv.writer(file_obj)
            writer.writerow(headers)
            for index in range(count):
                writer.writerow([column[index] if index < len(column) else "" for column in values])

    def _schedule(self, callback, ms):
        after_id = self.root.after(ms, callback)
        self._after_ids.append(after_id)
        return after_id

    def _on_close(self):
        for after_id in self._after_ids:
            try:
                self.root.after_cancel(after_id)
            except Exception:
                pass
        self.motor.disconnect()
        self.accel.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    ShakingTableApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
