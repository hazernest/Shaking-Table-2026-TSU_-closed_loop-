import argparse
import csv
import importlib
import math
import queue
import threading
import time
import tkinter as tk
from collections import deque
from pathlib import Path
from tkinter import filedialog
from tkinter import messagebox
from tkinter import ttk

import numpy as np
import serial
from serial.tools import list_ports


DEFAULT_BAUDRATE = 115200
DEFAULT_TIMEOUT = 2.0
DEFAULT_CONNECT_DELAY = 2.0
PREFERRED_PORT = "COM9"          # Motor MCU (Arduino Mega)
PREFERRED_ACCEL_PORT = ""         # Accel MCU (Arduino Uno) -- set to e.g. "COM10"
GRAPH_HISTORY = 180
GRAPH_ACCEL_RANGE = 2.0
GRAVITY_MS2 = 9.80665
GRAPH_ACCEL_RANGE_MS2 = GRAPH_ACCEL_RANGE * GRAVITY_MS2
RECORD_HISTORY = 600
RECORD_FILTER_DEFAULT_WINDOW = 5
RECORD_FILTER_MAX_WINDOW = 40
EXCEL_SAMPLE_INTERVAL_S = 0.03
CMPS2_TO_MPS2 = 0.01
LEAD_MM_PER_REV = 2.0
PULSES_PER_REV = 400
SAFE_ACCEL_LIMIT_MPS2 = 4.0
STROKE_LIMIT_M = 0.10
RESONANCE_MIN_SAMPLES = 64
RESONANCE_MIN_HZ = 0.5
RESONANCE_MAX_HZ = 25.0
RESONANCE_NOTCH_Q = 3.0
HIGHPASS_FILTER_DEFAULT_HZ = 0.05
HIGHPASS_FILTER_MIN_HZ = 0.01
HIGHPASS_FILTER_MAX_HZ = 0.50
STABILITY_WINDOW_SAMPLES = 25
STABILITY_MIN_SAMPLES = 60
STABILITY_STD_THRESHOLD_MS2 = 0.12
STABILITY_SPAN_THRESHOLD_MS2 = 0.35
STABILITY_CONFIRM_WINDOWS = 4
ARDUINO_QUEUE_SIZE = 16
SEND_LOOKAHEAD_S = 0.05          # send each command this many seconds before it's needed
QUEUE_POLL_INTERVAL_S = 0.05
REPLAY_UI_UPDATE_INTERVAL_S = 0.2
MAX_IMPORT_FEEDRATE_STEPS_S = 15000  # hard cap on steps/s for imported replay (~37.5 mm/s @ 2mm/rev 400spr)


def parse_imu_line(line):
    parts = line.split(",")
    if len(parts) != 10 or parts[0] != "IMU":
        return None

    try:
        values = [float(value) for value in parts[1:]]
    except ValueError:
        return None

    return {
        "gyro_x": values[0],
        "gyro_y": values[1],
        "gyro_z": values[2],
        "accel_x": values[3],
        "accel_y": values[4],
        "accel_z": values[5],
        "roll": values[6],
        "pitch": values[7],
        "yaw": values[8],
    }


def validate_command_values(steps_text, direction_text, feedrate_text, frequency_text):
    try:
        steps = int(steps_text)
        direction = int(direction_text)
        feedrate = int(feedrate_text)
        frequency = int(frequency_text)
    except ValueError as exc:
        raise ValueError("Steps, direction, feedrate, and frequency must be integers.") from exc

    if steps <= 0:
        raise ValueError("Steps must be greater than 0.")
    if direction not in (-1, 0, 1):
        raise ValueError("Direction must be -1, 0, or 1.")
    if feedrate <= 0:
        raise ValueError("Feedrate must be greater than 0.")
    if frequency <= 0:
        raise ValueError("Frequency must be greater than 0.")

    return steps, direction, feedrate, frequency


def build_command(steps, direction, feedrate):
    return f"{steps},{direction},{feedrate}\n"


def get_available_ports():
    ports = []
    for port in list_ports.comports():
        description = port.description or "Serial Port"
        ports.append(f"{port.device} {description}")

    ports.sort(key=lambda port_label: (not port_label.startswith(PREFERRED_PORT), port_label))
    return ports


def extract_port_device(port_label):
    if not port_label:
        return ""

    return port_label.split(" ", 1)[0]


def get_default_port_label(ports):
    for port_label in ports:
        if port_label.startswith(PREFERRED_PORT):
            return port_label

    return ports[0] if ports else ""


class ArduinoSerialClient:
    def __init__(self, port, baudrate=DEFAULT_BAUDRATE, timeout=DEFAULT_TIMEOUT):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.connection = None
        self.connection_lock = threading.Lock()
        self.reader_thread = None
        self.reader_running = False
        self.telemetry_callback = None
        self.message_callback = None
        self.queue_status_callback = None
        self.ok_count = 0          # incremented each time Arduino sends 'ok'

    def connect(self, telemetry_callback=None, message_callback=None, queue_status_callback=None):
        if self.connection and self.connection.is_open:
            self.telemetry_callback = telemetry_callback
            self.message_callback = message_callback
            self.queue_status_callback = queue_status_callback
            return

        self.telemetry_callback = telemetry_callback
        self.message_callback = message_callback
        self.queue_status_callback = queue_status_callback
        self.connection = serial.Serial(self.port, self.baudrate, timeout=0.1)
        self.connection.reset_input_buffer()
        self.connection.reset_output_buffer()
        time.sleep(DEFAULT_CONNECT_DELAY)
        self.connection.reset_input_buffer()
        self.reader_running = True
        self.reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.reader_thread.start()

    def disconnect(self):
        self.reader_running = False

        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)

        with self.connection_lock:
            if self.connection and self.connection.is_open:
                self.connection.close()

        self.connection = None
        self.reader_thread = None

    def is_connected(self):
        return self.connection is not None and self.connection.is_open

    def _read_loop(self):
        while self.reader_running:
            try:
                # Only lock to grab a reference; readline() runs lock-free so
                # send_command / request_queue_status are never blocked by a read.
                with self.connection_lock:
                    if not self.connection or not self.connection.is_open:
                        return
                    conn = self.connection

                raw_line = conn.readline()

                if not raw_line:
                    continue

                line = raw_line.decode(errors="ignore").strip()
                if not line:
                    continue

                telemetry = parse_imu_line(line)
                if telemetry is not None:
                    if self.telemetry_callback:
                        self.telemetry_callback(telemetry)
                    continue

                if line.startswith("QFREE,"):
                    parts = line.split(",")
                    if len(parts) >= 3:
                        try:
                            queue_free = int(parts[1])
                            is_active = int(parts[2])
                            if self.queue_status_callback:
                                self.queue_status_callback(queue_free, is_active)
                        except ValueError:
                            pass
                    continue

                if line.startswith("ENQ,"):
                    val = line[4:].strip()
                    if val == "FULL":
                        if self.queue_status_callback:
                            self.queue_status_callback(0, 1)
                    else:
                        try:
                            queue_free = int(val)
                            if self.queue_status_callback:
                                self.queue_status_callback(queue_free, 1)
                        except ValueError:
                            pass
                    continue

                if line == "ok":
                    self.ok_count += 1
                    continue

                if self.message_callback:
                    self.message_callback(f"Serial: {line}")
            except Exception as exc:
                if self.message_callback:
                    self.message_callback(f"Serial read error: {exc}")
                self.reader_running = False

    def send_command(self, steps, direction, feedrate):
        if not self.is_connected():
            raise RuntimeError("Serial port is not connected.")

        command = build_command(steps, direction, feedrate)
        with self.connection_lock:
            self.connection.write(command.encode("ascii"))
            self.connection.flush()
        return command.strip()

    def request_queue_status(self):
        if not self.is_connected():
            raise RuntimeError("Serial port is not connected.")

        with self.connection_lock:
            self.connection.write(b"Q?\n")
            self.connection.flush()

    def send_repeated_command(self, steps, direction, feedrate, frequency, on_message=None):
        temporary_connection = False
        if not self.is_connected():
            self.connect(message_callback=on_message)
            temporary_connection = True

        try:
            for index in range(frequency):
                # Wait for a free queue slot before sending (timeout 10 s)
                deadline = time.monotonic() + 10.0
                while self.queue_status_callback is not None:
                    # queue_status_callback updates queue_free_slots on the app side;
                    # we can't read it here directly, so request a status and yield
                    break  # app-level guard handled below; just proceed

                current_direction = get_direction_for_repeat(direction, index)
                command = self.send_command(steps, current_direction, feedrate)

                if on_message:
                    on_message(f"{index + 1}/{frequency} sent dir={current_direction}: {command}")
        finally:
            if temporary_connection:
                self.disconnect()


def get_direction_for_repeat(initial_direction, repeat_index):
    if initial_direction == 0:
        return 0

    return initial_direction if repeat_index % 2 == 0 else -initial_direction


def run_cli():
    ports = get_available_ports()
    if ports:
        print("Available ports:")
        for port in ports:
            print(f"  {port}")
    else:
        print("No serial ports detected automatically.")

    default_port = get_default_port_label(ports)
    port_input = input(f"Enter Arduino port [{default_port}]: ").strip()
    port = extract_port_device(port_input or default_port)
    baud_text = input(f"Enter baudrate [{DEFAULT_BAUDRATE}]: ").strip() or str(DEFAULT_BAUDRATE)
    steps_text = input("Enter steps: ").strip()
    direction_text = input("Enter direction (-1, 0, 1): ").strip()
    feedrate_text = input("Enter feedrate: ").strip()
    frequency_text = input("Enter frequency (repeat count): ").strip()

    try:
        baudrate = int(baud_text)
        steps, direction, feedrate, frequency = validate_command_values(
            steps_text, direction_text, feedrate_text, frequency_text
        )
        client = ArduinoSerialClient(port, baudrate=baudrate)
        client.send_repeated_command(
            steps,
            direction,
            feedrate,
            frequency,
            on_message=lambda message: print(message),
        )
    except Exception as exc:
        print(f"Error: {exc}")


class SerialSenderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Shaking Table Controller")
        self.root.geometry("1300x900")
        self.log_queue = queue.Queue()
        self.telemetry_queue = queue.Queue()
        self.client = None          # motor MCU (Arduino Mega)
        self.accel_client = None    # accel MCU (Arduino Uno)
        self.accel_history = deque(maxlen=GRAPH_HISTORY)
        self.record_accel_history = deque(maxlen=RECORD_HISTORY)
        self.record_time_history = deque(maxlen=RECORD_HISTORY)
        self.stability_window = deque(maxlen=STABILITY_WINDOW_SAMPLES)
        self.latest_telemetry = None
        self.recording_active = False
        self.send_in_progress = False
        self.waiting_for_stable = False
        self.stability_confirm_count = 0
        self.recording_started_at = None
        self.resonance_freq_hz = None
        self.resonance_magnitude = None
        self.imported_accel_history = []
        self.imported_accel_raw = []  # Raw data before baseline correction
        self.imported_sample_interval_s = EXCEL_SAMPLE_INTERVAL_S
        self._base_sample_interval_s = EXCEL_SAMPLE_INTERVAL_S  # never mutated; skip multiplies from this
        self.import_preview_window = None
        self.import_graph_window = None
        self.import_generated_rows = []
        self.import_merged_commands = []
        self.queue_free_slots = 0
        self.stepper_active_flag = 0
        self.import_sender_running = False
        self.import_sender_row_index = 0   # current row being executed (for graph progress line)
        self.last_telemetry_draw_time = 0.0

        self.port_var = tk.StringVar()
        self.accel_port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUDRATE))
        self.steps_var = tk.StringVar()
        self.direction_var = tk.StringVar(value="1")
        self.feedrate_var = tk.StringVar()
        self.frequency_var = tk.StringVar(value="1")
        self.connection_var = tk.StringVar(value="Motor: Disconnected")
        self.accel_connection_var = tk.StringVar(value="Accel: Disconnected")
        self.gyro_var = tk.StringVar(value="Gyro: --, --, --")
        self.accel_var = tk.StringVar(value="Accel: --, --, -- m/s^2")
        self.angle_var = tk.StringVar(value="Angle: --, --, --")
        self.record_summary_var = tk.StringVar(value="Send stepper command to record acceleration.")
        self.record_filter_enabled_var = tk.BooleanVar(value=False)
        self.record_filter_window_var = tk.IntVar(value=RECORD_FILTER_DEFAULT_WINDOW)
        self.input_shaping_mode_var = tk.IntVar(value=0)  # 0 = raw commands, 1 = input shaping
        self.highpass_cutoff_var = tk.DoubleVar(value=HIGHPASS_FILTER_DEFAULT_HZ)
        self.highpass_enabled_var = tk.BooleanVar(value=True)
        self.baseline_correction_enabled_var = tk.BooleanVar(value=True)
        self.max_displacement_cm_var = tk.DoubleVar(value=10.0)  # Max displacement in cm
        self.data_skip_var = tk.IntVar(value=1)          # use every Nth sample (1 = no skip)
        self.resonance_info_var = tk.StringVar(value="Resonance: not measured")
        self.show_imported_overlay_var = tk.BooleanVar(value=True)
        self._import_mode = 'accel'           # 'accel' or 'displacement'

        # ILC (Iterative Learning Control) state
        self._ilc_running = False
        self._ilc_stop_flag = [False]
        self._ilc_iteration = 0
        self._ilc_target_accel = []   # desired accel (original import, never mutated)
        self._ilc_current_accel = []  # input accel for current iteration
        self._ilc_history = []        # [(iteration, rmse), ...]
        self._ilc_status_var = tk.StringVar(value="Not started.")

        self._build_ui()
        self._refresh_ports()
        self._drain_log_queue()
        self._drain_telemetry_queue()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(250, self._auto_connect_default_port)

    def _build_ui(self):
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        self.notebook = ttk.Notebook(self.root)
        self.notebook.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)

        # Tab 0: IMU + Stepper monitor
        self.tab_monitor_frame = ttk.Frame(self.notebook, padding=12)
        self.notebook.add(self.tab_monitor_frame, text="  Monitor  ")

        # Tab 1: Imported earthquake graphs (populated on import)
        self.tab_graphs_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_graphs_frame, text="  Earthquake Graphs  ")
        ttk.Label(self.tab_graphs_frame,
                  text="Import an Excel file to populate this tab.",
                  foreground="#64748b").pack(padx=20, pady=40)

        # Tab 2: Data table (populated on import)
        self.tab_table_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_table_frame, text="  Data Table  ")
        ttk.Label(self.tab_table_frame,
                  text="Import an Excel file to populate this tab.",
                  foreground="#64748b").pack(padx=20, pady=40)

        # Tab 3: Correction Analysis (populated after each send)
        self.tab_correction_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_correction_frame, text="  Correction Analysis  ")
        ttk.Label(self.tab_correction_frame,
                  text="Run a send to populate this tab.",
                  foreground="#64748b").pack(padx=20, pady=40)

        frame = self.tab_monitor_frame
        frame.columnconfigure(0, weight=3)
        frame.columnconfigure(1, weight=2)
        frame.rowconfigure(0, weight=1)

        left_frame = ttk.LabelFrame(frame, text="IMU View", padding=10)
        left_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        left_frame.columnconfigure(0, weight=1)
        left_frame.rowconfigure(1, weight=1)
        left_frame.rowconfigure(3, weight=1)

        ttk.Label(left_frame, textvariable=self.gyro_var).grid(row=0, column=0, sticky="w", pady=(0, 4))
        self.orientation_canvas = tk.Canvas(left_frame, width=480, height=280, bg="#11161c", highlightthickness=0)
        self.orientation_canvas.grid(row=1, column=0, sticky="nsew")
        ttk.Label(left_frame, textvariable=self.accel_var).grid(row=2, column=0, sticky="w", pady=(8, 4))
        self.graph_canvas = tk.Canvas(left_frame, width=480, height=220, bg="#f5f7fa", highlightthickness=0)
        self.graph_canvas.grid(row=3, column=0, sticky="nsew")
        ttk.Label(left_frame, textvariable=self.angle_var).grid(row=4, column=0, sticky="w", pady=(8, 0))

        right_frame = ttk.LabelFrame(frame, text="Stepper Control", padding=10)
        right_frame.grid(row=0, column=1, sticky="nsew")
        right_frame.columnconfigure(1, weight=1)
        right_frame.rowconfigure(7, weight=1)
        right_frame.rowconfigure(10, weight=1)

        ttk.Label(right_frame, text="Motor MCU").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=4)
        self.port_combo = ttk.Combobox(right_frame, textvariable=self.port_var, state="normal")
        self.port_combo.grid(row=0, column=1, sticky="ew", pady=4)
        ttk.Button(right_frame, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, padx=(8, 0), pady=4)

        ttk.Label(right_frame, text="Accel MCU").grid(row=1, column=0, sticky="w", padx=(0, 8), pady=4)
        self.accel_port_combo = ttk.Combobox(right_frame, textvariable=self.accel_port_var, state="normal")
        self.accel_port_combo.grid(row=1, column=1, sticky="ew", pady=4)
        ttk.Button(right_frame, text="Refresh", command=self._refresh_ports).grid(row=1, column=2, padx=(8, 0), pady=4)

        ttk.Label(right_frame, text="Baudrate").grid(row=2, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Entry(right_frame, textvariable=self.baud_var).grid(row=2, column=1, columnspan=2, sticky="ew", pady=4)

        ttk.Label(right_frame, text="Steps").grid(row=3, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Entry(right_frame, textvariable=self.steps_var).grid(row=3, column=1, columnspan=2, sticky="ew", pady=4)

        ttk.Label(right_frame, text="Direction").grid(row=4, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Combobox(
            right_frame,
            textvariable=self.direction_var,
            values=("-1", "0", "1"),
            state="readonly",
        ).grid(row=4, column=1, columnspan=2, sticky="ew", pady=4)

        ttk.Label(right_frame, text="Feedrate").grid(row=5, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Entry(right_frame, textvariable=self.feedrate_var).grid(row=5, column=1, columnspan=2, sticky="ew", pady=4)

        ttk.Label(right_frame, text="Frequency").grid(row=6, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Entry(right_frame, textvariable=self.frequency_var).grid(row=6, column=1, columnspan=2, sticky="ew", pady=4)

        status_frame = ttk.Frame(right_frame)
        status_frame.grid(row=7, column=0, columnspan=3, sticky="ew", pady=(6, 6))
        status_frame.columnconfigure(0, weight=1)
        ttk.Label(status_frame, textvariable=self.connection_var).grid(row=0, column=0, sticky="w")
        ttk.Label(status_frame, textvariable=self.accel_connection_var).grid(row=1, column=0, sticky="w", pady=(2, 0))
        btn_frame = ttk.Frame(status_frame)
        btn_frame.grid(row=0, column=1, rowspan=2, sticky="e", padx=(8, 0))
        ttk.Button(btn_frame, text="Connect Motor", command=self._connect_motor_port).grid(row=0, column=0, padx=(0, 4))
        ttk.Button(btn_frame, text="Connect Accel MCU", command=self._connect_accel_port).grid(row=0, column=1, padx=(0, 4))
        ttk.Button(btn_frame, text="Disconnect All", command=self._disconnect_serial).grid(row=0, column=2)

        self.log_text = tk.Text(right_frame, height=12, width=42, state="disabled")
        self.log_text.grid(row=8, column=0, columnspan=3, sticky="nsew", pady=(2, 8))

        ttk.Button(right_frame, text="Send", command=self._start_send_thread).grid(row=9, column=0, columnspan=3, sticky="ew")

        record_frame = ttk.LabelFrame(right_frame, text="Accel Record (during send)", padding=8)
        record_frame.grid(row=10, column=0, columnspan=3, sticky="nsew", pady=(8, 0))
        record_frame.columnconfigure(0, weight=1)
        record_frame.rowconfigure(0, weight=1)

        self.record_canvas = tk.Canvas(record_frame, height=180, bg="#f8f9fb", highlightthickness=0)
        self.record_canvas.grid(row=0, column=0, sticky="nsew")

        filter_controls = ttk.Frame(record_frame)
        filter_controls.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        filter_controls.columnconfigure(1, weight=1)
        filter_controls.columnconfigure(3, weight=1)

        ttk.Checkbutton(
            filter_controls,
            text="Post-filter",
            variable=self.record_filter_enabled_var,
            command=self._on_record_filter_changed,
        ).grid(row=0, column=0, sticky="w", padx=(0, 8))

        ttk.Scale(
            filter_controls,
            from_=1,
            to=RECORD_FILTER_MAX_WINDOW,
            variable=self.record_filter_window_var,
            command=self._on_record_filter_slider_changed,
        ).grid(row=0, column=1, sticky="ew")

        ttk.Label(filter_controls, textvariable=self.record_filter_window_var, width=3).grid(row=0, column=2, sticky="e", padx=(8, 0))

        # Input shaping mode selection
        shaping_frame = ttk.LabelFrame(filter_controls, text="Command Mode", padding=(8, 4))
        shaping_frame.grid(row=0, column=3, sticky="w", padx=(12, 0))
        
        ttk.Radiobutton(
            shaping_frame,
            text="Raw",
            variable=self.input_shaping_mode_var,
            value=0,
            command=self._on_record_filter_changed,
        ).grid(row=0, column=0, sticky="w", padx=(0, 8))
        
        ttk.Radiobutton(
            shaping_frame,
            text="Input Shaping",
            variable=self.input_shaping_mode_var,
            value=1,
            command=self._on_record_filter_changed,
        ).grid(row=0, column=1, sticky="w")

        ttk.Checkbutton(
            filter_controls,
            text="Show imported",
            variable=self.show_imported_overlay_var,
            command=self._on_record_filter_changed,
        ).grid(row=0, column=4, sticky="w", padx=(12, 0))

        action_controls = ttk.Frame(record_frame)
        action_controls.grid(row=2, column=0, sticky="ew", pady=(6, 0))
        action_controls.columnconfigure(3, weight=1)

        ttk.Button(action_controls, text="Import Accel Data", command=self._import_accel_file).grid(row=0, column=0, sticky="w")
        ttk.Button(action_controls, text="Import Displacement Data", command=self._import_displacement_file).grid(row=0, column=1, sticky="w", padx=(6, 0))
        ttk.Button(action_controls, text="Measure resonance", command=self._measure_resonance).grid(row=0, column=2, sticky="w", padx=(8, 0))
        ttk.Label(action_controls, textvariable=self.resonance_info_var).grid(row=0, column=3, sticky="w", padx=(8, 0))
        ttk.Button(
            action_controls, text="Export CSV",
            command=lambda: self._export_graph_csv(
                {"time_s": [t for t in self.record_time_history],
                 "accel_x_mps2": [a for a in self.record_accel_history]},
                default_name="recorded_accel"
            )
        ).grid(row=0, column=4, sticky="e", padx=(8, 0))

        ttk.Label(record_frame, textvariable=self.record_summary_var).grid(row=3, column=0, sticky="w", pady=(6, 0))

        self._draw_orientation_view()
        self._draw_accel_graph()
        self._draw_record_graph()

    def _refresh_ports(self):
        ports = get_available_ports()
        self.port_combo["values"] = ports
        self.accel_port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(get_default_port_label(ports))
        if PREFERRED_ACCEL_PORT and self.accel_port_var.get() not in ports:
            for p in ports:
                if p.startswith(PREFERRED_ACCEL_PORT):
                    self.accel_port_var.set(p)
                    break

    def _auto_connect_default_port(self):
        if not (self.client and self.client.is_connected()):
            if self.port_var.get().strip():
                self._connect_motor_port()
        if not (self.accel_client and self.accel_client.is_connected()):
            if self.accel_port_var.get().strip():
                self._connect_accel_port()

    def _connect_motor_port(self):
        port_label = self.port_var.get().strip()
        baudrate_str = self.baud_var.get().strip()
        threading.Thread(
            target=self._do_connect_motor,
            args=(port_label, baudrate_str),
            daemon=True,
        ).start()

    def _do_connect_motor(self, port_label, baudrate_str):
        try:
            port = extract_port_device(port_label)
            baudrate = int(baudrate_str)
            if not port:
                raise ValueError("Motor MCU port is required.")
            if self.client and self.client.is_connected():
                if self.client.port == port and self.client.baudrate == baudrate:
                    return
                self._disconnect_motor()
            self.root.after(0, lambda: self.connection_var.set(f"Motor: Connecting {port}..."))
            self.client = ArduinoSerialClient(port, baudrate=baudrate)
            self.client.connect(
                telemetry_callback=None,
                message_callback=self._queue_log,
                queue_status_callback=self._queue_status_update,
            )
            try:
                self.client.request_queue_status()
            except Exception:
                pass
            self.root.after(0, lambda: self.connection_var.set(f"Motor: {port}"))
            self._queue_log(f"Motor MCU connected: {port}")
        except Exception as exc:
            self.root.after(0, lambda: self.connection_var.set("Motor: Disconnected"))
            self._queue_log(f"Motor connect error: {exc}")

    def _connect_accel_port(self):
        port_label = self.accel_port_var.get().strip()
        baudrate_str = self.baud_var.get().strip()
        threading.Thread(
            target=self._do_connect_accel,
            args=(port_label, baudrate_str),
            daemon=True,
        ).start()

    def _do_connect_accel(self, port_label, baudrate_str):
        try:
            port = extract_port_device(port_label)
            baudrate = int(baudrate_str)
            if not port:
                raise ValueError("Accel MCU port is required.")
            if self.accel_client and self.accel_client.is_connected():
                if self.accel_client.port == port and self.accel_client.baudrate == baudrate:
                    return
                self._disconnect_accel()
            self.root.after(0, lambda: self.accel_connection_var.set(f"Accel: Connecting {port}..."))
            self.accel_client = ArduinoSerialClient(port, baudrate=baudrate)
            self.accel_client.connect(
                telemetry_callback=self._queue_telemetry,
                message_callback=self._queue_log,
                queue_status_callback=None,
            )
            self.root.after(0, lambda: self.accel_connection_var.set(f"Accel: {port}"))
            self._queue_log(f"Accel MCU connected: {port}")
        except Exception as exc:
            self.root.after(0, lambda: self.accel_connection_var.set("Accel: Disconnected"))
            self._queue_log(f"Accel connect error: {exc}")

    # Keep old name so any remaining call sites still work
    def _connect_selected_port(self):
        self._connect_motor_port()

    def _disconnect_motor(self):
        if self.client:
            self.client.disconnect()
        self.client = None
        self.connection_var.set("Motor: Disconnected")
        self._queue_log("Motor MCU disconnected")

    def _disconnect_accel(self):
        if self.accel_client:
            self.accel_client.disconnect()
        self.accel_client = None
        self.accel_connection_var.set("Accel: Disconnected")
        self._queue_log("Accel MCU disconnected")

    def _disconnect_serial(self):
        """Disconnect both clients (used by window close and legacy call sites)."""
        self._disconnect_motor()
        self._disconnect_accel()

    def _append_log(self, message):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"{message}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _queue_log(self, message):
        self.log_queue.put(message)

    def _queue_telemetry(self, telemetry):
        self.telemetry_queue.put(telemetry)

    def _queue_status_update(self, queue_free, is_active):
        self.queue_free_slots = max(0, int(queue_free))
        self.stepper_active_flag = 1 if int(is_active) else 0

    def _drain_log_queue(self):
        while not self.log_queue.empty():
            self._append_log(self.log_queue.get())
        self._after_log_id = self.root.after(100, self._drain_log_queue)

    def _drain_telemetry_queue(self):
        updated = False
        while not self.telemetry_queue.empty():
            telemetry = self.telemetry_queue.get()
            self.latest_telemetry = telemetry
            accel_x_ms2 = telemetry["accel_x"] * GRAVITY_MS2
            accel_y_ms2 = telemetry["accel_y"] * GRAVITY_MS2
            accel_z_ms2 = telemetry["accel_z"] * GRAVITY_MS2
            self.accel_history.append(
                (
                    accel_x_ms2,
                    accel_y_ms2,
                    accel_z_ms2,
                )
            )

            if self.recording_active:
                if self.recording_started_at is None:
                    self.recording_started_at = time.monotonic()
                elapsed = time.monotonic() - self.recording_started_at
                self.record_time_history.append(elapsed)
                self.record_accel_history.append(accel_x_ms2)
                self._evaluate_record_stability(accel_x_ms2)

            updated = True

        if updated:
            self._update_telemetry_labels()
            now = time.monotonic()
            if self.import_sender_running:
                # During active send: only draw the record graph (cheapest canvas)
                if now - self.last_telemetry_draw_time >= 0.2:
                    self.last_telemetry_draw_time = now
                    self._draw_record_graph()
            else:
                draw_interval = 0.05
                if now - self.last_telemetry_draw_time >= draw_interval:
                    self.last_telemetry_draw_time = now
                    self._draw_orientation_view()
                    self._draw_accel_graph()
                    self._draw_record_graph()

            # Feed real-time graph in import graph window � only while sending
            if self.import_sender_running and getattr(self, '_rt_graph_active', False) and self.latest_telemetry:
                accel_x = self.latest_telemetry["accel_x"] * GRAVITY_MS2
                if not hasattr(self, '_rt_graph_history'):
                    self._rt_graph_history = []
                self._rt_graph_history.append(accel_x)
                # Keep at most 2000 samples in history
                if len(self._rt_graph_history) > 2000:
                    self._rt_graph_history = self._rt_graph_history[-2000:]
                rt_canvas = getattr(self, '_rt_canvas_ref', None)
                if rt_canvas and rt_canvas.winfo_exists() and now - getattr(self, '_last_rt_draw', 0) >= 0.1:
                    self._last_rt_draw = now
                    self._draw_rt_graph(rt_canvas)

        self._after_telemetry_id = self.root.after(50, self._drain_telemetry_queue)

    def _update_telemetry_labels(self):
        if not self.latest_telemetry:
            return

        telemetry = self.latest_telemetry
        self.gyro_var.set(
            f"Gyro: {telemetry['gyro_x']:.3f}, {telemetry['gyro_y']:.3f}, {telemetry['gyro_z']:.3f} deg/s"
        )
        self.accel_var.set(
            "Accel: "
            f"{telemetry['accel_x'] * GRAVITY_MS2:.3f}, "
            f"{telemetry['accel_y'] * GRAVITY_MS2:.3f}, "
            f"{telemetry['accel_z'] * GRAVITY_MS2:.3f} m/s^2"
        )
        self.angle_var.set(
            f"Angle: roll {telemetry['roll']:.2f}, pitch {telemetry['pitch']:.2f}, yaw {telemetry['yaw']:.2f}"
        )

    def _rotate_point(self, point, roll_rad, pitch_rad, yaw_rad):
        x_value, y_value, z_value = point

        cos_roll = math.cos(roll_rad)
        sin_roll = math.sin(roll_rad)
        y_roll = y_value * cos_roll - z_value * sin_roll
        z_roll = y_value * sin_roll + z_value * cos_roll

        cos_pitch = math.cos(pitch_rad)
        sin_pitch = math.sin(pitch_rad)
        x_pitch = x_value * cos_pitch + z_roll * sin_pitch
        z_pitch = -x_value * sin_pitch + z_roll * cos_pitch

        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        x_yaw = x_pitch * cos_yaw - y_roll * sin_yaw
        y_yaw = x_pitch * sin_yaw + y_roll * cos_yaw
        return x_yaw, y_yaw, z_pitch

    def _project_point(self, point, width, height, scale=90.0):
        x_value, y_value, z_value = point
        depth = 3.5 + z_value
        projected_x = width / 2 + (x_value * scale) / depth
        projected_y = height / 2 - (y_value * scale) / depth
        return projected_x, projected_y

    def _draw_orientation_view(self):
        canvas = self.orientation_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 480)
        height = max(canvas.winfo_height(), 280)

        canvas.create_text(12, 12, text="3D orientation", anchor="nw", fill="#dfe7ef", font=("TkDefaultFont", 11, "bold"))

        if not self.latest_telemetry:
            canvas.create_text(width / 2, height / 2, text="Waiting for IMU data", fill="#9ba8b5")
            return

        roll_rad = math.radians(self.latest_telemetry["roll"])
        pitch_rad = math.radians(self.latest_telemetry["pitch"])
        yaw_rad = math.radians(self.latest_telemetry["yaw"])

        cube_points = [
            (-1, -1, -1),
            (1, -1, -1),
            (1, 1, -1),
            (-1, 1, -1),
            (-1, -1, 1),
            (1, -1, 1),
            (1, 1, 1),
            (-1, 1, 1),
        ]
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]

        rotated_points = [self._rotate_point(point, roll_rad, pitch_rad, yaw_rad) for point in cube_points]
        projected_points = [self._project_point(point, width, height) for point in rotated_points]

        for start_index, end_index in edges:
            start_x, start_y = projected_points[start_index]
            end_x, end_y = projected_points[end_index]
            canvas.create_line(start_x, start_y, end_x, end_y, fill="#7cd1ff", width=2)

        accel_vector = (
            self.latest_telemetry["accel_x"],
            self.latest_telemetry["accel_y"],
            self.latest_telemetry["accel_z"],
        )
        vector_end = self._rotate_point(accel_vector, roll_rad, pitch_rad, yaw_rad)
        start_x, start_y = self._project_point((0.0, 0.0, 0.0), width, height)
        end_x, end_y = self._project_point(vector_end, width, height, scale=70.0)
        canvas.create_line(start_x, start_y, end_x, end_y, fill="#ff8f6b", width=3, arrow="last")

    def _draw_accel_graph(self):
        canvas = self.graph_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 480)
        height = max(canvas.winfo_height(), 220)
        margin = 24

        canvas.create_rectangle(0, 0, width, height, fill="#f5f7fa", outline="")
        canvas.create_text(12, 12, text="Compensated acceleration (m/s^2)", anchor="nw", fill="#243242", font=("TkDefaultFont", 11, "bold"))
        canvas.create_line(margin, height / 2, width - margin, height / 2, fill="#ccd6e0")
        canvas.create_line(margin, margin, margin, height - margin, fill="#ccd6e0")

        if len(self.accel_history) < 2:
            canvas.create_text(width / 2, height / 2, text="Waiting for IMU data", fill="#657381")
            return

        def to_points(axis_index):
            points = []
            usable_width = width - (2 * margin)
            usable_height = height - (2 * margin)
            for index, sample in enumerate(self.accel_history):
                x_pos = margin + (usable_width * index / max(len(self.accel_history) - 1, 1))
                normalized = max(-GRAPH_ACCEL_RANGE_MS2, min(GRAPH_ACCEL_RANGE_MS2, sample[axis_index])) / GRAPH_ACCEL_RANGE_MS2
                y_pos = margin + (usable_height / 2) - (normalized * usable_height / 2)
                points.extend((x_pos, y_pos))
            return points

        canvas.create_line(*to_points(0), fill="#d04a4a", width=2, smooth=True)
        canvas.create_line(*to_points(1), fill="#2b8a3e", width=2, smooth=True)
        canvas.create_line(*to_points(2), fill="#2463eb", width=2, smooth=True)
        canvas.create_text(width - margin, margin, text="ax", anchor="ne", fill="#d04a4a")
        canvas.create_text(width - margin, margin + 16, text="ay", anchor="ne", fill="#2b8a3e")
        canvas.create_text(width - margin, margin + 32, text="az", anchor="ne", fill="#2463eb")

    def _draw_record_graph(self):
        canvas = self.record_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 420)
        height = max(canvas.winfo_height(), 180)
        margin = 24

        canvas.create_rectangle(0, 0, width, height, fill="#f8f9fb", outline="")
        canvas.create_text(
            12,
            12,
            text="Recorded accel X (m/s^2)",
            anchor="nw",
            fill="#1f2937",
            font=("TkDefaultFont", 10, "bold"),
        )

        display_samples = self._get_record_display_samples()
        if len(display_samples) < 2:
            if self.recording_active and self.send_in_progress:
                message = "Recording while stepper command is running..."
            elif self.recording_active and self.waiting_for_stable:
                message = "Waiting for acceleration to stabilize..."
            elif self.recording_active:
                message = "Recording... waiting for telemetry"
            else:
                message = "No record yet"
            canvas.create_text(width / 2, height / 2, text=message, fill="#6b7280")
            return

        samples = display_samples
        min_value = min(samples)
        max_value = max(samples)
        min_index = samples.index(min_value)
        max_index = samples.index(max_value)

        y_min = min(0.0, min_value)
        y_max = max_value
        if abs(y_max - y_min) < 0.001:
            y_max = y_min + 1.0

        usable_width = width - (2 * margin)
        usable_height = height - (2 * margin)

        def to_xy(index, value):
            x_pos = margin + usable_width * index / max(len(samples) - 1, 1)
            normalized = (value - y_min) / (y_max - y_min)
            y_pos = height - margin - normalized * usable_height
            return x_pos, y_pos

        points = []
        for index, value in enumerate(samples):
            x_pos, y_pos = to_xy(index, value)
            points.extend((x_pos, y_pos))

        canvas.create_line(margin, height - margin, width - margin, height - margin, fill="#d1d5db")
        canvas.create_line(margin, margin, margin, height - margin, fill="#d1d5db")
        canvas.create_line(*points, fill="#2563eb", width=2, smooth=True)

        if self.show_imported_overlay_var.get() and len(self.imported_accel_history) >= 2:
            imported_points = []
            imported_count = len(self.imported_accel_history)
            for index, value in enumerate(self.imported_accel_history):
                x_pos = margin + usable_width * index / max(imported_count - 1, 1)
                normalized = (value - y_min) / (y_max - y_min)
                y_pos = height - margin - normalized * usable_height
                imported_points.extend((x_pos, y_pos))

            canvas.create_line(*imported_points, fill="#f59e0b", width=2, dash=(4, 3), smooth=True)
            canvas.create_text(width - margin, height - margin - 10, text="imported", anchor="se", fill="#b45309")

        min_x, min_y = to_xy(min_index, min_value)
        max_x, max_y = to_xy(max_index, max_value)

        canvas.create_oval(min_x - 4, min_y - 4, min_x + 4, min_y + 4, fill="#16a34a", outline="")
        canvas.create_text(min_x + 6, min_y - 10, text=f"min {min_value:.2f}", anchor="w", fill="#166534")

        canvas.create_oval(max_x - 4, max_y - 4, max_x + 4, max_y + 4, fill="#dc2626", outline="")
        canvas.create_text(max_x + 6, max_y - 10, text=f"max {max_value:.2f}", anchor="w", fill="#991b1b")

    def _get_record_display_samples(self):
        samples = list(self.record_accel_history)
        if not samples:
            return samples

        if not self.record_filter_enabled_var.get():
            return samples

        window = max(1, int(round(self.record_filter_window_var.get())))
        if window <= 1:
            return samples

        filtered_samples = []
        rolling_window = deque()
        rolling_sum = 0.0

        for value in samples:
            rolling_window.append(value)
            rolling_sum += value

            if len(rolling_window) > window:
                rolling_sum -= rolling_window.popleft()

            filtered_samples.append(rolling_sum / len(rolling_window))

        samples = filtered_samples

        if self.input_shaping_mode_var.get() == 1 and self.resonance_freq_hz is not None:
            return self._apply_notch_filter(samples, self.resonance_freq_hz, RESONANCE_NOTCH_Q)

        return samples

    def _estimate_record_sample_rate(self):
        times = list(self.record_time_history)
        if len(times) >= 2:
            span = times[-1] - times[0]
            if span > 0:
                return (len(times) - 1) / span

        return 1.0 / 0.03

    def _apply_notch_filter(self, samples, resonance_hz, q_factor):
        if len(samples) < 3:
            return samples

        sample_rate = self._estimate_record_sample_rate()
        if sample_rate <= 0 or resonance_hz <= 0 or resonance_hz >= (sample_rate * 0.49):
            return samples

        w0 = 2.0 * math.pi * resonance_hz / sample_rate
        alpha = math.sin(w0) / (2.0 * q_factor)
        cos_w0 = math.cos(w0)

        b0 = 1.0
        b1 = -2.0 * cos_w0
        b2 = 1.0
        a0 = 1.0 + alpha
        a1 = -2.0 * cos_w0
        a2 = 1.0 - alpha

        b0 /= a0
        b1 /= a0
        b2 /= a0
        a1 /= a0
        a2 /= a0

        output = []
        x1 = 0.0
        x2 = 0.0
        y1 = 0.0
        y2 = 0.0
        for x0 in samples:
            y0 = b0 * x0 + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2
            output.append(y0)
            x2 = x1
            x1 = x0
            y2 = y1
            y1 = y0

        return output

    def _measure_resonance(self):
        samples = list(self.record_accel_history)
        if len(samples) < RESONANCE_MIN_SAMPLES:
            self.resonance_freq_hz = None
            self.resonance_magnitude = None
            self.resonance_info_var.set("Resonance: need more recorded samples")
            self._draw_record_graph()
            return

        sample_rate = self._estimate_record_sample_rate()
        if sample_rate <= 0:
            self.resonance_info_var.set("Resonance: invalid sample rate")
            return

        centered = [value - (sum(samples) / len(samples)) for value in samples]
        n_samples = len(centered)
        max_bin = n_samples // 2
        best_freq = None
        best_mag = -1.0

        for k_value in range(1, max_bin):
            freq = (k_value * sample_rate) / n_samples
            if freq < RESONANCE_MIN_HZ or freq > min(RESONANCE_MAX_HZ, sample_rate * 0.49):
                continue

            real_part = 0.0
            imag_part = 0.0
            for n_index, sample in enumerate(centered):
                angle = (2.0 * math.pi * k_value * n_index) / n_samples
                real_part += sample * math.cos(angle)
                imag_part -= sample * math.sin(angle)

            magnitude = math.sqrt(real_part * real_part + imag_part * imag_part)
            if magnitude > best_mag:
                best_mag = magnitude
                best_freq = freq

        self.resonance_freq_hz = best_freq
        self.resonance_magnitude = best_mag if best_mag >= 0 else None

        if self.resonance_freq_hz is None:
            self.resonance_info_var.set("Resonance: no dominant peak found")
        else:
            self.resonance_info_var.set(
                f"Resonance: {self.resonance_freq_hz:.2f} Hz (Q={RESONANCE_NOTCH_Q:.1f})"
            )

        self._draw_record_graph()

    def _on_record_filter_changed(self):
        self._draw_record_graph()

    def _on_record_filter_slider_changed(self, _value):
        self._draw_record_graph()
    
    def _on_data_skip_changed(self, file_name, rows):
        self.data_skip_var.set(max(1, int(float(self.data_skip_var.get()))))
        self._schedule_reprocess()

    def _on_import_highpass_changed(self, file_name, original_rows):
        current_val = self.highpass_cutoff_var.get()
        self.highpass_cutoff_var.set(round(current_val, 2))
        self._schedule_reprocess()

    def _on_max_displacement_changed(self, file_name, original_rows):
        current_val = self.max_displacement_cm_var.get()
        self.max_displacement_cm_var.set(round(current_val, 1))
        self._schedule_reprocess()

    def _schedule_reprocess(self):
        """Debounce: cancel any pending reprocess and schedule a new one 350 ms out."""
        if hasattr(self, '_reprocess_after_id') and self._reprocess_after_id is not None:
            self.root.after_cancel(self._reprocess_after_id)
        self._reprocess_after_id = self.root.after(350, self._reprocess_imported_data)

    def _reprocess_imported_data(self):
        """Kick off background reprocessing; returns immediately so UI stays live."""
        self._reprocess_after_id = None
        if not self.imported_accel_raw:
            return
        # Snapshot all tk vars on the main thread before handing off
        skip = max(1, int(self.data_skip_var.get()))
        max_disp_m = self.max_displacement_cm_var.get() / 100.0
        use_shaping = self.input_shaping_mode_var.get() == 1 and self.resonance_freq_hz is not None
        threading.Thread(
            target=self._reprocess_bg,
            args=(skip, max_disp_m, use_shaping),
            daemon=True,
        ).start()

    def _reprocess_bg(self, skip, max_disp_m, use_shaping):
        """Background thread: full reprocess pipeline � no Tkinter calls here."""
        raw = self.imported_accel_raw[::skip]
        effective_dt = self._base_sample_interval_s * skip
        # temporarily update dt so all helpers use the right value
        self.imported_sample_interval_s = effective_dt

        if self._import_mode == 'displacement':
            # Displacement mode: no filtering, no FFT — use data directly
            self.imported_accel_history = raw
            new_rows = self._generate_motion_table_from_displacement(raw)
        else:
            corrected = self._apply_baseline_correction(raw.copy())
            if use_shaping:
                corrected = self._apply_input_shaping(corrected, self.resonance_freq_hz, effective_dt)

            scale = self._calculate_displacement_scale_factor(corrected, max_disp_m)
            scaled = [a * scale for a in corrected]

            self.imported_accel_history = scaled
            new_rows = self._generate_motion_table_from_accel(scaled)
        self.import_graph_current_rows = new_rows
        self.import_generated_rows = new_rows
        new_merged = self._merge_generated_commands(new_rows)
        self.import_merged_commands = new_merged

        # Hand results back to the main thread for UI updates
        self.root.after(0, lambda: self._reprocess_apply_ui(new_rows))
    
    def _reprocess_apply_ui(self, new_rows):
        """Main thread: redraw graphs, update treeview and stats label after background reprocess."""
        _input_series = ([r["unclamped_position_cm"] for r in new_rows]
                         if self._import_mode == 'displacement'
                         else [r["accel_mps2"] for r in new_rows])
        _input_label  = ("Displacement Input (cm)" if self._import_mode == 'displacement'
                         else "Acceleration (m/s\u00b2)")
        _input_color  = "#059669" if self._import_mode == 'displacement' else "#2563eb"
        self._import_graph_series = {
            "accel":     _input_series,
            "unclamped": [r["unclamped_position_cm"] for r in new_rows],
            "position":  [r["position_cm"]           for r in new_rows],
        }
        if hasattr(self, 'import_accel_canvas') and self.import_accel_canvas.winfo_exists():
            self._draw_import_series_graph(
                self.import_accel_canvas,
                self._import_graph_series["accel"],
                _input_label, _input_color,
            )
        if hasattr(self, 'import_position_canvas') and self.import_position_canvas.winfo_exists():
            self._draw_import_series_graph(
                self.import_position_canvas,
                self._import_graph_series["position"],
                "Position - Clamped to \u00b110 cm (cm)", "#dc2626",
            )
        if hasattr(self, '_redraw_accel'):
            self._redraw_accel()
        # Update stats label in preview window
        if hasattr(self, 'import_stats_var'):
            mc = self.import_merged_commands if hasattr(self, 'import_merged_commands') and self.import_merged_commands else []
            nonzero = sum(1 for r in new_rows if r["steps"] > 0)
            duration = new_rows[-1]["time_s"] if new_rows else 0.0
            self.import_stats_var.set(
                f"Rows: {len(new_rows)} | nonzero: {nonzero} | "
                f"merged: {len(mc)} | dt={self.imported_sample_interval_s:.3f}s | "
                f"duration: {duration:.1f}s"
            )
        self._update_import_table_window(new_rows)

    def _update_import_graph_progress(self, row_index, total_rows):
        """Redraw imported accel/displacement graph with progress line at row_index/total_rows."""
        if not hasattr(self, '_import_graph_series'):
            return
        frac = row_index / max(total_rows - 1, 1)
        _lbl = "Displacement Input (cm)" if self._import_mode == 'displacement' else "Acceleration (m/s\u00b2)"
        _col = "#059669"               if self._import_mode == 'displacement' else "#2563eb"
        if hasattr(self, 'import_accel_canvas') and self.import_accel_canvas.winfo_exists():
            self._draw_import_series_graph(
                self.import_accel_canvas,
                self._import_graph_series["accel"],
                _lbl, _col, progress_frac=frac,
            )
        if hasattr(self, 'import_position_canvas') and self.import_position_canvas.winfo_exists():
            self._draw_import_series_graph(
                self.import_position_canvas,
                self._import_graph_series["position"],
                "Position - Clamped to \u00b110 cm (cm)", "#dc2626", progress_frac=frac,
            )
        if hasattr(self, '_accel_sel'):
            # Restore selection rect on top of the redrawn graph
            s = self._accel_sel
            if s["x0"] is not None and s["x1"] is not None:
                canvas = self.import_accel_canvas
                h = max(canvas.winfo_height(), 200)
                if s["rect"]:
                    canvas.delete(s["rect"])
                s["rect"] = canvas.create_rectangle(
                    min(s["x0"], s["x1"]), 0, max(s["x0"], s["x1"]), h,
                    fill="#bfdbfe", outline="#3b82f6", stipple="gray50")

    def _calculate_displacement_scale_factor(self, accel_values_mps2, target_max_displacement_m):
        """Calculate scale factor to fit displacement within target max displacement"""
        # Use FFT method to get displacement
        displacement_m = self._accel_to_displacement_fft(accel_values_mps2)
        
        if not displacement_m:
            return 1.0
        
        # Find maximum displacement
        max_displacement = max(abs(d) for d in displacement_m)
        
        # If displacement is zero or very small, no scaling needed
        if max_displacement < 1e-6:
            return 1.0
        
        # Calculate scale factor
        scale_factor = target_max_displacement_m / max_displacement
        
        return scale_factor
    
    def _update_import_table_window(self, new_rows):
        """Update the import table window with new data using batched inserts."""
        if not hasattr(self, 'import_preview_window') or not self.import_preview_window:
            return
        if not self.import_preview_window.winfo_exists():
            return

        # Find the treeview
        tree = None
        for child in self.import_preview_window.winfo_children():
            if isinstance(child, ttk.Frame):
                for subchild in child.winfo_children():
                    if isinstance(subchild, ttk.Frame):
                        for item in subchild.winfo_children():
                            if isinstance(item, ttk.Treeview):
                                tree = item
                                break
        if tree is None:
            return

        tree.delete(*tree.get_children())
        win = self.import_preview_window

        def _batch(start):
            if not win.winfo_exists():
                return
            end = min(start + 300, len(new_rows))
            for r in new_rows[start:end]:
                tree.insert("", "end", values=(
                    r["index"], f"{r['time_s']:.2f}", f"{r['accel_cmps2']:.3f}",
                    f"{r['accel_mps2']:.4f}", r["steps"], r["feedrate"], r["direction"],
                    f"{r['position_cm']:.3f}", f"{r['unclamped_position_cm']:.3f}",
                ))
            if end < len(new_rows):
                win.after(1, lambda: _batch(end))

        _batch(0)

    def _import_displacement_file(self):
        """Import a displacement-vs-time file (first column in cm). Bypasses all FFT integration."""
        file_path = filedialog.askopenfilename(
            title="Open displacement file  (first column in cm)",
            filetypes=[
                ("Excel files", "*.xlsx *.xlsm"),
                ("CSV files", "*.csv"),
                ("All files", "*.*"),
            ],
        )
        if not file_path:
            return
        self._import_mode = 'displacement'
        self.record_summary_var.set("Loading displacement file\u2026")
        threading.Thread(
            target=self._import_displacement_bg_load,
            args=(file_path,),
            daemon=True,
        ).start()

    def _import_displacement_bg_load(self, file_path):
        """Background: load displacement (cm) and convert to metres."""
        try:
            # Reuse accel loader — it multiplies by 0.01, which is cm→m (same factor as cm/s²→m/s²)
            displacement_m = self._load_accel_series_from_file(file_path)
        except Exception as exc:
            self.root.after(0, lambda: messagebox.showerror("Import error", str(exc)))
            self.root.after(0, lambda: self.record_summary_var.set(""))
            return
        if len(displacement_m) < 2:
            self.root.after(0, lambda: messagebox.showerror(
                "Import error", "Need at least 2 numeric rows in the first data column."))
            self.root.after(0, lambda: self.record_summary_var.set(""))
            return

        # Zero-reference: subtract first sample so motion starts at current position
        d0 = displacement_m[0]
        displacement_m = [d - d0 for d in displacement_m]

        # Scale down if it exceeds stroke limit
        max_d = max(abs(d) for d in displacement_m)
        if max_d > STROKE_LIMIT_M:
            scale = (STROKE_LIMIT_M * 0.95) / max_d
            displacement_m = [d * scale for d in displacement_m]
            status_msg = (f"Imported {len(displacement_m)} displacement samples from "
                          f"{Path(file_path).name} (scaled {scale:.3f}\u00d7, "
                          f"dt={self.imported_sample_interval_s:.3f}s)")
        else:
            status_msg = (f"Imported {len(displacement_m)} displacement samples from "
                          f"{Path(file_path).name} (dt={self.imported_sample_interval_s:.3f}s)")

        # Store raw displacement (m) so reprocess/skip can access it
        self.imported_accel_raw = displacement_m
        self.imported_accel_history = displacement_m

        self.root.after(0, lambda: self.record_summary_var.set("Generating motion table\u2026"))
        threading.Thread(
            target=self._import_displacement_bg_generate,
            args=(file_path, displacement_m, status_msg),
            daemon=True,
        ).start()

    def _import_displacement_bg_generate(self, file_path, displacement_m, status_msg):
        generated_rows = self._generate_motion_table_from_displacement(displacement_m)
        merged_commands = self._merge_generated_commands(generated_rows)
        self.import_generated_rows = generated_rows
        self.import_merged_commands = merged_commands
        file_name = Path(file_path).name
        self.root.after(0, lambda: self._import_open_windows(file_name, generated_rows, merged_commands, status_msg))

    def _generate_motion_table_from_displacement(self, displacement_m_values):
        """Generate motion table directly from displacement (m). No integration needed."""
        dt = self.imported_sample_interval_s
        steps_per_meter = PULSES_PER_REV / (LEAD_MM_PER_REV / 1000.0)

        rows = []
        current_position_m = 0.0
        step_residual = 0.0

        for index, target_pos_m in enumerate(displacement_m_values):
            clamped = max(-STROKE_LIMIT_M, min(STROKE_LIMIT_M, target_pos_m))
            target_delta_m = clamped - current_position_m
            step_float = target_delta_m * steps_per_meter + step_residual
            signed_steps = int(round(step_float))
            step_residual = step_float - signed_steps

            if signed_steps > 0:
                direction = 1
                step_count = signed_steps
            elif signed_steps < 0:
                direction = -1
                step_count = -signed_steps
            else:
                direction = 0
                step_count = 0

            if step_count > 0:
                raw_feedrate = step_count / dt
                feedrate = max(1, min(MAX_IMPORT_FEEDRATE_STEPS_S, int(round(raw_feedrate))))
                actual_delta_m = signed_steps / steps_per_meter
                current_position_m += actual_delta_m
                current_position_m = max(-STROKE_LIMIT_M, min(STROKE_LIMIT_M, current_position_m))
            else:
                feedrate = 0

            rows.append({
                "index":                index,
                "time_s":               index * dt,
                "accel_cmps2":          0.0,
                "accel_mps2":           0.0,
                "steps":                step_count,
                "feedrate":             feedrate,
                "direction":            direction,
                "position_cm":          current_position_m * 100.0,
                "unclamped_position_cm": target_pos_m * 100.0,
            })

        return rows

    def _import_accel_file(self):
        """Import an acceleration-vs-time file (first column in cm/s\u00b2)."""
        self._import_mode = 'accel'
        file_path = filedialog.askopenfilename(
            title="Open acceleration file",
            filetypes=[
                ("Excel files", "*.xlsx *.xlsm"),
                ("CSV files", "*.csv"),
                ("All files", "*.*"),
            ],
        )
        if not file_path:
            return
        # Capture tk var on main thread before leaving
        use_shaping = self.input_shaping_mode_var.get() == 1
        self.record_summary_var.set("Loading file�")
        threading.Thread(
            target=self._import_bg_load,
            args=(file_path, use_shaping),
            daemon=True,
        ).start()

    def _import_bg_load(self, file_path, use_shaping):
        """Background thread: load file, baseline, FFT, optional shaping, scale factor."""
        try:
            imported_values = self._load_accel_series_from_file(file_path)
        except Exception as exc:
            self.root.after(0, lambda: messagebox.showerror("Import error", str(exc)))
            self.root.after(0, lambda: self.record_summary_var.set(""))
            return
        if len(imported_values) < 2:
            self.root.after(0, lambda: messagebox.showerror("Import error", "Need at least 2 numeric rows in the first data column."))
            self.root.after(0, lambda: self.record_summary_var.set(""))
            return

        raw = imported_values.copy()
        values = self._apply_baseline_correction(imported_values)

        initial_displacement_m = self._accel_to_displacement_fft(values)
        if initial_displacement_m:
            new_max = min(max(abs(d) for d in initial_displacement_m) * 100.0, 10.0)
        else:
            new_max = 10.0
        self.root.after(0, lambda v=new_max: self.max_displacement_cm_var.set(v))

        input_shaping_applied = use_shaping and self.resonance_freq_hz is not None
        if input_shaping_applied:
            values = self._apply_input_shaping(values, self.resonance_freq_hz, self.imported_sample_interval_s)

        scale_factor = self._calculate_optimal_scale_factor(values)
        self.root.after(0, lambda: self._import_main_dialog(file_path, raw, values, scale_factor, input_shaping_applied))

    def _import_main_dialog(self, file_path, raw, values, scale_factor, input_shaping_applied):
        """Main thread: optional scaling dialog, then kick off generation in background."""
        self.imported_accel_raw = raw
        if scale_factor < 1.0:
            response = messagebox.askyesno(
                "Amplitude Scaling Recommended",
                f"The earthquake displacement exceeds �{STROKE_LIMIT_M * 100:.0f}cm table limits.\n\n"
                f"Recommended scale factor: {scale_factor:.3f}\n"
                f"This will preserve the waveform shape while fitting within table stroke.\n\n"
                f"Original max accel: {max(abs(min(values)), abs(max(values))):.3f} m/s�\n"
                f"Scaled max accel: {max(abs(min(values)), abs(max(values))) * scale_factor:.3f} m/s�\n"
                f"{'Input shaping applied for ' + str(self.resonance_freq_hz) + ' Hz resonance' if input_shaping_applied else ''}\n\n"
                f"Apply scaling?"
            )
            if response:
                values = [a * scale_factor for a in values]
                status_msg = f"Imported {len(values)} samples from {Path(file_path).name} (scaled by {scale_factor:.3f}x"
            else:
                status_msg = f"Imported {len(values)} samples from {Path(file_path).name} (NO SCALING - will clip at �{STROKE_LIMIT_M * 100:.0f}cm"
            if input_shaping_applied:
                status_msg += f", input shaped for {self.resonance_freq_hz:.1f}Hz"
            status_msg += f", dt={self.imported_sample_interval_s:.2f}s)"
        else:
            status_msg = f"Imported {len(values)} samples from {Path(file_path).name} (fits within stroke limits"
            if input_shaping_applied:
                status_msg += f", input shaped for {self.resonance_freq_hz:.1f}Hz"
            status_msg += f", dt={self.imported_sample_interval_s:.2f}s)"

        self.imported_accel_history = values
        self.record_summary_var.set("Generating motion table�")
        threading.Thread(
            target=self._import_bg_generate,
            args=(file_path, values, status_msg),
            daemon=True,
        ).start()

    def _import_bg_generate(self, file_path, values, status_msg):
        """Background thread: generate motion table and merge commands."""
        generated_rows = self._generate_motion_table_from_accel(values)
        merged_commands = self._merge_generated_commands(generated_rows)
        self.import_generated_rows = generated_rows
        self.import_merged_commands = merged_commands
        file_name = Path(file_path).name
        self.root.after(0, lambda: self._import_open_windows(file_name, generated_rows, merged_commands, status_msg))

    def _import_open_windows(self, file_name, generated_rows, merged_commands, status_msg):
        """Main thread: set status, open preview + graph windows."""
        self.record_summary_var.set(status_msg)
        self._open_import_preview_window(file_name, generated_rows, merged_commands)
        self._draw_record_graph()

    def _calculate_optimal_scale_factor(self, accel_values_mps2):
        """
        Calculate the scaling factor needed to fit displacement within stroke limits.
        Uses FFT method to calculate displacement.
        Returns a value between 0 and 1.
        """
        # Use FFT method to get displacement
        displacement_m = self._accel_to_displacement_fft(accel_values_mps2)
        
        if not displacement_m:
            return 1.0
        
        # Find maximum displacement
        max_displacement = max(abs(d) for d in displacement_m)
        
        # If displacement fits, no scaling needed
        if max_displacement <= STROKE_LIMIT_M:
            return 1.0
        
        # Calculate required scale factor with 5% safety margin
        scale_factor = (STROKE_LIMIT_M * 0.95) / max_displacement
        
        return scale_factor

    def _load_accel_series_from_file(self, file_path):
        suffix = Path(file_path).suffix.lower()
        if suffix in {".xlsx", ".xlsm"}:
            return self._load_accel_series_from_excel(file_path)
        if suffix == ".csv":
            return self._load_accel_series_from_csv(file_path)

        raise ValueError("Unsupported file type. Use .xlsx, .xlsm, or .csv")

    def _load_accel_series_from_excel(self, file_path):
        try:
            openpyxl_module = importlib.import_module("openpyxl")
        except Exception:
            raise RuntimeError("openpyxl is required to import Excel files. Install with: pip install openpyxl")

        workbook = openpyxl_module.load_workbook(file_path, read_only=True, data_only=True)
        worksheet = workbook.active

        values = []
        for row in worksheet.iter_rows(min_col=1, max_col=1, values_only=True):
            if not row:
                continue

            raw_value = row[0]
            if raw_value is None:
                continue

            numeric_value = self._to_float(raw_value)
            if numeric_value is not None:
                values.append(numeric_value * CMPS2_TO_MPS2)

        workbook.close()
        return values

    def _load_accel_series_from_csv(self, file_path):
        values = []
        with open(file_path, "r", newline="", encoding="utf-8-sig") as file_obj:
            reader = csv.reader(file_obj)
            for row in reader:
                if not row:
                    continue

                numeric_value = self._to_float(row[0])
                if numeric_value is not None:
                    values.append(numeric_value * CMPS2_TO_MPS2)

        return values

    def _to_float(self, raw_value):
        if isinstance(raw_value, (int, float)):
            return float(raw_value)

        text = str(raw_value).strip()
        if not text:
            return None

        try:
            return float(text)
        except ValueError:
            return None

    def _apply_baseline_correction(self, accel_values):
        """
        Enhanced baseline correction to remove DC offset, polynomial drift, and 
        very low frequency components that cause position drift.
        
        This preserves earthquake frequency content (typically 0.1-20 Hz) while
        removing baseline drift that causes position to not return to zero.
        
        Steps:
        1. Remove mean (DC offset)
        2. Remove polynomial trend (quadratic drift)
        3. Apply high-pass filter (0.05 Hz) to remove very low frequency drift
        4. Apply post-integration correction to ensure zero final displacement
        """
        if not accel_values:
            return accel_values
        
        n = len(accel_values)
        
        # Step 1: Remove mean (DC offset)
        mean = sum(accel_values) / n
        detrended = [a - mean for a in accel_values]
        
        # Step 2: Remove polynomial trend (quadratic fit: y = ax� + bx + c)
        # This handles parabolic baseline drift better than linear
        if self.baseline_correction_enabled_var.get():
            corrected = self._remove_polynomial_trend(detrended)
        else:
            corrected = detrended
        
        # Step 3: Apply high-pass filter to remove very low frequencies
        # This removes drift without affecting earthquake content (typically > 0.1 Hz)
        if self.highpass_enabled_var.get():
            cutoff_hz = self.highpass_cutoff_var.get()
            corrected = self._apply_highpass_filter(corrected, cutoff_hz=cutoff_hz)
        
        # Step 4: Post-integration drift correction
        # Simulate integration to check final position, then remove residual drift
        if self.baseline_correction_enabled_var.get():
            corrected = self._remove_integration_drift(corrected)
        
        return corrected
        
        return corrected
    
    def _remove_polynomial_trend(self, accel_values):
        """
        Remove quadratic polynomial trend: y = ax� + bx + c
        Uses least squares fitting.
        """
        if len(accel_values) < 3:
            return accel_values
        
        n = len(accel_values)
        
        # Build normal equations for least squares
        # We need to solve: [X^T X] * [a, b, c]^T = X^T * y
        sum_x = sum(range(n))
        sum_x2 = sum(i * i for i in range(n))
        sum_x3 = sum(i * i * i for i in range(n))
        sum_x4 = sum(i * i * i * i for i in range(n))
        
        sum_y = sum(accel_values)
        sum_xy = sum(i * accel_values[i] for i in range(n))
        sum_x2y = sum(i * i * accel_values[i] for i in range(n))
        
        # Solve using Cramer's rule (simplified for 3x3 system)
        # For small datasets, numerical stability is okay
        try:
            # Determinant of coefficient matrix
            det = (n * sum_x2 * sum_x4 + 2 * sum_x * sum_x2 * sum_x3 - 
                   sum_x2 * sum_x2 * sum_x2 - n * sum_x3 * sum_x3 - sum_x4 * sum_x * sum_x)
            
            if abs(det) < 1e-10:
                # Matrix is singular, fall back to linear detrending
                m = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x)
                b = (sum_y - m * sum_x) / n
                return [accel_values[i] - (m * i + b) for i in range(n)]
            
            # Solve for coefficients a, b, c
            a = ((sum_x2y * (sum_x2 * n - sum_x * sum_x) + 
                  sum_xy * (sum_x * sum_x3 - sum_x2 * sum_x2) + 
                  sum_y * (sum_x2 * sum_x2 - sum_x3 * n)) / det)
            
            b = ((sum_x2y * (sum_x * sum_x - sum_x2 * n) + 
                  sum_xy * (sum_x4 * n - sum_x2 * sum_x2) + 
                  sum_y * (sum_x2 * sum_x2 - sum_x4 * sum_x)) / det)
            
            c = ((sum_x2y * (sum_x2 * sum_x - sum_x3 * sum_x) + 
                  sum_xy * (sum_x3 * sum_x - sum_x2 * sum_x2) + 
                  sum_y * (sum_x2 * sum_x2 - sum_x3 * sum_x)) / det)
            
            # Remove polynomial trend
            return [accel_values[i] - (a * i * i + b * i + c) for i in range(n)]
            
        except (ZeroDivisionError, OverflowError):
            # Fall back to linear detrending on numerical issues
            m = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x)
            b = (sum_y - m * sum_x) / n
            return [accel_values[i] - (m * i + b) for i in range(n)]
    
    def _apply_highpass_filter(self, accel_values, cutoff_hz):
        """
        Apply simple high-pass filter (1st order Butterworth) to remove 
        very low frequency drift without affecting earthquake frequencies.
        
        Cutoff at 0.05 Hz preserves all earthquake content (> 0.1 Hz).
        """
        if len(accel_values) < 2:
            return accel_values
        
        dt = self.imported_sample_interval_s
        sample_rate = 1.0 / dt
        
        # Calculate filter coefficient (1st order high-pass)
        RC = 1.0 / (2.0 * math.pi * cutoff_hz)
        alpha = RC / (RC + dt)
        
        # Apply filter: y[i] = alpha * (y[i-1] + x[i] - x[i-1])
        filtered = [accel_values[0]]  # First sample passes through
        
        for i in range(1, len(accel_values)):
            filtered_value = alpha * (filtered[i-1] + accel_values[i] - accel_values[i-1])
            filtered.append(filtered_value)
        
        return filtered
    
    def _remove_integration_drift(self, accel_values):
        """
        Remove residual drift by simulating double integration, measuring
        final position/velocity, and removing a corrective linear ramp.
        
        This ensures position returns to zero at the end while preserving
        the earthquake's frequency content.
        
        Mathematical approach:
        - Add correction: a_corr(t) = a_0 + a_1*t
        - After integration: v(T) = a_0*T + a_1*T�/2
        - After 2nd integration: x(T) = a_0*T�/2 + a_1*T�/6
        - Solve for a_0, a_1 such that final v=0, x=0
        """
        if len(accel_values) < 2:
            return accel_values
        
        dt = self.imported_sample_interval_s
        
        # Simulate double integration
        velocity = 0.0
        position = 0.0
        
        for accel in accel_values:
            velocity += accel * dt
            position += velocity * dt
        
        final_velocity = velocity
        final_position = position
        
        # If final position/velocity are small enough, no correction needed
        if abs(final_position) < 0.001 and abs(final_velocity) < 0.001:
            return accel_values
        
        n = len(accel_values)
        total_time = n * dt
        T = total_time
        
        # Solve for correction coefficients:
        # a_0*T + a_1*T�/2 = -v_f
        # a_0*T�/2 + a_1*T�/6 = -x_f
        # 
        # Solution:
        # a_1 = 12*x_f/T� - 6*v_f/T�
        # a_0 = -6*x_f/T� + 2*v_f/T
        
        a_1 = 12.0 * final_position / (T ** 3) - 6.0 * final_velocity / (T ** 2)
        a_0 = -6.0 * final_position / (T ** 2) + 2.0 * final_velocity / T
        
        # Apply correction: a_corr(t) = a_0 + a_1*t
        corrected = []
        for i, accel in enumerate(accel_values):
            t = i * dt
            correction = a_0 + a_1 * t
            corrected.append(accel + correction)
        
        return corrected

    def _apply_input_shaping(self, accel_values, resonance_hz, sample_interval):
        """
        Apply Zero Vibration (ZV) input shaping to suppress resonance.
        This modifies the commanded acceleration to cancel vibrations at the resonant frequency.
        
        Input shaping convolves the command with a shaped impulse sequence that cancels
        residual vibrations at the system's natural frequency.
        """
        if not accel_values or resonance_hz is None or resonance_hz <= 0:
            return accel_values
        
        # ZV shaper parameters
        omega_n = 2.0 * math.pi * resonance_hz  # Natural frequency (rad/s)
        zeta = 1.0 / (2.0 * RESONANCE_NOTCH_Q)  # Damping ratio from Q factor
        omega_d = omega_n * math.sqrt(1.0 - zeta * zeta)  # Damped frequency
        
        # ZV shaper impulse times and amplitudes
        T_v = math.pi / omega_d  # Time between impulses
        K = math.exp(-zeta * omega_n * T_v)  # Decay factor
        
        # Normalized amplitudes
        A1 = 1.0 / (1.0 + K)
        A2 = K / (1.0 + K)
        
        # Convert time to samples
        samples_delay = int(round(T_v / sample_interval))
        
        if samples_delay <= 0 or samples_delay >= len(accel_values):
            return accel_values
        
        # Convolve with ZV shaper: output[n] = A1*input[n] + A2*input[n-delay]
        shaped = []
        for i in range(len(accel_values)):
            if i < samples_delay:
                # Before delay, only first impulse contributes
                shaped.append(A1 * accel_values[i])
            else:
                # After delay, both impulses contribute
                shaped.append(A1 * accel_values[i] + A2 * accel_values[i - samples_delay])
        
        return shaped

    def _accel_to_displacement_fft(self, accel_values_mps2):
        """
        Convert acceleration to displacement using FFT method.
        This is more accurate than double integration as it naturally removes DC drift.
        
        Mathematical relationship:
        X(f) = -A(f) / (2pf)�
        
        Where:
        - X(f) = displacement in frequency domain
        - A(f) = acceleration in frequency domain
        - f = frequency
        
        Uses numpy's fast FFT for O(n log n) performance.
        """
        if not accel_values_mps2:
            return []
        
        n = len(accel_values_mps2)
        dt = self.imported_sample_interval_s
        sample_rate = 1.0 / dt
        
        # Convert to numpy array
        accel_array = np.array(accel_values_mps2)
        
        # Perform FFT on acceleration
        accel_fft = np.fft.fft(accel_array)
        
        # Get frequency bins
        freqs = np.fft.fftfreq(n, dt)
        
        # Convert acceleration to displacement in frequency domain
        # X(f) = -A(f) / (2pf)�
        disp_fft = np.zeros_like(accel_fft, dtype=complex)
        
        # Skip DC component (f=0) and very low frequencies to avoid division by zero
        # Frequencies below 0.05 Hz are typically drift, not earthquake content
        mask = np.abs(freqs) >= 0.05
        omega = 2.0 * np.pi * freqs[mask]
        disp_fft[mask] = -accel_fft[mask] / (omega * omega)
        
        # Perform IFFT to get displacement in time domain
        displacement_array = np.fft.ifft(disp_fft).real

        # -- Critical: zero the starting position --------------------------------
        # The IFFT result can have any initial value. If displacement[0] != 0,
        # the motion table will try to jump from position 0 to displacement[0]
        # in one dt interval, producing an enormous feedrate that faults the driver.
        displacement_array -= displacement_array[0]

        return displacement_array.tolist()
    
    def _generate_motion_table_from_accel(self, accel_values_mps2):
        """
        Generate motion table from acceleration using FFT-based displacement calculation.
        This replaces the double integration method with a more accurate frequency domain approach.
        """
        dt = self.imported_sample_interval_s
        steps_per_meter = PULSES_PER_REV / (LEAD_MM_PER_REV / 1000.0)
        
        # Convert acceleration to displacement using FFT method
        unclamped_displacement_m = self._accel_to_displacement_fft(accel_values_mps2)
        
        rows = []
        current_position_m = 0.0
        step_residual = 0.0

        for index, accel_value in enumerate(accel_values_mps2):
            # Get target displacement from FFT calculation
            target_position_m = unclamped_displacement_m[index]
            
            # Apply acceleration limiting (still useful for safety)
            limited_accel = max(-SAFE_ACCEL_LIMIT_MPS2, min(SAFE_ACCEL_LIMIT_MPS2, accel_value))
            
            # Clamp position to stroke limits
            clamped_position_m = target_position_m
            if clamped_position_m > STROKE_LIMIT_M:
                clamped_position_m = STROKE_LIMIT_M
            elif clamped_position_m < -STROKE_LIMIT_M:
                clamped_position_m = -STROKE_LIMIT_M
            
            # Calculate step command
            target_delta_m = clamped_position_m - current_position_m
            step_float = target_delta_m * steps_per_meter + step_residual
            signed_steps = int(round(step_float))
            step_residual = step_float - signed_steps

            if signed_steps > 0:
                direction = 1
                step_count = signed_steps
            elif signed_steps < 0:
                direction = -1
                step_count = -signed_steps
            else:
                direction = 0
                step_count = 0

            if step_count > 0:
                raw_feedrate = step_count / dt
                # Cap feedrate to prevent driver faults from FFT artefacts or large single-step deltas
                feedrate = max(1, min(MAX_IMPORT_FEEDRATE_STEPS_S, int(round(raw_feedrate))))
                actual_delta_m = signed_steps / steps_per_meter
                current_position_m += actual_delta_m
                if current_position_m > STROKE_LIMIT_M:
                    current_position_m = STROKE_LIMIT_M
                elif current_position_m < -STROKE_LIMIT_M:
                    current_position_m = -STROKE_LIMIT_M
            else:
                feedrate = 0

            rows.append(
                {
                    "index": index,
                    "time_s": index * dt,
                    "accel_cmps2": accel_value / CMPS2_TO_MPS2,
                    "accel_mps2": limited_accel,
                    "steps": step_count,
                    "feedrate": feedrate,
                    "direction": direction,
                    "position_cm": current_position_m * 100.0,
                    "unclamped_position_cm": target_position_m * 100.0,
                }
            )

        return rows

    def _merge_generated_commands(self, rows):
        merged = []

        for row in rows:
            if row["steps"] <= 0:
                continue

            if not merged:
                merged.append(
                    {
                        "steps": row["steps"],
                        "direction": row["direction"],
                        "feedrate": row["feedrate"],
                        "row_start": row["index"],
                        "row_end": row["index"],
                    }
                )
                continue

            last = merged[-1]
            if (
                row["direction"] == last["direction"]
                and row["feedrate"] == last["feedrate"]
                and row["index"] == last["row_end"] + 1
            ):
                last["steps"] += row["steps"]
                last["row_end"] = row["index"]
            else:
                merged.append(
                    {
                        "steps": row["steps"],
                        "direction": row["direction"],
                        "feedrate": row["feedrate"],
                        "row_start": row["index"],
                        "row_end": row["index"],
                    }
                )

        return merged

    def _open_import_preview_window(self, file_name, rows, merged_commands):
        # Repopulate the Data Table tab in-place
        for w in self.tab_table_frame.winfo_children():
            w.destroy()

        window = self.tab_table_frame
        self.import_preview_window = self.tab_table_frame

        outer = ttk.Frame(window, padding=10)
        outer.grid(row=0, column=0, sticky="nsew")
        window.columnconfigure(0, weight=1)
        window.rowconfigure(0, weight=1)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(1, weight=1)

        header_text = (
            f"Generated from accel X (dt={self.imported_sample_interval_s:.2f}s, "
            f"lead={LEAD_MM_PER_REV:.1f} mm/rev, pulse/rev={PULSES_PER_REV}, "
            f"a_limit={SAFE_ACCEL_LIMIT_MPS2:.2f} m/s^2, stroke=+/-{STROKE_LIMIT_M * 100:.1f} cm)"
        )
        ttk.Label(outer, text=header_text).grid(row=0, column=0, sticky="w", pady=(0, 8))

        table_frame = ttk.Frame(outer)
        table_frame.grid(row=1, column=0, sticky="nsew")
        table_frame.columnconfigure(0, weight=1)
        table_frame.rowconfigure(0, weight=1)

        columns = (
            "index",
            "time_s",
            "accel_cmps2",
            "accel_mps2",
            "steps",
            "feedrate",
            "direction",
            "position_cm",
            "unclamped_position_cm",
        )
        tree = ttk.Treeview(table_frame, columns=columns, show="headings", height=18)
        tree.grid(row=0, column=0, sticky="nsew")

        headings = {
            "index": "i",
            "time_s": "time (s)",
            "accel_cmps2": "accel (cm/s^2)",
            "accel_mps2": "accel (m/s^2)",
            "steps": "step",
            "feedrate": "feedrate",
            "direction": "dir",
            "position_cm": "pos (cm)",
            "unclamped_position_cm": "actual pos (cm)",
        }

        widths = {
            "index": 60,
            "time_s": 90,
            "accel_cmps2": 130,
            "accel_mps2": 120,
            "steps": 90,
            "feedrate": 100,
            "direction": 60,
            "position_cm": 100,
            "unclamped_position_cm": 120,
        }

        for column in columns:
            tree.heading(column, text=headings[column])
            tree.column(column, width=widths[column], anchor="center", stretch=False)

        def _batch_insert(start):
            if not window.winfo_exists():
                return
            batch_end = min(start + 300, len(rows))
            for i in range(start, batch_end):
                r = rows[i]
                tree.insert(
                    "",
                    "end",
                    values=(
                        r["index"],
                        f"{r['time_s']:.2f}",
                        f"{r['accel_cmps2']:.3f}",
                        f"{r['accel_mps2']:.4f}",
                        r["steps"],
                        r["feedrate"],
                        r["direction"],
                        f"{r['position_cm']:.3f}",
                        f"{r['unclamped_position_cm']:.3f}",
                    ),
                )
            if batch_end < len(rows):
                window.after(1, lambda: _batch_insert(batch_end))

        _batch_insert(0)

        y_scroll = ttk.Scrollbar(table_frame, orient="vertical", command=tree.yview)
        y_scroll.grid(row=0, column=1, sticky="ns")
        tree.configure(yscrollcommand=y_scroll.set)

        nonzero_commands = sum(1 for row in rows if row["steps"] > 0)
        self.import_stats_var = tk.StringVar()
        self.import_stats_var.set(
            f"Rows: {len(rows)} | nonzero: {nonzero_commands} | "
            f"merged: {len(merged_commands)} | dt={self.imported_sample_interval_s:.3f}s | "
            f"duration: {rows[-1]['time_s']:.1f}s" if rows else "No data"
        )
        ttk.Label(outer, textvariable=self.import_stats_var).grid(row=2, column=0, sticky="w", pady=(8, 0))

        # Shared send state � graph tab will use the same vars & button lists
        self._send_status_var = tk.StringVar(value="Ready to send.")
        self._send_progress_var = tk.DoubleVar(value=0.0)
        self._send_stop_flag = [False]
        self._all_send_btns = []
        self._all_stop_btns = []

        bottom_frame = ttk.Frame(outer)
        bottom_frame.grid(row=3, column=0, sticky="ew", pady=(8, 0))
        bottom_frame.columnconfigure(3, weight=1)

        send_btn = ttk.Button(
            bottom_frame,
            text="Send to Arduino",
            command=lambda: self._start_import_send(
                self.import_generated_rows, self.import_merged_commands, tree
            ),
        )
        send_btn.grid(row=0, column=0, sticky="w")
        self._all_send_btns.append(send_btn)

        stop_btn = ttk.Button(
            bottom_frame,
            text="Stop",
            state="disabled",
            command=lambda: self._send_stop_flag.__setitem__(0, True),
        )
        stop_btn.grid(row=0, column=1, sticky="w", padx=(8, 0))
        self._all_stop_btns.append(stop_btn)

        export_btn = ttk.Button(
            bottom_frame,
            text="Export Table",
            command=lambda: self._export_generated_table(self.import_generated_rows, file_name, self._send_status_var),
        )
        export_btn.grid(row=0, column=2, sticky="w", padx=(8, 0))

        ttk.Label(bottom_frame, textvariable=self._send_status_var).grid(
            row=0, column=3, sticky="w", padx=(12, 0)
        )

        progress_bar = ttk.Progressbar(
            outer,
            variable=self._send_progress_var,
            maximum=100.0,
            length=400,
        )
        progress_bar.grid(row=4, column=0, sticky="ew", pady=(6, 0))

        self._open_import_graph_window(file_name, rows)

    def _open_import_graph_window(self, file_name, rows):
        # Repopulate the Earthquake Graphs tab in-place
        for w in self.tab_graphs_frame.winfo_children():
            w.destroy()

        graph_window = self.tab_graphs_frame
        self.import_graph_window = self.tab_graphs_frame

        outer = ttk.Frame(graph_window, padding=10)
        outer.grid(row=0, column=0, sticky="nsew")
        graph_window.columnconfigure(0, weight=1)
        graph_window.rowconfigure(0, weight=1)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(2, weight=1)  # imported accel
        outer.rowconfigure(4, weight=1)  # real-time accel
        outer.rowconfigure(6, weight=1)  # position graph

        # -- Controls ------------------------------------------------------------
        filter_frame = ttk.LabelFrame(outer, text="Controls", padding=(8, 4))
        filter_frame.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        filter_frame.columnconfigure(1, weight=1)
        filter_frame.columnconfigure(4, weight=1)

        if self._import_mode == 'accel':
            ttk.Label(filter_frame, text="High-pass (Hz):").grid(row=0, column=0, sticky="w", padx=(0, 4))
            ttk.Scale(filter_frame, from_=HIGHPASS_FILTER_MIN_HZ, to=HIGHPASS_FILTER_MAX_HZ,
                      variable=self.highpass_cutoff_var,
                      command=lambda v: self._on_import_highpass_changed(file_name, rows),
                      ).grid(row=0, column=1, sticky="ew")
            ttk.Label(filter_frame, textvariable=self.highpass_cutoff_var, width=5).grid(row=0, column=2, sticky="e", padx=(4, 16))
            ttk.Checkbutton(filter_frame, text="Enabled",
                            variable=self.highpass_enabled_var,
                            command=self._schedule_reprocess,
                            ).grid(row=0, column=3, sticky="w", padx=(0, 8))
            ttk.Label(filter_frame, text="\u2190 less drift removal | more \u2192", foreground="#64748b").grid(row=0, column=4, sticky="w")

            ttk.Label(filter_frame, text="Baseline correction:").grid(row=1, column=0, sticky="w", padx=(0, 4), pady=(4, 0))
            ttk.Checkbutton(filter_frame, text="Enabled  (polynomial detrend + integration drift removal)",
                            variable=self.baseline_correction_enabled_var,
                            command=self._schedule_reprocess,
                            ).grid(row=1, column=1, columnspan=4, sticky="w", pady=(4, 0))

            ttk.Label(filter_frame, text="Max displacement (cm):").grid(row=2, column=0, sticky="w", padx=(0, 4), pady=(4, 0))
            ttk.Scale(filter_frame, from_=0.5, to=10.0, variable=self.max_displacement_cm_var,
                      command=lambda v: self._on_max_displacement_changed(file_name, rows),
                      ).grid(row=2, column=1, sticky="ew", pady=(4, 0))
            ttk.Label(filter_frame, textvariable=self.max_displacement_cm_var, width=5).grid(row=2, column=2, sticky="e", padx=(4, 16), pady=(4, 0))
            ttk.Label(filter_frame, text="scales amplitude to fit stroke limit", foreground="#64748b").grid(row=2, column=3, sticky="w", pady=(4, 0))

            _skip_row = 3
        else:
            ttk.Label(filter_frame,
                      text="\u2705  Displacement input \u2014 no integration or filtering needed.",
                      foreground="#059669", font=("TkDefaultFont", 9, "bold"),
                      ).grid(row=0, column=0, columnspan=5, sticky="w", pady=(0, 4))
            _skip_row = 1

        ttk.Label(filter_frame, text="Data skip (every Nth):").grid(row=_skip_row, column=0, sticky="w", padx=(0, 4), pady=(4, 0))
        ttk.Scale(filter_frame, from_=1, to=20, variable=self.data_skip_var,
                  command=lambda v: self._on_data_skip_changed(file_name, rows),
                  ).grid(row=_skip_row, column=1, sticky="ew", pady=(4, 0))
        ttk.Label(filter_frame, textvariable=self.data_skip_var, width=5).grid(row=_skip_row, column=2, sticky="e", padx=(4, 16), pady=(4, 0))
        ttk.Label(filter_frame, text="1 = all samples  |  higher = fewer commands, coarser motion",
                  foreground="#64748b").grid(row=_skip_row, column=3, sticky="w", pady=(4, 0))

        # -- Send / Stop buttons -------------------------------------------------
        send_frame = ttk.Frame(outer)
        send_frame.grid(row=1, column=0, sticky="ew", pady=(0, 6))

        graph_send_btn = ttk.Button(send_frame, text="Send to Arduino",
                                    command=lambda: self._start_import_send(
                                        self.import_generated_rows, self.import_merged_commands,
                                        type('_T', (), {'get_children': lambda s: ()})()))
        graph_send_btn.grid(row=0, column=0)
        self._all_send_btns.append(graph_send_btn)

        graph_stop_btn = ttk.Button(send_frame, text="Stop", state="disabled",
                                    command=lambda: self._send_stop_flag.__setitem__(0, True))
        graph_stop_btn.grid(row=0, column=1, padx=(6, 0))
        self._all_stop_btns.append(graph_stop_btn)

        ttk.Button(send_frame, text="Manual Stepper", command=self._open_manual_stepper_window).grid(row=0, column=2, padx=(16, 0))
        ttk.Label(send_frame, textvariable=self._send_status_var, foreground="#374151").grid(row=0, column=3, sticky="w", padx=(12, 0))

        # -- Imported acceleration graph (with click-drag selection) ---------------------
        accel_lf_title = ("Imported Displacement (cm) \u2014 drag to select range"
                          if self._import_mode == 'displacement'
                          else "Imported Acceleration (m/s\u00b2) \u2014 drag to select range")
        accel_lf = ttk.LabelFrame(outer, text=accel_lf_title, padding=2)
        accel_lf.grid(row=2, column=0, sticky="nsew", pady=(0, 6))
        accel_lf.columnconfigure(0, weight=1)
        accel_lf.rowconfigure(0, weight=1)
        self.import_accel_canvas = tk.Canvas(accel_lf, height=200, bg="#f8fafc", highlightthickness=0)
        self.import_accel_canvas.grid(row=0, column=0, sticky="nsew")
        accel_canvas = self.import_accel_canvas

        # Export button overlay (bottom-right of canvas via place, drawn after canvas is mapped)
        def _place_accel_export_btn(event=None):
            if self._import_mode == 'displacement':
                _csv_key, _csv_name = "displacement_cm", "imported_displacement"
            else:
                _csv_key, _csv_name = "accel_mps2",      "imported_accel"
            ttk.Button(
                accel_lf, text="Export CSV",
                command=lambda k=_csv_key, n=_csv_name: self._export_graph_csv(
                    {k: list(self._import_graph_series.get("accel", []))},
                    default_name=n
                )
            ).place(relx=1.0, rely=1.0, anchor="se", x=-4, y=-4)
        accel_lf.after(100, _place_accel_export_btn)

        # Selection state for drag
        self._accel_sel = {"x0": None, "x1": None, "rect": None, "dragging": False}

        def _sel_start(event):
            self._accel_sel["x0"] = event.x
            self._accel_sel["x1"] = event.x
            self._accel_sel["dragging"] = True
            if self._accel_sel["rect"]:
                accel_canvas.delete(self._accel_sel["rect"])
            h = max(accel_canvas.winfo_height(), 200)
            self._accel_sel["rect"] = accel_canvas.create_rectangle(
                event.x, 0, event.x, h, fill="#bfdbfe", outline="#3b82f6", stipple="gray50")

        def _sel_drag(event):
            if not self._accel_sel["dragging"]:
                return
            self._accel_sel["x1"] = event.x
            h = max(accel_canvas.winfo_height(), 200)
            accel_canvas.coords(self._accel_sel["rect"],
                                min(self._accel_sel["x0"], event.x), 0,
                                max(self._accel_sel["x0"], event.x), h)

        def _sel_end(event):
            if not self._accel_sel["dragging"]:
                return
            self._accel_sel["x1"] = event.x
            self._accel_sel["dragging"] = False
            _show_selection_send()

        def _sel_clear(event=None):
            if self._accel_sel["rect"]:
                accel_canvas.delete(self._accel_sel["rect"])
                self._accel_sel["rect"] = None
            self._accel_sel["x0"] = self._accel_sel["x1"] = None
            # redraw without selection
            _redraw_accel()

        def _show_selection_send():
            x0 = self._accel_sel["x0"]
            x1 = self._accel_sel["x1"]
            if x0 is None or x1 is None or abs(x0 - x1) < 3:
                return
            w = max(accel_canvas.winfo_width(), 920)
            margin = 28
            usable = w - 2 * margin
            series = self._import_graph_series.get("accel", []) if hasattr(self, '_import_graph_series') else []
            n = len(series)
            if n < 2:
                return
            frac0 = max(0.0, (min(x0, x1) - margin) / usable)
            frac1 = min(1.0, (max(x0, x1) - margin) / usable)
            i0 = int(frac0 * (n - 1))
            i1 = int(frac1 * (n - 1))
            if i0 >= i1:
                return
            all_rows = self.import_generated_rows if hasattr(self, 'import_generated_rows') and self.import_generated_rows else []
            sel_rows = all_rows[i0:i1 + 1]
            sel_merged = self._merge_generated_commands(sel_rows)
            self._send_status_var.set(f"Selection: rows {i0}�{i1} ({len(sel_merged)} merged commands). Click Send to dispatch.")
            # Temporarily override commands for the send button
            self._graph_sel_rows = sel_rows
            self._graph_sel_merged = sel_merged

        def _redraw_accel(frac=None):
            if not accel_canvas.winfo_exists():
                return
            series = self._import_graph_series.get("accel", []) if hasattr(self, '_import_graph_series') else []
            if series:
                self._draw_import_series_graph(accel_canvas, series, "Acceleration (m/s�)", "#2563eb", progress_frac=frac)
            # Re-draw selection rect on top
            if self._accel_sel["x0"] is not None and self._accel_sel["x1"] is not None:
                h = max(accel_canvas.winfo_height(), 200)
                if self._accel_sel["rect"]:
                    accel_canvas.delete(self._accel_sel["rect"])
                self._accel_sel["rect"] = accel_canvas.create_rectangle(
                    min(self._accel_sel["x0"], self._accel_sel["x1"]), 0,
                    max(self._accel_sel["x0"], self._accel_sel["x1"]), h,
                    fill="#bfdbfe", outline="#3b82f6", stipple="gray50")
        self._redraw_accel = _redraw_accel

        accel_canvas.bind("<ButtonPress-1>", _sel_start)
        accel_canvas.bind("<B1-Motion>", _sel_drag)
        accel_canvas.bind("<ButtonRelease-1>", _sel_end)
        accel_canvas.bind("<Double-Button-1>", _sel_clear)

        sel_info = ttk.Label(outer, text="Double-click graph to clear selection", foreground="#64748b",
                             font=("TkDefaultFont", 8))
        sel_info.grid(row=3, column=0, sticky="w")

        # Override graph send button to use selection if available
        def _graph_send_smart():
            if hasattr(self, '_graph_sel_merged') and self._graph_sel_merged:
                rw = getattr(self, '_graph_sel_rows', self.import_generated_rows or [])
                mc = self._graph_sel_merged
                self._send_status_var.set(f"Sending selection ({len(mc)} commands)�")
            else:
                rw = self.import_generated_rows if hasattr(self, 'import_generated_rows') and self.import_generated_rows else []
                mc = self.import_merged_commands if hasattr(self, 'import_merged_commands') and self.import_merged_commands else []
            if not mc:
                self._send_status_var.set("No commands � adjust sliders first.")
                return
            fake_tree = type('_T', (), {'get_children': lambda s: ()})()            
            self._start_import_send(rw, mc, fake_tree)
        graph_send_btn.configure(command=_graph_send_smart)

        # -- Real-time acceleration graph from Accel MCU ---------------------------
        rt_lf = ttk.LabelFrame(outer, text="Real-time Acceleration from Accel MCU (m/s�)", padding=2)
        rt_lf.grid(row=4, column=0, sticky="nsew")
        rt_lf.columnconfigure(0, weight=1)
        rt_lf.rowconfigure(0, weight=1)
        self.import_rt_canvas = tk.Canvas(rt_lf, height=200, bg="#f8fafc", highlightthickness=0)
        self.import_rt_canvas.grid(row=0, column=0, sticky="nsew")
        rt_canvas = self.import_rt_canvas

        rt_btn_frame = ttk.Frame(outer)
        rt_btn_frame.grid(row=5, column=0, sticky="ew", pady=(4, 0))
        rt_btn_frame.columnconfigure(1, weight=1)
        ttk.Button(rt_btn_frame, text="Clear Real-time Graph",
                   command=lambda: self._clear_rt_graph(rt_canvas)).grid(row=0, column=0)
        ttk.Button(
            rt_btn_frame, text="Export CSV",
            command=lambda: self._export_graph_csv(
                {"accel_x_mps2": list(self._rt_graph_history)},
                default_name="realtime_accel"
            )
        ).grid(row=0, column=2, sticky="e")

        # -- Position graph ------------------------------------------------------
        pos_lf = ttk.LabelFrame(outer, text="Position � Clamped to \u00b110 cm", padding=2)
        pos_lf.grid(row=6, column=0, sticky="nsew", pady=(6, 0))
        pos_lf.columnconfigure(0, weight=1)
        pos_lf.rowconfigure(0, weight=1)
        self.import_position_canvas = tk.Canvas(pos_lf, height=200, bg="#f8fafc", highlightthickness=0)
        self.import_position_canvas.grid(row=0, column=0, sticky="nsew")
        pos_canvas = self.import_position_canvas

        def _place_pos_export_btn():
            ttk.Button(
                pos_lf, text="Export CSV",
                command=lambda: self._export_graph_csv(
                    {"position_cm": list(self._import_graph_series.get("position", []))},
                    default_name="position"
                )
            ).place(relx=1.0, rely=1.0, anchor="se", x=-4, y=-4)
        pos_lf.after(100, _place_pos_export_btn)

        # Compute fixed scale from imported data
        accel_series = [r["accel_mps2"] for r in rows]
        rt_scale = max(abs(v) for v in accel_series) if accel_series else GRAPH_ACCEL_RANGE_MS2
        rt_scale = max(rt_scale, 0.5)
        self._rt_graph_scale = rt_scale
        self._rt_graph_history = []

        # -- Store refs & cache series --------------------------------------------
        self.import_graph_file_name = file_name
        self.import_graph_current_rows = rows
        self._import_graph_series = {
            "accel":     ([r["unclamped_position_cm"] for r in rows]
                          if self._import_mode == 'displacement'
                          else [r["accel_mps2"] for r in rows]),
            "unclamped": [r["unclamped_position_cm"] for r in rows],
            "position":  [r["position_cm"]           for r in rows],
        }

        # Defer initial draws
        def _draw_all():
            if not graph_window.winfo_exists():
                return
            _redraw_accel()
            self._draw_rt_graph(rt_canvas)
            if pos_canvas.winfo_exists():
                self._draw_import_series_graph(
                    pos_canvas,
                    self._import_graph_series["position"],
                    "Position - Clamped to \u00b110 cm (cm)", "#dc2626",
                )

        self.root.after(50, _draw_all)

        # -- Hook telemetry into real-time graph ----------------------------------
        self._rt_canvas_ref = rt_canvas
        self._rt_graph_active = True

        # Switch to the Earthquake Graphs tab
        self.notebook.select(self.tab_graphs_frame)

    # -- Real-time graph helpers -------------------------------------------------

    def _clear_rt_graph(self, canvas):
        self._rt_graph_history = []
        self._draw_rt_graph(canvas)

    def _draw_rt_graph(self, canvas=None):
        if canvas is None:
            canvas = getattr(self, '_rt_canvas_ref', None)
        if canvas is None or not canvas.winfo_exists():
            return
        canvas.delete("all")
        width  = max(canvas.winfo_width(), 920)
        height = max(canvas.winfo_height(), 200)
        margin = 28

        # Use same background/style as _draw_import_series_graph
        canvas.create_rectangle(0, 0, width, height, fill="#f8fafc", outline="")
        canvas.create_text(10, 10, text="Real-time Acceleration (m/s�)", anchor="nw",
                           fill="#1f2937", font=("TkDefaultFont", 10, "bold"))
        canvas.create_line(margin, margin, margin, height - margin, fill="#cbd5e1")
        canvas.create_line(margin, height - margin, width - margin, height - margin, fill="#cbd5e1")

        history = getattr(self, '_rt_graph_history', [])
        if len(history) < 2:
            canvas.create_text(width / 2, height / 2, text="Waiting for Accel MCU data�",
                               fill="#64748b")
            return

        # Y axis: auto-scale to what's currently in history
        min_value = min(history)
        max_value = max(history)
        if abs(max_value - min_value) < 1e-9:
            max_value = min_value + 1.0

        usable_w = width - 2 * margin
        usable_h = height - 2 * margin
        MAX_POINTS = max(int(usable_w), 500)
        if len(history) > MAX_POINTS:
            step = len(history) / MAX_POINTS
            sampled = [history[int(i * step)] for i in range(MAX_POINTS)]
        else:
            sampled = history
        n = len(sampled)

        points = []
        for i, v in enumerate(sampled):
            x = margin + usable_w * i / max(n - 1, 1)
            y_norm = (v - min_value) / (max_value - min_value)
            y = height - margin - y_norm * usable_h
            points.extend((x, y))
        canvas.create_line(*points, fill="#2563eb", width=2)

        canvas.create_text(width - margin, margin,
                           text=f"max {max_value:.3f}", anchor="ne", fill="#334155")
        canvas.create_text(width - margin, margin + 16,
                           text=f"min {min_value:.3f}", anchor="ne", fill="#334155")

    # -- Manual stepper window ---------------------------------------------------

    def _open_manual_stepper_window(self):
        if hasattr(self, '_manual_win') and self._manual_win and self._manual_win.winfo_exists():
            self._manual_win.lift()
            return
        win = tk.Toplevel(self.root)
        self._manual_win = win
        win.title("Manual Stepper Control")
        win.resizable(False, False)
        f = ttk.Frame(win, padding=16)
        f.grid(row=0, column=0)

        steps_var    = tk.StringVar(value="400")
        feedrate_var = tk.StringVar(value="4000")

        ttk.Label(f, text="Steps:").grid(row=0, column=0, sticky="w")
        ttk.Entry(f, textvariable=steps_var, width=10).grid(row=0, column=1, padx=(4, 0))
        ttk.Label(f, text="Feedrate:").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(f, textvariable=feedrate_var, width=10).grid(row=1, column=1, padx=(4, 0), pady=(6, 0))

        status_var = tk.StringVar(value="")
        ttk.Label(f, textvariable=status_var, foreground="#374151").grid(row=2, column=0, columnspan=2, sticky="w", pady=(8, 0))

        def _send_manual(direction):
            if not self.client or not self.client.is_connected():
                status_var.set("Motor MCU not connected.")
                return
            try:
                steps = int(steps_var.get())
                feedrate = int(feedrate_var.get())
            except ValueError:
                status_var.set("Invalid steps or feedrate.")
                return
            def _worker():
                deadline = time.monotonic() + 10.0
                while self.queue_free_slots <= 0:
                    if time.monotonic() > deadline:
                        self.root.after(0, lambda: status_var.set("Timeout waiting for free slot."))
                        return
                    time.sleep(0.02)
                try:
                    self.client.send_command(steps, direction, feedrate)
                    self.queue_free_slots = max(0, self.queue_free_slots - 1)
                    self.root.after(0, lambda d=direction, s=steps: status_var.set(
                        f"Sent: {s} steps, dir={d}"))
                except Exception as exc:
                    self.root.after(0, lambda e=str(exc): status_var.set(f"Error: {e}"))
            threading.Thread(target=_worker, daemon=True).start()

        btn_f = ttk.Frame(f)
        btn_f.grid(row=3, column=0, columnspan=2, pady=(12, 0))
        ttk.Button(btn_f, text="\u25C4 Backward", width=12,
                   command=lambda: _send_manual(-1)).grid(row=0, column=0, padx=(0, 8))
        ttk.Button(btn_f, text="Forward \u25BA", width=12,
                   command=lambda: _send_manual(1)).grid(row=0, column=1)

    def _draw_import_series_graph(self, canvas, values, title, line_color, progress_frac=None):
        canvas.delete("all")
        width = max(canvas.winfo_width(), 920)
        height = max(canvas.winfo_height(), 180)
        margin = 28

        canvas.create_rectangle(0, 0, width, height, fill="#f8fafc", outline="")
        canvas.create_text(
            10,
            10,
            text=title,
            anchor="nw",
            fill="#1f2937",
            font=("TkDefaultFont", 10, "bold"),
        )
        canvas.create_line(margin, margin, margin, height - margin, fill="#cbd5e1")
        canvas.create_line(margin, height - margin, width - margin, height - margin, fill="#cbd5e1")

        if len(values) < 2:
            canvas.create_text(width / 2, height / 2, text="Not enough samples", fill="#64748b")
            return

        min_value = min(values)
        max_value = max(values)
        if abs(max_value - min_value) < 1e-9:
            max_value = min_value + 1.0

        usable_width = width - (2 * margin)
        usable_height = height - (2 * margin)

        # Downsample to at most one point per pixel column to keep create_line fast
        MAX_POINTS = max(int(usable_width), 500)
        count = len(values)
        if count > MAX_POINTS:
            step = count / MAX_POINTS
            sampled = [values[int(i * step)] for i in range(MAX_POINTS)]
        else:
            sampled = values
        n = len(sampled)

        points = []
        for i, value in enumerate(sampled):
            x_pos = margin + usable_width * i / max(n - 1, 1)
            y_norm = (value - min_value) / (max_value - min_value)
            y_pos = height - margin - y_norm * usable_height
            points.extend((x_pos, y_pos))

        canvas.create_line(*points, fill=line_color, width=2)

        # Progress line: vertical red line at current send position
        if progress_frac is not None and 0.0 <= progress_frac <= 1.0:
            px = margin + usable_width * progress_frac
            canvas.create_line(px, margin, px, height - margin, fill="#ef4444", width=2, dash=(4, 3))
            canvas.create_text(px + 4, margin + 2, text=f"{progress_frac*100:.0f}%",
                                anchor="nw", fill="#ef4444", font=("TkDefaultFont", 8))

        canvas.create_text(width - margin, margin, text=f"max {max_value:.3f}", anchor="ne", fill="#334155")
        canvas.create_text(width - margin, margin + 16, text=f"min {min_value:.3f}", anchor="ne", fill="#334155")

    def _export_generated_table(self, rows, source_name, status_var=None):
        default_stem = Path(source_name).stem if source_name else "earthquake_table"
        output_path = filedialog.asksaveasfilename(
            title="Export calculated earthquake table",
            defaultextension=".xlsx",
            initialfile=f"{default_stem}_calculated_table.xlsx",
            filetypes=[
                ("Excel file", "*.xlsx"),
                ("CSV file", "*.csv"),
            ],
        )
        if not output_path:
            return

        headers = [
            "index",
            "time_s",
            "accel_cmps2",
            "accel_mps2",
            "steps",
            "feedrate",
            "direction",
            "position_cm",
            "unclamped_position_cm",
        ]

        output_suffix = Path(output_path).suffix.lower()

        try:
            if output_suffix == ".csv":
                with open(output_path, "w", newline="", encoding="utf-8") as file_obj:
                    writer = csv.writer(file_obj)
                    writer.writerow(headers)
                    for row in rows:
                        writer.writerow([
                            row["index"],
                            round(row["time_s"], 6),
                            round(row["accel_cmps2"], 6),
                            round(row["accel_mps2"], 6),
                            row["steps"],
                            row["feedrate"],
                            row["direction"],
                            round(row["position_cm"], 6),
                            round(row["unclamped_position_cm"], 6),
                        ])
            else:
                try:
                    openpyxl_module = importlib.import_module("openpyxl")
                except Exception:
                    raise RuntimeError("openpyxl is required for Excel export. Install with: pip install openpyxl")

                workbook = openpyxl_module.Workbook()
                worksheet = workbook.active
                worksheet.title = "calculated_table"
                worksheet.append(headers)

                for row in rows:
                    worksheet.append([
                        row["index"],
                        row["time_s"],
                        row["accel_cmps2"],
                        row["accel_mps2"],
                        row["steps"],
                        row["feedrate"],
                        row["direction"],
                        row["position_cm"],
                        row["unclamped_position_cm"],
                    ])

                workbook.save(output_path)

            if status_var is not None:
                status_var.set(f"Exported table: {Path(output_path).name}")
        except Exception as exc:
            messagebox.showerror("Export error", str(exc))

    def _start_import_send(self, rows, merged_commands, tree):
        if not merged_commands:
            messagebox.showerror("Send error", "No commands available to send.")
            return

        if self.import_sender_running:
            messagebox.showwarning("Send in progress", "Imported replay is already running.")
            return

        if not self.client or not self.client.is_connected():
            self._connect_selected_port()
            if not self.client or not self.client.is_connected():
                messagebox.showerror(
                    "Connection error",
                    "Connect to the Arduino first before sending.",
                )
                return

        self._send_stop_flag[0] = False
        for b in self._all_send_btns:
            b.configure(state="disabled")
        for b in self._all_stop_btns:
            b.configure(state="normal")
        self._send_status_var.set("Sending...")
        self._send_progress_var.set(0.0)
        self.import_sender_running = True
        self.queue_free_slots = ARDUINO_QUEUE_SIZE  # assume full queue until first QFREE reply
        self._rt_graph_history = []   # fresh graph for this send session
        self._start_accel_recording()
        try:
            self.client.request_queue_status()
        except Exception:
            pass

        # Skip tree_ids collection for large trees � scrolling is disabled during send anyway
        tree_ids = tree.get_children() if len(tree.get_children()) < 500 else ()

        threading.Thread(
            target=self._import_send_worker,
            args=(rows, merged_commands, tree, tree_ids),
            daemon=True,
        ).start()

    def _import_send_worker(self, rows, merged_commands, tree, tree_ids):
        """ok-gated send: send next command only after Arduino confirms previous one finished.
        First ARDUINO_QUEUE_SIZE commands are burst-sent to pre-fill the queue; after that
        each send waits for an 'ok' from the Arduino (commandRunning -> done transition).
        """
        status_var   = self._send_status_var
        progress_var = self._send_progress_var
        stop_flag    = self._send_stop_flag

        total_commands = len(merged_commands)
        if total_commands == 0:
            self.import_sender_running = False
            self.root.after(0, lambda: [b.configure(state="normal")  for b in self._all_send_btns])
            self.root.after(0, lambda: [b.configure(state="disabled") for b in self._all_stop_btns])
            return

        had_error = False
        sent_count = 0
        ok_expected = 0        # how many ok's we've sent into the queue
        ok_at_start = self.client.ok_count  # baseline
        t0 = time.monotonic()
        last_ui_time = t0

        for i, command in enumerate(merged_commands):
            if stop_flag[0]:
                self.root.after(0, lambda: status_var.set("Stopped."))
                break

            # After the initial burst, wait for an ok before sending the next command
            if i >= ARDUINO_QUEUE_SIZE:
                deadline = time.monotonic() + 30.0
                while (self.client.ok_count - ok_at_start) < ok_expected:
                    if time.monotonic() > deadline:
                        had_error = True
                        self.root.after(0, lambda: status_var.set("Error: timed out waiting for ok."))
                        break
                    if stop_flag[0]:
                        break
                    time.sleep(0.005)
                if had_error or stop_flag[0]:
                    break

            # Also guard against a full queue (ENQ,FULL safety net)
            deadline = time.monotonic() + 10.0
            while self.queue_free_slots <= 0:
                if time.monotonic() > deadline:
                    had_error = True
                    self.root.after(0, lambda: status_var.set("Error: timed out waiting for free queue slot."))
                    break
                time.sleep(0.005)
            if had_error:
                break

            try:
                self.client.send_command(
                    command["steps"],
                    command["direction"],
                    command["feedrate"],
                )
            except Exception as exc:
                had_error = True
                self.root.after(0, lambda e=str(exc): status_var.set(f"Error: {e}"))
                self.root.after(0, lambda e=str(exc): self._finish_accel_recording(error_text=e))
                break

            sent_count += 1
            ok_expected += 1
            self.import_sender_row_index = command.get("row_end", command.get("row_start", sent_count))

            now = time.monotonic()
            if now - last_ui_time >= REPLAY_UI_UPDATE_INTERVAL_S:
                last_ui_time = now
                pct = sent_count / total_commands * 100.0
                self.root.after(0, lambda p=pct: progress_var.set(p))
                self.root.after(0, lambda s=sent_count, t=total_commands: status_var.set(
                    f"Sent {s}/{t} | ok {self.client.ok_count - ok_at_start}/{ok_expected}"
                ))
                row_idx = self.import_sender_row_index
                total_rows = len(rows)
                self.root.after(0, lambda ri=row_idx, tr=total_rows:
                    self._update_import_graph_progress(ri, tr))

        if not had_error and not stop_flag[0]:
            # Wait for all queued commands to finish (all ok's received)
            deadline = time.monotonic() + 120.0
            while (self.client.ok_count - ok_at_start) < ok_expected:
                if time.monotonic() > deadline:
                    break
                time.sleep(0.01)
            self.root.after(0, lambda s=sent_count, t=total_commands: status_var.set(
                f"Done. {s}/{t} commands executed."
            ))

        self.import_sender_running = False
        self.root.after(0, lambda: [b.configure(state="normal")  for b in self._all_send_btns])
        self.root.after(0, lambda: [b.configure(state="disabled") for b in self._all_stop_btns])
        self.root.after(0, lambda: progress_var.set(100.0 if not had_error else progress_var.get()))
        if not had_error and not stop_flag[0]:
            self.root.after(0, self._mark_send_complete_wait_for_stable)

    def _start_send_thread(self):
        try:
            baudrate = int(self.baud_var.get().strip())
            steps, direction, feedrate, frequency = validate_command_values(
                self.steps_var.get(),
                self.direction_var.get(),
                self.feedrate_var.get(),
                self.frequency_var.get(),
            )
        except Exception as exc:
            messagebox.showerror("Invalid input", str(exc))
            return

        if not self.client or not self.client.is_connected():
            self._connect_selected_port()
            if not self.client or not self.client.is_connected():
                messagebox.showerror("Connection error", "Unable to connect to the selected serial port.")
                return

        self._start_accel_recording()
        port = self.client.port

        thread = threading.Thread(
            target=self._send_commands,
            args=(port, baudrate, steps, direction, feedrate, frequency),
            daemon=True,
        )
        thread.start()

    def _send_commands(self, port, baudrate, steps, direction, feedrate, frequency):
        self._queue_log(
            f"Sending {frequency} command(s): steps={steps}, initial_direction={direction}, feedrate={feedrate}"
        )
        try:
            if not self.client or not self.client.is_connected():
                raise RuntimeError("Serial port is not connected.")

            ok_at_start = self.client.ok_count

            for index in range(frequency):
                # Wait for ok from previous command before sending next (skip wait for first)
                if index > 0:
                    deadline = time.monotonic() + 30.0
                    while self.client.ok_count - ok_at_start < index:
                        if time.monotonic() > deadline:
                            raise RuntimeError("Timed out waiting for ok from Arduino.")
                        time.sleep(0.005)

                # Also guard against a full queue
                deadline = time.monotonic() + 10.0
                while self.queue_free_slots <= 0:
                    if time.monotonic() > deadline:
                        raise RuntimeError("Timed out waiting for free queue slot.")
                    try:
                        self.client.request_queue_status()
                    except Exception:
                        pass
                    time.sleep(0.02)

                current_direction = get_direction_for_repeat(direction, index)
                command = self.client.send_command(steps, current_direction, feedrate)
                self.queue_free_slots = max(0, self.queue_free_slots - 1)
                self._queue_log(f"{index + 1}/{frequency} sent dir={current_direction}: {command}")

            # Wait for the last command's ok
            deadline = time.monotonic() + 60.0
            while self.client.ok_count - ok_at_start < frequency:
                if time.monotonic() > deadline:
                    break
                time.sleep(0.01)

            self._queue_log("Stepper commands sent. Continuing to record until acceleration stabilizes...")
            self.root.after(0, self._mark_send_complete_wait_for_stable)
        except Exception as exc:
            self._queue_log(f"Error: {exc}")
            self.root.after(0, lambda: self._finish_accel_recording(error_text=str(exc)))

    def _start_accel_recording(self):
        self.record_accel_history.clear()
        self.record_time_history.clear()
        self.stability_window.clear()
        self.recording_active = True
        self.send_in_progress = True
        self.waiting_for_stable = False
        self.stability_confirm_count = 0
        self.recording_started_at = None
        self.record_summary_var.set("Recording while stepper commands are running...")
        self._draw_record_graph()

    def _mark_send_complete_wait_for_stable(self):
        if not self.recording_active:
            return

        self.send_in_progress = False
        self.waiting_for_stable = True
        self.stability_confirm_count = 0
        self.record_summary_var.set("Stepper command sent. Waiting for acceleration to stabilize...")
        self._draw_record_graph()

    def _evaluate_record_stability(self, sample_value):
        if not self.recording_active or self.send_in_progress or not self.waiting_for_stable:
            return

        self.stability_window.append(sample_value)

        if len(self.record_accel_history) < STABILITY_MIN_SAMPLES:
            return

        if len(self.stability_window) < STABILITY_WINDOW_SAMPLES:
            return

        window_samples = list(self.stability_window)
        mean_value = sum(window_samples) / len(window_samples)
        variance = sum((value - mean_value) * (value - mean_value) for value in window_samples) / len(window_samples)
        std_value = math.sqrt(variance)
        span_value = max(window_samples) - min(window_samples)

        if std_value <= STABILITY_STD_THRESHOLD_MS2 and span_value <= STABILITY_SPAN_THRESHOLD_MS2:
            self.stability_confirm_count += 1
        else:
            self.stability_confirm_count = 0

        if self.stability_confirm_count >= STABILITY_CONFIRM_WINDOWS:
            self._finish_accel_recording(std_value=std_value, span_value=span_value)

    def _finish_accel_recording(self, std_value=None, span_value=None, error_text=None):
        self.recording_active = False
        self.send_in_progress = False
        self.waiting_for_stable = False
        self.stability_window.clear()
        self.stability_confirm_count = 0
        self.recording_started_at = None

        if error_text:
            self.record_summary_var.set(f"Recording stopped due to send error: {error_text}")
        elif not self.record_accel_history:
            self.record_summary_var.set("No acceleration samples captured during send.")
        else:
            samples = list(self.record_accel_history)
            summary = (
                f"Samples: {len(samples)} | X min: {min(samples):.3f} m/s^2 | "
                f"X max: {max(samples):.3f} m/s^2"
            )
            if std_value is not None and span_value is not None:
                summary += f" | Stable (std {std_value:.3f}, span {span_value:.3f})"
            self.record_summary_var.set(summary)
        self._draw_record_graph()
        # Update correction analysis tab whenever a send finishes
        self.root.after(100, self._build_correction_tab)
        # Advance ILC if a calibration run just finished
        if self._ilc_running:
            self.root.after(300, self._ilc_next_iteration)

    # -- ILC engine -----------------------------------------------------------
    def _start_ilc(self, learning_gain, max_iterations, rmse_threshold):
        """Kick off the first ILC run. Subsequent runs are triggered by _ilc_next_iteration."""
        if self._import_mode == 'displacement':
            messagebox.showerror("Auto-Calibrate",
                                 "ILC requires acceleration data.\nPlease import using 'Import Accel Data'.")
            return
        if not hasattr(self, '_import_graph_series') or not self._import_graph_series.get('accel'):
            messagebox.showerror("Auto-Calibrate", "Import earthquake data first.")
            return
        if not self.client or not self.client.is_connected():
            messagebox.showerror("Auto-Calibrate", "Connect Motor MCU first.")
            return
        if self.import_sender_running:
            messagebox.showwarning("Auto-Calibrate", "A send is already in progress.")
            return

        self._ilc_learning_gain   = learning_gain
        self._ilc_max_iterations  = max_iterations
        self._ilc_rmse_threshold  = rmse_threshold
        self._ilc_target_accel    = list(self._import_graph_series['accel'])
        self._ilc_current_accel   = list(self._ilc_target_accel)  # start from original
        self._ilc_iteration       = 0
        self._ilc_history         = []
        self._ilc_running         = True
        self._ilc_stop_flag[0]    = False
        self._rt_graph_history    = []

        self._ilc_status_var.set(
            f"ILC started — run 1/{max_iterations} — L={learning_gain}, threshold={rmse_threshold} m/s²")
        self.notebook.select(self.tab_correction_frame)

        rows = self.import_generated_rows
        mc   = self.import_merged_commands
        if not mc:
            messagebox.showerror("Auto-Calibrate", "No merged commands. Import data first.")
            self._ilc_running = False
            return
        fake_tree = type('_T', (), {'get_children': lambda s: ()})()  # noqa: E731
        self._start_import_send(rows, mc, fake_tree)

    def _ilc_next_iteration(self):
        """Called after each send+recording cycle. Computes error, updates input, sends again."""
        if not self._ilc_running:
            return

        if self._ilc_stop_flag[0]:
            self._ilc_finish("Stopped by user.")
            return

        actual  = list(self._rt_graph_history)
        target  = self._ilc_target_accel
        current = self._ilc_current_accel

        if not actual or not target:
            self._ilc_finish("Error: no recorded data from IMU.")
            return

        # Resample actual (100 Hz IMU) to match target length
        n_t = len(target)
        n_a = len(actual)
        if n_a >= 2:
            resampled = [actual[int(i / max(n_t - 1, 1) * (n_a - 1))] for i in range(n_t)]
        else:
            resampled = [actual[0]] * n_t

        # Error: desired − actual
        errors = [t - a for t, a in zip(target, resampled)]
        rmse   = math.sqrt(sum(e * e for e in errors) / len(errors))

        iteration = self._ilc_iteration + 1
        self._ilc_history.append((iteration, rmse))

        hist = "  →  ".join(f"iter {it}: {r:.5f}" for it, r in self._ilc_history)
        self._ilc_status_var.set(
            f"Iter {iteration}/{self._ilc_max_iterations}  RMSE={rmse:.5f}  |  {hist}")

        # Rebuild correction tab so history shows live
        self.root.after(0, self._build_correction_tab)

        # Check convergence
        if rmse <= self._ilc_rmse_threshold:
            self._ilc_finish(f"Converged after {iteration} iterations. Final RMSE={rmse:.5f} m/s²")
            return
        if iteration >= self._ilc_max_iterations:
            self._ilc_finish(
                f"Max iterations ({self._ilc_max_iterations}) reached. Final RMSE={rmse:.5f} m/s²")
            return

        # ILC update rule: u_{k+1}[i] = u_k[i] + L * e_k[i]
        L = self._ilc_learning_gain
        new_accel = [
            max(-SAFE_ACCEL_LIMIT_MPS2,
                min(SAFE_ACCEL_LIMIT_MPS2, c + L * e))
            for c, e in zip(current, errors)
        ]
        self._ilc_current_accel = new_accel
        self._ilc_iteration     = iteration

        # Regenerate motion table from updated accel
        new_rows = self._generate_motion_table_from_accel(new_accel)
        self.import_generated_rows    = new_rows
        self.import_merged_commands   = self._merge_generated_commands(new_rows)
        # Keep _import_graph_series in sync so graphs show current iteration input
        self._import_graph_series['accel']    = [r['accel_mps2'] for r in new_rows]
        self._import_graph_series['position'] = [r['position_cm'] for r in new_rows]
        if hasattr(self, '_redraw_accel'):
            self.root.after(0, self._redraw_accel)

        # Clear telemetry for next run
        self._rt_graph_history = []

        self._ilc_status_var.set(
            f"Sending iter {iteration + 1}/{self._ilc_max_iterations}  (prev RMSE={rmse:.5f})…")

        fake_tree = type('_T', (), {'get_children': lambda s: ()})()  # noqa: E731
        self.root.after(500, lambda: self._start_import_send(
            self.import_generated_rows, self.import_merged_commands, fake_tree))

    def _ilc_finish(self, message):
        """Clean up ILC state and surface the result."""
        self._ilc_running = False
        self._ilc_status_var.set(message)
        self.root.after(0, self._build_correction_tab)
        # Apply the converged accel back as permanent import data
        if self._ilc_current_accel and not self._ilc_stop_flag[0]:
            new_rows = self._generate_motion_table_from_accel(self._ilc_current_accel)
            self.import_generated_rows  = new_rows
            self.import_merged_commands = self._merge_generated_commands(new_rows)
            self._import_graph_series['accel']    = [r['accel_mps2'] for r in new_rows]
            self._import_graph_series['position'] = [r['position_cm'] for r in new_rows]
            if hasattr(self, '_redraw_accel'):
                self.root.after(100, self._redraw_accel)

    def _on_close(self):
        # Cancel pending after-callbacks before destroying the window
        for attr in ('_after_log_id', '_after_telemetry_id'):
            after_id = getattr(self, attr, None)
            if after_id is not None:
                try:
                    self.root.after_cancel(after_id)
                except Exception:
                    pass
        self._disconnect_serial()
        self.root.destroy()

    # -- CSV export helper ----------------------------------------------------
    def _export_graph_csv(self, series_dict, default_name="graph_data"):
        """Save one or more named series to a CSV file.
        series_dict: {column_header: [values]}
        """
        if not series_dict:
            messagebox.showwarning("Export", "No data to export.")
            return
        path = filedialog.asksaveasfilename(
            title="Export graph data",
            defaultextension=".csv",
            initialfile=f"{default_name}.csv",
            filetypes=[("CSV file", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            headers = list(series_dict.keys())
            columns = list(series_dict.values())
            n = max(len(c) for c in columns)
            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(headers)
                for i in range(n):
                    writer.writerow([col[i] if i < len(col) else "" for col in columns])
            messagebox.showinfo("Export", f"Saved {n} rows to:\n{path}")
        except Exception as exc:
            messagebox.showerror("Export error", str(exc))

    # -- Correction Analysis tab ------------------------------------------------
    def _build_correction_tab(self):
        """Build/refresh the Correction Analysis tab after a send completes."""
        # Need both expected (imported) and actual (recorded) series
        expected = (self._import_graph_series.get("accel", [])
                    if hasattr(self, '_import_graph_series') else [])
        actual_raw = list(self.record_accel_history)   # timestamped at ~100 Hz from IMU
        rt_history = getattr(self, '_rt_graph_history', [])  # accel captured during send

        # Prefer rt_graph_history (captured exactly during send window) over full record history
        actual = rt_history if rt_history else actual_raw

        for w in self.tab_correction_frame.winfo_children():
            w.destroy()

        outer = ttk.Frame(self.tab_correction_frame, padding=10)
        outer.pack(fill="both", expand=True)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(1, weight=2)
        outer.rowconfigure(3, weight=1)

        # -- Stats row --------------------------------------------------
        stats_var = tk.StringVar(value="No data yet.")
        ttk.Label(outer, textvariable=stats_var, foreground="#1e3a5f",
                  font=("TkDefaultFont", 10, "bold")).grid(row=0, column=0, sticky="w", pady=(0, 6))

        # -- Overlay graph: expected (blue) vs actual (red) -------------------------
        overlay_lf = ttk.LabelFrame(outer, text="Expected vs Actual Acceleration (m/s�)", padding=2)
        overlay_lf.grid(row=1, column=0, sticky="nsew", pady=(0, 6))
        overlay_lf.columnconfigure(0, weight=1)
        overlay_lf.rowconfigure(0, weight=1)
        overlay_canvas = tk.Canvas(overlay_lf, height=280, bg="#f8fafc", highlightthickness=0)
        overlay_canvas.grid(row=0, column=0, sticky="nsew")

        def _place_overlay_export():
            ttk.Button(
                overlay_lf, text="Export CSV",
                command=lambda: self._export_graph_csv(
                    {"expected_mps2": expected,
                     "actual_mps2": (rt_history if rt_history else actual_raw)},
                    default_name="correction_overlay"
                )
            ).place(relx=1.0, rely=1.0, anchor="se", x=-4, y=-4)
        overlay_lf.after(120, _place_overlay_export)
        err_lf = ttk.LabelFrame(outer, text="Per-sample Error (actual - expected) (m/s�)", padding=2)
        err_lf.grid(row=3, column=0, sticky="nsew", pady=(0, 6))
        err_lf.columnconfigure(0, weight=1)
        err_lf.rowconfigure(0, weight=1)
        err_canvas = tk.Canvas(err_lf, height=180, bg="#f8fafc", highlightthickness=0)
        err_canvas.grid(row=0, column=0, sticky="nsew")

        # Export wired after _draw_correction_graphs populates errors list
        _correction_errors = []   # filled by _draw_correction_graphs closure

        def _place_err_export():
            ttk.Button(
                err_lf, text="Export CSV",
                command=lambda: self._export_graph_csv(
                    {"error_mps2": list(_correction_errors)},
                    default_name="correction_errors"
                )
            ).place(relx=1.0, rely=1.0, anchor="se", x=-4, y=-4)
        err_lf.after(120, _place_err_export)
        ctrl_frame = ttk.LabelFrame(outer, text="Feedforward Gain Correction", padding=(8, 4))
        ctrl_frame.grid(row=4, column=0, sticky="ew", pady=(0, 4))
        ctrl_frame.columnconfigure(1, weight=1)

        gain_var = tk.DoubleVar(value=1.0)
        apply_status_var = tk.StringVar(value="")

        ttk.Label(ctrl_frame, text="Gain:").grid(row=0, column=0, sticky="w", padx=(0, 6))
        gain_entry = ttk.Entry(ctrl_frame, textvariable=gain_var, width=8)
        gain_entry.grid(row=0, column=1, sticky="w")
        ttk.Label(ctrl_frame, text="(multiply all input steps by this factor � 2.0 doubles amplitude)",
                  foreground="#64748b").grid(row=0, column=2, sticky="w", padx=(8, 0))

        def _apply_gain():
            if not hasattr(self, 'import_generated_rows') or not self.import_generated_rows:
                apply_status_var.set("No imported data to correct.")
                return
            try:
                g = float(gain_var.get())
                if g <= 0:
                    raise ValueError
            except (ValueError, tk.TclError):
                apply_status_var.set("Invalid gain value.")
                return
            # Apply gain: scale steps and feedrate proportionally
            corrected = []
            for r in self.import_generated_rows:
                cr = dict(r)
                cr["steps"] = max(0, round(r["steps"] * g))
                corrected.append(cr)
            self.import_generated_rows = corrected
            self.import_merged_commands = self._merge_generated_commands(corrected)
            apply_status_var.set(
                f"Gain {g:.3f} applied � {len(self.import_merged_commands)} merged commands ready. "
                "Use Send to Arduino in Earthquake Graphs tab."
            )
            # Refresh cached series so graphs reflect corrected data
            if hasattr(self, '_import_graph_series'):
                self._import_graph_series["accel"] = [r["accel_mps2"] for r in corrected]
                self._import_graph_series["position"] = [r["position_cm"] for r in corrected]
            if hasattr(self, '_redraw_accel'):
                self._redraw_accel()

        def _suggest_gain():
            """Compute recommended gain from ratio of expected peak to actual peak."""
            if not expected or not actual:
                apply_status_var.set("Need both expected and actual data.")
                return
            exp_peak = max(abs(v) for v in expected) if expected else 0
            act_peak = max(abs(v) for v in actual) if actual else 0
            if act_peak < 1e-6:
                apply_status_var.set("Actual peak too small to compute gain.")
                return
            suggested = exp_peak / act_peak
            gain_var.set(round(suggested, 3))
            apply_status_var.set(f"Suggested gain: {suggested:.3f} (expected peak / actual peak)")

        ttk.Button(ctrl_frame, text="Suggest Gain", command=_suggest_gain).grid(
            row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Button(ctrl_frame, text="Apply Gain & Update Commands", command=_apply_gain).grid(
            row=1, column=1, sticky="w", pady=(6, 0), padx=(6, 0))
        ttk.Label(ctrl_frame, textvariable=apply_status_var, foreground="#1e40af").grid(
            row=2, column=0, columnspan=3, sticky="w", pady=(4, 0))

        # -- ILC section --------------------------------------------------------
        ilc_frame = ttk.LabelFrame(outer, text="Auto-Calibrate  (Iterative Learning Control)",
                                   padding=(8, 4))
        ilc_frame.grid(row=5, column=0, sticky="ew", pady=(6, 0))
        ilc_frame.columnconfigure(1, weight=1)

        ilc_gain_var  = tk.DoubleVar(value=0.5)
        ilc_iter_var  = tk.IntVar(value=5)
        ilc_rmse_var  = tk.DoubleVar(value=0.005)

        ttk.Label(ilc_frame, text="Learning gain L:").grid(row=0, column=0, sticky="w", padx=(0, 4))
        ttk.Entry(ilc_frame, textvariable=ilc_gain_var, width=7).grid(row=0, column=1, sticky="w")
        ttk.Label(ilc_frame, text="(0.1 = cautious  0.8 = fast,  stay ≤0.8)",
                  foreground="#64748b").grid(row=0, column=2, sticky="w", padx=(8, 0))

        ttk.Label(ilc_frame, text="Max iterations:").grid(row=1, column=0, sticky="w", padx=(0, 4), pady=(4,0))
        ttk.Entry(ilc_frame, textvariable=ilc_iter_var, width=7).grid(row=1, column=1, sticky="w", pady=(4,0))

        ttk.Label(ilc_frame, text="Stop RMSE (m/s²):").grid(row=2, column=0, sticky="w", padx=(0, 4), pady=(4,0))
        ttk.Entry(ilc_frame, textvariable=ilc_rmse_var, width=7).grid(row=2, column=1, sticky="w", pady=(4,0))
        ttk.Label(ilc_frame, text="stop when RMSE drops below this",
                  foreground="#64748b").grid(row=2, column=2, sticky="w", padx=(8, 0))

        btn_row = ttk.Frame(ilc_frame)
        btn_row.grid(row=3, column=0, columnspan=3, sticky="w", pady=(8, 0))

        def _start_ilc_ui():
            try:
                L   = float(ilc_gain_var.get())
                mx  = int(ilc_iter_var.get())
                thr = float(ilc_rmse_var.get())
                if not (0 < L <= 1.0):
                    raise ValueError("gain")
                if mx < 1:
                    raise ValueError("iterations")
            except (ValueError, tk.TclError) as exc:
                self._ilc_status_var.set(f"Invalid parameter: {exc}")
                return
            self._start_ilc(L, mx, thr)

        ttk.Button(btn_row, text="Start Auto-Calibrate", command=_start_ilc_ui).grid(
            row=0, column=0, padx=(0, 6))
        ttk.Button(btn_row, text="Stop",
                   command=lambda: self._ilc_stop_flag.__setitem__(0, True)).grid(row=0, column=1)

        ttk.Label(ilc_frame, textvariable=self._ilc_status_var,
                  foreground="#1e40af", wraplength=700).grid(
            row=4, column=0, columnspan=3, sticky="w", pady=(6, 0))

        # History log
        if self._ilc_history:
            hist_text = "  |  ".join(
                f"iter {it}: RMSE={rmse:.5f}" for it, rmse in self._ilc_history
            )
            ttk.Label(ilc_frame, text=hist_text,
                      foreground="#374151", wraplength=700,
                      font=("TkDefaultFont", 8)).grid(
                row=5, column=0, columnspan=3, sticky="w", pady=(2, 0))

        # -- Draw everything once canvas is sized --------------------------------
        def _draw_correction_graphs():
            if not overlay_canvas.winfo_exists():
                return

            if not expected or not actual:
                overlay_canvas.create_text(500, 140, text="Not enough data � run a send first.",
                                           fill="#64748b")
                return

            # Resample actual to same length as expected for point-by-point comparison
            n_exp = len(expected)
            n_act = len(actual)
            if n_act >= 2:
                resampled_actual = [
                    actual[int(i / (n_exp - 1) * (n_act - 1))] for i in range(n_exp)
                ]
            else:
                resampled_actual = [actual[0]] * n_exp

            errors = [a - e for a, e in zip(resampled_actual, expected)]
            _correction_errors[:] = errors   # update shared list for export button
            rmse = math.sqrt(sum(e * e for e in errors) / len(errors)) if errors else 0.0
            max_err = max(abs(e) for e in errors) if errors else 0.0
            exp_peak = max(abs(v) for v in expected)
            act_peak = max(abs(v) for v in resampled_actual)
            stats_var.set(
                f"RMSE: {rmse:.4f} m/s�  |  Max error: {max_err:.4f} m/s�  |  "
                f"Expected peak: {exp_peak:.4f}  |  Actual peak: {act_peak:.4f}  |  "
                f"Peak ratio (exp/act): {(exp_peak / act_peak if act_peak > 1e-6 else 0):.3f}"
            )

            # Overlay graph
            self._draw_overlay_graph(overlay_canvas, expected, resampled_actual)

            # Error graph
            self._draw_import_series_graph(err_canvas, errors,
                                           "Error (actual - expected) m/s�", "#dc2626")

        self.tab_correction_frame.after(60, _draw_correction_graphs)

    def _draw_overlay_graph(self, canvas, series_a, series_b):
        """Draw two series on the same canvas. series_a=expected (blue), series_b=actual (red)."""
        canvas.delete("all")
        width  = max(canvas.winfo_width(), 900)
        height = max(canvas.winfo_height(), 280)
        margin = 32
        canvas.create_rectangle(0, 0, width, height, fill="#f8fafc", outline="")
        canvas.create_line(margin, margin, margin, height - margin, fill="#cbd5e1")
        canvas.create_line(margin, height - margin, width - margin, height - margin, fill="#cbd5e1")

        all_vals = series_a + series_b
        if len(all_vals) < 2:
            canvas.create_text(width / 2, height / 2, text="No data", fill="#64748b")
            return
        min_v = min(all_vals)
        max_v = max(all_vals)
        if abs(max_v - min_v) < 1e-9:
            max_v = min_v + 1.0

        uw = width - 2 * margin
        uh = height - 2 * margin
        MAX_POINTS = max(int(uw), 500)

        def _make_points(series):
            n = len(series)
            if n > MAX_POINTS:
                step = n / MAX_POINTS
                s = [series[int(i * step)] for i in range(MAX_POINTS)]
            else:
                s = series
            pts = []
            for i, v in enumerate(s):
                x = margin + uw * i / max(len(s) - 1, 1)
                y = height - margin - (v - min_v) / (max_v - min_v) * uh
                pts.extend((x, y))
            return pts

        canvas.create_line(*_make_points(series_a), fill="#2563eb", width=2)
        canvas.create_line(*_make_points(series_b), fill="#dc2626", width=2)

        # Legend
        canvas.create_rectangle(width - 160, margin, width - margin, margin + 36,
                                 fill="#f1f5f9", outline="#cbd5e1")
        canvas.create_line(width - 155, margin + 12, width - 130, margin + 12,
                           fill="#2563eb", width=2)
        canvas.create_text(width - 126, margin + 12, text="Expected", anchor="w",
                           fill="#1e3a5f", font=("TkDefaultFont", 8))
        canvas.create_line(width - 155, margin + 26, width - 130, margin + 26,
                           fill="#dc2626", width=2)
        canvas.create_text(width - 126, margin + 26, text="Actual", anchor="w",
                           fill="#991b1b", font=("TkDefaultFont", 8))

        canvas.create_text(width - margin, height - margin + 2,
                           text=f"max {max_v:.3f}", anchor="ne", fill="#334155")
        canvas.create_text(width - margin, height - margin + 14,
                           text=f"min {min_v:.3f}", anchor="ne", fill="#334155")


def run_ui():
    root = tk.Tk()
    app = SerialSenderApp(root)
    root.mainloop()


def main():
    parser = argparse.ArgumentParser(description="Send stepper commands to an Arduino over serial.")
    parser.add_argument(
        "--cli",
        action="store_true",
        help="Run in terminal prompt mode instead of the Tkinter UI.",
    )
    args = parser.parse_args()

    if args.cli:
        run_cli()
        return

    run_ui()


if __name__ == "__main__":
    main()