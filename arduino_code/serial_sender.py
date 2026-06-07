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
PREFERRED_PORT = "/dev/cu.usbmodem1101"
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
QUEUE_POLL_INTERVAL_S = 0.05
REPLAY_UI_UPDATE_INTERVAL_S = 0.2


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
                with self.connection_lock:
                    if not self.connection or not self.connection.is_open:
                        return
                    raw_line = self.connection.readline()

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
        self.root.title("Stepper And IMU Monitor")
        self.log_queue = queue.Queue()
        self.telemetry_queue = queue.Queue()
        self.client = None
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
        self.import_preview_window = None
        self.import_graph_window = None
        self.import_generated_rows = []
        self.import_merged_commands = []
        self.queue_free_slots = 0
        self.stepper_active_flag = 0
        self.import_sender_running = False
        self.last_telemetry_draw_time = 0.0

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUDRATE))
        self.steps_var = tk.StringVar()
        self.direction_var = tk.StringVar(value="1")
        self.feedrate_var = tk.StringVar()
        self.frequency_var = tk.StringVar(value="1")
        self.connection_var = tk.StringVar(value="Disconnected")
        self.gyro_var = tk.StringVar(value="Gyro: --, --, --")
        self.accel_var = tk.StringVar(value="Accel: --, --, -- m/s^2")
        self.angle_var = tk.StringVar(value="Angle: --, --, --")
        self.record_summary_var = tk.StringVar(value="Send stepper command to record acceleration.")
        self.record_filter_enabled_var = tk.BooleanVar(value=False)
        self.record_filter_window_var = tk.IntVar(value=RECORD_FILTER_DEFAULT_WINDOW)
        self.input_shaping_mode_var = tk.IntVar(value=0)  # 0 = raw commands, 1 = input shaping
        self.highpass_cutoff_var = tk.DoubleVar(value=HIGHPASS_FILTER_DEFAULT_HZ)
        self.max_displacement_cm_var = tk.DoubleVar(value=10.0)  # Max displacement in cm
        self.resonance_info_var = tk.StringVar(value="Resonance: not measured")
        self.show_imported_overlay_var = tk.BooleanVar(value=True)

        self._build_ui()
        self._refresh_ports()
        self._drain_log_queue()
        self._drain_telemetry_queue()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(250, self._auto_connect_default_port)

    def _build_ui(self):
        frame = ttk.Frame(self.root, padding=12)
        frame.grid(row=0, column=0, sticky="nsew")

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
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
        right_frame.rowconfigure(9, weight=1)

        ttk.Label(right_frame, text="Port").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=4)
        self.port_combo = ttk.Combobox(right_frame, textvariable=self.port_var, state="normal")
        self.port_combo.grid(row=0, column=1, sticky="ew", pady=4)
        ttk.Button(right_frame, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, padx=(8, 0), pady=4)

        ttk.Label(right_frame, text="Baudrate").grid(row=1, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Entry(right_frame, textvariable=self.baud_var).grid(row=1, column=1, columnspan=2, sticky="ew", pady=4)

        ttk.Label(right_frame, text="Steps").grid(row=2, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Entry(right_frame, textvariable=self.steps_var).grid(row=2, column=1, columnspan=2, sticky="ew", pady=4)

        ttk.Label(right_frame, text="Direction").grid(row=3, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Combobox(
            right_frame,
            textvariable=self.direction_var,
            values=("-1", "0", "1"),
            state="readonly",
        ).grid(row=3, column=1, columnspan=2, sticky="ew", pady=4)

        ttk.Label(right_frame, text="Feedrate").grid(row=4, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Entry(right_frame, textvariable=self.feedrate_var).grid(row=4, column=1, columnspan=2, sticky="ew", pady=4)

        ttk.Label(right_frame, text="Frequency").grid(row=5, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Entry(right_frame, textvariable=self.frequency_var).grid(row=5, column=1, columnspan=2, sticky="ew", pady=4)

        status_frame = ttk.Frame(right_frame)
        status_frame.grid(row=6, column=0, columnspan=3, sticky="ew", pady=(6, 6))
        status_frame.columnconfigure(0, weight=1)
        ttk.Label(status_frame, textvariable=self.connection_var).grid(row=0, column=0, sticky="w")
        ttk.Button(status_frame, text="Connect", command=self._connect_selected_port).grid(row=0, column=1, padx=(8, 0))
        ttk.Button(status_frame, text="Disconnect", command=self._disconnect_serial).grid(row=0, column=2, padx=(8, 0))

        self.log_text = tk.Text(right_frame, height=12, width=42, state="disabled")
        self.log_text.grid(row=7, column=0, columnspan=3, sticky="nsew", pady=(2, 8))

        ttk.Button(right_frame, text="Send", command=self._start_send_thread).grid(row=8, column=0, columnspan=3, sticky="ew")

        record_frame = ttk.LabelFrame(right_frame, text="Accel Record (during send)", padding=8)
        record_frame.grid(row=9, column=0, columnspan=3, sticky="nsew", pady=(8, 0))
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
        action_controls.columnconfigure(2, weight=1)

        ttk.Button(action_controls, text="Import Excel", command=self._import_accel_file).grid(row=0, column=0, sticky="w")

        ttk.Button(action_controls, text="Measure resonance", command=self._measure_resonance).grid(row=0, column=1, sticky="w", padx=(8, 0))
        ttk.Label(action_controls, textvariable=self.resonance_info_var).grid(row=0, column=2, sticky="w", padx=(8, 0))

        ttk.Label(record_frame, textvariable=self.record_summary_var).grid(row=3, column=0, sticky="w", pady=(6, 0))

        self._draw_orientation_view()
        self._draw_accel_graph()
        self._draw_record_graph()

    def _refresh_ports(self):
        ports = get_available_ports()
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(get_default_port_label(ports))

    def _auto_connect_default_port(self):
        if self.client and self.client.is_connected():
            return

        if self.port_var.get().strip():
            self._connect_selected_port()

    def _connect_selected_port(self):
        try:
            port_label = self.port_var.get().strip()
            port = extract_port_device(port_label)
            baudrate = int(self.baud_var.get().strip())
            if not port:
                raise ValueError("Port is required.")

            if self.client and self.client.is_connected():
                if self.client.port == port and self.client.baudrate == baudrate:
                    return
                self._disconnect_serial()

            self.connection_var.set(f"Connecting to {port}...")
            self.client = ArduinoSerialClient(port, baudrate=baudrate)
            self.client.connect(
                telemetry_callback=self._queue_telemetry,
                message_callback=self._queue_log,
                queue_status_callback=self._queue_status_update,
            )
            try:
                self.client.request_queue_status()
            except Exception:
                pass
            self.connection_var.set(f"Connected: {port}")
            self._queue_log(f"Connected to {port}")
        except Exception as exc:
            self.connection_var.set("Disconnected")
            self._queue_log(f"Connect error: {exc}")

    def _disconnect_serial(self):
        if self.client:
            self.client.disconnect()
        self.client = None
        self.connection_var.set("Disconnected")
        self._queue_log("Disconnected")

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
        self.root.after(100, self._drain_log_queue)

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
            draw_interval = 0.15 if self.import_sender_running else 0.05
            if now - self.last_telemetry_draw_time >= draw_interval:
                self.last_telemetry_draw_time = now
                self._draw_orientation_view()
                self._draw_accel_graph()
                self._draw_record_graph()

        self.root.after(50, self._drain_telemetry_queue)

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
    
    def _on_import_highpass_changed(self, file_name, original_rows):
        """Called when high-pass filter slider changes in import graph window"""
        # Update the displayed value to show 2 decimal places
        current_val = self.highpass_cutoff_var.get()
        self.highpass_cutoff_var.set(round(current_val, 2))
        
        # Reprocess data
        self._reprocess_imported_data()
    
    def _on_max_displacement_changed(self, file_name, original_rows):
        """Called when max displacement slider changes in import graph window"""
        # Update the displayed value to show 1 decimal place
        current_val = self.max_displacement_cm_var.get()
        self.max_displacement_cm_var.set(round(current_val, 1))
        
        # Reprocess data
        self._reprocess_imported_data()
    
    def _reprocess_imported_data(self):
        """Reprocess imported data with current filter and displacement settings"""
        if not self.imported_accel_raw:
            return
        
        # Apply baseline correction with current cutoff frequency
        corrected_values = self._apply_baseline_correction(self.imported_accel_raw.copy())
        
        # Apply input shaping if enabled
        if self.input_shaping_mode_var.get() == 1 and self.resonance_freq_hz is not None:
            corrected_values = self._apply_input_shaping(
                corrected_values,
                self.resonance_freq_hz,
                self.imported_sample_interval_s
            )
        
        # Calculate scale factor based on max displacement slider
        max_displacement_m = self.max_displacement_cm_var.get() / 100.0
        scale_factor = self._calculate_displacement_scale_factor(corrected_values, max_displacement_m)
        
        # Apply scale factor to acceleration
        scaled_values = [a * scale_factor for a in corrected_values]
        
        # Update stored data
        self.imported_accel_history = scaled_values
        
        # Regenerate motion table
        new_rows = self._generate_motion_table_from_accel(scaled_values)
        self.import_graph_current_rows = new_rows
        self.import_generated_rows = new_rows
        
        # Redraw all three graphs
        if hasattr(self, 'import_accel_canvas') and self.import_accel_canvas.winfo_exists():
            self._draw_import_series_graph(
                self.import_accel_canvas,
                [row["accel_mps2"] for row in new_rows],
                "Acceleration (m/s^2)",
                "#2563eb",
            )
        
        if hasattr(self, 'import_position_canvas') and self.import_position_canvas.winfo_exists():
            self._draw_import_series_graph(
                self.import_position_canvas,
                [row["position_cm"] for row in new_rows],
                "Position - Clamped to ±10cm (cm)",
                "#dc2626",
            )
        
        if hasattr(self, 'import_unclamped_canvas') and self.import_unclamped_canvas.winfo_exists():
            self._draw_import_series_graph(
                self.import_unclamped_canvas,
                [row["unclamped_position_cm"] for row in new_rows],
                "Position - Actual Earthquake Displacement (cm)",
                "#16a34a",
            )
        
        # Update table window if it exists
        self._update_import_table_window(new_rows)
    
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
        """Update the import table window with new data"""
        if not hasattr(self, 'import_preview_window') or not self.import_preview_window:
            return
        
        if not self.import_preview_window.winfo_exists():
            return
        
        # Find the treeview in the window
        for child in self.import_preview_window.winfo_children():
            if isinstance(child, ttk.Frame):
                for subchild in child.winfo_children():
                    if isinstance(subchild, ttk.Frame):
                        for item in subchild.winfo_children():
                            if isinstance(item, ttk.Treeview):
                                # Clear existing items
                                item.delete(*item.get_children())
                                
                                # Insert new data
                                for row in new_rows:
                                    item.insert(
                                        "",
                                        "end",
                                        values=(
                                            row["index"],
                                            f"{row['time_s']:.2f}",
                                            f"{row['accel_cmps2']:.3f}",
                                            f"{row['accel_mps2']:.4f}",
                                            row["steps"],
                                            row["feedrate"],
                                            row["direction"],
                                            f"{row['position_cm']:.3f}",
                                            f"{row['unclamped_position_cm']:.3f}",
                                        ),
                                    )
                                return

    def _import_accel_file(self):
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

        try:
            imported_values = self._load_accel_series_from_file(file_path)
        except Exception as exc:
            messagebox.showerror("Import error", str(exc))
            return

        if len(imported_values) < 2:
            messagebox.showerror("Import error", "Need at least 2 numeric rows in the first data column.")
            return

        # Store raw data before baseline correction for re-processing when slider changes
        self.imported_accel_raw = imported_values.copy()

        # Apply baseline correction (less critical with FFT method, but still helps clean data)
        # FFT method naturally removes DC drift by skipping f=0 bin
        imported_values = self._apply_baseline_correction(imported_values)
        
        # Calculate initial max displacement to set slider default
        initial_displacement_m = self._accel_to_displacement_fft(imported_values)
        if initial_displacement_m:
            initial_max_disp = max(abs(d) for d in initial_displacement_m)
            # Set slider to actual displacement, capped at 10cm (stroke limit)
            self.max_displacement_cm_var.set(min(initial_max_disp * 100.0, 10.0))
        else:
            self.max_displacement_cm_var.set(10.0)
        
        # Apply input shaping for resonance compensation (if enabled and resonance measured)
        input_shaping_applied = False
        if self.input_shaping_mode_var.get() == 1 and self.resonance_freq_hz is not None:
            imported_values = self._apply_input_shaping(
                imported_values, 
                self.resonance_freq_hz, 
                self.imported_sample_interval_s
            )
            input_shaping_applied = True
        
        # Calculate optimal scaling factor to fit displacement within stroke limits
        scale_factor = self._calculate_optimal_scale_factor(imported_values)
        
        if scale_factor < 1.0:
            response = messagebox.askyesno(
                "Amplitude Scaling Recommended",
                f"The earthquake displacement exceeds ±{STROKE_LIMIT_M * 100:.0f}cm table limits.\n\n"
                f"Recommended scale factor: {scale_factor:.3f}\n"
                f"This will preserve the waveform shape while fitting within table stroke.\n\n"
                f"Original max accel: {max(abs(min(imported_values)), abs(max(imported_values))):.3f} m/s²\n"
                f"Scaled max accel: {max(abs(min(imported_values)), abs(max(imported_values))) * scale_factor:.3f} m/s²\n"
                f"{'Input shaping applied for ' + str(self.resonance_freq_hz) + ' Hz resonance' if input_shaping_applied else ''}\n\n"
                f"Apply scaling?"
            )
            
            if response:
                imported_values = [a * scale_factor for a in imported_values]
                status_msg = f"Imported {len(imported_values)} samples from {Path(file_path).name} (scaled by {scale_factor:.3f}x"
                if input_shaping_applied:
                    status_msg += f", input shaped for {self.resonance_freq_hz:.1f}Hz"
                status_msg += f", dt={self.imported_sample_interval_s:.2f}s)"
                self.record_summary_var.set(status_msg)
            else:
                status_msg = f"Imported {len(imported_values)} samples from {Path(file_path).name} (NO SCALING - will clip at ±{STROKE_LIMIT_M * 100:.0f}cm"
                if input_shaping_applied:
                    status_msg += f", input shaped for {self.resonance_freq_hz:.1f}Hz"
                status_msg += ")"
                self.record_summary_var.set(status_msg)
        else:
            status_msg = f"Imported {len(imported_values)} samples from {Path(file_path).name} (fits within stroke limits"
            if input_shaping_applied:
                status_msg += f", input shaped for {self.resonance_freq_hz:.1f}Hz"
            status_msg += f", dt={self.imported_sample_interval_s:.2f}s)"
            self.record_summary_var.set(status_msg)
        
        self.imported_accel_history = imported_values
        generated_rows = self._generate_motion_table_from_accel(imported_values)
        merged_commands = self._merge_generated_commands(generated_rows)
        self.import_generated_rows = generated_rows
        self.import_merged_commands = merged_commands
        file_name = Path(file_path).name
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
        
        # Step 2: Remove polynomial trend (quadratic fit: y = ax² + bx + c)
        # This handles parabolic baseline drift better than linear
        corrected = self._remove_polynomial_trend(detrended)
        
        # Step 3: Apply high-pass filter to remove very low frequencies
        # This removes drift without affecting earthquake content (typically > 0.1 Hz)
        cutoff_hz = self.highpass_cutoff_var.get()
        corrected = self._apply_highpass_filter(corrected, cutoff_hz=cutoff_hz)
        
        # Step 4: Post-integration drift correction
        # Simulate integration to check final position, then remove residual drift
        corrected = self._remove_integration_drift(corrected)
        
        return corrected
    
    def _remove_polynomial_trend(self, accel_values):
        """
        Remove quadratic polynomial trend: y = ax² + bx + c
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
        - After integration: v(T) = a_0*T + a_1*T²/2
        - After 2nd integration: x(T) = a_0*T²/2 + a_1*T³/6
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
        # a_0*T + a_1*T²/2 = -v_f
        # a_0*T²/2 + a_1*T³/6 = -x_f
        # 
        # Solution:
        # a_1 = 12*x_f/T³ - 6*v_f/T²
        # a_0 = -6*x_f/T² + 2*v_f/T
        
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
        X(f) = -A(f) / (2πf)²
        
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
        # X(f) = -A(f) / (2πf)²
        disp_fft = np.zeros_like(accel_fft, dtype=complex)
        
        # Skip DC component (f=0) and very low frequencies to avoid division by zero
        # Frequencies below 0.05 Hz are typically drift, not earthquake content
        mask = np.abs(freqs) >= 0.05
        omega = 2.0 * np.pi * freqs[mask]
        disp_fft[mask] = -accel_fft[mask] / (omega * omega)
        
        # Perform IFFT to get displacement in time domain
        displacement_array = np.fft.ifft(disp_fft).real
        
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
                feedrate = max(1, int(round(step_count / dt)))
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
        if self.import_preview_window and self.import_preview_window.winfo_exists():
            self.import_preview_window.destroy()

        window = tk.Toplevel(self.root)
        self.import_preview_window = window
        window.title(f"Imported Earthquake Table - {file_name}")
        window.geometry("980x560")

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

        for row in rows:
            tree.insert(
                "",
                "end",
                values=(
                    row["index"],
                    f"{row['time_s']:.2f}",
                    f"{row['accel_cmps2']:.3f}",
                    f"{row['accel_mps2']:.4f}",
                    row["steps"],
                    row["feedrate"],
                    row["direction"],
                    f"{row['position_cm']:.3f}",
                    f"{row['unclamped_position_cm']:.3f}",
                ),
            )

        y_scroll = ttk.Scrollbar(table_frame, orient="vertical", command=tree.yview)
        y_scroll.grid(row=0, column=1, sticky="ns")
        tree.configure(yscrollcommand=y_scroll.set)

        nonzero_commands = sum(1 for row in rows if row["steps"] > 0)
        ttk.Label(
            outer,
            text=(
                f"Rows: {len(rows)} | nonzero commands: {nonzero_commands} | "
                f"merged commands: {len(merged_commands)} | dt={self.imported_sample_interval_s:.3f}s"
            ),
        ).grid(row=2, column=0, sticky="w", pady=(8, 0))

        import_status_var = tk.StringVar(value="Ready to send.")
        import_progress_var = tk.DoubleVar(value=0.0)
        import_stop_flag = [False]

        bottom_frame = ttk.Frame(outer)
        bottom_frame.grid(row=3, column=0, sticky="ew", pady=(8, 0))
        bottom_frame.columnconfigure(3, weight=1)

        send_btn = ttk.Button(
            bottom_frame,
            text="Send to Arduino",
            command=lambda: self._start_import_send(
                rows, merged_commands, tree, import_status_var, import_progress_var,
                import_stop_flag, send_btn, stop_btn
            ),
        )
        send_btn.grid(row=0, column=0, sticky="w")

        stop_btn = ttk.Button(
            bottom_frame,
            text="Stop",
            state="disabled",
            command=lambda: import_stop_flag.__setitem__(0, True),
        )
        stop_btn.grid(row=0, column=1, sticky="w", padx=(8, 0))

        export_btn = ttk.Button(
            bottom_frame,
            text="Export Table",
            command=lambda: self._export_generated_table(rows, file_name, import_status_var),
        )
        export_btn.grid(row=0, column=2, sticky="w", padx=(8, 0))

        ttk.Label(bottom_frame, textvariable=import_status_var).grid(
            row=0, column=3, sticky="w", padx=(12, 0)
        )

        progress_bar = ttk.Progressbar(
            outer,
            variable=import_progress_var,
            maximum=100.0,
            length=400,
        )
        progress_bar.grid(row=4, column=0, sticky="ew", pady=(6, 0))

        self._open_import_graph_window(file_name, rows)

    def _open_import_graph_window(self, file_name, rows):
        if self.import_graph_window and self.import_graph_window.winfo_exists():
            self.import_graph_window.destroy()

        graph_window = tk.Toplevel(self.root)
        self.import_graph_window = graph_window
        graph_window.title(f"Imported Earthquake Graphs - {file_name}")
        graph_window.geometry("980x680")

        outer = ttk.Frame(graph_window, padding=10)
        outer.grid(row=0, column=0, sticky="nsew")
        graph_window.columnconfigure(0, weight=1)
        graph_window.rowconfigure(0, weight=1)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(1, weight=1)
        outer.rowconfigure(2, weight=1)
        outer.rowconfigure(3, weight=1)

        # Filter and displacement controls at top
        filter_frame = ttk.Frame(outer)
        filter_frame.grid(row=0, column=0, sticky="ew", pady=(0, 12))
        filter_frame.columnconfigure(1, weight=1)
        filter_frame.columnconfigure(4, weight=1)
        
        # High-pass filter slider
        ttk.Label(filter_frame, text="High-pass filter (Hz):").grid(row=0, column=0, sticky="w", padx=(0, 8))
        
        ttk.Scale(
            filter_frame,
            from_=HIGHPASS_FILTER_MIN_HZ,
            to=HIGHPASS_FILTER_MAX_HZ,
            variable=self.highpass_cutoff_var,
            command=lambda v: self._on_import_highpass_changed(file_name, rows),
        ).grid(row=0, column=1, sticky="ew")
        
        ttk.Label(filter_frame, textvariable=self.highpass_cutoff_var, width=6).grid(row=0, column=2, sticky="e", padx=(8, 0))
        
        ttk.Label(filter_frame, text="← Less | More →").grid(row=0, column=3, sticky="w", padx=(8, 16))
        
        # Max displacement slider
        ttk.Label(filter_frame, text="Max displacement (cm):").grid(row=0, column=4, sticky="w", padx=(0, 8))
        
        ttk.Scale(
            filter_frame,
            from_=0.5,
            to=10.0,
            variable=self.max_displacement_cm_var,
            command=lambda v: self._on_max_displacement_changed(file_name, rows),
        ).grid(row=0, column=5, sticky="ew")
        
        ttk.Label(filter_frame, textvariable=self.max_displacement_cm_var, width=6).grid(row=0, column=6, sticky="e", padx=(8, 0))
        
        ttk.Label(filter_frame, text="(scales acceleration, updates table in real-time)").grid(row=0, column=7, sticky="w", padx=(8, 0))

        # Store canvas references for updating
        self.import_accel_canvas = tk.Canvas(outer, height=200, bg="#f8fafc", highlightthickness=0)
        self.import_accel_canvas.grid(row=1, column=0, sticky="nsew", pady=(0, 8))
        
        accel_canvas = self.import_accel_canvas

        self.import_position_canvas = tk.Canvas(outer, height=200, bg="#f8fafc", highlightthickness=0)
        self.import_position_canvas.grid(row=2, column=0, sticky="nsew", pady=(0, 8))
        position_canvas = self.import_position_canvas
        
        self.import_unclamped_canvas = tk.Canvas(outer, height=200, bg="#f8fafc", highlightthickness=0)
        self.import_unclamped_canvas.grid(row=3, column=0, sticky="nsew")
        unclamped_canvas = self.import_unclamped_canvas
        
        # Store current file name and rows for re-processing
        self.import_graph_file_name = file_name
        self.import_graph_current_rows = rows

        self._draw_import_series_graph(
            accel_canvas,
            [row["accel_mps2"] for row in rows],
            "Acceleration (m/s^2)",
            "#2563eb",
        )
        self._draw_import_series_graph(
            position_canvas,
            [row["position_cm"] for row in rows],
            "Position - Clamped to ±10cm (cm)",
            "#dc2626",
        )
        self._draw_import_series_graph(
            unclamped_canvas,
            [row["unclamped_position_cm"] for row in rows],
            "Position - Actual Earthquake Displacement (cm)",
            "#16a34a",
        )

    def _draw_import_series_graph(self, canvas, values, title, line_color):
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

        points = []
        count = len(values)
        for index, value in enumerate(values):
            x_pos = margin + usable_width * index / max(count - 1, 1)
            y_norm = (value - min_value) / (max_value - min_value)
            y_pos = height - margin - y_norm * usable_height
            points.extend((x_pos, y_pos))

        canvas.create_line(*points, fill=line_color, width=2, smooth=True)
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

    def _start_import_send(self, rows, merged_commands, tree, status_var, progress_var, stop_flag,
                            send_btn, stop_btn):
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

        stop_flag[0] = False
        send_btn.configure(state="disabled")
        stop_btn.configure(state="normal")
        status_var.set("Sending...")
        progress_var.set(0.0)
        self.import_sender_running = True
        self._start_accel_recording()
        try:
            self.client.request_queue_status()
        except Exception:
            pass

        threading.Thread(
            target=self._import_send_worker,
            args=(rows, merged_commands, tree, status_var, progress_var, stop_flag, send_btn, stop_btn),
            daemon=True,
        ).start()

    def _import_send_worker(self, rows, merged_commands, tree, status_var, progress_var, stop_flag,
                            send_btn, stop_btn):
        total_commands = len(merged_commands)
        sent_count = 0
        tree_ids = tree.get_children()
        next_command_index = 0
        last_poll_time = 0.0
        last_ui_time = 0.0
        had_error = False

        while True:
            if stop_flag[0]:
                self.root.after(0, lambda: status_var.set("Stopped."))
                break

            now = time.monotonic()
            if now - last_poll_time >= QUEUE_POLL_INTERVAL_S:
                last_poll_time = now
                try:
                    self.client.request_queue_status()
                except Exception as exc:
                    had_error = True
                    self.root.after(0, lambda e=str(exc): status_var.set(f"Error: {e}"))
                    self.root.after(0, lambda e=str(exc): self._finish_accel_recording(error_text=e))
                    break

            free_slots = self.queue_free_slots
            while free_slots > 0 and next_command_index < total_commands:
                command = merged_commands[next_command_index]
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
                next_command_index += 1
                free_slots -= 1

                if sent_count % 20 == 0 or sent_count == total_commands:
                    self._queue_log(
                        f"Replay top-up: sent {sent_count}/{total_commands} merged commands"
                    )

            if had_error:
                break

            done_sending = next_command_index >= total_commands
            queue_empty = self.queue_free_slots >= ARDUINO_QUEUE_SIZE
            stepper_idle = self.stepper_active_flag == 0
            if done_sending and queue_empty and stepper_idle:
                self.root.after(
                    0,
                    lambda s=sent_count: status_var.set(
                        f"Done. {s}/{total_commands} merged commands sent."
                    ),
                )
                break

            if now - last_ui_time >= REPLAY_UI_UPDATE_INTERVAL_S:
                last_ui_time = now
                progress_pct = next_command_index / max(total_commands, 1) * 100.0
                self.root.after(0, lambda pct=progress_pct: progress_var.set(pct))

                if next_command_index > 0:
                    last_row = merged_commands[next_command_index - 1]["row_end"]
                else:
                    last_row = 0

                if 0 <= last_row < len(tree_ids):
                    tree_id = tree_ids[last_row]
                    self.root.after(0, lambda tid=tree_id: (tree.selection_set(tid), tree.see(tid)))

                self.root.after(
                    0,
                    lambda idx=next_command_index, qf=self.queue_free_slots: status_var.set(
                        f"Commands {idx}/{total_commands} | qfree {qf}/{ARDUINO_QUEUE_SIZE}"
                    ),
                )

            time.sleep(0.005)

        self.import_sender_running = False
        self.root.after(0, lambda: send_btn.configure(state="normal"))
        self.root.after(0, lambda: stop_btn.configure(state="disabled"))
        self.root.after(0, lambda: progress_var.set(100.0 if not had_error else progress_var.get()))
        if not had_error:
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

            self.client.send_repeated_command(
                steps,
                direction,
                feedrate,
                frequency,
                on_message=self._queue_log,
            )
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

    def _on_close(self):
        self._disconnect_serial()
        self.root.destroy()


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