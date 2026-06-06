import argparse
import math
import queue
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import messagebox
from tkinter import ttk

import serial
from serial.tools import list_ports


DEFAULT_BAUDRATE = 115200
DEFAULT_TIMEOUT = 2.0
DEFAULT_CONNECT_DELAY = 2.0
PREFERRED_PORT = "/dev/cu.usbmodem1101"
GRAPH_HISTORY = 180
GRAPH_ACCEL_RANGE = 2.0


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

    def connect(self, telemetry_callback=None, message_callback=None):
        if self.connection and self.connection.is_open:
            self.telemetry_callback = telemetry_callback
            self.message_callback = message_callback
            return

        self.telemetry_callback = telemetry_callback
        self.message_callback = message_callback
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
        self.latest_telemetry = None

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUDRATE))
        self.steps_var = tk.StringVar()
        self.direction_var = tk.StringVar(value="1")
        self.feedrate_var = tk.StringVar()
        self.frequency_var = tk.StringVar(value="1")
        self.connection_var = tk.StringVar(value="Disconnected")
        self.gyro_var = tk.StringVar(value="Gyro: --, --, --")
        self.accel_var = tk.StringVar(value="Accel: --, --, --")
        self.angle_var = tk.StringVar(value="Angle: --, --, --")

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

        self._draw_orientation_view()
        self._draw_accel_graph()

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
            )
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

    def _drain_log_queue(self):
        while not self.log_queue.empty():
            self._append_log(self.log_queue.get())
        self.root.after(100, self._drain_log_queue)

    def _drain_telemetry_queue(self):
        updated = False
        while not self.telemetry_queue.empty():
            telemetry = self.telemetry_queue.get()
            self.latest_telemetry = telemetry
            self.accel_history.append(
                (telemetry["accel_x"], telemetry["accel_y"], telemetry["accel_z"])
            )
            updated = True

        if updated:
            self._update_telemetry_labels()
            self._draw_orientation_view()
            self._draw_accel_graph()

        self.root.after(50, self._drain_telemetry_queue)

    def _update_telemetry_labels(self):
        if not self.latest_telemetry:
            return

        telemetry = self.latest_telemetry
        self.gyro_var.set(
            f"Gyro: {telemetry['gyro_x']:.3f}, {telemetry['gyro_y']:.3f}, {telemetry['gyro_z']:.3f} deg/s"
        )
        self.accel_var.set(
            f"Accel: {telemetry['accel_x']:.3f}, {telemetry['accel_y']:.3f}, {telemetry['accel_z']:.3f} g"
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
        canvas.create_text(12, 12, text="Compensated acceleration", anchor="nw", fill="#243242", font=("TkDefaultFont", 11, "bold"))
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
                normalized = max(-GRAPH_ACCEL_RANGE, min(GRAPH_ACCEL_RANGE, sample[axis_index])) / GRAPH_ACCEL_RANGE
                y_pos = margin + (usable_height / 2) - (normalized * usable_height / 2)
                points.extend((x_pos, y_pos))
            return points

        canvas.create_line(*to_points(0), fill="#d04a4a", width=2, smooth=True)
        canvas.create_line(*to_points(1), fill="#2b8a3e", width=2, smooth=True)
        canvas.create_line(*to_points(2), fill="#2463eb", width=2, smooth=True)
        canvas.create_text(width - margin, margin, text="ax", anchor="ne", fill="#d04a4a")
        canvas.create_text(width - margin, margin + 16, text="ay", anchor="ne", fill="#2b8a3e")
        canvas.create_text(width - margin, margin + 32, text="az", anchor="ne", fill="#2463eb")

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
            self._queue_log("Done.")
        except Exception as exc:
            self._queue_log(f"Error: {exc}")

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