import csv
import math
import queue
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from .analysis import correlation, finite_difference_accel_from_displacement, resample_nearest, rmse
from .config import (
    ACCEL_COLOR,
    ACTUAL_COLOR,
    CMPS2_TO_MPS2,
    CM_TO_M,
    DEFAULT_BAUDRATE,
    DEFAULT_SAMPLE_INTERVAL_S,
    DISP_COLOR,
    ERROR_COLOR,
    GRAPH_AXIS,
    GRAPH_BG,
    GRAPH_GRID,
    PREFERRED_ACCEL_PORT,
    PREFERRED_MOTOR_PORT,
)
from .data_io import load_first_numeric_column
from .motion import generate_rows_from_accel, generate_rows_from_displacement, merge_rows, prepare_accel
from .serial_client import SerialClient, available_ports


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

        self.notebook.add(self.tab_monitor, text="Monitor")
        self.notebook.add(self.tab_graphs, text="Earthquake Graphs")
        self.notebook.add(self.tab_table, text="Data Table")
        self.notebook.add(self.tab_correction, text="Correction Analysis")
        self.notebook.add(self.tab_calibration, text="Calibration")

        self._build_monitor_tab()
        self._build_graphs_tab()
        self._build_table_tab()
        self._build_correction_tab()
        self._build_calibration_tab()

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
        ttk.Button(live, text="Clear Graph", command=lambda: self._clear_live()).grid(row=2, column=0, sticky="w", pady=(6, 0))
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

        rt = ttk.LabelFrame(tab, text="Measured Acceleration from IMU", padding=4)
        rt.grid(row=1, column=0, sticky="nsew", pady=(8, 0))
        rt.columnconfigure(0, weight=1)
        rt.rowconfigure(0, weight=1)
        self.rt_canvas = tk.Canvas(rt, bg=GRAPH_BG, height=210, highlightthickness=0)
        self.rt_canvas.grid(row=0, column=0, sticky="nsew")

        pos = ttk.LabelFrame(tab, text="Commanded Position", padding=4)
        pos.grid(row=2, column=0, sticky="nsew", pady=(8, 0))
        pos.columnconfigure(0, weight=1)
        pos.rowconfigure(0, weight=1)
        self.pos_canvas = tk.Canvas(pos, bg=GRAPH_BG, height=210, highlightthickness=0)
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
        for col, text in headings.items():
            self.table.heading(col, text=text)
            self.table.column(col, width=110, anchor="center")
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
        bias = ttk.LabelFrame(tab, text="IMU Bias Calibration", padding=8)
        bias.grid(row=0, column=0, sticky="ew")
        ttk.Label(bias, text="Keep the table stationary, then collect bias samples.").grid(row=0, column=0, columnspan=3, sticky="w")
        ttk.Button(bias, text="Calibrate 30 seconds", command=lambda: self._start_bias_calibration(30.0)).grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Button(bias, text="Calibrate 10 seconds", command=lambda: self._start_bias_calibration(10.0)).grid(row=1, column=1, sticky="w", padx=(8, 0), pady=(8, 0))
        ttk.Label(bias, textvariable=self.noise_var, foreground="#374151").grid(row=1, column=2, sticky="w", padx=(12, 0), pady=(8, 0))

        sweep = ttk.LabelFrame(tab, text="Frequency Sweep", padding=8)
        sweep.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        ttk.Label(sweep, text="Sweep identification is reserved for the next hardware-safe step. Bias calibration and response plotting are ready first.").grid(row=0, column=0, sticky="w")

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
            self._set_import(path, "accel", raw, rows, accel, [r["unclamped_position_cm"] / 100.0 for r in rows])
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
            displacement = [v - start for v in displacement]
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
        floats = []
        for part in parts:
            try:
                floats.append(float(part))
            except ValueError:
                continue
        if not floats:
            return None
        return floats[0]

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
        variance = sum((v - mean) ** 2 for v in self.bias_samples) / len(self.bias_samples)
        std = math.sqrt(variance)
        self.bias_var.set(mean)
        self.noise_var.set(f"Bias: {mean:.5f} m/s2 | noise std: {std:.5f} | samples: {len(self.bias_samples)}")

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
            input_series = [v * 100.0 for v in self.input_displacement_m]
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
        self._draw_series(self.pos_canvas, [r["position_cm"] for r in self.rows], "Commanded position (cm)", DISP_COLOR)

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
        for i in range(5):
            y = margin + i * usable_h / 4
            canvas.create_line(margin, y, width - margin, y, fill=GRAPH_GRID)
        canvas.create_line(margin, margin, margin, height - margin, fill=GRAPH_AXIS)
        canvas.create_line(margin, height - margin, width - margin, height - margin, fill=GRAPH_AXIS)
        points = []
        n = len(values)
        for i, value in enumerate(values):
            x = margin + i * usable_w / (n - 1)
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
        self.correction_summary_var.set(f"RMSE: {rmse_text} | correlation: {corr_text} | samples: target {len(self.target_accel)}, measured {len(self.rt_accel)}")
        self._draw_overlay(self.overlay_canvas, self.target_accel, actual)
        errors = [self.target_accel[i] - actual[i] for i in range(min(len(self.target_accel), len(actual)))]
        self._draw_series(self.error_canvas, errors, "target - measured (m/s2)", ERROR_COLOR)

    def _draw_overlay(self, canvas, expected, actual):
        self._draw_series(canvas, expected, "Blue target, red measured", ACCEL_COLOR)
        if not actual or canvas is None or not canvas.winfo_exists():
            return
        width = max(canvas.winfo_width(), 600)
        height = max(canvas.winfo_height(), 180)
        margin = 34
        combined = expected + actual
        vmin = min(combined)
        vmax = max(combined)
        if abs(vmax - vmin) < 1e-12:
            return
        usable_w = width - 2 * margin
        usable_h = height - 2 * margin
        n = len(actual)
        points = []
        for i, value in enumerate(actual):
            x = margin + i * usable_w / max(n - 1, 1)
            y = margin + (vmax - value) * usable_h / (vmax - vmin)
            points.extend((x, y))
        canvas.create_line(*points, fill=ACTUAL_COLOR, width=2)

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
        count = max((len(v) for v in values), default=0)
        with open(path, "w", newline="", encoding="utf-8") as file_obj:
            writer = csv.writer(file_obj)
            writer.writerow(headers)
            for i in range(count):
                writer.writerow([col[i] if i < len(col) else "" for col in values])

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
    app = ShakingTableApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
