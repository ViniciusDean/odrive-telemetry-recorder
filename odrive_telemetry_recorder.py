from __future__ import annotations

import csv
import ctypes
import json
import math
import os
import queue
import struct
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk

import hid


VID = 0x1209
PID = 0x0D40
REPORT_ID = 0x01
HISTORY_INTERVAL_S = 0.02
MAX_HISTORY = 180_000
SETTINGS_FILE_NAME = "odrive_telemetry_recorder_settings.json"


def enable_windows_dpi_awareness() -> None:
    """Prevent Windows from bitmap-scaling the Tkinter window on HiDPI monitors."""
    if sys.platform != "win32":
        return
    try:
        # Per-monitor V2 keeps the UI sharp when moving it between monitors.
        ctypes.windll.user32.SetProcessDpiAwarenessContext(ctypes.c_void_p(-4))
        return
    except Exception:
        pass
    try:
        ctypes.windll.shcore.SetProcessDpiAwareness(2)
        return
    except Exception:
        pass
    try:
        ctypes.windll.user32.SetProcessDPIAware()
    except Exception:
        pass


enable_windows_dpi_awareness()


def application_directory() -> Path:
    """Use the executable folder when packaged, never PyInstaller's temp folder."""
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent
    return Path(__file__).resolve().parent


@dataclass(frozen=True)
class Sample:
    t: float
    position_deg: float
    rpm: float
    iq: float
    torque: float
    vbus: float
    ibus: float
    ibrake: float


def parse_report(raw: bytes | list[int], wheel_range_deg: float) -> Sample | None:
    data = bytes(raw)
    # hidapi on Windows returns the report ID, while some backends do not.
    # Do not infer it from the first payload byte: button 1 can also be 0x01.
    if len(data) == 31:
        if data[0] != REPORT_ID:
            return None
        data = data[1:]
    elif len(data) != 30:
        return None
    position_raw = struct.unpack_from("<h", data, 8)[0]
    turns_s = struct.unpack_from("<h", data, 10)[0] / 1000.0
    return Sample(
        t=time.perf_counter(),
        position_deg=position_raw / 32767.0 * wheel_range_deg / 2.0,
        rpm=turns_s * 60.0,
        iq=struct.unpack_from("<h", data, 12)[0] / 1000.0,
        torque=struct.unpack_from("<h", data, 20)[0] / 1000.0,
        vbus=struct.unpack_from("<h", data, 24)[0] / 100.0,
        ibus=struct.unpack_from("<h", data, 26)[0] / 100.0,
        ibrake=struct.unpack_from("<h", data, 28)[0] / 100.0,
    )


class HidReader(threading.Thread):
    def __init__(self, path: bytes, range_getter, output: queue.Queue):
        super().__init__(daemon=True)
        self.path = path
        self.range_getter = range_getter
        self.output = output
        self.stop_event = threading.Event()
        self.opened = threading.Event()
        self.finished = threading.Event()
        self.error: str | None = None
        self.device = None

    def stop(self) -> None:
        self.stop_event.set()
        # A close unblocks a pending native HID read, so the next session does
        # not race the old reader for the same device handle.
        if self.device:
            try:
                self.device.close()
            except Exception:
                pass

    def run(self) -> None:
        try:
            self.device = hid.device()
            self.device.open_path(self.path)
            self.device.set_nonblocking(1)
            self.opened.set()
            while not self.stop_event.is_set():
                raw = self.device.read(64)
                if not raw:
                    time.sleep(0.001)
                    continue
                sample = parse_report(raw, self.range_getter())
                if sample:
                    try:
                        self.output.put_nowait(sample)
                    except queue.Full:
                        pass
        except Exception as exc:
            if not self.stop_event.is_set():
                self.error = str(exc)
        finally:
            if self.device:
                try:
                    self.device.close()
                except Exception:
                    pass
            self.finished.set()


class Session:
    def __init__(self, directory: Path, max_torque: float, current_limit: float, brake_ohm: float, phase_ohm: float):
        self.start = time.perf_counter()
        self.max_torque = max_torque
        self.current_limit = current_limit
        self.brake_ohm = brake_ohm
        self.phase_ohm = phase_ohm
        self.count = 0
        self.torque_clip_count = 0
        self.clip_count = 0
        self.last_history = -math.inf
        self.history: list[tuple[float, float, float]] = []
        self.truncated = False
        self.extrema = {key: [math.inf, -math.inf] for key in (
            "vbus", "torque", "iq", "ibus", "ibrake", "rpm", "p_brake", "p_copper"
        )}
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.path = directory / f"odrive_telemetry_{stamp}.csv"
        self.file = self.path.open("w", newline="", encoding="utf-8", buffering=1_048_576)
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            "t_s", "vbus_V", "ibus_A", "ibrake_A", "iq_A", "torque_Nm",
            "speed_rpm", "position_deg", "p_brake_W", "p_copper_W", "p_mech_W",
        ])
        self.pending_flush = 0

    def duration(self) -> float:
        return time.perf_counter() - self.start

    def track(self, key: str, value: float) -> None:
        if math.isfinite(value):
            self.extrema[key][0] = min(self.extrema[key][0], value)
            self.extrema[key][1] = max(self.extrema[key][1], value)

    def add(self, sample: Sample) -> None:
        elapsed = sample.t - self.start
        p_brake = self.brake_ohm * sample.ibrake * sample.ibrake
        p_copper = 1.5 * self.phase_ohm * sample.iq * sample.iq
        p_mech = sample.torque * sample.rpm / 60.0 * 2.0 * math.pi
        self.count += 1
        self.torque_clip_count += int(abs(sample.torque) >= self.max_torque * 0.98)
        self.clip_count += int(abs(sample.iq) >= self.current_limit * 0.95)
        for key, value in (
            ("vbus", sample.vbus), ("torque", sample.torque), ("iq", sample.iq),
            ("ibus", sample.ibus), ("ibrake", sample.ibrake), ("rpm", sample.rpm),
            ("p_brake", p_brake), ("p_copper", p_copper),
        ):
            self.track(key, value)
        self.writer.writerow([
            f"{elapsed:.6f}", f"{sample.vbus:.4f}", f"{sample.ibus:.4f}",
            f"{sample.ibrake:.4f}", f"{sample.iq:.4f}", f"{sample.torque:.4f}",
            f"{sample.rpm:.4f}", f"{sample.position_deg:.4f}", f"{p_brake:.4f}",
            f"{p_copper:.4f}", f"{p_mech:.4f}",
        ])
        self.pending_flush += 1
        if self.pending_flush >= 1_000:
            self.file.flush()
            self.pending_flush = 0
        if elapsed - self.last_history >= HISTORY_INTERVAL_S:
            if len(self.history) < MAX_HISTORY:
                self.history.append((elapsed, sample.vbus, sample.torque))
            else:
                self.truncated = True
            self.last_history = elapsed

    def close(self) -> None:
        self.file.flush()
        self.file.close()

    def write_summary(self) -> Path:
        duration = self.duration()
        summary_path = self.path.with_name(self.path.stem + "_summary.csv")
        with summary_path.open("w", newline="", encoding="utf-8") as summary_file:
            writer = csv.writer(summary_file)
            writer.writerow(["metric", "value", "unit"])
            writer.writerows([
                ("duration", f"{duration:.3f}", "s"),
                ("hid_samples", self.count, "samples"),
                ("average_hid_rate", f"{self.count / duration:.2f}" if duration else "0", "Hz"),
                ("max_torque_setting", f"{self.max_torque:.3f}", "Nm"),
                ("ffb_clip_percent", f"{self.torque_clip_count / self.count * 100:.3f}" if self.count else "0", "%"),
                ("current_limit_setting", f"{self.current_limit:.3f}", "A"),
                ("current_clip_percent", f"{self.clip_count / self.count * 100:.3f}" if self.count else "0", "%"),
                ("vbus_min", f"{self.extrema['vbus'][0]:.4f}", "V"),
                ("vbus_max", f"{self.extrema['vbus'][1]:.4f}", "V"),
                ("torque_min", f"{self.extrema['torque'][0]:.4f}", "Nm"),
                ("torque_max", f"{self.extrema['torque'][1]:.4f}", "Nm"),
                ("iq_min", f"{self.extrema['iq'][0]:.4f}", "A"),
                ("iq_max", f"{self.extrema['iq'][1]:.4f}", "A"),
                ("ibus_min", f"{self.extrema['ibus'][0]:.4f}", "A"),
                ("ibus_max", f"{self.extrema['ibus'][1]:.4f}", "A"),
                ("ibrake_min", f"{self.extrema['ibrake'][0]:.4f}", "A"),
                ("ibrake_max", f"{self.extrema['ibrake'][1]:.4f}", "A"),
                ("rpm_min", f"{self.extrema['rpm'][0]:.4f}", "RPM"),
                ("rpm_max", f"{self.extrema['rpm'][1]:.4f}", "RPM"),
                ("brake_power_max", f"{self.extrema['p_brake'][1]:.4f}", "W"),
                ("copper_power_max", f"{self.extrema['p_copper'][1]:.4f}", "W"),
            ])
        return summary_path


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ODrive Telemetry Recorder")
        self.geometry("1180x760")
        self.minsize(900, 620)
        self.configure(bg="#10141b")
        self.queue: queue.Queue[Sample] = queue.Queue(maxsize=20_000)
        self.device_info: dict | None = None
        self.reader: HidReader | None = None
        self.session: Session | None = None
        self.output_dir = application_directory() / "recordings"
        self.settings_path = application_directory() / SETTINGS_FILE_NAME
        self.settings = self.load_settings()
        self.last_render = 0.0

        self.range_var = tk.StringVar(value=self.settings.get("wheel_range", "900"))
        self.max_torque_var = tk.StringVar(value=self.settings.get("max_torque", "10"))
        self.current_var = tk.StringVar(value=self.settings.get("current_limit", "20"))
        self.brake_var = tk.StringVar(value=self.settings.get("brake_resistance", "6"))
        self.phase_var = tk.StringVar(value=self.settings.get("phase_resistance", "0.329"))
        self.status_var = tk.StringVar(value="Desconectado. Clique em Procurar volante.")
        self.file_var = tk.StringVar(value="Nenhuma sessao gravada.")
        self.values = {key: tk.StringVar(value="--") for key in (
            "duration", "rate", "vbus", "torque", "iq", "ibus", "ibrake",
            "rpm", "ffb_clip", "clip", "p_brake", "p_copper"
        )}
        self._style()
        self._ui()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(100, self.pump)

    def _style(self) -> None:
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure(".", background="#10141b", foreground="#e7edf5", font=("Segoe UI", 10))
        style.configure("TFrame", background="#10141b")
        style.configure("Card.TFrame", background="#171d27")
        style.configure("TLabel", background="#10141b", foreground="#e7edf5")
        style.configure("Card.TLabel", background="#171d27", foreground="#e7edf5")
        style.configure("Muted.TLabel", background="#10141b", foreground="#94a3b8")
        style.configure("MetricName.TLabel", background="#1c2431", foreground="#9aa8ba", font=("Segoe UI", 9))
        style.configure("MetricValue.TLabel", background="#1c2431", foreground="#f6f8fc", font=("Cascadia Mono", 13, "bold"))
        style.configure("MetricHint.TLabel", background="#1c2431", foreground="#7f8ca0", font=("Cascadia Mono", 8))
        style.configure("Accent.TButton", background="#635bff", foreground="white", padding=(14, 8), font=("Segoe UI", 10, "bold"))
        style.map("Accent.TButton", background=[("active", "#5148e5"), ("disabled", "#2d3141")])
        style.configure("TButton", background="#252e3d", foreground="#e7edf5", padding=(12, 7))
        style.map("TButton", background=[("active", "#303c50"), ("disabled", "#1a202b")])
        style.configure("TEntry", fieldbackground="#0d1118", foreground="#e7edf5", insertcolor="#e7edf5")

    @staticmethod
    def card(parent) -> ttk.Frame:
        return ttk.Frame(parent, style="Card.TFrame", padding=18)

    def _ui(self) -> None:
        root = ttk.Frame(self, padding=20)
        root.pack(fill="both", expand=True)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(3, weight=1)

        header = self.card(root)
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)
        ttk.Label(header, text="ODrive Telemetry Recorder", style="Card.TLabel", font=("Segoe UI", 18, "bold")).grid(row=0, column=0, sticky="w")
        ttk.Label(header, text="App externo: le a telemetria HID sem abrir o configurador ou enviar comandos ao motor.", style="Card.TLabel").grid(row=1, column=0, sticky="w", pady=(5, 0))
        ttk.Label(header, textvariable=self.status_var, style="Card.TLabel").grid(row=2, column=0, sticky="w", pady=(12, 0))
        controls = ttk.Frame(header, style="Card.TFrame")
        controls.grid(row=0, column=1, rowspan=3, sticky="e")
        self.scan_button = ttk.Button(controls, text="Procurar volante", command=self.scan)
        self.scan_button.grid(row=0, column=0, padx=(0, 8))
        self.start_button = ttk.Button(controls, text="Iniciar gravacao", style="Accent.TButton", command=self.start_recording, state="disabled")
        self.start_button.grid(row=0, column=1, padx=(0, 8))
        self.stop_button = ttk.Button(controls, text="Parar e analisar", command=self.stop_recording, state="disabled")
        self.stop_button.grid(row=0, column=2)
        ttk.Button(controls, text="Abrir CSVs", command=self.open_folder).grid(row=1, column=0, columnspan=3, sticky="ew", pady=(8, 0))

        settings = self.card(root)
        settings.grid(row=1, column=0, sticky="ew", pady=(12, 0))
        ttk.Label(settings, text="Parametros para analise", style="Card.TLabel", font=("Segoe UI", 11, "bold")).grid(row=0, column=0, columnspan=10, sticky="w")
        fields = [
            ("Range do volante", self.range_var, "graus"),
            ("Max torque FFB", self.max_torque_var, "Nm"),
            ("Current limit", self.current_var, "A"),
            ("Resistor de freio", self.brake_var, "ohm"),
            ("R de fase", self.phase_var, "ohm"),
        ]
        for index, (label, variable, unit) in enumerate(fields):
            col = index * 2
            ttk.Label(settings, text=label, style="Card.TLabel").grid(row=1, column=col, sticky="w", pady=(10, 2))
            if label == "Max torque FFB":
                ttk.Spinbox(
                    settings,
                    textvariable=variable,
                    from_=0.5,
                    to=30.0,
                    increment=0.5,
                    width=11,
                    format="%.1f",
                ).grid(row=2, column=col, sticky="w")
            else:
                ttk.Entry(settings, textvariable=variable, width=11).grid(row=2, column=col, sticky="w")
            ttk.Label(settings, text=unit, style="Card.TLabel").grid(row=2, column=col + 1, sticky="w", padx=(6, 18))
        ttk.Label(settings, text="Use os valores da sua configuracao. Max torque e usado para detectar clipping de FFB; os demais afetam potencia e clipping fisico.", style="Card.TLabel").grid(row=3, column=0, columnspan=10, sticky="w", pady=(12, 0))

        summary = self.card(root)
        summary.grid(row=2, column=0, sticky="ew", pady=(12, 0))
        ttk.Label(summary, text="Resumo da sessao", style="Card.TLabel", font=("Segoe UI", 11, "bold")).grid(row=0, column=0, columnspan=5, sticky="w")
        metrics = [
            ("Duracao", "duration", "amostras / taxa HID"), ("Vbus", "vbus", "min / max"),
            ("Torque", "torque", "min / max"), ("Iq", "iq", "pico absoluto"),
            ("Ibus", "ibus", "regen / consumo"), ("I do freio", "ibrake", "pico absoluto"),
            ("RPM", "rpm", "min / max"), ("Clip de corrente", "clip", ">= 95% do limite"),
            ("Clip FFB", "ffb_clip", ">= 98% do max Nm"),
            ("P freio", "p_brake", "pico estimado"), ("P cobre", "p_copper", "pico estimado"),
        ]
        for index, (title, key, hint) in enumerate(metrics):
            row, col = 1 + index // 5, index % 5
            tile = tk.Frame(summary, bg="#1c2431", padx=10, pady=8)
            tile.grid(row=row, column=col, sticky="nsew", padx=4, pady=4)
            ttk.Label(tile, text=title, style="MetricName.TLabel").pack(anchor="w")
            ttk.Label(tile, textvariable=self.values[key], style="MetricValue.TLabel").pack(anchor="w", pady=(5, 2))
            ttk.Label(tile, text=hint, style="MetricHint.TLabel").pack(anchor="w")
            summary.columnconfigure(col, weight=1)

        chart = self.card(root)
        chart.grid(row=3, column=0, sticky="nsew", pady=(12, 0))
        chart.columnconfigure(0, weight=1)
        chart.rowconfigure(1, weight=1)
        ttk.Label(chart, text="Historico da sessao", style="Card.TLabel", font=("Segoe UI", 11, "bold")).grid(row=0, column=0, sticky="w")
        ttk.Label(chart, text="Vbus azul, torque vermelho. CSV grava todas as amostras HID.", style="Card.TLabel").grid(row=0, column=0, sticky="e")
        self.canvas = tk.Canvas(chart, background="#0d1118", highlightthickness=0, height=230)
        self.canvas.grid(row=1, column=0, sticky="nsew", pady=(10, 0))
        self.canvas.bind("<Configure>", lambda _event: self.draw_chart())

        footer = ttk.Frame(root)
        footer.grid(row=4, column=0, sticky="ew", pady=(10, 0))
        footer.columnconfigure(0, weight=1)
        ttk.Label(footer, textvariable=self.file_var, style="Muted.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Button(footer, text="Abrir pasta das gravacoes", command=self.open_folder).grid(row=0, column=1, sticky="e")

    @staticmethod
    def number(value: tk.StringVar, label: str) -> float:
        try:
            result = float(value.get().replace(",", "."))
        except ValueError as exc:
            raise ValueError(f"{label} invalido.") from exc
        if result <= 0:
            raise ValueError(f"{label} deve ser maior que zero.")
        return result

    def load_settings(self) -> dict[str, str]:
        try:
            settings = json.loads(self.settings_path.read_text(encoding="utf-8"))
            if isinstance(settings, dict):
                return {key: str(value) for key, value in settings.items()}
        except (OSError, json.JSONDecodeError):
            pass
        return {}

    def save_settings(self) -> None:
        settings = {
            "wheel_range": self.range_var.get().strip(),
            "max_torque": self.max_torque_var.get().strip(),
            "current_limit": self.current_var.get().strip(),
            "brake_resistance": self.brake_var.get().strip(),
            "phase_resistance": self.phase_var.get().strip(),
        }
        temporary_path = self.settings_path.with_suffix(".tmp")
        try:
            temporary_path.write_text(json.dumps(settings, indent=2) + "\n", encoding="utf-8")
            temporary_path.replace(self.settings_path)
        except OSError:
            try:
                temporary_path.unlink(missing_ok=True)
            except OSError:
                pass

    def wheel_range(self) -> float:
        try:
            return self.number(self.range_var, "Range")
        except ValueError:
            return 900.0

    def scan(self) -> None:
        devices = hid.enumerate(VID, PID)
        if not devices:
            self.device_info = None
            self.start_button.configure(state="disabled")
            self.status_var.set("ODrive-Wheel HID nao encontrado. Conecte o USB e tente novamente.")
            return
        self.device_info = devices[0]
        name = self.device_info.get("product_string") or "ODrive-Wheel HID"
        self.status_var.set(f"Volante encontrado: {name}. Pronto para gravar.")
        self.start_button.configure(state="normal")

    def start_recording(self) -> None:
        if self.reader and self.reader.is_alive():
            self.status_var.set("A sessao anterior ainda esta liberando o HID. Aguarde alguns instantes.")
            return
        if not self.device_info:
            self.scan()
            if not self.device_info:
                return
        try:
            max_torque = self.number(self.max_torque_var, "Max torque FFB")
            current = self.number(self.current_var, "Current limit")
            brake = self.number(self.brake_var, "Resistor de freio")
            phase = self.number(self.phase_var, "R de fase")
        except ValueError as exc:
            messagebox.showerror("Parametro invalido", str(exc), parent=self)
            return
        self.save_settings()
        while True:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                break
        self.session = Session(self.output_dir, max_torque, current, brake, phase)
        self.reader = HidReader(self.device_info["path"], self.wheel_range, self.queue)
        self.reader.start()
        self.scan_button.configure(state="disabled")
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.status_var.set("Conectando ao volante...")
        self.file_var.set(f"Gravando: {self.session.path.name}")
        self.after(50, self.await_reader_open)

    def await_reader_open(self) -> None:
        reader = self.reader
        if not reader:
            return
        if reader.error:
            self.abort_unstarted_session(f"Nao foi possivel abrir o HID: {reader.error}")
        elif reader.opened.is_set():
            self.status_var.set("Gravando telemetria HID em CSV...")
        elif reader.finished.is_set():
            self.abort_unstarted_session("O leitor HID foi encerrado antes de abrir o volante.")
        else:
            self.after(50, self.await_reader_open)

    def abort_unstarted_session(self, message: str) -> None:
        if self.reader:
            self.reader.stop()
            self.reader.join(timeout=2.0)
        self.reader = None
        if self.session:
            self.session.close()
            try:
                if self.session.count == 0:
                    self.session.path.unlink(missing_ok=True)
            except OSError:
                pass
        self.session = None
        self.status_var.set(message)
        self.file_var.set("Nenhuma sessao gravada.")
        self.scan_button.configure(state="normal")
        self.start_button.configure(state="normal" if self.device_info else "disabled")
        self.stop_button.configure(state="disabled")

    def stop_recording(self) -> None:
        if self.reader:
            self.reader.stop()
            self.reader.join(timeout=2.0)
            if self.reader.is_alive():
                self.status_var.set("O leitor HID ainda nao encerrou. Feche e reabra o app antes de iniciar outra sessao.")
                return
            self.reader = None
        self.drain()
        if self.session:
            self.session.close()
            summary_path = self.session.write_summary()
            self.status_var.set(f"Sessao finalizada. CSVs salvos em: {self.output_dir}")
            self.file_var.set(f"CSVs salvos: {self.session.path.name} e {summary_path.name}")
        self.scan_button.configure(state="normal")
        self.start_button.configure(state="normal" if self.device_info else "disabled")
        self.stop_button.configure(state="disabled")
        self.refresh()
        self.draw_chart()

    def drain(self) -> None:
        if not self.session:
            return
        while True:
            try:
                self.session.add(self.queue.get_nowait())
            except queue.Empty:
                return

    def pump(self) -> None:
        if self.reader and self.reader.error:
            error = self.reader.error
            self.stop_recording()
            self.status_var.set(f"Leitura HID interrompida: {error}")
        self.drain()
        if self.session and time.perf_counter() - self.last_render > 0.2:
            self.refresh()
            self.draw_chart()
            self.last_render = time.perf_counter()
        self.after(100, self.pump)

    @staticmethod
    def fmt_range(bounds: list[float], digits: int, unit: str) -> str:
        return "--" if not math.isfinite(bounds[0]) else f"{bounds[0]:.{digits}f} / {bounds[1]:.{digits}f} {unit}"

    def refresh(self) -> None:
        if not self.session:
            return
        s, e = self.session, self.session.extrema
        duration = s.duration()
        self.values["duration"].set(f"{duration:.1f} s")
        self.values["rate"].set(f"{s.count:,} / {s.count / duration:.0f} Hz" if duration else "--")
        self.values["vbus"].set(self.fmt_range(e["vbus"], 2, "V"))
        self.values["torque"].set(self.fmt_range(e["torque"], 2, "Nm"))
        self.values["iq"].set("--" if not math.isfinite(e["iq"][0]) else f"{max(abs(e['iq'][0]), abs(e['iq'][1])):.2f} A")
        self.values["ibus"].set(self.fmt_range(e["ibus"], 2, "A"))
        self.values["ibrake"].set("--" if not math.isfinite(e["ibrake"][0]) else f"{max(abs(e['ibrake'][0]), abs(e['ibrake'][1])):.2f} A")
        self.values["rpm"].set(self.fmt_range(e["rpm"], 0, "RPM"))
        self.values["ffb_clip"].set(f"{s.torque_clip_count / s.count * 100:.2f} %" if s.count else "--")
        self.values["clip"].set(f"{s.clip_count / s.count * 100:.2f} %" if s.count else "--")
        self.values["p_brake"].set("--" if not math.isfinite(e["p_brake"][1]) else f"{e['p_brake'][1]:.1f} W")
        self.values["p_copper"].set("--" if not math.isfinite(e["p_copper"][1]) else f"{e['p_copper'][1]:.1f} W")

    def draw_chart(self) -> None:
        self.canvas.delete("all")
        if not self.session or not self.session.history:
            self.canvas.create_text(max(self.canvas.winfo_width(), 1) / 2, max(self.canvas.winfo_height(), 1) / 2, text="Inicie uma gravacao para ver Vbus e torque.", fill="#94a3b8", font=("Segoe UI", 11))
            return
        history = self.session.history
        width, height = max(self.canvas.winfo_width(), 1), max(self.canvas.winfo_height(), 1)
        left, right, top, bottom = 62, 62, 18, 28
        plot_w, plot_h = width - left - right, height - top - bottom
        if plot_w < 2 or plot_h < 2:
            return
        duration = max(history[-1][0], 0.001)
        torque_peak = max(1.0, max(abs(item[2]) for item in history))
        v_min, v_max = min(item[1] for item in history), max(item[1] for item in history)
        pad = max(0.5, (v_max - v_min) * 0.15)
        v_min, v_max = v_min - pad, v_max + pad
        v_span = max(0.1, v_max - v_min)
        for index in range(5):
            y = top + index * plot_h / 4
            self.canvas.create_line(left, y, width - right, y, fill="#293241")
        self.canvas.create_line(left, top + plot_h / 2, width - right, top + plot_h / 2, fill="#566274")
        step = max(1, len(history) // max(1, int(plot_w)))
        v_points, t_points = [], []
        for index in range(0, len(history), step):
            t, vbus, torque = history[index]
            x = left + t / duration * plot_w
            v_points.extend((x, top + (v_max - vbus) / v_span * plot_h))
            t_points.extend((x, top + (torque_peak - torque) / (2 * torque_peak) * plot_h))
        if len(v_points) >= 4:
            self.canvas.create_line(*v_points, fill="#4fc3f7", width=2)
            self.canvas.create_line(*t_points, fill="#ef5350", width=2)
        mono = ("Cascadia Mono", 9)
        self.canvas.create_text(5, top, anchor="nw", text=f"+{torque_peak:.1f} Nm", fill="#ef5350", font=mono)
        self.canvas.create_text(5, height - bottom, anchor="sw", text=f"-{torque_peak:.1f} Nm", fill="#ef5350", font=mono)
        self.canvas.create_text(width - 5, top, anchor="ne", text=f"{v_max:.1f} V", fill="#4fc3f7", font=mono)
        self.canvas.create_text(width - 5, height - bottom, anchor="se", text=f"{v_min:.1f} V", fill="#4fc3f7", font=mono)
        self.canvas.create_text(left, height - 6, anchor="sw", text="0 s", fill="#94a3b8", font=mono)
        self.canvas.create_text(width - right, height - 6, anchor="se", text=f"{duration:.1f} s", fill="#94a3b8", font=mono)

    def open_folder(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        os.startfile(self.output_dir)

    def on_close(self) -> None:
        self.save_settings()
        if self.reader:
            self.stop_recording()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
