"""Tkinter dashboard for observing the existing live gesture classifier.

This module deliberately consumes ``iter_prediction_results`` rather than
reimplementing feature extraction or KNN prediction.  The displayed values are
the classifier's existing display-only class proximity scores, normalized to
sum to 100%; they are not calibrated statistical probabilities.
"""

from __future__ import annotations

from pathlib import Path
import queue
import threading
import tkinter as tk
from tkinter import ttk

from .app import collect_baseline, iter_prediction_results
from .knn import CLASSES, load_model
from .sensor import Sensor


DISPLAY_NAMES = {
    "rest": "Rest",
    "wrist_up": "Wrist Up",
    "spread": "Spread",
    "fist": "Fist",
}
COLORS = {
    "rest": "#64748b",
    "wrist_up": "#38bdf8",
    "spread": "#a78bfa",
    "fist": "#fb7185",
}
Prediction = tuple[str, dict[str, float]]


class DetectionMonitor:
    """Run the unchanged detector in a worker and render its outputs."""

    def __init__(self, port: str, profile_path: Path) -> None:
        self.port = port
        self.profile_path = profile_path
        self.root = tk.Tk()
        self.root.title("eFlesh Detection Monitor")
        self.root.minsize(620, 470)
        self.root.configure(background="#0f172a")
        self._outputs: queue.Queue[Prediction] = queue.Queue(maxsize=1)
        self._status_updates: queue.Queue[str] = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._status = tk.StringVar(value="Connecting to sensor…")
        self._label = tk.StringVar(value="Waiting for readings")
        self._percentage = {label: tk.StringVar(value="0%") for label in CLASSES}
        self._bars: dict[str, ttk.Progressbar] = {}
        self._build()
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def _build(self) -> None:
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("Monitor.Horizontal.TProgressbar", troughcolor="#1e293b")

        body = tk.Frame(self.root, bg="#0f172a", padx=36, pady=30)
        body.pack(fill="both", expand=True)

        tk.Label(
            body,
            text="LIVE GESTURE DETECTION",
            bg="#0f172a",
            fg="#94a3b8",
            font=("Helvetica", 12, "bold"),
        ).pack(anchor="w")
        tk.Label(
            body,
            textvariable=self._label,
            bg="#0f172a",
            fg="#f8fafc",
            font=("Helvetica", 30, "bold"),
            pady=8,
        ).pack(anchor="w")
        tk.Label(
            body,
            textvariable=self._status,
            bg="#0f172a",
            fg="#cbd5e1",
            font=("Helvetica", 12),
        ).pack(anchor="w", pady=(0, 24))

        for label in CLASSES:
            row = tk.Frame(body, bg="#0f172a", pady=7)
            row.pack(fill="x")
            tk.Label(
                row,
                text=DISPLAY_NAMES[label],
                width=12,
                anchor="w",
                bg="#0f172a",
                fg="#e2e8f0",
                font=("Helvetica", 14, "bold"),
            ).pack(side="left")
            bar_style = f"{label}.Monitor.Horizontal.TProgressbar"
            style.configure(bar_style, background=COLORS[label])
            bar = ttk.Progressbar(
                row, maximum=100, mode="determinate", style=bar_style
            )
            bar.pack(side="left", fill="x", expand=True, padx=(12, 14))
            self._bars[label] = bar
            tk.Label(
                row,
                textvariable=self._percentage[label],
                width=5,
                anchor="e",
                bg="#0f172a",
                fg="#f8fafc",
                font=("Helvetica", 14, "bold"),
            ).pack(side="right")

        tk.Label(
            body,
            text=(
                "Scores show relative KNN proximity, normalized across gestures. "
                "They are a monitoring signal, not calibrated probabilities."
            ),
            justify="left",
            wraplength=560,
            bg="#0f172a",
            fg="#94a3b8",
            font=("Helvetica", 10),
            pady=24,
        ).pack(anchor="w")

    def start(self) -> None:
        self._thread = threading.Thread(
            target=self._run_detector, name="eflesh-monitor", daemon=True
        )
        self._thread.start()
        self.root.after(50, self._refresh)
        self.root.mainloop()

    def close(self) -> None:
        self._stop.set()
        self.root.destroy()

    def _publish(self, prediction: Prediction) -> None:
        try:
            self._outputs.put_nowait(prediction)
        except queue.Full:
            self._outputs.get_nowait()
            self._outputs.put_nowait(prediction)

    def _set_status(self, value: str) -> None:
        """Pass worker status to Tk's main thread without touching Tk there."""
        try:
            self._status_updates.put_nowait(value)
        except queue.Full:
            self._status_updates.get_nowait()
            self._status_updates.put_nowait(value)

    def _run_detector(self) -> None:
        try:
            model = load_model(self.profile_path)
            with Sensor(self.port) as sensor:
                self._set_status("Keep your hand relaxed while baseline is collected…")
                baseline = collect_baseline(sensor)
                self._set_status("Live — reading timestamped sensor frames")
                for label, scores in iter_prediction_results(sensor, model, baseline):
                    if self._stop.is_set():
                        return
                    self._publish((label, scores))
        except Exception as exc:
            if not self._stop.is_set():
                self._set_status(f"Sensor error: {exc}")

    def _refresh(self) -> None:
        try:
            while True:
                self._status.set(self._status_updates.get_nowait())
        except queue.Empty:
            pass
        try:
            while True:
                label, scores = self._outputs.get_nowait()
                self._label.set(DISPLAY_NAMES[label])
                for gesture, score in scores.items():
                    percentage = score * 100
                    self._bars[gesture]["value"] = percentage
                    self._percentage[gesture].set(f"{percentage:.0f}%")
        except queue.Empty:
            pass
        if self.root.winfo_exists():
            self.root.after(50, self._refresh)


def monitor(port: str, profile_path: Path) -> None:
    """Open the desktop monitor for a calibrated profile and serial port."""
    DetectionMonitor(port, profile_path).start()
