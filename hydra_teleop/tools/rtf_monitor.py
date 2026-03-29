#!/usr/bin/env python3
"""
RTF (Real Time Factor) monitor for Gazebo.
Polls /world/{world}/stats via `gz topic` in a background thread and prints RTF.
"""

import re
import subprocess
import threading


_RTF_RE = re.compile(r"real_time_factor:\s*([\d.]+(?:e[+-]?\d+)?)", re.IGNORECASE)


class RtfMonitor:
    """
    Spawns a background thread that periodically fetches one message from
    /world/{world_name}/stats and prints the real_time_factor.
    """

    def __init__(self, world_name: str = "airport", interval_s: float = 5.0):
        self._topic = f"/world/{world_name}/stats"
        self._interval = interval_s
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="rtf-monitor")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None

    def _fetch_rtf(self) -> float | None:
        try:
            result = subprocess.run(
                ["gz", "topic", "-e", "-t", self._topic, "-n", "1"],
                capture_output=True,
                text=True,
                timeout=3.0,
            )
            m = _RTF_RE.search(result.stdout)
            if m:
                return float(m.group(1))
        except Exception:
            pass
        return None

    def _run(self) -> None:
        while not self._stop.is_set():
            rtf = self._fetch_rtf()
            if rtf is not None:
                print(f"[RTF] {rtf:.3f}")
            self._stop.wait(self._interval)
