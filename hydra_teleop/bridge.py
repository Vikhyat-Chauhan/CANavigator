#!/usr/bin/env python3
# hydra_teleop/bridge.py
import subprocess
import threading
from typing import Iterable, Optional, Sequence, Tuple, Union

Triplet = Tuple[str, str, str]  # (topic, ros_type, gz_type)


class RosGzBridge:
    """
    Owns a single `ros2 run ros_gz_bridge parameter_bridge ...` process
    with one or more mappings, and streams its logs to stdout.
    """
    def __init__(self, maps: Sequence[Triplet]):
        self._maps: Sequence[Triplet] = list(maps)
        self._proc: Optional[subprocess.Popen] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()

    def start(self):
        if self._proc and self._proc.poll() is None:
            return  # already running

        args = ["ros2", "run", "ros_gz_bridge", "parameter_bridge"]
        args += [f"{t}@{r}@{g}" for (t, r, g) in self._maps]
        print("[param-bridge] CMD:", " ".join(args))

        self._proc = subprocess.Popen(
            args,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

        def _reader():
            assert self._proc and self._proc.stdout
            for line in self._proc.stdout:
                if self._stop_evt.is_set():
                    break
                #if line:
                    #print(f"[param-bridge:{self._proc.pid}] {line}", end="")

        self._reader_thread = threading.Thread(target=_reader, daemon=True)
        self._reader_thread.start()

    def stop(self, wait_timeout: float = 3.0):
        self._stop_evt.set()
        try:
            if self._proc and self._proc.poll() is None:
                self._proc.terminate()  # SIGTERM (15)
                try:
                    self._proc.wait(timeout=wait_timeout)
                except subprocess.TimeoutExpired:
                    print("[param-bridge] Bridge didn't exit in time; killing…")
                    self._proc.kill()
                    self._proc.wait()
        finally:
            if self._reader_thread:
                self._reader_thread.join(timeout=0.5)
            self._proc = None
            self._reader_thread = None
            self._stop_evt.clear()

    def __del__(self):
        try:
            self.stop()
        except Exception:
            pass


def start_parameter_bridge(
    mappings: Union[Triplet, Iterable[Triplet]]
) -> RosGzBridge:
    """
    Start a parameter bridge for one or many triplets.
      - mappings: (topic, ros_type, gz_type) OR an iterable of them
    Returns a manager you can .stop() later.
    """
    if isinstance(mappings, tuple) and len(mappings) == 3 and all(isinstance(x, str) for x in mappings):
        maps_list = [mappings]
    else:
        maps_list = list(mappings)  # type: ignore[arg-type]

    bridge = RosGzBridge(maps_list)
    bridge.start()
    return bridge
