#!/usr/bin/env python3
# hydra_teleop/teleop.py
from __future__ import annotations

import time
import threading
import logging

from typing import Tuple, Optional

from .transport import GzVelPub
from ..config import TeleopConfig
from ..simulation.physics import DronePhysics


class GzTeleop:
    """
    Realism-upgraded velocity teleop.

    SAME Public API:
      - start()
      - set_cmd(vx, vy, vz, wz)
      - publish_once(linear, angular)
      - stop()
      - shutdown()

    Implementation detail:
      - All motion shaping (physics/env) is delegated to DronePhysics.
      - This class only handles threading, logging, and publishing.
    """

    def __init__(self, topic: str, cfg: TeleopConfig):
        if getattr(cfg, "rate_hz", 0) <= 0:
            raise ValueError(f"rate_hz must be > 0, got {cfg.rate_hz}")

        self.topic = topic
        self.cfg = cfg

        self._logger = logging.getLogger(__name__)
        self._pub = GzVelPub(self.topic)

        # Physics model (encapsulates states & tunables)
        self._physics = DronePhysics(cfg)

        # Threading
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        # Cached last output (for publish_once side effects if needed)
        self._last_vx = self._last_vy = self._last_vz = 0.0
        self._last_wz = 0.0

    # -------------- private --------------
    def _publish_loop(self) -> None:
        period = 1.0 / self.cfg.rate_hz
        next_t = time.time()
        last_t = next_t

        while not self._stop_event.is_set():
            now = time.time()
            dt = max(1e-4, now - last_t)
            last_t = now

            # Step physics and publish
            with self._lock:
                vx, vy, vz, wz = self._physics.step(dt)
                self._last_vx, self._last_vy, self._last_vz, self._last_wz = vx, vy, vz, wz

            try:
                self._pub.send((vx, vy, vz), (0.0, 0.0, wz))
            except Exception:
                # Publishing failures shouldn't kill the loop
                pass

            next_t += period
            time.sleep(max(0.0, next_t - time.time()))

    # -------------- public API --------------
    def start(self) -> None:
        self._logger.info({
                            "physics" : True
                        },
                        extra = {
                            "event" : "Started"
                        })  
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()

    def set_cmd(self, vx: float, vy: float, vz: float, wz: float) -> None:
        # Only push the desired setpoint into physics (it handles latency)
        with self._lock:
            self._physics.update_cmd(vx, vy, vz, wz)

    def publish_once(
        self,
        linear: Tuple[float, float, float],
        angular: Tuple[float, float, float],
    ) -> None:
        # Direct one-shot publish (bypasses physics shaping)
        self._logger.info({
                            "vx" : linear[0],
                            "vy" : linear[1],
                            "vz" : linear[2],
                            "wx" : angular[0],
                            "wx" : angular[0],
                            "wx" : angular[0],
                        },
                        extra = {
                            "event" : "PUBLISH_ONCE"
                        })  
        try:
            self._pub.send(linear, angular)
        except Exception:
            pass

    def stop(self) -> None:
        self._logger.info({
                            "event" : "STOP"
                        })
        self.set_cmd(0.0, 0.0, 0.0, 0.0)

    def shutdown(self, join_timeout: float = 0.3) -> None:
        # Zero commands and publish a final zero
        with self._lock:
            self._physics.update_cmd(0.0, 0.0, 0.0, 0.0)
        try:
            self._pub.send((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        except Exception:
            pass

        # Small guard for in-flight publish
        time.sleep(max(0.02, 2.0 / self.cfg.rate_hz))

        # Stop loop
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=join_timeout)
        self._thread = None
