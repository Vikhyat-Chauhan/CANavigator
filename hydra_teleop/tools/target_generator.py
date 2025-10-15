#!/usr/bin/env python3
"""
Hydra Tool Plugin: Target (sphere) generator (simple, keeps TeleopConfig dependency)
- Picks a safe (x,y) that avoids restricted rectangles (from nofly meta)
- Keeps a minimum distance from the drone start (from TeleopConfig or explicit cfg)
- Writes an SDF for a static sphere + a small meta JSON
"""

from dataclasses import dataclass
from typing import Tuple, List, Optional
import os, json, random

# --- Map bounds (match your world) ---
X_MIN, X_MAX = -100.0, 100.0
Y_MIN, Y_MAX =  -50.0,  50.0


@dataclass
class TargetGenCfg:
    # outputs
    out_sdf: str = os.path.join("models/generated", "generated_target.sdf")
    out_meta: str = os.path.join("models/generated", "generated_target_meta.json")
    # input (from NoFlyGenerator meta)
    in_meta_nofly: str = os.path.join("models/generated", "generated_nofly_meta.json")

    # target model params
    z: float = 0.0
    radius: float = 0.6
    color_rgba: Tuple[float, float, float, float] = (1.0, 0.9, 0.0, 1.0)  # golden-ish
    name: str = "target_sphere"

    # placement constraints
    seed: Optional[int] = None
    margin_walls: float = 5.0
    margin_rect: float = 1.0
    max_tries: int = 5000  # attempts to sample a safe spot

    # keep target away from drone start (optional override)
    avoid_start_xy: Optional[Tuple[float, float]] = None  # if None, pulled from TeleopConfig
    min_dist_start: float = 25.0  # meters of separation from start

    # world bounds (override if needed)
    x_min: float = X_MIN
    x_max: float = X_MAX
    y_min: float = Y_MIN
    y_max: float = Y_MAX


class TargetGenerator:
    """
    Generate a spherical target that avoids no-fly rectangles and the drone start.
    API: run() -> (out_sdf, out_meta)
    """

    def __init__(self, teleop_cfg, gen_cfg: TargetGenCfg):
        # Keep TeleopConfig dependency via this arg; we read start (x,y) from it if provided.
        self.teleop_cfg = teleop_cfg
        self.cfg = gen_cfg

    # ---------- public ----------
    def run(self) -> Tuple[str, str]:
        rects = self._load_restricted_rects(self.cfg.in_meta_nofly)

        x, y = self._pick_safe_xy(
            rects=rects,
            margin_walls=self.cfg.margin_walls,
            margin_rect=self.cfg.margin_rect,
            seed=self.cfg.seed,
            max_tries=self.cfg.max_tries,
            x_min=self.cfg.x_min, x_max=self.cfg.x_max,
            y_min=self.cfg.y_min, y_max=self.cfg.y_max,
            radius=self.cfg.radius,
        )

        sdf_xml = self._make_sphere_sdf(
            self.cfg.name, self.cfg.radius, self.cfg.color_rgba, x, y, self.cfg.z
        )

        os.makedirs(os.path.dirname(self.cfg.out_sdf), exist_ok=True)
        self._write_text(self.cfg.out_sdf, sdf_xml)

        meta = {
            "name": self.cfg.name,
            "x": x, "y": y, "z": self.cfg.z,
            "radius": self.cfg.radius,
            "color": self.cfg.color_rgba,
            "in_meta_nofly": self.cfg.in_meta_nofly,
            "avoid_start_xy": self._resolve_start_xy(),  # what we actually used
            "min_dist_start": self.cfg.min_dist_start,
        }
        self._write_text(self.cfg.out_meta, json.dumps(meta, indent=2))
        return self.cfg.out_sdf, self.cfg.out_meta

    # ---------- helpers ----------
    def _resolve_start_xy(self) -> Optional[Tuple[float, float]]:
        """Prefer explicit cfg.avoid_start_xy; else read from TeleopConfig if available."""
        if self.cfg.avoid_start_xy is not None:
            return self.cfg.avoid_start_xy
        if self.teleop_cfg is None:
            return None
        # Common attribute names you likely have in TeleopConfig:
       
        sx = getattr(self.teleop_cfg, "start_x", None)
        sy = getattr(self.teleop_cfg, "start_y", None)
        if sx is not None and sy is not None:
            return (float(sx), float(sy))
        return None

    @staticmethod
    def _point_in_rect_with_margin(x: float, y: float,
                                   cx: float, cy: float, w: float, h: float,
                                   margin: float = 0.0) -> bool:
        half_w = w * 0.5 + margin
        half_h = h * 0.5 + margin
        return (abs(x - cx) <= half_w) and (abs(y - cy) <= half_h)

    def _load_restricted_rects(self, meta_path: str) -> List[Tuple[float, float, float, float]]:
        if not os.path.isfile(meta_path):
            return []
        try:
            with open(meta_path) as f:
                meta = json.load(f)
            rects = meta.get("rectangles_xywh", [])
            return [tuple(map(float, r)) for r in rects]  # (cx, cy, w, h)
        except Exception:
            return []

    def _pick_safe_xy(
        self,
        rects: List[Tuple[float, float, float, float]],
        margin_walls: float,
        margin_rect: float,
        seed: Optional[int],
        max_tries: int,
        x_min: float, x_max: float,
        y_min: float, y_max: float,
        radius: float,
    ) -> Tuple[float, float]:
        rnd = random.Random(seed)

        # Shrink world bounds so the whole sphere stays within walls.
        wall_margin = margin_walls + radius
        left   = x_min + wall_margin
        right  = x_max - wall_margin
        bottom = y_min + wall_margin
        top    = y_max - wall_margin
        if left > right or bottom > top:
            raise RuntimeError(
                f"[hydra.tools] Sample bounds invalid after margins; "
                f"reduce margin_walls or radius."
            )

        keepout = margin_rect + radius
        start_xy = self._resolve_start_xy()
        min_d2 = (self.cfg.min_dist_start ** 2) if start_xy is not None else 0.0

        def is_safe(px: float, py: float) -> bool:
            # 1) outside all expanded rectangles
            for (cx, cy, w, h) in rects:
                if self._point_in_rect_with_margin(px, py, cx, cy, w, h, keepout):
                    return False
            # 2) far enough from start
            if start_xy is not None:
                dx = px - start_xy[0]
                dy = py - start_xy[1]
                if dx*dx + dy*dy < min_d2:
                    return False
            return True

        # 1) fast rejection sampling
        for _ in range(max_tries):
            x = rnd.uniform(left, right)
            y = rnd.uniform(bottom, top)
            if is_safe(x, y):
                return x, y

        # 2) simple grid sweep fallback
        step = max(2.0 * radius, 0.5)
        xg = left
        while xg <= right + 1e-9:
            yg = bottom
            while yg <= top + 1e-9:
                if is_safe(xg, yg):
                    return xg, yg
                yg += step
            xg += step

        raise RuntimeError(
            "[hydra.tools] Could not place target: no collision-free position "
            "satisfies rectangles, radius/margins, and min_dist_start."
        )

    def _make_sphere_sdf(self, name: str, radius: float,
                         rgba: Tuple[float, float, float, float],
                         x: float, y: float, z: float) -> str:
        r, g, b, a = rgba
        return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
    <link name="link">
      <visual name="vis">
        <geometry><sphere><radius>{radius:.3f}</radius></sphere></geometry>
        <material>
          <diffuse>{r} {g} {b} {a}</diffuse>
          <ambient>{r} {g} {b} {a}</ambient>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

    def _write_text(self, path: str, text: str):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            f.write(text)
