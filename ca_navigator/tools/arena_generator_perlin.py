#!/usr/bin/env python3
# ca_navigator/tools/ca_nav_gen_merged.py
#
# Call from main just like your existing plugin: ctor + run().
# Also exports a top-level run(...) shortcut.
#
# Deterministic: one master seed drives both NFZ + target.

from dataclasses import dataclass
from typing import Tuple, List, Optional, Dict
import os, json, random

import numpy as np
from noise import pnoise2

# --- Bounds / legacy Z, matching your NFZ plugin (thin slab) ---
X_MIN, X_MAX = -100.0, 100.0
Y_MIN, Y_MAX =  -50.0,  50.0
CELL_M = 5.0
Z_THICK = 5.0
Z_EPS   = 0.0

# ---------------- Configs (mirror your style) ----------------

@dataclass
class NoFlyGenCfg:
    out_sdf: str
    out_meta: str
    density: float = 0.2
    corr_len_m: float = 10.0
    seed: int = 0
    pass_through: bool = True
    visual_alpha: float = 0.0
    color_rgb: Tuple[float, float, float] = (1.0, 0.0, 0.0)
    x_min: float = X_MIN
    x_max: float = X_MAX
    y_min: float = Y_MIN
    y_max: float = Y_MAX
    cell_m: float = CELL_M

@dataclass
class TargetGenCfg:
    out_sdf: str
    out_meta: str
    in_meta_nofly: str
    z: float = 0.0
    radius: float = 0.6
    color_rgba: Tuple[float, float, float, float] = (1.0, 0.9, 0.0, 1.0)
    name: str = "target_sphere"
    seed: Optional[int] = None
    margin_walls: float = 5.0
    margin_rect: float = 1.0
    max_tries: int = 5000
    avoid_start_xy: Optional[Tuple[float, float]] = None
    min_dist_start: float = 25.0
    x_min: float = X_MIN
    x_max: float = X_MAX
    y_min: float = Y_MIN
    y_max: float = Y_MAX

@dataclass
class ArenaGenCfg:
    # master seed; target seed = seed + target_seed_offset
    seed: int = 42
    target_seed_offset: int = 1
    outdir: str = os.path.join("models", "generated")
    # NFZ knobs
    density: float = 0.2
    corr_len_m: float = 10.0
    pass_through: bool = True
    visual_alpha: float = 0.0
    # Target knobs
    target_radius: float = 0.6
    target_min_dist: float = 25.0
    target_margin_walls: float = 5.0
    target_margin_rect: float = 1.0
    # Bounds
    x_min: float = X_MIN; x_max: float = X_MAX
    y_min: float = Y_MIN; y_max: float = Y_MAX
    cell_m: float = CELL_M

# ---------------- Implementation (same logic as your plugins) ----------------
# NFZ bits mirror your existing nofly_generator (Perlin grid → rect merge → SDF+meta). :contentReference[oaicite:0]{index=0}

class _NoFly:
    def __init__(self, cfg: NoFlyGenCfg):
        self.cfg = cfg

    def run(self) -> Tuple[str, str]:
        xs = np.arange(self.cfg.x_min, self.cfg.x_max, self.cfg.cell_m, dtype=float)
        ys = np.arange(self.cfg.y_min, self.cfg.y_max, self.cfg.cell_m, dtype=float)
        nmap = self._sample_perlin(xs, ys, self.cfg.corr_len_m, self.cfg.seed)
        mask, thr = self._mask_by_density(nmap, self.cfg.density)
        rects = self._merge_rects(mask)
        os.makedirs(os.path.dirname(self.cfg.out_sdf), exist_ok=True)
        self._write_sdf(rects, xs, ys)
        self._write_meta(xs, ys, rects, thr)
        return self.cfg.out_sdf, self.cfg.out_meta

    def _sample_perlin(self, xs, ys, corr_len_m, seed):
        freq = 1.0 / max(corr_len_m, 1e-6)
        z = np.zeros((len(xs), len(ys)), dtype=np.float32)
        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                zx = (x - xs[0]) * freq + seed * 1.1337
                zy = (y - ys[0]) * freq - seed * 0.7331
                z[i, j] = pnoise2(zx, zy, octaves=3, repeatx=1024, repeaty=1024, base=seed)
        zmin = float(np.min(z)); zptp = float(np.ptp(z))
        return (z - zmin) / max(zptp, 1e-9)

    @staticmethod
    def _mask_by_density(noise_map, density):
        d = min(max(float(density), 0.0), 1.0)
        thr = float(np.quantile(noise_map, 1.0 - d))
        return (noise_map >= thr), thr

    @staticmethod
    def _merge_rects(mask):
        used = np.zeros_like(mask, dtype=bool)
        rects: List[Tuple[int, int, int, int]] = []
        nx, ny = mask.shape
        for i in range(nx):
            j = 0
            while j < ny:
                if mask[i, j] and not used[i, j]:
                    w = 1
                    while j + w < ny and mask[i, j + w] and not used[i, j + w]:
                        w += 1
                    h = 1; good = True
                    while i + h < nx and good:
                        for jj in range(j, j + w):
                            if not (mask[i + h, jj] and not used[i + h, jj]):
                                good = False; break
                        if good: h += 1
                    for ii in range(i, i + h):
                        for jj in range(j, j + w):
                            used[ii, jj] = True
                    rects.append((i, j, h, w)); j += w
                else:
                    j += 1
        return rects

    def _rect_to_world(self, xs, ys, rect):
        i0, j0, h, w = rect
        x0, y0 = xs[i0], ys[j0]
        w_m, h_m = w * self.cfg.cell_m, h * self.cfg.cell_m
        return float(x0 + w_m/2.0), float(y0 + h_m/2.0), float(w_m), float(h_m)

    def _write_sdf(self, rects, xs, ys):
        r, g, b = self.cfg.color_rgb
        transparency = max(0.0, min(1.0, self.cfg.visual_alpha)); a = 1.0 - transparency
        lines = [
            '<?xml version="1.0"?>','<sdf version="1.9">','  <model name="restricted_zones">',
            '    <static>true</static>','    <pose>0 0 0 0 0 0</pose>','    <link name="link">'
        ]
        for k, rect in enumerate(rects):
            cx, cy, w_m, h_m = self._rect_to_world(xs, ys, rect)
            zc, zs = float(Z_EPS), float(Z_THICK)
            if not self.cfg.pass_through:
                lines += [
                    f'      <collision name="c{k}">', f'        <pose>{cx:.3f} {cy:.3f} {zc:.3f} 0 0 0</pose>',
                    f'        <geometry><box><size>{w_m:.3f} {h_m:.3f} {zs:.3f}</size></box></geometry>', '      </collision>'
                ]
            lines += [
                f'      <visual name="v{k}">', f'        <pose>{cx:.3f} {cy:.3f} {zc:.3f} 0 0 0</pose>',
                f'        <geometry><box><size>{w_m:.3f} {h_m:.3f} {zs:.3f}</size></box></geometry>',
                f'        <transparency>{transparency:.4f}</transparency>',
                '        <material>', f'          <diffuse>{r:.4f} {g:.4f} {b:.4f} {a:.4f}</diffuse>',
                f'          <ambient>{r:.4f} {g:.4f} {b:.4f} {a:.4f}</ambient>', '          <specular>0.1 0.1 0.1 1</specular>',
                '        </material>', '      </visual>'
            ]
        lines += ['    </link>','  </model>','</sdf>']
        os.makedirs(os.path.dirname(self.cfg.out_sdf), exist_ok=True)
        with open(self.cfg.out_sdf, "w") as f: f.write("\n".join(lines))

    def _write_meta(self, xs, ys, rects, thr):
        meta = {
            "x_min": float(self.cfg.x_min), "x_max": float(self.cfg.x_max),
            "y_min": float(self.cfg.y_min), "y_max": float(self.cfg.y_max),
            "cell_m": float(self.cfg.cell_m), "density": float(self.cfg.density),
            "corr_len_m": float(self.cfg.corr_len_m), "seed": int(self.cfg.seed),
            "threshold": float(thr), "pass_through": bool(self.cfg.pass_through),
            "visual_alpha": float(self.cfg.visual_alpha),
            "color_rgb": [float(c) for c in self.cfg.color_rgb],
            "rectangles_xywh": [self._rect_to_world(xs, ys, r) for r in rects],
            "height_mode": {"type": "legacy_only", "legacy": {"z_center": float(Z_EPS), "z_size": float(Z_THICK)}}
        }
        with open(self.cfg.out_meta, "w") as f: json.dump(meta, f, indent=2)

class _Target:
    def __init__(self, cfg: TargetGenCfg, start_xy: Optional[Tuple[float,float]]):
        self.cfg = cfg; self.start_xy = start_xy

    def run(self) -> Tuple[str, str]:
        rects = self._load_rects(self.cfg.in_meta_nofly)
        x, y = self._pick_xy(rects)
        self._write_sdf(x, y)
        self._write_meta(x, y)
        return self.cfg.out_sdf, self.cfg.out_meta

    @staticmethod
    def _load_rects(meta_path: str) -> List[Tuple[float, float, float, float]]:
        if not os.path.isfile(meta_path): return []
        with open(meta_path) as f: meta = json.load(f)
        return [tuple(map(float, r)) for r in meta.get("rectangles_xywh", [])]

    def _pick_xy(self, rects):
        rnd = random.Random(self.cfg.seed)
        wall = self.cfg.margin_walls + self.cfg.radius
        left, right = self.cfg.x_min + wall, self.cfg.x_max - wall
        bottom, top = self.cfg.y_min + wall, self.cfg.y_max - wall
        keepout = self.cfg.margin_rect + self.cfg.radius
        min_d2 = (self.cfg.min_dist_start ** 2) if self.start_xy else 0.0

        def safe(px, py):
            for (cx, cy, w, h) in rects:
                hw, hh = w*0.5 + keepout, h*0.5 + keepout
                if abs(px - cx) <= hw and abs(py - cy) <= hh: return False
            if self.start_xy:
                dx, dy = px - self.start_xy[0], py - self.start_xy[1]
                if dx*dx + dy*dy < min_d2: return False
            return True

        for _ in range(self.cfg.max_tries):
            x, y = rnd.uniform(left, right), rnd.uniform(bottom, top)
            if safe(x, y): return x, y

        # deterministic grid fallback
        step = max(2.0 * self.cfg.radius, 0.5)
        xg = left
        while xg <= right + 1e-9:
            yg = bottom
            while yg <= top + 1e-9:
                if safe(xg, yg): return xg, yg
                yg += step
            xg += step
        raise RuntimeError("No collision-free target position found.")

    def _write_sdf(self, x, y):
        r,g,b,a = self.cfg.color_rgba
        xml = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{self.cfg.name}">
    <static>true</static>
    <pose>{x:.3f} {y:.3f} {self.cfg.z:.3f} 0 0 0</pose>
    <link name="link">
      <visual name="vis">
        <geometry><sphere><radius>{self.cfg.radius:.3f}</radius></sphere></geometry>
        <material><diffuse>{r} {g} {b} {a}</diffuse><ambient>{r} {g} {b} {a}</ambient><specular>0.2 0.2 0.2 1</specular></material>
      </visual>
    </link>
  </model>
</sdf>"""
        os.makedirs(os.path.dirname(self.cfg.out_sdf), exist_ok=True)
        with open(self.cfg.out_sdf, "w") as f: f.write(xml)

    def _write_meta(self, x, y):
        meta = {
            "name": self.cfg.name, "x": x, "y": y, "z": self.cfg.z,
            "radius": self.cfg.radius, "color": self.cfg.color_rgba,
            "in_meta_nofly": self.cfg.in_meta_nofly,
            "avoid_start_xy": self.start_xy, "min_dist_start": self.cfg.min_dist_start,
        }
        with open(self.cfg.out_meta, "w") as f: json.dump(meta, f, indent=2)

# ---------------- Public, main-like entrypoint ----------------

class ArenaGenerator:
    """Use like your existing plugin: gen = ArenaGenerator(teleop_cfg, cfg); gen.run()."""

    def __init__(self, teleop_cfg, cfg: ArenaGenCfg):
        self.teleop_cfg = teleop_cfg
        self.cfg = cfg

    def run(self) -> Dict[str, str]:
        # master determinism
        random.seed(self.cfg.seed)
        np.random.seed(self.cfg.seed)

        outdir = self.cfg.outdir
        os.makedirs(outdir, exist_ok=True)

        nfz_sdf  = os.path.join(outdir, "generated_nofly.sdf")
        nfz_meta = os.path.join(outdir, "generated_nofly_meta.json")
        tgt_sdf  = os.path.join(outdir, "generated_target.sdf")
        tgt_meta = os.path.join(outdir, "generated_target_meta.json")

        # 1) No-fly
        nfz = _NoFly(NoFlyGenCfg(
            out_sdf=nfz_sdf, out_meta=nfz_meta,
            density=self.cfg.density, corr_len_m=self.cfg.corr_len_m,
            seed=self.cfg.seed, pass_through=self.cfg.pass_through, visual_alpha=self.cfg.visual_alpha,
            x_min=self.cfg.x_min, x_max=self.cfg.x_max, y_min=self.cfg.y_min, y_max=self.cfg.y_max, cell_m=self.cfg.cell_m,
        ))
        nfz.run()

        # 2) Target (seed derived from master)
        target_seed = int(self.cfg.seed) + int(self.cfg.target_seed_offset)
        start_xy = None
        if self.teleop_cfg is not None:
            sx = getattr(self.teleop_cfg, "start_x", None)
            sy = getattr(self.teleop_cfg, "start_y", None)
            if sx is not None and sy is not None: start_xy = (float(sx), float(sy))

        tgt = _Target(TargetGenCfg(
            out_sdf=tgt_sdf, out_meta=tgt_meta, in_meta_nofly=nfz_meta,
            radius=self.cfg.target_radius, seed=target_seed,
            min_dist_start=self.cfg.target_min_dist, margin_walls=self.cfg.target_margin_walls, margin_rect=self.cfg.target_margin_rect,
            x_min=self.cfg.x_min, x_max=self.cfg.x_max, y_min=self.cfg.y_min, y_max=self.cfg.y_max,
        ), start_xy=start_xy)
        tgt.run()

        return {"nofly_sdf": nfz_sdf, "nofly_meta": nfz_meta, "target_sdf": tgt_sdf, "target_meta": tgt_meta}

# Optional: function-style call if you prefer calling a function from main
def run(teleop_cfg=None, **kwargs) -> Dict[str, str]:
    """
    Example:
      from ca_navigator.tools import ca_nav_gen_merged as merged
      merged.run(self.teleop_cfg, seed=42, density=0.25, corr_len_m=12, outdir="models/generated")
    """
    return ArenaGenerator(teleop_cfg, ArenaGenCfg(**kwargs)).run()
