#!/usr/bin/env python3
"""
Hydra Plugin: Procedural No-Fly Zone Generator
Generates an SDF + metadata file representing restricted zones via Perlin noise.

Simplified height model:
- **Legacy only**: every rectangle is a thin slab at Z_EPS with thickness Z_THICK.

Notes:
- All previous "custom/tall height" options are now ignored (kept in NoFlyGenCfg
  only for backward compatibility).
"""

from dataclasses import dataclass
from typing import List, Tuple
import os, json
import numpy as np

try:
    from noise import pnoise2
except ImportError as e:
    raise RuntimeError(
        "Missing dependency: 'noise'. Install with `pip install noise`."
    ) from e


# --- Default world bounds (XY footprint must match your world) ---
X_MIN, X_MAX = -100.0, 100.0
Y_MIN, Y_MAX = -50.0, 50.0
CELL_M = 5.0

# --- Legacy Z geometry (single mode) ---
Z_THICK = 5.0     # slab thickness
Z_EPS = 0.00      # slab center Z


@dataclass
class NoFlyGenCfg:
    out_sdf: str = os.path.join("models/generated", "generated_nofly.sdf")
    out_meta: str = os.path.join("models/generated", "generated_nofly_meta.json")
    density: float = 0.2
    corr_len_m: float = 10.0
    seed: int = 0
    pass_through: bool = True          # if False, add collisions; else visuals-only
    visual_alpha: float = 0.0
    color_rgb: Tuple[float, float, float] = (1.0, 0.0, 0.0)
    x_min: float = X_MIN
    x_max: float = X_MAX
    y_min: float = Y_MIN
    y_max: float = Y_MAX
    cell_m: float = CELL_M

    # ---------------- BACKWARD COMPATIBILITY (IGNORED) ----------------
    # The following fields existed for "custom/tall height" support and
    # are now NO-OPS. They are retained to avoid breaking existing code
    # that constructs NoFlyGenCfg with these attributes.
    custom_count: int = 0
    custom_frac: float = 0.0
    ground_z: float = 0.0
    above_m: float = 0.0
    below_m: float = 0.0
    z_min: float = -20.0
    z_max: float = 30.0
    selection_seed_offset: int = 777
    # ------------------------------------------------------------------


class NoFlyGenerator:
    """Hydra tool plugin for generating no-fly zones procedurally (legacy height only)."""

    def __init__(self, teleop_cfg, gen_cfg: NoFlyGenCfg):
        self.teleop_cfg = teleop_cfg
        self.cfg = gen_cfg
        self._rects: List[Tuple[int, int, int, int]] = []

    def run(self) -> Tuple[str, str]:
        xs, ys = self._build_grid()
        nmap = self._sample_perlin(xs, ys, self.cfg.corr_len_m, self.cfg.seed)
        mask, thr = self._mask_by_density(nmap, self.cfg.density)
        rects = self._merge_rects(mask)
        self._rects = rects

        os.makedirs(os.path.dirname(self.cfg.out_sdf), exist_ok=True)
        self._write_sdf(rects, xs, ys)
        self._write_meta(xs, ys, rects, thr)
        return self.cfg.out_sdf, self.cfg.out_meta

    def shutdown(self):
        pass

    # -------------------------------------------------------------------------
    # Internal methods
    # -------------------------------------------------------------------------

    def _build_grid(self):
        xs = np.arange(self.cfg.x_min, self.cfg.x_max, self.cfg.cell_m)
        ys = np.arange(self.cfg.y_min, self.cfg.y_max, self.cfg.cell_m)
        return xs, ys

    def _sample_perlin(self, xs, ys, corr_len_m, seed):
        freq = 1.0 / max(corr_len_m, 1e-6)
        z = np.zeros((len(xs), len(ys)), dtype=np.float32)
        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                zx = (x - xs[0]) * freq + seed * 1.1337
                zy = (y - ys[0]) * freq - seed * 0.7331
                z[i, j] = pnoise2(
                    zx, zy,
                    octaves=3,
                    repeatx=1024,
                    repeaty=1024,
                    base=seed
                )
        # Normalize to [0,1]
        return (z - float(np.min(z))) / max(float(np.ptp(z)), 1e-9)

    def _mask_by_density(self, noise_map, density):
        thr = float(np.quantile(noise_map, 1.0 - min(max(density, 0.0), 1.0)))
        return (noise_map >= thr), float(thr)

    def _merge_rects(self, mask):
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
                    h = 1
                    good = True
                    while i + h < nx and good:
                        for jj in range(j, j + w):
                            if not (mask[i + h, jj] and not used[i + h, jj]):
                                good = False
                                break
                        if good:
                            h += 1
                    for ii in range(i, i + h):
                        for jj in range(j, j + w):
                            used[ii, jj] = True
                    rects.append((i, j, h, w))
                    j += w
                else:
                    j += 1
        return rects

    def _rect_to_world(self, xs, ys, rect):
        i0, j0, h, w = rect
        x0, y0 = xs[i0], ys[j0]
        w_m, h_m = w * self.cfg.cell_m, h * self.cfg.cell_m
        # Cast to Python floats so JSON is happy
        return float(x0 + w_m / 2.0), float(y0 + h_m / 2.0), float(w_m), float(h_m)

    # -------------------------------------------------------------------------

    def _write_sdf(self, rects, xs, ys):
        r, g, b = self.cfg.color_rgb
        transparency = max(0.0, min(1.0, self.cfg.visual_alpha))
        a = 1.0 - transparency

        lines = [
            '<?xml version="1.0"?>',
            '<sdf version="1.9">',
            '  <model name="restricted_zones">',
            '    <static>true</static>',
            '    <pose>0 0 0 0 0 0</pose>',
            '    <link name="link">'
        ]
        for k, rect in enumerate(rects):
            cx, cy, w_m, h_m = self._rect_to_world(xs, ys, rect)

            # Legacy-only Z (single mode)
            zc, zs = float(Z_EPS), float(Z_THICK)

            if not self.cfg.pass_through:
                lines += [
                    f'      <collision name="c{k}">',
                    f'        <pose>{cx:.3f} {cy:.3f} {zc:.3f} 0 0 0</pose>',
                    f'        <geometry><box><size>{w_m:.3f} {h_m:.3f} {zs:.3f}</size></box></geometry>',
                    '      </collision>'
                ]
            lines += [
                f'      <visual name="v{k}">',
                f'        <pose>{cx:.3f} {cy:.3f} {zc:.3f} 0 0 0</pose>',
                f'        <geometry><box><size>{w_m:.3f} {h_m:.3f} {zs:.3f}</size></box></geometry>',
                f'        <transparency>{transparency:.4f}</transparency>',
                '        <material>',
                f'          <diffuse>{r:.4f} {g:.4f} {b:.4f} {a:.4f}</diffuse>',
                f'          <ambient>{r:.4f} {g:.4f} {b:.4f} {a:.4f}</ambient>',
                '          <specular>0.1 0.1 0.1 1</specular>',
                '        </material>',
                '      </visual>'
            ]
        lines += ['    </link>', '  </model>', '</sdf>']
        with open(self.cfg.out_sdf, "w") as f:
            f.write("\n".join(lines))

    def _write_meta(self, xs, ys, rects, thr):
        # Metadata reflects the simplified, legacy-only height mode.
        meta = {
            "x_min": float(self.cfg.x_min), "x_max": float(self.cfg.x_max),
            "y_min": float(self.cfg.y_min), "y_max": float(self.cfg.y_max),
            "cell_m": float(self.cfg.cell_m), "density": float(self.cfg.density),
            "corr_len_m": float(self.cfg.corr_len_m), "seed": int(self.cfg.seed),
            "threshold": float(thr),
            "pass_through": bool(self.cfg.pass_through),
            "visual_alpha": float(self.cfg.visual_alpha),
            "color_rgb": [float(c) for c in self.cfg.color_rgb],
            "rectangles_xywh": [self._rect_to_world(xs, ys, r) for r in rects],
            "height_mode": {
                "type": "legacy_only",
                "legacy": {"z_center": float(Z_EPS), "z_size": float(Z_THICK)}
            }
        }
        with open(self.cfg.out_meta, "w") as f:
            json.dump(meta, f, indent=2)
