#!/usr/bin/env python3
# hydra_teleop/tools/hydra_gen_merged.py
#
# City-style NFZ generator only:
# - Roads carved as free space (major/minor grid)
# - Blocks subdivided into lots; lots optionally filled with "buildings" (rectangles)
# Deterministic via a single master seed (also drives target placement with offset).
#
# Public usage:
#   from hydra_teleop.tools import hydra_gen_merged as merged
#   paths = merged.run(seed=101, outdir="models/generated",
#                      city_major_m=60.0, city_minor_m=30.0,
#                      road_w_m=6.0, lot_w_m=14.0, lot_h_m=12.0,
#                      lot_setback_m=1.5, lot_fill_prob=0.9, lot_jitter_m=2.0)
# -----------------------------------------------------------------------------
# References (IEEE style) for city-style NFZ generation & target placement
#
# [1] P. Parish and P. Müller, “Procedural Modeling of Cities,” Proc. SIGGRAPH, 2001.
# [2] G. Chen, G. Esch, P. Wonka, and P. Müller, “Interactive Procedural Street Modeling,”
#     ACM Trans. Graph. (SIGGRAPH Asia), vol. 27, no. 5, 2008.
# [3] D. Galin, A. Peytavie, N. Maréchal, and E. Guérin, “Procedural Generation of Roads,”
#     Computer Graphics Forum (Eurographics), vol. 29, no. 2, pp. 429–438, 2010.
# [4] D. Galin, E. Guérin, A. Peytavie, and N. Maréchal, “Authoring Hierarchical Road Networks,”
#     Computer Graphics Forum (Eurographics), vol. 30, no. 7, pp. 2021–2030, 2011.
# [5] D. Aliaga, C. Vanegas, and B. Benes, “Interactive Example-Based Urban Layout Synthesis,”
#     ACM Trans. Graph., vol. 27, no. 5, 2008 / surveys & follow-ups 2009–2012 (parcels/blocks).
# [6] P. Müller, P. Wonka, S. Haegler, A. Ulmer, and L. Van Gool, “Procedural Modeling of
#     Buildings,” Proc. SIGGRAPH, 2006.
# [7] P. Wonka, M. Wimmer, F. Sillion, and W. Ribarsky, “Instant Architecture,” ACM Trans.
#     Graph. (SIGGRAPH), vol. 22, no. 3, pp. 669–677, 2003.
#
# Code mapping:
#   - Major/minor grids & road widths: [1], [2]   (deterministic grid w/ minor irregularities)
#   - Blocks → lot subdivision + setbacks: [5]    (parceling inside blocks)
#   - Per-lot building rectangles (NFZ proxies): [6], [7]
#   - Hierarchical/weighted road rationale (optional extension): [3], [4]
# -----------------------------------------------------------------------------


from dataclasses import dataclass
from typing import Tuple, List, Optional, Dict
import os, json, random

# --- Bounds / legacy Z (thin slab for visuals/collisions) ---
X_MIN, X_MAX = -100.0, 100.0
Y_MIN, Y_MAX =  -50.0,  50.0
Z_THICK = 5.0
Z_EPS   = 0.0

# ---------------- Configs ----------------

@dataclass
class NoFlyGenCfg:
    out_sdf: str
    out_meta: str
    # Visual/physics
    pass_through: bool = True
    visual_alpha: float = 0.0
    color_rgb: Tuple[float, float, float] = (1.0, 0.0, 0.0)
    # Domain
    x_min: float = X_MIN
    x_max: float = X_MAX
    y_min: float = Y_MIN
    y_max: float = Y_MAX

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
    # Master seed; target seed = seed + target_seed_offset
    seed: int = 42
    target_seed_offset: int = 1
    outdir: str = os.path.join("models", "generated")

    # --- City NFZ knobs ---
    '''
    city_major_m: float = 40.0     # spacing of “avenues” (coarse grid)
    city_minor_m: float = 20.0     # spacing of “streets” (fine grid)
    road_w_m: float   = 4.0        # road width carved as free (no NFZ)
    lot_w_m: float    = 12.0       # lot width inside blocks
    lot_h_m: float    = 10.0       # lot height inside blocks
    lot_setback_m: float = 1.0     # sidewalk from road into block
    lot_fill_prob: float = 0.85    # probability a lot has a “building”
    lot_jitter_m: float = 2.0      # small size jitter to avoid perfect grid look
    '''
    city_major_m   = 60.0
    city_minor_m   = 22.0
    road_w_m       = 4.5
    lot_w_m        = 13.0
    lot_h_m        = 11.0
    lot_setback_m  = 0.8
    lot_fill_prob  = 0.9
    lot_jitter_m   = 2.0

    # Visual/physics
    pass_through: bool = True
    visual_alpha: float = 0.0
    color_rgb: Tuple[float, float, float] = (1.0, 0.0, 0.0)

    # Bounds
    x_min: float = X_MIN; x_max: float = X_MAX
    y_min: float = Y_MIN; y_max: float = Y_MAX

    # Target knobs
    target_radius: float = 0.6
    target_min_dist: float = 25.0
    target_margin_walls: float = 5.0
    target_margin_rect: float = 1.0

# ---------------- City NFZ Implementation ----------------

class _NoFlyCity:
    """City-like NFZs: roads carved as free space; occupied lots become NFZ rectangles."""
    def __init__(self, cfg: NoFlyGenCfg, seed: int,
                 major, minor, road_w, lot_w, lot_h, setback, fill_p, jitter):
        self.cfg = cfg
        self.seed = int(seed)
        self.major, self.minor = float(major), float(minor)
        self.road_w = float(road_w)
        self.lot_w, self.lot_h = float(lot_w), float(lot_h)
        self.setback = float(setback)
        self.fill_p = float(fill_p)
        self.jitter = float(jitter)

    def run(self) -> Tuple[str, str]:
        rng = random.Random(self.seed)

        # 1) Road lines on both axes; near a road = free space.
        x_roads = self._axis_lines(self.cfg.x_min, self.cfg.x_max, self.major, self.minor)
        y_roads = self._axis_lines(self.cfg.y_min, self.cfg.y_max, self.major, self.minor)

        # 2) Rectangular blocks between adjacent road lines.
        x_blocks = self._adjacent_pairs(x_roads, self.cfg.x_min, self.cfg.x_max)
        y_blocks = self._adjacent_pairs(y_roads, self.cfg.y_min, self.cfg.y_max)

        rects_xywh: List[Tuple[float, float, float, float]] = []
        road_half = self.road_w * 0.5

        for (xa, xb) in x_blocks:
            for (ya, yb) in y_blocks:
                # sidewalks/setbacks inside each block
                # Subdivide domain into blocks between adjacent road lines; apply sidewalk/setback [5]
                xa_i = xa + road_half + self.setback
                xb_i = xb - road_half - self.setback
                ya_i = ya + road_half + self.setback
                yb_i = yb - road_half - self.setback
                if xb_i <= xa_i or yb_i <= ya_i:
                    continue  # too small block

                # 3) Subdivide block into lots; randomly fill lots with buildings (NFZs)
                # Regular parcel/lot slicing of blocks; simple tiling variant in spirit of parceling literature [5]
                x_lots = self._tiles(xa_i, xb_i, self.lot_w)
                y_lots = self._tiles(ya_i, yb_i, self.lot_h)
                for (lx0, lx1) in x_lots:
                    for (ly0, ly1) in y_lots:
                        # Random lot occupancy and mild massing jitter to avoid perfect grid look [1], [2];
                        # each occupied lot emits a rectangular NFZ proxy for a building footprint [6], [7]
                        if rng.random() > self.fill_p:
                            continue  # leave as courtyard/park/parking (free)
                        # Jitter size a bit and clamp into lot
                        w = max(1e-3, min(lx1 - lx0, (lx1 - lx0) - rng.uniform(0, self.jitter)))
                        h = max(1e-3, min(ly1 - ly0, (ly1 - ly0) - rng.uniform(0, self.jitter)))
                        cx = (lx0 + lx1) * 0.5
                        cy = (ly0 + ly1) * 0.5
                        rects_xywh.append((cx, cy, w, h))

        # 4) Emit SDF + meta
        os.makedirs(os.path.dirname(self.cfg.out_sdf), exist_ok=True)
        self._write_sdf(rects_xywh)
        self._write_meta(rects_xywh)
        return self.cfg.out_sdf, self.cfg.out_meta

    # ----- helpers -----
    def _axis_lines(self, vmin, vmax, major, minor):
        # Major/minor orthogonal grids motivated by procedural city layout: [1], [2]
        # Deterministic placement for reproducibility; jitter applied at lot scale (not here).
        # Centered grids for symmetry; include domain edges.
        lines = set([vmin, vmax, 0.0])
        def add(step):
            if step == 0: return
            k = 0
            # forward
            while True:
                pos = 0.0 + k*step
                if pos > vmax + 2*abs(step): break
                if pos >= vmin - 2*abs(step): lines.add(pos)
                k += 1
            # backward
            k = 1
            while True:
                pos = 0.0 - k*step
                if pos < vmin - 2*abs(step): break
                if pos <= vmax + 2*abs(step): lines.add(pos)
                k += 1
        add(major); add(minor)
        dedup = sorted(lines)
        # squeeze near-duplicates
        out = []
        eps = 1e-6
        for L in dedup:
            if not out or abs(L - out[-1]) > eps:
                out.append(L)
        return out

    @staticmethod
    def _adjacent_pairs(lines, vmin, vmax):
        pairs = []
        last = None
        for L in lines:
            if last is None:
                last = L; continue
            a, b = last, L
            if b > a:
                pairs.append((a, b))
            last = L
        return [(max(vmin, a), min(vmax, b)) for (a, b) in pairs]

    @staticmethod
    def _tiles(a, b, target):
        spans = []
        if target <= 0:
            return [(a, b)]
        x = a
        while x + target < b:
            spans.append((x, x + target))
            x += target
        if x < b:
            spans.append((x, b))
        return spans

    def _write_sdf(self, rects):
        r, g, b = self.cfg.color_rgb
        transparency = max(0.0, min(1.0, self.cfg.visual_alpha)); a = 1.0 - transparency
        lines = [
            '<?xml version="1.0"?>','<sdf version="1.9">','  <model name="restricted_zones">',
            '    <static>true</static>','    <pose>0 0 0 0 0 0</pose>','    <link name="link">'
        ]
        for k, (cx, cy, w_m, h_m) in enumerate(rects):
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
        with open(self.cfg.out_sdf, "w") as f: f.write("\n".join(lines))

    def _write_meta(self, rects):
        meta = {
            "x_min": float(self.cfg.x_min), "x_max": float(self.cfg.x_max),
            "y_min": float(self.cfg.y_min), "y_max": float(self.cfg.y_max),
            "mode": "city",
            "seed": int(self.seed),
            "grid": {"major_m": self.major, "minor_m": self.minor, "road_w_m": self.road_w},
            "lot": {"w_m": self.lot_w, "h_m": self.lot_h, "setback_m": self.setback,
                    "fill_prob": self.fill_p, "jitter_m": self.jitter},
            "pass_through": bool(self.cfg.pass_through),
            "visual_alpha": float(self.cfg.visual_alpha),
            "color_rgb": [float(c) for c in self.cfg.color_rgb],
            "rectangles_xywh": [(float(cx), float(cy), float(w), float(h)) for (cx, cy, w, h) in rects],
            "height_mode": {"type": "legacy_only", "legacy": {"z_center": float(Z_EPS), "z_size": float(Z_THICK)}}
        }
        with open(self.cfg.out_meta, "w") as f: json.dump(meta, f, indent=2)

# ---------------- Target generator (unchanged) ----------------

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

# ---------------- Public entrypoints ----------------

class ArenaGenerator:
    """Use like your existing plugin: gen = ArenaGenerator(teleop_cfg, cfg); gen.run()."""
    def __init__(self, teleop_cfg, cfg: ArenaGenCfg):
        self.teleop_cfg = teleop_cfg
        self.cfg = cfg

    def run(self) -> Dict[str, str]:
        # Deterministic RNG
        random.seed(self.cfg.seed)

        outdir = self.cfg.outdir
        os.makedirs(outdir, exist_ok=True)

        nfz_sdf  = os.path.join(outdir, "generated_nofly.sdf")
        nfz_meta = os.path.join(outdir, "generated_nofly_meta.json")
        tgt_sdf  = os.path.join(outdir, "generated_target.sdf")
        tgt_meta = os.path.join(outdir, "generated_target_meta.json")

        # 1) No-fly (city)
        nfz = _NoFlyCity(
            NoFlyGenCfg(
                out_sdf=nfz_sdf, out_meta=nfz_meta,
                pass_through=self.cfg.pass_through, visual_alpha=self.cfg.visual_alpha,
                color_rgb=self.cfg.color_rgb,
                x_min=self.cfg.x_min, x_max=self.cfg.x_max, y_min=self.cfg.y_min, y_max=self.cfg.y_max
            ),
            seed=self.cfg.seed,
            major=self.cfg.city_major_m, minor=self.cfg.city_minor_m, road_w=self.cfg.road_w_m,
            lot_w=self.cfg.lot_w_m, lot_h=self.cfg.lot_h_m, setback=self.cfg.lot_setback_m,
            fill_p=self.cfg.lot_fill_prob, jitter=self.cfg.lot_jitter_m
        )
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
            min_dist_start=self.cfg.target_min_dist,
            margin_walls=self.cfg.target_margin_walls, margin_rect=self.cfg.target_margin_rect,
            x_min=self.cfg.x_min, x_max=self.cfg.x_max, y_min=self.cfg.y_min, y_max=self.cfg.y_max,
        ), start_xy=start_xy)
        tgt.run()

        return {"nofly_sdf": nfz_sdf, "nofly_meta": nfz_meta, "target_sdf": tgt_sdf, "target_meta": tgt_meta}

def run(teleop_cfg=None, **kwargs) -> Dict[str, str]:
    """
    Convenience:
      from hydra_teleop.tools import hydra_gen_merged as merged
      merged.run(self.teleop_cfg, seed=101, outdir="models/generated",
                 city_major_m=60.0, city_minor_m=30.0, road_w_m=6.0,
                 lot_w_m=14.0, lot_h_m=12.0, lot_setback_m=1.5,
                 lot_fill_prob=0.9, lot_jitter_m=2.0)
    """
    return ArenaGenerator(teleop_cfg, ArenaGenCfg(**kwargs)).run()

'''
@inproceedings{Parish2001Cities,
  author = {Parish, Y. I. H. and M{\"u}ller, P.},
  title = {Procedural Modeling of Cities},
  booktitle = {Proc. SIGGRAPH},
  year = {2001}
}

@article{Chen2008Streets,
  author = {Chen, G. and Esch, G. and Wonka, P. and M{\"u}ller, P.},
  title = {Interactive Procedural Street Modeling},
  journal = {ACM Trans. Graph.},
  volume = {27},
  number = {5},
  year = {2008}
}

@article{Galin2010Roads,
  author = {Galin, D. and Peytavie, A. and Mar{\'e}chal, N. and Gu{\'e}rin, E.},
  title = {Procedural Generation of Roads},
  journal = {Computer Graphics Forum},
  volume = {29},
  number = {2},
  pages = {429--438},
  year = {2010}
}

@article{Galin2011HierarchicalRoads,
  author = {Galin, D. and Gu{\'e}rin, E. and Peytavie, A. and Mar{\'e}chal, N.},
  title = {Authoring Hierarchical Road Networks},
  journal = {Computer Graphics Forum},
  volume = {30},
  number = {7},
  pages = {2021--2030},
  year = {2011}
}

@inproceedings{Mueller2006Buildings,
  author = {M{\"u}ller, P. and Wonka, P. and Haegler, S. and Ulmer, A. and Van Gool, L.},
  title = {Procedural Modeling of Buildings},
  booktitle = {Proc. SIGGRAPH},
  year = {2006}
}

@article{Wonka2003InstantArchitecture,
  author = {Wonka, P. and Wimmer, M. and Sillion, F. and Ribarsky, W.},
  title = {Instant Architecture},
  journal = {ACM Trans. Graph.},
  volume = {22},
  number = {3},
  pages = {669--677},
  year = {2003}
}

@article{Aliaga2012Parcels,
  author = {Aliaga, D. and Vanegas, C. and Benes, B.},
  title = {Procedural Generation of Parcels in Urban Modeling},
  journal = {Computer Graphics Forum},
  year = {2012}
}
'''