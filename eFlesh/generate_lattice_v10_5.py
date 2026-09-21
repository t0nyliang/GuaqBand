"""
eFlesh forearm lattice generator -- v10.5.

v10.5 = v10.4's pocket-floor fix taken one step further because v10.4's
0.6mm floor still printed a rough (not-flat) magnet seat -- user reported
the magnet would not seat fully (residue), "almost there, better than
before" (2026-07-23). The floor bridge over the ~15%-dense lattice still
sags enough at 3 layers that the solid-fill layers on top do not fully
recover a flat seat. v10.5 raises PLUG_FLOOR_MARGIN 0.6 -> 0.8mm (4 solid
layers) -- one layer short of v9.1's known-clean 1.0mm -- to give the
bridge enough recovery layers for a flat seat, while still keeping ~+10%
more sub-magnet compliance than v9.1 (vs +21% at 0.6mm / +31% at 0.4mm).

This is a deliberate point on the sensitivity-vs-printability trade: the
sub-magnet compliance gain and the floor-bridge problem are the SAME lever
(thinner floor = more compliant lattice under the magnet = worse bridge).
The dominant sensitivity win (~3.8x) is the closed sensor gap, which is
independent of this file; the floor lever is only worth ~+/-10-20% on top.
If 0.8mm STILL will not seat a magnet cleanly, fall back to v9.1's proven
1.0mm floor -- that costs little total sensitivity and is the safe choice.

The clearance-support fix from v10.4 is retained (clearance bottom sits at
PLUG_Z_MIN so the sub-magnet lattice rises to support the floor underside).

---- v10.4 fix rationale (retained) ----

v10.3/v10.2 printed with RESIDUE inside the magnet pockets (user caught it
mid-print, 2026-07-23). Diagnosed directly on lattice_v10_3.stl: the cavity
interiors are geometrically clean (0% solid inside), so this is NOT a mesh
bug -- it is a PRINT-BRIDGE failure of the thin plug floor. Two causes stack:
  1. The plug floor was only 0.4mm (2 layers) -- v9.1's clean-printing floor
     was 1.0mm (5 layers).
  2. The pocket CLEARANCE cut extended 0.5mm tall / centered on the plug,
     i.e. ~0.25mm BELOW the plug floor, stripping away the lattice that
     would otherwise support the floor's underside. Measured: 0% lattice in
     the 0.2mm directly under the floor at every pocket -> the 0.4mm floor
     was bridging over open void and sagged/strung into the cavity.

v10.4 fix, chosen to keep the agreed goal (MAX compliant lattice under the
magnet for sensitivity -- see below) while printing cleanly:
  A. The clearance no longer over-cuts below the floor: its bottom is placed
     exactly at PLUG_Z_MIN, so the sub-magnet lattice now rises to touch the
     floor's underside and support the bridge. This costs ~zero compliance
     (it only fills a previously-cleared void with compliant lattice) and is
     a strict improvement to both printability and the load path.
  B. PLUG_FLOOR_MARGIN 0.4 -> 0.6mm (2 -> 3 solid layers). Still far thinner
     than v9.1's 1.0mm, so most of the sub-magnet compliance gain is kept;
     combined with (A)'s support that is enough for a reliable bridge.
The magnet cavity itself (radius, bore, depth, Z, pause layer) remains
byte-identical -- only the solid floor thickness and the clearance extent
change. If any pocket floor STILL sags on the first print, the next safe
step is PLUG_FLOOR_MARGIN 0.8 (4 layers); do NOT go back below 0.6.

Same geometry-and-goal lineage as v10.2/v10.3 below; this version exists because the SENSOR
STACK was measured and the operating point is now known, and that math
confirms v10.2's lattice design is the right one for the real application.

Application (clarified by user): the pad is strapped FIRMLY onto the
forearm (lattice/touch side against the skin, base + MLX90393 on the
outside) and must resolve VERY LIGHT tendon movements on top of that
static preload. That is force-controlled small-signal sensing about a
preloaded operating point -- NOT a fingertip press. Consequences:

  1. Stiffening the touch surface does nothing here (in a series stack the
     same force reaches the sub-magnet band regardless of the top's
     stiffness), so the top is kept soft/compliant to conform to the arm.
  2. The only mechanical lever is C_sub -- the compliance of the lattice
     BELOW the magnet -- exactly what v10.2 maximized by thinning the plug
     floor (see PLUG_FLOOR_MARGIN below).

Sensor stack (measured): MLX90393 now ~1mm below the base bottom (was 4mm;
the user closed a 4mm air gap that was costing ~6-7x of signal at 1/r^4).
Magnet center now sits r ~= 7.6mm from the sensing element. Because signal
~ C_sub * m / r^4, this ~3.8x gap improvement dwarfs every lattice tweak in
the v8.3->v10.x series -- but it is banked in hardware and orthogonal to
this file.

Why a SOFT, high-travel sub-magnet band is correct for THIS operating point
(the key point the earlier versions could not evaluate without the gap
number): a firm preload compresses the soft sub-magnet band and rides the
magnet DOWN toward the sensor, cutting r and RAISING sensitivity at 1/r^4.
Estimated: unloaded r 7.6mm (~15mT static); firm preload sinking ~1.5mm ->
r ~6.1mm (~29mT, ~2.4x the unloaded gradient); ~2mm sink -> r ~5.6mm
(~37mT, approaching but still under the MLX90393 +-50mT range). So firm
preload parks the magnet in the high-gradient sweet spot -- provided the
band stays soft and has enough travel and does not densify/saturate. The
0.4mm floor gives the MAXIMUM compliant-lattice height (hence maximum
travel headroom) achievable without moving the magnet.

EMPIRICAL KNOB (the one thing simulation cannot settle -- needs a print
test): the exact firm-preload force and the TPU stress-strain curve decide
whether the operating point lands mid-plateau (ideal) or near densification
(dead) / near magnetic saturation. If, under your firm strap, the live
signal to a light tendon touch is strong -> done. If it reads flat/dead or
the MLX pins near +-50mT, the band is over-compressed: either lower the MLX
gain first (config, free), or regenerate with a stiffer sub-magnet lattice
(raise YOUNG_BY_K[0] off its floor) -- do NOT thicken the floor, which only
removes travel. This is a stiffness-to-preload match, tunable in one shot
once the real preload is on the arm.

---- (unchanged design rationale carried from v10.2) ----

Goal for this version: genuinely raise sensor SENSITIVITY (how much the
live magnetometer reading moves when the pad is pressed), while keeping the
touch surface as soft as it can be *and still print* -- explicitly not
repeating v10's mistake.

Why the previous versions stalled (measured, not guessed -- see project
memory eflesh-sensitivity-load-path):

  - v8.3 -> v9 -> v9.1 lowered the Young's-modulus target, but that lever
    saturates: by E=0.0001 the inversion geometry stops changing, so the
    physical struts (and the real stiffness) stop changing too. v9.1 sits
    on that floor. You cannot make this topology's struts softer via E.
  - v10 dropped CELL_SIZE 8.0 -> 6.5 to force two clean Z layers. In this
    thin (~2-cell-tall) part that did measure softer, but ONLY because the
    struts got thinner (0.628 -> 0.510mm) -- which is exactly why it printed
    with missing/sagging struts and debris in the pockets. Cell size here is
    a printability knob, not a free softness knob: thinner struts = softer
    AND less printable, thicker = firmer AND more printable. No free lunch.
  - v10.1 kept CELL_SIZE=8 and only moved WHICH Z layer is truncated
    (Z_GRID_SHIFT). Measured result: ~4% STIFFER than v9.1 overall and ~6%
    stiffer in the top 5mm, because it put the dense full cell under the
    finger. It was a printability fix that accidentally firmed the pad, and
    it did nothing for sensitivity.

The real bottleneck (measured on the output STLs, ∑dz/A(z) proxy):
Only ~31% of the pad's compliance is the lattice BELOW the magnet plug --
the sole material whose compression actually moves a magnet toward the
sensor. ~50% sits ABOVE the magnet and can only feel squishy; under
displacement-controlled pressing it cannot move the magnet at all. Every
"softer lattice" version so far tuned the ~half that can't affect the
reading.

v10.2's change -- exactly ONE, everything else identical to v9.1:

  PLUG_FLOOR_MARGIN: 1.0 -> 0.4 mm

The magnet pocket itself is byte-identical: same cavity radius, same
CAVITY_CLEARANCE bore, same MAGNET_DEPTH, same CAVITY_Z_MIN/MAX, same
PAUSE_Z_LOC / pause layer, same insertion procedure. The ONLY thing that
changes is the thickness of the solid disc the magnet rests ON: it drops
from 1.0mm to 0.4mm (2 print layers at 0.2mm), and the 0.6mm of solid it
gives up is replaced by compliant lattice. That grows the compliant band
directly under each magnet from ~2.8mm to ~3.4mm (+~21% of load-bearing
lattice under the magnet), which raises magnet displacement per unit press
under BOTH force- and displacement-controlled pressing -- the first change
in this whole series that targets the ~31% that the reading actually
depends on.

Touch surface: kept exactly as v9.1 (no Z_GRID_SHIFT). v9.1's arrangement
leaves the material-sparse truncated cell under the finger, which is the
SOFTEST the touch surface can be while still printing cleanly (it's the
config that is known to print). Making the top any softer is the thin-strut
path that broke v10, so "as soft as possible but printable" == v9.1's top.
v10.2 therefore feels at least as soft as v9.1 (slightly softer over each
magnet, where solid plug floor became lattice) and reads more sensitively.

Printability watch-item unique to this version: the 0.4mm plug floor now
bridges over the lattice below it instead of a 1.0mm slab. The lattice
provides frequent support points so the unsupported spans are short, but
this 0.4mm bridge is the one new print risk -- inspect the pocket floors
on the first print. If any pocket floor sags, bump PLUG_FLOOR_MARGIN to
0.6 (3 layers) and regenerate; you keep most of the sensitivity gain.

All dimensions, pouch cavity size/location, bore, base, cell size, Young's
modulus, and grid alignment are unchanged from v9.1.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    import trimesh

from magnet_layout import POCKET_CENTERS, POCKET_ORDER

# ── paths ──────────────────────────────────────────────────────────────────
NOTEBOOK_DIR = Path("microstructure/microstructure_inflators").resolve()
MATOPT_DIR = Path("microstructure/matopt").resolve()
CUT_CELLS_CLI = NOTEBOOK_DIR / "build/isosurface_inflator/cut_cells_cli"
BOX_SURFACE = Path("shapes/forearm_pad_box_v10_5.obj")
RAW_OBJ = Path("output/forearm_lattice_raw_v10_5.obj")
FINAL = Path("output/lattice_v10_5.stl")
SLICER = Path("output/slicer_settings_lattice_v10_5.txt")

# ── part geometry (agreed dimensions, unchanged from v9.1) ──────────────────
BOX_X, BOX_Y, LATTICE_H = 30.0, 30.0, 13.0
BASE_H = 1.0
# Bottom Z boundary is Z=0, which lands exactly on a cut_cells_cli grid line
# (no Z_GRID_SHIFT in this version), so the raw lattice bottom comes out flush
# with Z=0 and the base/lattice union has full overlap -- 0.02 is safe here,
# same as v9.1. (The base-deletion bug in v10.1 was specific to its shifted,
# non-grid-aligned bottom; it does not apply to v10.2.)
BASE_OVERLAP = 0.02

# ── lattice parameters (unchanged from v9.1) ────────────────────────────────
NU = 0.09
CELL_SIZE = 8.0
RESOLUTION = 50

# Grid-alignment shift in X/Y so the box center lands on a cell boundary of
# cut_cells_cli's world-anchored grid (keeps the 4 pockets mirror-symmetric
# within the cell grid). Unchanged from v9.1.
GRID_SHIFT = 1.0

YOUNG_BY_K = {-1: 0.0001, 0: 0.0001, 1: 0.0002}


def young(_i: int, _j: int, k: int) -> float:
    return YOUNG_BY_K.get(k, 0.005)


# ── magnet pouch parameters (cavity/bore/pause UNCHANGED from v9.1) ─────────
MAGNET_RADIUS = 4.8 / 2.0
MAGNET_DEPTH = 1.6
PAUSE_Z_LOC = 6.5
POCKET_BOTTOM_OVERCUT = 0.10

PLUG_RADIUS = MAGNET_RADIUS + 1.0        # unchanged -- based on true MAGNET_RADIUS, not the oversized cavity bore

# *** THE ONLY SUBSTANTIVE CHANGE IN v10.2 ***
# Solid floor under the magnet cut from 1.0mm to 0.4mm (2 print layers). The
# 0.6mm of solid given up becomes compliant lattice directly under the magnet,
# growing the load-bearing band from ~2.8mm to ~3.4mm (+~21%). This is the
# sub-magnet compliance that actually moves the magnet toward the sensor --
# see module docstring. The magnet cavity itself (radius/depth/Z/bore) is
# completely untouched; only the disc the magnet sits on gets thinner.
PLUG_FLOOR_MARGIN = 0.8

PLUG_ROOF_MARGIN = 1.0                   # unchanged -- keeps magnet retention above the cavity
CLEARANCE_OVERLAP = 0.15

# Cavity bore is cut slightly larger than the magnet itself for an easier
# fit; 0.25mm on radius = 0.5mm on diameter, within the requested <=1mm cap.
CAVITY_CLEARANCE = 0.25
CAVITY_RADIUS = MAGNET_RADIUS + CAVITY_CLEARANCE

CAVITY_DEPTH = MAGNET_DEPTH + POCKET_BOTTOM_OVERCUT
CAVITY_Z_MAX = PAUSE_Z_LOC
CAVITY_Z_MIN = CAVITY_Z_MAX - CAVITY_DEPTH
PLUG_Z_MIN = CAVITY_Z_MIN - PLUG_FLOOR_MARGIN
PLUG_Z_MAX = CAVITY_Z_MAX + PLUG_ROOF_MARGIN
PLUG_HEIGHT = PLUG_Z_MAX - PLUG_Z_MIN

LAYER_HEIGHT = 0.20
FIRST_LAYER_H = 0.25

_trimesh_module = None


def load_trimesh():
    global _trimesh_module
    if _trimesh_module is None:
        import trimesh as trimesh_module

        _trimesh_module = trimesh_module
    return _trimesh_module


def edge_stats(mesh: trimesh.Trimesh) -> tuple[int, int]:
    counts = np.bincount(mesh.edges_unique_inverse, minlength=len(mesh.edges_unique))
    boundary = int(np.count_nonzero(counts == 1))
    nonmanifold = int(np.count_nonzero(counts > 2))
    return boundary, nonmanifold


def drop_tiny_debris(
    mesh: trimesh.Trimesh,
    max_debris_faces: int = 1000,
    protect_xy: tuple[tuple[float, float], ...] = (),
    protect_radius: float = 5.0,
) -> trimesh.Trimesh:
    """Drop small disconnected shells left by the inflator at surface/edge
    boundaries. protect_xy exempts components whose centroid is near a
    given (x, y) -- e.g. the magnet pocket centers -- from being dropped
    regardless of size, since a magnet cavity's own boundary is
    topologically its own small closed shell (v8.1 bug: this function
    deleted those as "debris", silently sealing every pocket solid without
    tripping any watertight/volume check).
    """
    t0 = time.monotonic()
    components = mesh.split(only_watertight=False)
    print(f"    [drop_tiny_debris] split() into {len(components)} shells took {time.monotonic()-t0:.1f}s", flush=True)
    if len(components) <= 1:
        return mesh

    def is_protected(c: trimesh.Trimesh) -> bool:
        if not protect_xy:
            return False
        cx, cy = c.vertices[:, 0].mean(), c.vertices[:, 1].mean()
        return any((cx - px) ** 2 + (cy - py) ** 2 < protect_radius ** 2 for px, py in protect_xy)

    kept = [c for c in components if len(c.faces) > max_debris_faces or is_protected(c)]
    if not kept:
        return mesh
    if len(kept) == len(components):
        return mesh
    tm = load_trimesh()
    merged = tm.util.concatenate(kept)
    print(f"  Dropped {len(components) - len(kept)} tiny debris shell(s) (<= {max_debris_faces} faces)", flush=True)
    return merged


def box_at(min_corner: tuple[float, float, float], extents: tuple[float, float, float]) -> trimesh.Trimesh:
    tm = load_trimesh()
    box = tm.creation.box(extents=extents)
    box.apply_translation(np.asarray(min_corner, dtype=float) + np.asarray(extents, dtype=float) / 2.0)
    return box


def build_input_surface() -> None:
    """Box shifted by +GRID_SHIFT in X/Y so its center lands on a cell
    boundary of cut_cells_cli's world-anchored grid (see module docstring).
    Unchanged from v9.1 -- no Z_GRID_SHIFT.
    """
    tm = load_trimesh()
    box = tm.creation.box(extents=[BOX_X, BOX_Y, LATTICE_H])
    box.apply_translation([BOX_X / 2.0 + GRID_SHIFT, BOX_Y / 2.0 + GRID_SHIFT, LATTICE_H / 2.0])
    BOX_SURFACE.parent.mkdir(parents=True, exist_ok=True)
    box.export(BOX_SURFACE)
    print(f"Saved shifted input surface: {BOX_SURFACE} ({len(box.faces)} faces), shift=+{GRID_SHIFT}mm")


def run_cut_cells_cli() -> None:
    import json
    import copy
    import os
    import sys

    import meshio

    sys.path.insert(0, str(MATOPT_DIR / "tools/material2geometry"))
    from material2geometry import Material2Geometry

    mat2geo = Material2Geometry(in_path=str(MATOPT_DIR / "tools/material2geometry/0646_geo_1_coeffs.txt"))

    m = meshio.read(str(BOX_SURFACE))
    v = m.points.astype(float)
    bbox_min = np.amin(v, axis=0)
    bbox_max = np.amax(v, axis=0)

    corner0 = list(map(int, np.ceil(bbox_min / CELL_SIZE) - 1))
    corner1 = list(map(int, np.floor(bbox_max / CELL_SIZE)))
    print(f"Pad bbox: {bbox_min} -> {bbox_max}")
    print(f"Grid corners: {corner0} -> {corner1}")

    pattern_file = NOTEBOOK_DIR / "data/patterns/3D/reference_wires/pattern0646.wire"
    entry = {"params": [], "symmetry": "Cubic", "pattern": str(pattern_file), "index": [0, 0, 0]}
    patterns = []
    for i in range(corner0[0], 1 + corner1[0]):
        for j in range(corner0[1], 1 + corner1[1]):
            for k in range(corner0[2], 1 + corner1[2]):
                geo_params = mat2geo.evaluate(NU, young(i, j, k))
                entry["params"] = geo_params
                entry["index"] = [i, j, k]
                patterns.append(copy.deepcopy(entry))

    with open("data.json", "w") as fp:
        json.dump(patterns, fp)
    print(f"Generated {len(patterns)} cell entries (cell_size={CELL_SIZE}, res={RESOLUTION})")

    RAW_OBJ.parent.mkdir(parents=True, exist_ok=True)
    cmd = (
        f"{CUT_CELLS_CLI} -p data.json --surface {BOX_SURFACE}"
        f" --gridSize {CELL_SIZE} -o {RAW_OBJ} -r {RESOLUTION}"
    )
    print("Running:", cmd)
    ret = os.system(cmd)
    print("Return code:", ret)
    if ret != 0 or not RAW_OBJ.exists():
        raise SystemExit("cut_cells_cli failed")


def build_pocket_plug() -> trimesh.Trimesh:
    """Solid cylindrical plug with a single blind magnet cavity pre-carved,
    opening upward at PAUSE_Z_LOC. Centered at local (0, 0); caller
    translates to each (shifted) magnet location.
    """
    tm = load_trimesh()
    plug = tm.creation.cylinder(radius=PLUG_RADIUS, height=PLUG_HEIGHT, sections=64)
    plug.apply_translation([0.0, 0.0, (PLUG_Z_MIN + PLUG_Z_MAX) / 2.0])

    cavity = tm.creation.cylinder(radius=CAVITY_RADIUS, height=CAVITY_DEPTH + 0.02, sections=64)
    cavity.apply_translation([0.0, 0.0, (CAVITY_Z_MIN + CAVITY_Z_MAX) / 2.0])

    plug = plug.difference(cavity, engine="manifold")
    return plug


def add_magnet_pockets(lattice: trimesh.Trimesh) -> trimesh.Trimesh:
    tm = load_trimesh()
    for idx, (x0, y0) in enumerate(POCKET_CENTERS, start=1):
        x, y = x0 + GRID_SHIFT, y0 + GRID_SHIFT
        # v10.4: clearance bottom sits exactly at PLUG_Z_MIN (no over-cut
        # below the floor), so the sub-magnet lattice rises to touch and
        # support the floor's underside. The 0.5mm extra height is spent
        # entirely ABOVE the plug (clean pocket opening), not below it.
        clr_h = PLUG_HEIGHT + 0.5
        clearance = tm.creation.cylinder(radius=PLUG_RADIUS - CLEARANCE_OVERLAP, height=clr_h, sections=64)
        clearance.apply_translation([x, y, PLUG_Z_MIN + clr_h / 2.0])
        t0 = time.monotonic()
        lattice = lattice.difference(clearance, engine="manifold")
        print(f"    [pocket {idx}] clearance diff: {time.monotonic()-t0:.1f}s, faces={len(lattice.faces):,}", flush=True)

        plug = build_pocket_plug()
        plug.apply_translation([x, y, 0.0])
        t0 = time.monotonic()
        lattice = lattice.union(plug, engine="manifold")
        print(f"    [pocket {idx}] plug union: {time.monotonic()-t0:.1f}s, faces={len(lattice.faces):,}", flush=True)
        print(f"  Pocket {idx}/{len(POCKET_CENTERS)} plug placed at X={x0}, Y={y0} (shifted: {x},{y})", flush=True)
    return lattice


def verify_pockets_open(mesh: trimesh.Trimesh, x_offset: float = 0.0, y_offset: float = 0.0, z_offset: float = 0.0) -> None:
    """Containment check: the magnet cavity's mid-point must be empty space,
    not solid (regression guard for the v8.1 sealed-pocket bug).
    """
    mid_z = (CAVITY_Z_MIN + CAVITY_Z_MAX) / 2.0 + z_offset
    test_pts = np.array([[x + x_offset, y + y_offset, mid_z] for x, y in POCKET_CENTERS])
    inside = mesh.contains(test_pts)
    for (x, y), is_solid in zip(POCKET_CENTERS, inside):
        if is_solid:
            raise RuntimeError(
                f"Pocket at ({x},{y}) is SOLID at its cavity midpoint (z={mid_z}) -- "
                "the magnet cavity got sealed shut somewhere in the pipeline."
            )
    print(f"  Pocket cavities verified open at all {len(POCKET_CENTERS)} locations (z={mid_z:.2f})", flush=True)


def verify_pockets_symmetric(mesh: trimesh.Trimesh, x_offset: float = 0.0, y_offset: float = 0.0, z_offset: float = 0.0) -> None:
    """Regression guard for the v8.1 asymmetric-lattice bug: within the
    clearance radius and the plug's own Z-range (minus a small margin to
    avoid edge effects at the floor/roof caps), the ONLY geometry present
    should be the cavity wall -- identical face count at every pocket. If
    the surrounding lattice fails to clear fully at one pocket (as happened
    at (21,21) in v8.1, verified directly), that pocket's count differs.
    """
    centers = mesh.triangles_center
    zlo = PLUG_Z_MIN + 0.05 + z_offset
    zhi = PLUG_Z_MAX - 0.05 + z_offset
    clear_r = PLUG_RADIUS - CLEARANCE_OVERLAP - 0.05
    counts = []
    for x, y in POCKET_CENTERS:
        dist = np.sqrt((centers[:, 0] - (x + x_offset)) ** 2 + (centers[:, 1] - (y + y_offset)) ** 2)
        mask = (dist < clear_r) & (centers[:, 2] > zlo) & (centers[:, 2] < zhi)
        counts.append(int(np.count_nonzero(mask)))
    if len(set(counts)) != 1:
        raise RuntimeError(f"Pocket geometry is NOT symmetric: face counts inside clearance radius = {counts}")
    print(f"  Pocket symmetry verified: {counts[0]} faces inside clearance radius at every pocket", flush=True)


def build() -> trimesh.Trimesh:
    tm = load_trimesh()

    print("=" * 62)
    print("PHASE 1 - raw surface-conforming lattice (grid-shifted)")
    print("=" * 62)
    build_input_surface()
    run_cut_cells_cli()

    t0 = time.monotonic()
    raw = tm.load(RAW_OBJ, force="mesh")
    print(f"  Raw lattice: {len(raw.faces):,} faces, watertight={raw.is_watertight} (load took {time.monotonic()-t0:.1f}s)", flush=True)
    t0 = time.monotonic()
    raw.update_faces(raw.nondegenerate_faces())
    raw.merge_vertices()
    raw.remove_unreferenced_vertices()
    raw.fix_normals()
    print(f"  Cleanup ops took {time.monotonic()-t0:.1f}s", flush=True)
    raw = drop_tiny_debris(raw)
    print(f"  Cleaned: {len(raw.faces):,} faces", flush=True)

    print("\n" + "=" * 62)
    print("PHASE 2 - solid magnet-pocket plugs")
    print("=" * 62, flush=True)
    shifted_centers = tuple((x + GRID_SHIFT, y + GRID_SHIFT) for x, y in POCKET_CENTERS)
    lattice = add_magnet_pockets(raw)
    lattice = drop_tiny_debris(lattice, protect_xy=shifted_centers)
    verify_pockets_open(lattice, x_offset=GRID_SHIFT, y_offset=GRID_SHIFT)
    verify_pockets_symmetric(lattice, x_offset=GRID_SHIFT, y_offset=GRID_SHIFT)

    print("\n" + "=" * 62)
    print("PHASE 3 - shift back + base plate union")
    print("=" * 62, flush=True)
    lattice.apply_translation([-GRID_SHIFT, -GRID_SHIFT, BASE_H])
    base = box_at((0.0, 0.0, 0.0), (BOX_X, BOX_Y, BASE_H + BASE_OVERLAP))
    for _ in range(3):
        base = base.subdivide()
    t0 = time.monotonic()
    final = base.union(lattice, engine="manifold")
    print(f"  Base union took {time.monotonic()-t0:.1f}s", flush=True)
    final = drop_tiny_debris(final, protect_xy=POCKET_CENTERS)
    final.fix_normals()

    print(f"  Final: {len(final.faces):,} faces, watertight={final.is_watertight}, volume={final.is_volume}")
    return final


def fix_stl_roundtrip(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """STL has no shared-vertex topology; reloading it re-infers connectivity
    from raw float32 coordinates, which can leave a small number of hairline
    non-manifold seams at boolean-union boundaries. The in-memory mesh
    before export is already a clean, verified volume -- any such defect is
    purely an STL round-trip artifact. Deliberately no repair-and-patch
    cycle here (v8.1 bug: that cascaded into deleting a magnet pocket's
    cavity floor and walls); just a safe vertex merge.
    """
    mesh = mesh.copy()
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.merge_vertices(digits_vertex=5)
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.remove_unreferenced_vertices()
    mesh.fix_normals()
    mesh = drop_tiny_debris(mesh, protect_xy=POCKET_CENTERS)
    return mesh


def save_slicer_settings(height: float, triangles: int) -> None:
    abs_pause_z = BASE_H + PAUSE_Z_LOC
    n_exact = (abs_pause_z - FIRST_LAYER_H) / LAYER_HEIGHT + 1
    pause_layer = int(n_exact)
    pause_z_actual = FIRST_LAYER_H + (pause_layer - 1) * LAYER_HEIGHT
    pocket_lines = "\n".join(
        f"  {idx}. X={x:.1f}mm, Y={y:.1f}mm"
        for idx, (x, y) in enumerate(POCKET_CENTERS, start=1)
    )
    text = f"""=== ORCASLICER SETTINGS FOR EFLESH FOREARM PAD v10.5 (0.8mm magnet floor for a clean seat, still ~+10% vs v9.1) ===
File: lattice_v10_5.stl
Geometry: {BOX_X} x {BOX_Y} x {height:.2f}mm  (1mm base + 13mm lattice)

Material: TPU 95A
Nozzle: 0.4mm
Layer height: {LAYER_HEIGHT}mm
First layer height: {FIRST_LAYER_H}mm
Print speed: 20mm/s
Bed temperature: 45C / Nozzle: 220-230C
Brim: 10mm / Supports: None / Retraction: 0.5-1mm

PAUSE AT LAYER: {pause_layer}  (printed Z = {pause_z_actual:.2f}mm)
  Pocket opens at Z = {abs_pause_z}mm

*** v10.5 POCKET-FLOOR (0.6mm still gave a rough seat -> now 0.8mm) ***
  The magnet floor is now 0.8mm (4 layers) AND the pocket clearance no longer
  cuts away the lattice under the floor, so the floor bridges over supported
  lattice with enough recovery layers for a flat magnet seat. If a magnet
  STILL will not seat fully, set PLUG_FLOOR_MARGIN = 1.0 (v9.1's proven floor).
  To be safe on the bridge layers (floor prints at Z ~= {BASE_H + (PAUSE_Z_LOC - (MAGNET_DEPTH+POCKET_BOTTOM_OVERCUT) - PLUG_FLOOR_MARGIN):.2f}-{BASE_H + (PAUSE_Z_LOC - (MAGNET_DEPTH+POCKET_BOTTOM_OVERCUT)):.2f}mm):
    - Slow those layers to ~10mm/s (Bridge speed) and set Bridge flow ~1.05
    - Max part cooling fan on the floor + cavity-wall layers
    - Enable "Avoid crossing walls/perimeters" and wipe-on-retract so the
      nozzle does not ooze/string across the open cavity mouth
  INSPECT the 4 pocket floors at the pause layer before inserting magnets.
  If any floor still sags: raise PLUG_FLOOR_MARGIN to 0.8 (4 layers) and
  regenerate -- do NOT go back below 0.6.

MAGNET INSERTION (UNCHANGED from v9.1):
  4 x N52, 4.8mm dia x 1.6mm thick
  Pocket bore: {CAVITY_RADIUS*2:.1f}mm dia (+{CAVITY_CLEARANCE*2:.1f}mm clearance over magnet dia)
  Centers ({POCKET_ORDER}):
{pocket_lines}
  Polarity in pocket order: N S N S
"""
    SLICER.parent.mkdir(parents=True, exist_ok=True)
    SLICER.write_text(text, encoding="utf-8")
    print(f"Saved: {SLICER} ({triangles:,} triangles in STL)")


def main() -> None:
    tm = load_trimesh()
    final = build()
    FINAL.parent.mkdir(parents=True, exist_ok=True)
    final.export(FINAL)

    reloaded = tm.load(FINAL, force="mesh")
    reloaded = fix_stl_roundtrip(reloaded)
    reloaded.export(FINAL)
    final = reloaded

    verify_pockets_open(final, z_offset=BASE_H)
    verify_pockets_symmetric(final, z_offset=BASE_H)

    boundary, nonmanifold = edge_stats(final)
    print(
        f"\nOn-disk check: watertight={final.is_watertight} volume={final.is_volume} "
        f"boundary_edges={boundary} nonmanifold_edges={nonmanifold}"
    )
    save_slicer_settings(float(final.bounding_box.extents[2]), len(final.faces))
    print(f"Saved: {FINAL}")
    print(f"  BBox: {final.bounding_box.extents.round(4)} mm")


if __name__ == "__main__":
    main()
