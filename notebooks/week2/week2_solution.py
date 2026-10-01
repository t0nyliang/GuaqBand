"""Reference answers for week2.ipynb -- the whole pipeline in one command.

    .\\.venv\\Scripts\\python.exe .\\notebooks\\week2\\week2_solution.py     (Windows)
    ./.venv/bin/python notebooks/week2/week2_solution.py                   (macOS)

Rebuilds eFlesh's lattice_v10_5.stl from the committed raw lattice and writes it
to notebooks/week2/output/. Two uses:

* check the environment before teaching -- if this runs, the worksheet will;
* the answer key, when a learner is stuck on one exercise.

Every value below is what the notebook's `None` blanks are asking for, in the
same order, with the matching exercise named. Section by section this mirrors
eFlesh/generate_lattice_v10_5.py.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parent))

from week2_lattice import (
    describe_mesh,
    drop_tiny_debris,
    find_repo_root,
    load_material2geometry,
    load_raw_lattice,
    output_dir,
    slicer_notes,
)

TIMINGS: list[tuple[str, float]] = []


def stage(label: str, start: float) -> None:
    TIMINGS.append((label, time.monotonic() - start))


def main() -> Path:
    repo_root = find_repo_root()
    out = output_dir()
    overall = time.monotonic()

    # -- Exercise 1: input surface + grid shift ---------------------------
    BOX_X, BOX_Y, LATTICE_H = 30.0, 30.0, 13.0
    GRID_SHIFT = 1.0
    pad_box = trimesh.creation.box(extents=[BOX_X, BOX_Y, LATTICE_H])
    pad_box.apply_translation(
        [BOX_X / 2 + GRID_SHIFT, BOX_Y / 2 + GRID_SHIFT, LATTICE_H / 2]
    )
    pad_box.export(out / "forearm_pad_box_v10_5.obj")
    print(f"input surface bounds {pad_box.bounds.tolist()}")

    # -- Exercise 2: cell grid + per-layer stiffness ----------------------
    CELL_SIZE, NU, RESOLUTION = 8.0, 0.09, 50
    YOUNG_BY_K = {-1: 0.0001, 0: 0.0001, 1: 0.0002}
    lo, hi = pad_box.bounds
    corner0 = np.ceil(lo / CELL_SIZE).astype(int) - 1
    corner1 = np.floor(hi / CELL_SIZE).astype(int)
    n_cells = int(np.prod(corner1 - corner0 + 1))
    print(f"cell grid {corner0.tolist()} .. {corner1.tolist()} = {n_cells} cells")

    mat2geo = load_material2geometry(repo_root)
    thinnest = min(p * CELL_SIZE for p in mat2geo.evaluate(NU, YOUNG_BY_K[0])[5:9])
    print(f"thinnest strut {thinnest:.3f} mm at E={YOUNG_BY_K[0]}")

    # -- Exercise 3: load the inflated lattice ----------------------------
    t = time.monotonic()
    raw = load_raw_lattice()
    stage("load raw lattice", t)
    describe_mesh(raw, "raw lattice")
    np.testing.assert_allclose(raw.bounds, pad_box.bounds, atol=0.05)

    # -- Exercise 4: clean the raw mesh -----------------------------------
    t = time.monotonic()
    raw.update_faces(raw.nondegenerate_faces())
    raw.merge_vertices()
    raw.remove_unreferenced_vertices()
    raw.fix_normals()
    raw = drop_tiny_debris(raw)
    stage("clean raw mesh", t)

    # -- Exercise 5: the magnet plug --------------------------------------
    MAGNET_RADIUS = 4.8 / 2.0
    MAGNET_DEPTH = 1.6
    PAUSE_Z_LOC = 6.5
    POCKET_BOTTOM_OVERCUT = 0.10
    CAVITY_CLEARANCE = 0.25
    PLUG_FLOOR_MARGIN = 0.8
    PLUG_ROOF_MARGIN = 1.0
    PLUG_RADIUS = MAGNET_RADIUS + 1.0

    CAVITY_DEPTH = MAGNET_DEPTH + POCKET_BOTTOM_OVERCUT
    CAVITY_Z_MAX = PAUSE_Z_LOC
    CAVITY_Z_MIN = CAVITY_Z_MAX - CAVITY_DEPTH
    CAVITY_RADIUS = MAGNET_RADIUS + CAVITY_CLEARANCE
    PLUG_Z_MIN = CAVITY_Z_MIN - PLUG_FLOOR_MARGIN
    PLUG_Z_MAX = CAVITY_Z_MAX + PLUG_ROOF_MARGIN
    PLUG_HEIGHT = PLUG_Z_MAX - PLUG_Z_MIN

    def build_pocket_plug() -> trimesh.Trimesh:
        plug = trimesh.creation.cylinder(radius=PLUG_RADIUS, height=PLUG_HEIGHT, sections=64)
        cavity = trimesh.creation.cylinder(
            radius=CAVITY_RADIUS, height=CAVITY_DEPTH + 0.02, sections=64
        )
        plug.apply_translation([0.0, 0.0, (PLUG_Z_MIN + PLUG_Z_MAX) / 2.0])
        cavity.apply_translation([0.0, 0.0, (CAVITY_Z_MIN + CAVITY_Z_MAX) / 2.0])
        return plug.difference(cavity, engine="manifold")

    # -- Exercise 6: weld the plugs in ------------------------------------
    POCKET_CENTERS = ((9.0, 9.0), (21.0, 9.0), (9.0, 21.0), (21.0, 21.0))
    POLARITY = "N S N S"
    CLEARANCE_OVERLAP = 0.15

    t = time.monotonic()
    lattice = raw.copy()
    for x0, y0 in POCKET_CENTERS:
        x, y = x0 + GRID_SHIFT, y0 + GRID_SHIFT
        clr_h = PLUG_HEIGHT + 0.5
        clearance = trimesh.creation.cylinder(
            radius=PLUG_RADIUS - CLEARANCE_OVERLAP, height=clr_h, sections=64
        )
        clearance.apply_translation([x, y, PLUG_Z_MIN + clr_h / 2.0])
        lattice = lattice.difference(clearance, engine="manifold")

        plug = build_pocket_plug()
        plug.apply_translation([x, y, 0.0])
        lattice = lattice.union(plug, engine="manifold")

    shifted_centers = tuple((x + GRID_SHIFT, y + GRID_SHIFT) for x, y in POCKET_CENTERS)
    lattice = drop_tiny_debris(lattice, protect_xy=shifted_centers)
    stage("weld 4 plugs", t)
    describe_mesh(lattice, "lattice + plugs")

    # -- Exercise 7: shift back + base plate ------------------------------
    BASE_H, BASE_OVERLAP = 1.0, 0.02
    t = time.monotonic()
    stacked = lattice.copy()
    stacked.apply_translation([-GRID_SHIFT, -GRID_SHIFT, BASE_H])
    base = trimesh.creation.box(extents=(BOX_X, BOX_Y, BASE_H + BASE_OVERLAP))
    base.apply_translation([BOX_X / 2, BOX_Y / 2, (BASE_H + BASE_OVERLAP) / 2])
    for _ in range(3):
        base = base.subdivide()
    final = base.union(stacked, engine="manifold")
    final = drop_tiny_debris(final, protect_xy=POCKET_CENTERS)
    final.fix_normals()
    stage("base plate", t)
    describe_mesh(final, "final part")

    # -- Exercise 8: verify the pockets -----------------------------------
    t = time.monotonic()
    mid_z = (CAVITY_Z_MIN + CAVITY_Z_MAX) / 2.0 + BASE_H
    test_points = np.array([[x, y, mid_z] for x, y in POCKET_CENTERS])
    for (x, y), solid in zip(POCKET_CENTERS, final.contains(test_points)):
        assert not solid, f"pocket at ({x}, {y}) is sealed solid"

    centres = final.triangles_center
    zlo, zhi = PLUG_Z_MIN + 0.05 + BASE_H, PLUG_Z_MAX - 0.05 + BASE_H
    clear_r = PLUG_RADIUS - CLEARANCE_OVERLAP - 0.05
    counts = []
    for x, y in POCKET_CENTERS:
        dist = np.sqrt((centres[:, 0] - x) ** 2 + (centres[:, 1] - y) ** 2)
        mask = (dist < clear_r) & (centres[:, 2] > zlo) & (centres[:, 2] < zhi)
        counts.append(int(np.count_nonzero(mask)))
    assert len(set(counts)) == 1, f"pockets are not identical: {counts}"
    stage("verify pockets", t)
    print(f"  4 cavities open, all identical at {counts[0]} faces")

    # -- Exercise 9: export -----------------------------------------------
    t = time.monotonic()
    final_path = out / "lattice_v10_5.stl"
    final.export(final_path)
    reloaded = trimesh.load(final_path, force="mesh")
    reloaded.update_faces(reloaded.nondegenerate_faces())
    reloaded.merge_vertices(digits_vertex=5)
    reloaded.update_faces(reloaded.nondegenerate_faces())
    reloaded.remove_unreferenced_vertices()
    reloaded.fix_normals()
    reloaded = drop_tiny_debris(reloaded, protect_xy=POCKET_CENTERS)
    reloaded.export(final_path)
    stage("export + round trip", t)

    print()
    print(slicer_notes(PAUSE_Z_LOC, BASE_H, POLARITY, POCKET_CENTERS))

    reference_path = repo_root / "eFlesh/output/lattice_v10_5.stl"
    if reference_path.exists():
        reference = trimesh.load(reference_path, force="mesh")
        print(f"{'':22}{'yours':>14}{'eFlesh v10_5':>16}")
        print(f"{'triangles':22}{len(reloaded.faces):>14,}{len(reference.faces):>16,}")
        print(f"{'solid volume (mm^3)':22}{reloaded.volume:>14,.1f}{reference.volume:>16,.1f}")

    width = max(len(label) for label, _ in TIMINGS)
    print("\ntiming")
    for label, seconds in TIMINGS:
        print(f"  {label:<{width}}  {seconds:6.2f}s")
    print(f"  {'TOTAL':<{width}}  {time.monotonic() - overall:6.2f}s")
    print(f"\nwrote {final_path}  ({final_path.stat().st_size / 1e6:.1f} MB)")
    return final_path


if __name__ == "__main__":
    main()
