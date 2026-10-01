"""The filled-in answers to week2.ipynb, as one runnable script.

    .\\.venv\\Scripts\\python.exe .\\notebooks\\week2\\week2_answers.py     (Windows)
    ./.venv/bin/python notebooks/week2/week2_answers.py                   (Mac)

Two uses:

  * Check the setup before teaching. If this runs, the notebook will.
  * Look up one answer when someone is stuck.

The sections below match the notebook's eight steps, in order, and use the same
variable names. Takes about 30 seconds and writes touch_pad.stl.
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
    drop_debris,
    find_repo_root,
    inflate_lattice,
    load_material_model,
    output_dir,
    read_cell_pattern,
    slicer_notes,
)


def main() -> Path:
    REPO = find_repo_root()
    OUT = output_dir()
    started = time.monotonic()

    # -- Step 1: the outline box ------------------------------------------
    BOX_X, BOX_Y, HEIGHT = 30.0, 30.0, 13.0
    SHIFT = 1.0

    box = trimesh.creation.box(extents=[BOX_X, BOX_Y, HEIGHT])
    box.apply_translation([BOX_X / 2 + SHIFT, BOX_Y / 2 + SHIFT, HEIGHT / 2])
    print("step 1  box spans", box.bounds[0], "to", box.bounds[1])

    # -- Step 2: cell size and softness -----------------------------------
    CELL = 8.0
    SOFTNESS = {-1: 0.0001, 0: 0.0001, 1: 0.0002}

    low, high = box.bounds
    first_cell = np.ceil(low / CELL).astype(int) - 1
    last_cell = np.floor(high / CELL).astype(int)
    n_cells = int(np.prod(last_cell - first_cell + 1))

    material = load_material_model(REPO)
    thinnest = min(t * CELL for t in material.thicknesses(SOFTNESS[0]))
    print(f"step 2  {n_cells} cells, thinnest strut {thinnest:.3f} mm")

    # -- Step 3: fill the box with lattice --------------------------------
    points, lines = read_cell_pattern(REPO)
    THICKNESS = float(np.mean([t * CELL for t in material.thicknesses(SOFTNESS[0])]))

    marker = time.monotonic()
    lattice = inflate_lattice(
        REPO,
        size=(BOX_X, BOX_Y, HEIGHT),
        start=(SHIFT, SHIFT, 0.0),
        cell_size=CELL,
        strut_radius=THICKNESS / 2,
        first_cell=first_cell,
        last_cell=last_cell,
    )
    print(f"step 3  {len(points)} points and {len(lines)} lines per cell, "
          f"strut {THICKNESS:.3f} mm  ({time.monotonic() - marker:.1f}s)")
    describe_mesh(lattice, "        lattice")

    # -- Step 4: tidy up the mesh -----------------------------------------
    lattice.update_faces(lattice.nondegenerate_faces())
    for name in ("merge_vertices", "remove_unreferenced_vertices", "fix_normals"):
        getattr(lattice, name)()
    lattice = drop_debris(lattice)
    print(f"step 4  tidied to {len(lattice.faces):,} triangles")

    # -- Step 5: one magnet pocket ----------------------------------------
    MAGNET_RADIUS = 4.8 / 2
    MAGNET_THICK = 1.6
    MAGNET_HEIGHT = 6.5
    EXTRA_WIDTH = 0.25
    EXTRA_DEPTH = 0.10
    SOLID_BELOW = 0.8
    SOLID_ABOVE = 1.0
    PLUG_RADIUS = MAGNET_RADIUS + 1.0

    POCKET_DEPTH = MAGNET_THICK + EXTRA_DEPTH
    POCKET_TOP = MAGNET_HEIGHT
    POCKET_BOTTOM = POCKET_TOP - POCKET_DEPTH

    POCKET_RADIUS = MAGNET_RADIUS + EXTRA_WIDTH
    PLUG_BOTTOM = POCKET_BOTTOM - SOLID_BELOW
    PLUG_TOP = POCKET_TOP + SOLID_ABOVE
    PLUG_HEIGHT = PLUG_TOP - PLUG_BOTTOM

    def make_plug():
        plug = trimesh.creation.cylinder(radius=PLUG_RADIUS, height=PLUG_HEIGHT, sections=64)
        pocket = trimesh.creation.cylinder(
            radius=POCKET_RADIUS, height=POCKET_DEPTH + 0.02, sections=64
        )
        plug.apply_translation([0.0, 0.0, (PLUG_BOTTOM + PLUG_TOP) / 2])
        pocket.apply_translation([0.0, 0.0, (POCKET_BOTTOM + POCKET_TOP) / 2])
        return plug.difference(pocket, engine="manifold")

    print(f"step 5  plug {PLUG_BOTTOM} to {PLUG_TOP} mm, pocket "
          f"{POCKET_BOTTOM} to {POCKET_TOP} mm")

    # -- Step 6: four pockets in the lattice ------------------------------
    MAGNET_SPOTS = ((9.0, 9.0), (21.0, 9.0), (9.0, 21.0), (21.0, 21.0))
    POLES = "N S N S"
    OVERLAP = 0.15

    marker = time.monotonic()
    for x0, y0 in MAGNET_SPOTS:
        x, y = x0 + SHIFT, y0 + SHIFT
        hole_height = PLUG_HEIGHT + 0.5
        hole = trimesh.creation.cylinder(
            radius=PLUG_RADIUS - OVERLAP, height=hole_height, sections=64
        )
        hole.apply_translation([x, y, PLUG_BOTTOM + hole_height / 2])
        lattice = lattice.difference(hole, engine="manifold")

        plug = make_plug()
        plug.apply_translation([x, y, 0.0])
        lattice = lattice.union(plug, engine="manifold")

    shifted_spots = tuple((x + SHIFT, y + SHIFT) for x, y in MAGNET_SPOTS)
    lattice = drop_debris(lattice, keep_near=shifted_spots)
    print(f"step 6  four pockets placed  ({time.monotonic() - marker:.1f}s)")

    # -- Step 7: the base plate -------------------------------------------
    BASE = 1.0
    BITE = 0.02

    pad = lattice.copy()
    pad.apply_translation([-SHIFT, -SHIFT, BASE])

    plate = trimesh.creation.box(extents=(BOX_X, BOX_Y, BASE + BITE))
    plate.apply_translation([BOX_X / 2, BOX_Y / 2, (BASE + BITE) / 2])
    for _ in range(3):
        plate = plate.subdivide()

    part = plate.union(pad, engine="manifold")
    part = drop_debris(part, keep_near=MAGNET_SPOTS)
    part.fix_normals()
    describe_mesh(part, "step 7  finished part")

    # -- Step 8: check it, then export ------------------------------------
    check_height = (POCKET_BOTTOM + POCKET_TOP) / 2 + BASE
    probes = np.array([[x, y, check_height] for x, y in MAGNET_SPOTS])
    for (x, y), is_solid in zip(MAGNET_SPOTS, part.contains(probes)):
        assert not is_solid, f"pocket at ({x}, {y}) is solid -- it got sealed up"

    centres = part.triangles_center
    inside = ((centres[:, 2] > PLUG_BOTTOM + BASE + 0.05)
              & (centres[:, 2] < PLUG_TOP + BASE - 0.05))
    counts = []
    for x, y in MAGNET_SPOTS:
        distance = np.sqrt((centres[:, 0] - x) ** 2 + (centres[:, 1] - y) ** 2)
        counts.append(int(np.count_nonzero(inside & (distance < PLUG_RADIUS - OVERLAP - 0.05))))
    assert len(set(counts)) == 1, f"pockets differ: {counts} -- check SHIFT"
    print(f"step 8  four pockets hollow and identical at {counts[0]} triangles")

    NAME = "touch_pad.stl"
    path = OUT / NAME
    part.export(path)

    print()
    print(slicer_notes(MAGNET_HEIGHT, BASE, POLES, MAGNET_SPOTS))

    real_path = REPO / "eFlesh/output/lattice_v10_5.stl"
    if real_path.exists():
        real = trimesh.load(real_path, force="mesh")
        gap = abs(100 * part.volume / real.volume - 100)
        print(f"yours {part.volume:,.0f} mm3 vs real pad {real.volume:,.0f} mm3 "
              f"-- within {gap:.0f}%")

    print(f"\nwrote {path}  ({path.stat().st_size / 1e6:.0f} MB)")
    print(f"total {time.monotonic() - started:.0f}s")
    return path


if __name__ == "__main__":
    main()
