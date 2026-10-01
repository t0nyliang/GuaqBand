"""Support code for week2.ipynb -- the eFlesh forearm-lattice worksheet.

The worksheet reproduces ``eFlesh/generate_lattice_v10_5.py``. This module
holds the parts that are machinery rather than lesson: loading the raw
lattice that eFlesh's compiled ``cut_cells_cli`` produced, that script's own
``drop_tiny_debris`` helper, and the plots used to inspect the result.

Geometry conventions used throughout:

* millimetres, right-handed, Z up;
* ``GRID_SHIFT`` moves the pad to ``1 <= x,y <= 31`` while the lattice is
  built, so the pad centre lands on a cell boundary of the inflater's
  world-anchored grid; it is shifted back before the base plate goes on;
* the finished part occupies ``0 <= x,y <= 30`` and ``0 <= z <= 14``.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

__all__ = [
    "find_repo_root",
    "output_dir",
    "data_dir",
    "load_raw_lattice",
    "load_material2geometry",
    "drop_tiny_debris",
    "strut_segments",
    "inflate_lattice",
    "describe_mesh",
    "plot_cross_section",
    "plot_side_section",
    "plot_wall_faces",
    "slicer_notes",
]


# -- project paths ---------------------------------------------------------
def find_repo_root(start: Path | None = None) -> Path:
    """Walk upward until the folder holding the vendored eFlesh pipeline."""
    start = (start or Path.cwd()).resolve()
    for candidate in (start, *start.parents):
        if (candidate / "eFlesh" / "generate_lattice_v10_5.py").exists():
            return candidate
    raise FileNotFoundError("Open Jupyter from inside the project repository.")


def output_dir() -> Path:
    """Scratch folder for the worksheet's OBJ/STL files (git-ignored).

    Created on first use, so a fresh checkout has no output folder at all.
    """
    path = Path(__file__).resolve().parent / "output"
    path.mkdir(parents=True, exist_ok=True)
    return path


RAW_LATTICE_FILE = "forearm_lattice_raw_v10_5.obj.gz"


def data_dir() -> Path:
    """Folder holding the pre-generated raw lattice."""
    return Path(__file__).resolve().parent / "data"


def load_raw_lattice(path: Path | None = None):
    """The surface-conforming lattice that ``cut_cells_cli`` produced.

    This is the one step of eFlesh's pipeline that cannot run on a Mac or
    Windows laptop: the inflater is a C++ program that only builds on Linux
    and takes about an hour to compile. It is run once by
    ``.github/workflows/generate-raw-lattice.yml`` and its output committed
    here, so everything downstream is reproducible anywhere.
    """
    import gzip
    import trimesh

    path = Path(path) if path is not None else data_dir() / RAW_LATTICE_FILE
    if not path.exists():
        raise FileNotFoundError(
            f"{path.name} is missing from {path.parent}. "
            "It is produced by the 'Generate raw lattice (cut_cells_cli)' "
            "GitHub Action -- see notebooks/week2/data/README.md."
        )

    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rb") as handle:
        mesh = trimesh.load(handle, file_type="obj", force="mesh")
    return mesh


def load_material2geometry(repo_root: Path):
    """eFlesh's modulus-to-geometry inversion, straight from the vendored tree.

    ``evaluate(nu, E)`` returns the 13 geometry parameters of cut-cell pattern
    0646 for a target Poisson's ratio and Young's modulus. Entries 5 to 8 are
    strut thicknesses as a fraction of a cell. Pure Python and numpy, so unlike
    the inflater it runs anywhere.
    """
    import contextlib
    import io
    import sys

    tools = Path(repo_root) / "eFlesh/microstructure/matopt/tools/material2geometry"
    coeffs = tools / "0646_geo_1_coeffs.txt"
    if not coeffs.exists():
        raise FileNotFoundError(f"material2geometry coefficients not found at {coeffs}")

    if str(tools) not in sys.path:
        sys.path.insert(0, str(tools))
    from material2geometry import Material2Geometry

    # The constructor prints spline diagnostics; keep the notebook output clean.
    with contextlib.redirect_stdout(io.StringIO()):
        return Material2Geometry(in_path=str(coeffs))


def drop_tiny_debris(mesh, max_debris_faces: int = 1000, protect_xy=(), protect_radius: float = 5.0):
    """Verbatim from ``eFlesh/generate_lattice_v10_5.py``.

    Drops small disconnected shells left by the inflator at surface/edge
    boundaries. ``protect_xy`` exempts components whose centroid is near a
    given (x, y) -- e.g. the magnet pocket centres -- from being dropped
    regardless of size, since a magnet cavity's own boundary is
    topologically its own small closed shell. That exemption is the fix for
    the v8.1 bug in which this function deleted every pocket, silently
    sealing them solid without tripping any watertight or volume check.
    """
    import trimesh

    components = mesh.split(only_watertight=False)
    if len(components) <= 1:
        return mesh

    def is_protected(c) -> bool:
        if not protect_xy:
            return False
        cx, cy = c.vertices[:, 0].mean(), c.vertices[:, 1].mean()
        return any((cx - px) ** 2 + (cy - py) ** 2 < protect_radius ** 2 for px, py in protect_xy)

    kept = [c for c in components if len(c.faces) > max_debris_faces or is_protected(c)]
    if not kept or len(kept) == len(components):
        return mesh
    print(f"  dropped {len(components) - len(kept)} tiny debris shell(s)")
    return trimesh.util.concatenate(kept)


# -- strut layout ----------------------------------------------------------
def strut_segments(box, cell_size: float) -> np.ndarray:
    """Endpoints of every beam in a body-centred-cubic unit-cell grid.

    One cell contributes its axis-aligned edges (shared with its neighbours)
    plus eight corner-to-centre diagonals. The grid is anchored at the origin
    and runs one cell past the box on every side, so beams are trimmed by the
    box later instead of stopping short of its faces.

    Returns an (n_segments, 2, 3) array of millimetre coordinates.
    """
    nx, ny, nz = (int(np.floor(length / cell_size)) + 2 for length in box)

    def node(i, j, k):
        return np.array([i, j, k], dtype=float) * cell_size

    segments = []
    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                for di, dj, dk in ((1, 0, 0), (0, 1, 0), (0, 0, 1)):
                    if i + di < nx and j + dj < ny and k + dk < nz:
                        segments.append((node(i, j, k), node(i + di, j + dj, k + dk)))

    for i in range(nx - 1):
        for j in range(ny - 1):
            for k in range(nz - 1):
                centre = node(i + 0.5, j + 0.5, k + 0.5)
                for di in (0, 1):
                    for dj in (0, 1):
                        for dk in (0, 1):
                            segments.append((node(i + di, j + dj, k + dk), centre))

    return np.asarray(segments, dtype=float)


# -- inflater (stand-in for cut_cells_cli) ---------------------------------
def _segment_distance_field(segments, axes, beam_radius):
    """Distance to the nearest beam surface, negative inside a beam."""
    xs, ys, zs = axes
    field = np.full((xs.size, ys.size, zs.size), 1e3, dtype=np.float32)
    margin = beam_radius + float(max(np.diff(xs)[0], np.diff(ys)[0], np.diff(zs)[0]))

    for start, end in segments:
        lo, hi = np.minimum(start, end) - margin, np.maximum(start, end) + margin
        slices = []
        for axis_values, low, high in zip(axes, lo, hi):
            i0 = int(np.searchsorted(axis_values, low))
            i1 = int(np.searchsorted(axis_values, high)) + 1
            slices.append(slice(i0, min(i1, axis_values.size)))
        if any(s.start >= s.stop for s in slices):
            continue

        gx = xs[slices[0]][:, None, None]
        gy = ys[slices[1]][None, :, None]
        gz = zs[slices[2]][None, None, :]
        along = end - start
        length_sq = float(along @ along)
        px, py, pz = gx - start[0], gy - start[1], gz - start[2]
        t = np.clip((px * along[0] + py * along[1] + pz * along[2]) / length_sq, 0.0, 1.0)
        dx, dy, dz = px - t * along[0], py - t * along[1], pz - t * along[2]
        distance = np.sqrt(dx * dx + dy * dy + dz * dz) - beam_radius

        block = field[tuple(slices)]
        np.minimum(block, distance.astype(np.float32), out=block)

    return field


def inflate_lattice(box, cell_size: float, beam_radius: float, voxel: float = 0.21):
    """Build the raw surface-conforming lattice that fills ``box``.

    eFlesh runs its compiled ``cut_cells_cli`` inflater at this point (see
    ``eFlesh/microstructure/microstructure_inflators/cut-cell.ipynb``). That
    binary only builds on Linux and takes roughly an hour to compile, so the
    worksheet inflates an equivalent beam lattice from a signed-distance
    field instead: beams of radius ``beam_radius`` on a ``cell_size`` grid,
    intersected with the box, then meshed with marching cubes.

    Returns a watertight ``trimesh.Trimesh`` whose bounds are ``box``.
    """
    if box is None or cell_size is None or beam_radius is None:
        raise ValueError(
            "inflate_lattice needs box, cell_size and beam_radius -- fill in the Nones."
        )

    import trimesh
    from skimage import measure

    pad = 1.0
    # The half-voxel offset keeps the box faces off the sample planes. A zero
    # crossing landing exactly on a grid node makes marching cubes emit
    # degenerate triangles, and the mesh then comes out non-watertight.
    axes = tuple(
        np.arange(-pad + voxel / 2.0, length + pad, voxel, dtype=np.float32)
        for length in box
    )

    segments = strut_segments(box, cell_size)
    field = _segment_distance_field(segments, axes, beam_radius)

    grids = np.meshgrid(*axes, indexing="ij")
    box_field = np.maximum.reduce(
        [np.maximum(-grid, grid - length) for grid, length in zip(grids, box)]
    )
    field = np.maximum(field, box_field.astype(np.float32))

    vertices, faces, _, _ = measure.marching_cubes(field, level=0.0, spacing=(voxel,) * 3)
    vertices = vertices + np.array([axis[0] for axis in axes])

    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
    mesh.fix_normals()
    return mesh


# -- reporting -------------------------------------------------------------
def describe_mesh(mesh, label: str = "mesh") -> None:
    extents = mesh.bounding_box.extents
    print(f"{label}: {len(mesh.faces):,} triangles, watertight={mesh.is_watertight}")
    print(f"  bounding box  {extents[0]:.2f} x {extents[1]:.2f} x {extents[2]:.2f} mm")
    print(
        f"  solid volume  {mesh.volume:,.1f} mm^3"
        f"  ({100 * mesh.volume / float(np.prod(extents)):.1f}% of the bounding box)"
    )


def plot_cross_section(
    triangles,
    z: float,
    slab: float = 0.25,
    title: str = "",
    circles=(),
    circle_radius: float = 0.0,
) -> None:
    """Scatter every triangle corner within ``slab`` mm of height ``z``.

    ``circles`` draws a dashed outline at each (x, y) -- handy for checking
    that the magnet pouches really are empty where they should be.
    """
    points = np.asarray(triangles).reshape(-1, 3)
    keep = np.abs(points[:, 2] - z) <= slab
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.scatter(points[keep, 0], points[keep, 1], s=0.4, color="tab:blue", alpha=0.5)
    for x, y in circles:
        ax.add_patch(plt.Circle((x, y), circle_radius, fill=False, ls="--", color="tab:red"))
    ax.set(
        title=title or f"Cross-section at z = {z:.2f} mm",
        xlabel="x (mm)",
        ylabel="y (mm)",
        xlim=(-1, 31),
        ylim=(-1, 31),
    )
    ax.set_aspect("equal")
    plt.show()


def plot_side_section(triangles, y: float, title: str = "") -> None:
    """Exact vertical cross-section through the part at height ``y``.

    Unlike the scatter above this cuts the mesh and draws the real outline, so
    a magnet plug shows up as a rectangle with the cavity drawn inside it.
    """
    import trimesh

    triangles = np.asarray(triangles)
    mesh = trimesh.Trimesh(
        vertices=triangles.reshape(-1, 3),
        faces=np.arange(triangles.size // 3).reshape(-1, 3),
        process=False,
    )
    section = mesh.section(plane_origin=[0.0, y, 0.0], plane_normal=[0.0, 1.0, 0.0])
    if section is None:
        raise ValueError(f"nothing to cut at y = {y} mm")

    fig, ax = plt.subplots(figsize=(10, 5))
    for path in section.discrete:
        ax.plot(path[:, 0], path[:, 2], color="tab:blue", lw=0.9)
    ax.set(
        title=title or f"Vertical section at y = {y:.1f} mm",
        xlabel="x (mm)",
        ylabel="z (mm)",
        xlim=(-1, 31),
        ylim=(-1, 15),
    )
    ax.set_aspect("equal")
    plt.show()


def plot_wall_faces(face_centres, mask, pouch_centres, pouch_radius: float) -> None:
    """Top view of the faces a wall check accepted, one marker per face."""
    face_centres = np.asarray(face_centres)
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.scatter(
        face_centres[mask, 0],
        face_centres[mask, 1],
        s=3,
        color="tab:red",
        label="accepted wall faces",
    )
    for x, y in pouch_centres:
        ax.add_patch(plt.Circle((x, y), pouch_radius, fill=False, ls="--", color="0.4"))
    ax.set(
        title="Detected pouch walls",
        xlabel="x (mm)",
        ylabel="y (mm)",
        xlim=(0, 30),
        ylim=(0, 30),
    )
    ax.set_aspect("equal")
    ax.legend(loc="upper right")
    plt.show()


def slicer_notes(
    pouch_z_top: float,
    base_height: float,
    polarity: str,
    pouch_centres,
    layer_height: float = 0.20,
    first_layer_height: float = 0.25,
) -> str:
    """OrcaSlicer crib sheet, including the layer to pause the print on."""
    pause_z = base_height + pouch_z_top
    pause_layer = int((pause_z - first_layer_height) / layer_height + 1)
    actual_z = first_layer_height + (pause_layer - 1) * layer_height
    pockets = "\n".join(
        f"    {n}. X={x:.1f} mm, Y={y:.1f} mm   pole {pole}"
        for n, ((x, y), pole) in enumerate(zip(pouch_centres, polarity.split()), start=1)
    )
    return f"""=== SLICER SETTINGS: lattice_v10_5.stl ===
  Material            TPU 95A
  Nozzle              0.4 mm
  Layer height        {layer_height} mm   (first layer {first_layer_height} mm)
  Print speed         20 mm/s
  Bed / nozzle        45 C / 220-230 C
  Brim 10 mm, no supports, retraction 0.5-1 mm

  PAUSE AT LAYER      {pause_layer}   (printed z = {actual_z:.2f} mm)
  The nozzle reaches the top of the open cavities at z = {pause_z:.2f} mm. Pause
  there, drop the magnets in, and resume -- the roof prints over them.

  Magnets             4 x N52, 4.8 mm dia x 1.6 mm thick
{pockets}
  Alternating poles keep each magnet's field distinguishable at its sensor.
"""
