# Pre-generated raw lattice

`week2.ipynb` needs one file here:

```
forearm_lattice_raw_v10_5.obj.gz
```

It is the **raw surface-conforming lattice** — the output of eFlesh's
`cut_cells_cli` inflater, before magnet pockets or the base plate. Roughly 740k
triangles, ~29 MB as OBJ, ~10 MB gzipped.

## Why it is committed instead of generated

`cut_cells_cli` is a C++ program. It is not a Python package — it has to be
compiled from source, which needs Linux plus CGAL, Boost, GMP, MPFR, TBB and
SuiteSparse, and takes about an hour. eFlesh only ships a bash build script and
only tests Ubuntu.

A learner on macOS or Windows cannot run that step. So it is run **once**, in
CI, and the result committed. Every other step of `generate_lattice_v10_5.py` is
pure Python and runs anywhere, which is what the worksheet rebuilds.

## How to produce it

1. Push this repository to GitHub.
2. **Actions → Generate raw lattice (cut_cells_cli) → Run workflow.**
   Expect one to three hours: the job compiles OpenVDB, CGAL, libigl and nlopt
   from source on a 4-core runner. The job's timeout is set to 360 minutes.
3. Download the `raw-lattice-v10-5` artifact.
4. Copy `forearm_lattice_raw_v10_5.obj.gz` into this folder and commit it.

The artifact also contains `lattice_v10_5.stl`, which should match the committed
`eFlesh/output/lattice_v10_5.stl`. The workflow prints a comparison of the two —
check it before committing. If they differ, the generator or its inputs changed
and the reference part is no longer what the worksheet reproduces.

## When it needs regenerating

Only if a parameter that feeds the inflater changes in
`eFlesh/generate_lattice_v10_5.py`:

`CELL_SIZE`, `NU`, `YOUNG_BY_K`, `RESOLUTION`, `GRID_SHIFT`, `BOX_X`, `BOX_Y`,
`LATTICE_H`.

Everything else the worksheet touches — plug floor and roof, cavity clearance,
pocket centres, pause height, base thickness — is applied downstream in Python
and needs no rebuild.

Exercise 3 asserts the loaded lattice's bounding box matches the input surface
built in Exercise 1, so a mismatched pair fails loudly rather than quietly
producing the wrong part.
