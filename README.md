# 3D Data Processing – Lab 3: Full Point Cloud Registration

This project implements a **full point cloud registration pipeline** in C++ using **Open3D**: global feature-based alignment followed by local refinement with **ICP**, comparing an **SVD-based** and a **Levenberg–Marquardt (Ceres)-based** ICP formulation.

The pipeline:

1. Load source and target point clouds.
2. Run a **descriptor-based global registration** (FPFH + RANSAC) to get a coarse initial alignment.
3. Perturb the alignment with configurable rotation/translation noise.
4. Refine it with **ICP**, in one of two modes:
   - `svd` — closed-form point-to-point ICP via SVD,
   - `lm` — point-to-point ICP solved as a nonlinear least-squares problem with **Ceres** (Levenberg–Marquardt).
5. Report RMSE, runtime and iteration count for each configuration; optionally sweep over noise levels (`all` mode).

---

## Author

* [@adelinapopa02](https://github.com/adelinapopa02)

---

## Project overview

| File | Role |
|---|---|
| `Registration.h` / `.cpp` | Core class: global (descriptor-based) registration, noisy-transform generation, SVD and LM ICP, RMSE computation, visualization and I/O. |
| `registration_trial.cpp` | CLI entry point: loads the two clouds, runs global registration, then ICP in the requested mode. |
| `TERMINAL_RESULTS.txt` | Logged runs (RMSE, timing, iterations) on the test datasets for both ICP variants. |
| `Lab3 - Full Cloud Registration.pdf` | Assignment write-up / report. |

---

## Dependencies

- [Open3D](http://www.open3d.org/) (C++)
- [Ceres Solver](http://ceres-solver.org/)
- Eigen3

## Build

```bash
mkdir build && cd build
cmake ..
make
```

## Run

```bash
./registration <source point cloud> <target point cloud> <mode>
```

`<mode>` is `svd`, `lm`, or `all` (sweeps multiple noise levels for both methods).

### Example

```bash
./registration ../data/bunny/source.ply ../data/bunny/target.ply svd
./registration ../data/bunny/source.ply ../data/bunny/target.ply lm
```
