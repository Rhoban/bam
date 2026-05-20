Using Identified Parameters in MuJoCo Warp via mjlab (GPU)
==========================================================

Status: draft scaffold for release documentation.

Intended content
----------------

- Parameter ingestion path in mjlab pipelines
- Mapping between BAM friction model coefficients and Warp runtime code
- Numerical parity checks between CPU and GPU simulations

Planned validation checklist
----------------------------

- Same initial state and command traces on CPU/GPU
- Error metrics (MAE) computed over aligned trajectories
- Stability checks for high-load and low-speed regimes
