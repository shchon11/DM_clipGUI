# Regression after the CUDA-LK / scheduling changes (2026-09-27, feat/speed)

## Reduced run, 4 windows, from scratch (nt_regress/speed/reduced_v3, 3 workers)

## RGB vs lidar_odo/final (14/14 within 3 sigma)

sigma_run {'axis_mm': np.float64(12.5), 'pos_mm': np.float64(17.5), 'rot_deg': np.float64(0.075), 'f_px': 0.9}, sigma_ref {'axis_mm': 5.0, 'pos_mm': 9.0, 'rot_deg': 0.03, 'f_px': 0.5}

| camera | dpos V fwd/left/up mm | |dpos| mm | d along-axis mm | drot deg | df px | dcx/dcy px | far-field px (med/max) | max z | pass |
|---|---|---|---|---|---|---|---|---|---|
| camera_front1 | [-6.7, -12.1, 11.1] | 17.8 | -2.7 | 0.022 | +0.89 | -0.0/+0.1 | 0.35/0.58 | 0.90 | True |
| camera_front2 | [0.5, 6.7, 4.7] | 8.2 | +0.4 | 0.026 | +0.35 | +0.3/-0.4 | 0.13/0.22 | 0.42 | True |
| camera_front3 | [-1.2, -1.1, 2.3] | 2.8 | -1.9 | 0.032 | +0.38 | -0.6/+0.0 | 0.15/0.26 | 0.40 | True |
| camera_front4 | [5.3, 5.8, 5.3] | 9.5 | +2.9 | 0.042 | -0.45 | -0.8/+0.0 | 0.14/0.23 | 0.52 | True |
| camera_front5 | [2.2, -2.6, 6.4] | 7.3 | +2.0 | 0.051 | +0.17 | -0.7/-0.6 | 0.12/0.34 | 0.63 | True |
| camera_front6 | [2.7, -1.4, 4.9] | 5.7 | +2.0 | 0.041 | +0.39 | +0.6/-0.4 | 0.09/0.2 | 0.51 | True |
| camera_front7 | [-1.6, -1.1, 2.2] | 3.0 | -1.6 | 0.044 | +0.37 | -0.6/-0.4 | 0.13/0.27 | 0.54 | True |
| camera_front8 | [0.6, 8.3, 4.5] | 9.4 | +0.6 | 0.01 | +0.23 | -0.0/-0.0 | 0.13/0.18 | 0.48 | True |
| camera_front9 | [1.1, 8.6, 4.2] | 9.7 | +1.0 | 0.054 | +0.25 | -0.7/+0.5 | 0.12/0.28 | 0.67 | True |
| camera_rear_left | [-33.0, -36.9, 15.5] | 51.9 | +6.8 | 0.181 | +1.24 | +3.4/-0.8 | 0.53/1.52 | 2.64 | True |
| camera_rear_right | [-2.9, 22.7, 15.2] | 27.5 | -11.0 | 0.141 | +1.7 | -2.8/-0.8 | 0.89/1.84 | 1.75 | True |
| camera_side_left | [2.6, -8.7, 2.8] | 9.5 | -7.5 | 0.019 | +0.36 | -0.1/+0.1 | 0.4/0.64 | 0.56 | True |
| camera_side_right | [4.8, -9.8, -3.2] | 11.3 | +10.7 | 0.018 | -0.98 | -0.6/+0.1 | 0.49/0.97 | 0.95 | True |
| camera_top | [4.2, -1.7, 18.0] | 18.6 | +4.0 | 0.094 | +0.03 | -1.8/-0.4 | 0.17/0.56 | 1.17 | True |

## Thermal vs thermal_lo/thermal_lo_calib.yaml

sigma_run {'axis_mm': np.float64(29.047375096555626), 'lat_mm': np.float64(9.682458365518542), 'rot_deg': np.float64(0.09682458365518543), 'fx_px': np.float64(1.9364916731037085), 'dt_ms': np.float64(1.9364916731037085)}, sigma_ref {'axis_mm': 15.0, 'lat_mm': 5.0, 'rot_deg': 0.05, 'fx_px': 1.0, 'dt_ms': 1.0}

| camera | d (cam x/y/z) mm | drot deg | dcx/dcy px | far-field px (med/max) | dfx px | d time offset ms | max z | pass |
|---|---|---|---|---|---|---|---|---|
| thermal_left | [4.5, -0.8, -7.4] | 0.391 | [3.4, 3.5] | 0.54/1.06 | -0.14 | -0.59 | 0.42 | True |
| thermal_right | [-3.8, 0.9, -9.2] | 0.203 | [2.6, 0.7] | 0.3/0.64 | +0.09 | +0.63 | 0.35 | True |

## Full run, 25 windows (nt_regress/speed/full_v3): extraction and LiDAR odometry reused, tracks and solves fresh

- Extraction reused from nt_regress/full (unpruned): sweeps, kept JPEGs and cam_index byte-identical to full_final's for S02, S07, W13 (current extraction code = full_final's).
- LO reused from full_final (LO code unchanged since 0bb9956): a fresh LO chain on W02 and S04 gives T_w_L within 4e-13 m (KISS-ICP's threaded reduction is not bit-deterministic), map points within 1 float32 ulp (5e-7 m) for 3e-6 of the points.
- Thermal (the thermal chain finished before the run was stopped on request): thermal_left d cam [3.4, 1.0, -2.0] mm, far-field 0.18 px, dfx -0.35 px, dt -0.53 ms, max z 0.49; thermal_right [-3.5, 3.0, -12.6] mm, 0.12 px, +0.19 px, -0.19 ms, max z 0.59 -> 2/2 within 3 sigma (full_final, OpenCL LK: max z 0.48 / 0.64).
- RGB (25 windows): not measured - the run was stopped during the RGB solves (tracks done). The 4-window run above passes 14/14.
