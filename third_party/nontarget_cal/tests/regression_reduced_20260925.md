# Regression vs the validated 2026-09-24 results


## RGB vs lidar_odo/final (14/14 within 3 sigma)

sigma_run {'axis_mm': np.float64(12.5), 'pos_mm': np.float64(17.5), 'rot_deg': np.float64(0.075), 'f_px': 0.9}, sigma_ref {'axis_mm': 5.0, 'pos_mm': 9.0, 'rot_deg': 0.03, 'f_px': 0.5}

| camera | dpos V fwd/left/up mm | |dpos| mm | d along-axis mm | drot deg | df px | dcx/dcy px | far-field px (med/max) | max z | pass |
|---|---|---|---|---|---|---|---|---|---|
| camera_front1 | [-3.3, -10.1, 9.6] | 14.3 | -0.0 | 0.025 | +0.77 | +0.1/+0.2 | 0.3/0.51 | 0.74 | True |
| camera_front2 | [1.1, 4.3, 5.4] | 7.0 | +1.0 | 0.026 | +0.35 | -0.3/-0.5 | 0.13/0.19 | 0.36 | True |
| camera_front3 | [1.1, 1.1, 1.6] | 2.2 | +0.5 | 0.046 | +0.31 | -0.9/+0.0 | 0.16/0.25 | 0.57 | True |
| camera_front4 | [8.0, 2.4, 4.3] | 9.4 | +6.6 | 0.042 | -0.61 | -0.9/+0.1 | 0.23/0.35 | 0.60 | True |
| camera_front5 | [0.7, -0.3, 4.3] | 4.4 | +0.6 | 0.05 | +0.24 | -0.8/-0.4 | 0.12/0.33 | 0.62 | True |
| camera_front6 | [1.9, -1.2, 4.7] | 5.3 | +1.3 | 0.047 | +0.29 | +0.7/-0.4 | 0.09/0.2 | 0.58 | True |
| camera_front7 | [0.0, 1.5, 1.0] | 1.8 | +0.0 | 0.037 | +0.35 | -0.5/-0.4 | 0.1/0.25 | 0.46 | True |
| camera_front8 | [1.2, 9.9, 4.3] | 10.9 | +1.2 | 0.027 | +0.37 | -0.4/-0.1 | 0.11/0.24 | 0.55 | True |
| camera_front9 | [4.1, 5.8, 4.4] | 8.4 | +4.1 | 0.054 | +0.06 | -0.9/+0.3 | 0.13/0.43 | 0.67 | True |
| camera_rear_left | [-37.5, -25.2, 17.2] | 48.3 | +17.1 | 0.218 | +0.82 | +4.1/-0.9 | 0.6/1.72 | 2.70 | True |
| camera_rear_right | [3.0, 20.9, 16.7] | 27.0 | -14.6 | 0.12 | +1.97 | -2.3/-0.9 | 1.01/1.84 | 1.92 | True |
| camera_side_left | [2.4, -10.8, 2.8] | 11.4 | -9.5 | 0.027 | +0.25 | +0.1/+0.2 | 0.37/0.59 | 0.71 | True |
| camera_side_right | [4.7, -9.8, -3.6] | 11.4 | +10.7 | 0.018 | -1.37 | -0.7/+0.1 | 0.58/1.2 | 1.33 | True |
| camera_top | [0.3, -0.5, 18.3] | 18.3 | +0.0 | 0.085 | +0.19 | -1.5/-0.5 | 0.15/0.56 | 1.05 | True |

## Thermal vs thermal_lo/thermal_lo_calib.yaml

sigma_run {'axis_mm': np.float64(29.047375096555626), 'lat_mm': np.float64(9.682458365518542), 'rot_deg': np.float64(0.09682458365518543), 'fx_px': np.float64(1.9364916731037085), 'dt_ms': np.float64(1.9364916731037085)}, sigma_ref {'axis_mm': 15.0, 'lat_mm': 5.0, 'rot_deg': 0.05, 'fx_px': 1.0, 'dt_ms': 1.0}

| camera | d (cam x/y/z) mm | drot deg | dcx/dcy px | far-field px (med/max) | dfx px | d time offset ms | max z | pass |
|---|---|---|---|---|---|---|---|---|
| thermal_left | [4.7, -0.6, -8.2] | 0.346 | [3.3, 2.9] | 0.51/0.88 | -0.04 | -0.29 | 0.43 | True |
| thermal_right | [-3.9, 0.9, -8.8] | 0.208 | [2.7, 0.6] | 0.3/0.64 | +0.11 | +0.71 | 0.36 | True |
