# Regression vs the validated 2026-09-24 results


## RGB vs lidar_odo/final (14/14 within 3 sigma)

sigma_run {'axis_mm': 10.0, 'pos_mm': 14.0, 'rot_deg': 0.06, 'f_px': 0.9}, sigma_ref {'axis_mm': 5.0, 'pos_mm': 9.0, 'rot_deg': 0.03, 'f_px': 0.5}

| camera | dpos V fwd/left/up mm | |dpos| mm | d along-axis mm | drot deg | df px | dcx/dcy px | far-field px (med/max) | max z | pass |
|---|---|---|---|---|---|---|---|---|---|
| camera_front1 | [-3.8, -6.4, 0.2] | 7.4 | -3.5 | 0.03 | +0.05 | +0.6/-0.2 | 0.15/0.29 | 0.45 | True |
| camera_front2 | [-0.4, 1.2, -1.2] | 1.7 | -0.4 | 0.019 | +0.03 | +0.3/-0.2 | 0.07/0.18 | 0.28 | True |
| camera_front3 | [-6.5, -6.3, -2.1] | 9.2 | -5.5 | 0.038 | +0.25 | -0.6/+0.7 | 0.25/0.44 | 0.57 | True |
| camera_front4 | [0.2, -0.3, -1.9] | 1.9 | +0.3 | 0.019 | -0.17 | +0.3/+0.0 | 0.05/0.12 | 0.29 | True |
| camera_front5 | [0.5, -1.9, -0.6] | 2.0 | +0.5 | 0.012 | -0.18 | +0.2/-0.1 | 0.07/0.14 | 0.18 | True |
| camera_front6 | [2.9, -1.2, 0.6] | 3.2 | +2.3 | 0.006 | -0.24 | +0.1/+0.1 | 0.1/0.19 | 0.23 | True |
| camera_front7 | [1.6, -0.4, -0.7] | 1.8 | +1.6 | 0.006 | -0.16 | +0.1/+0.1 | 0.06/0.1 | 0.15 | True |
| camera_front8 | [1.1, 5.0, -1.2] | 5.3 | +1.2 | 0.008 | -0.11 | +0.2/+0.0 | 0.07/0.16 | 0.32 | True |
| camera_front9 | [0.6, 3.7, -2.1] | 4.3 | +0.6 | 0.011 | -0.09 | +0.2/+0.0 | 0.07/0.17 | 0.26 | True |
| camera_rear_left | [0.6, -6.5, 9.4] | 11.5 | -3.9 | 0.009 | -0.18 | -0.3/+0.1 | 0.12/0.24 | 0.69 | True |
| camera_rear_right | [0.1, -3.8, 8.4] | 9.3 | +2.4 | 0.044 | -1.09 | +1.3/+0.2 | 0.63/1.09 | 1.06 | True |
| camera_side_left | [1.1, -9.6, 0.6] | 9.7 | -8.8 | 0.002 | -0.3 | -0.0/-0.0 | 0.13/0.17 | 0.79 | True |
| camera_side_right | [1.0, 1.1, -2.3] | 2.7 | -0.8 | 0.014 | -1.03 | -0.1/-0.0 | 0.44/0.83 | 1.00 | True |
| camera_top | [3.5, 5.2, 5.6] | 8.4 | +3.4 | 0.052 | -0.21 | -1.0/+0.1 | 0.12/0.23 | 0.78 | True |

## Thermal vs thermal_lo/thermal_lo_calib.yaml

sigma_run {'axis_mm': np.float64(15.0), 'lat_mm': np.float64(5.0), 'rot_deg': np.float64(0.05), 'fx_px': np.float64(1.0), 'dt_ms': np.float64(1.0)}, sigma_ref {'axis_mm': 15.0, 'lat_mm': 5.0, 'rot_deg': 0.05, 'fx_px': 1.0, 'dt_ms': 1.0}

| camera | d (cam x/y/z) mm | drot deg | dcx/dcy px | far-field px (med/max) | dfx px | d time offset ms | max z | pass |
|---|---|---|---|---|---|---|---|---|
| thermal_left | [3.3, 1.3, -3.3] | 0.303 | [0.4, 3.6] | 0.16/0.79 | -0.32 | -0.34 | 0.47 | True |
| thermal_right | [-3.6, 2.9, -11.8] | 0.08 | [-0.2, 1.0] | 0.11/0.22 | +0.13 | -0.2 | 0.56 | True |
