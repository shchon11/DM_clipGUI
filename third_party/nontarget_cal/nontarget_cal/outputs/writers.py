"""Deliverable files, in the team format of /hdd/DM_calib/deliverables (make_rgb_deliverables.py,
make_thermal_deliverables.py):

  extrinsic/<cam>.yaml   camera, parent_frame os_lidar, child_frame, R, t, T_cam_lidar, rpy_deg_xyz,
                         quaternion_xyzw + camera_position_in_os_lidar_m, camera_position_vehicle_m,
                         time offset, method, data, empirical uncertainty, validation numbers
  intrinsic/<cam>.yaml   RGB: equidistant (Kannala-Brandt, D = k1..k4); thermal: plumb_bob (k1 k2)
  camera_info/<cam>.yaml ROS sensor_msgs/CameraInfo-style YAML (distortion_model equidistant / plumb_bob)
  calib_json/<cam>.json  online_calib format (loadable by online_calib/evaluate.py:Calib)
  rig.yaml               rig = reference camera optical frame, T_rig_lidar, per camera T_cam_rig, positions
"""
from __future__ import annotations

import json
import zipfile
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as Rot

EXT_DESC = "x_cam = R @ x_lidar + t  (lidar -> camera). Camera optical frame (OpenCV: x right, y down, z forward)."
POS_AXES = "forward / left / up, origin at the Ouster (os_lidar x points BACKWARD)"


def fl(a):
    return [float(x) for x in np.asarray(a).ravel()]


def mat(a):
    return [fl(r) for r in np.asarray(a)]


def _dump(obj, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(obj, sort_keys=False, allow_unicode=True, width=120))


def fov_kb(K, D, W, H):
    pts = np.array([[[0, H / 2]], [[W - 1, H / 2]], [[W / 2, 0]], [[W / 2, H - 1]]], np.float64)
    n = cv2.fisheye.undistortPoints(pts, K, D).reshape(-1, 2)
    ang = np.degrees(np.arctan(np.abs(n)))
    return float(ang[0, 0] + ang[1, 0]), float(ang[2, 1] + ang[3, 1])


def fov_pinhole(K, D, W, H):
    def ang(u, v):
        xd, yd = (u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1]
        x, y = xd, yd
        for _ in range(20):
            r2 = x * x + y * y
            dd = 1 + D[0] * r2 + D[1] * r2 * r2
            x, y = xd / dd, yd / dd
        return x, y
    xl, _ = ang(0, K[1, 2]); xr, _ = ang(W - 1, K[1, 2]); _, yt = ang(K[0, 2], 0); _, yb = ang(K[0, 2], H - 1)
    return float(np.degrees(np.arctan(-xl) + np.arctan(xr))), float(np.degrees(np.arctan(-yt) + np.arctan(yb)))


def camera_info(name, W, H, K, D, model):
    P = np.zeros((3, 4)); P[:3, :3] = K
    return {"image_width": int(W), "image_height": int(H), "camera_name": name,
            "camera_matrix": {"rows": 3, "cols": 3, "data": fl(K)},
            "distortion_model": model,
            "distortion_coefficients": {"rows": 1, "cols": len(D), "data": fl(D)},
            "rectification_matrix": {"rows": 3, "cols": 3, "data": fl(np.eye(3))},
            "projection_matrix": {"rows": 3, "cols": 4, "data": fl(P)}}


def write_rgb(res, out: Path, R_L_V, metrics: dict, meta: dict, W=1920, H=1200, cfg=None):
    files = []
    for c, v in res["cameras"].items():
        T = np.array(v["T_cam_lidar"]); R, t = T[:3, :3], T[:3, 3]; C = -R.T @ t
        fx, fy, cx, cy, k1, k2, k3, k4 = v["intr_kb"]
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]]); D = np.array([k1, k2, k3, k4])
        hf, vf = fov_kb(K, D, W, H)
        m = metrics.get(c, {})
        ext = {"camera": c, "parent_frame": "os_lidar", "child_frame": c, "description": EXT_DESC,
               "R": mat(R), "t": fl(t), "T_cam_lidar": mat(T),
               "rpy_deg_xyz": fl(Rot.from_matrix(R).as_euler("xyz", degrees=True)),
               "quaternion_xyzw": fl(Rot.from_matrix(R).as_quat()),
               "camera_position_in_os_lidar_m": fl(C),
               "camera_position_vehicle_m": {"value": [round(x, 4) for x in fl(R_L_V.T @ C)], "axes": POS_AXES},
               "time_offset_s": float(res.get("dt_ms", 0.0)) * 1e-3,
               "topic": meta.get("topics", {}).get(c),
               "method": "targetless (nontarget_cal): KLT tracks + trajectory fixed from refined LiDAR odometry + "
                         "landmark-to-LiDAR-plane ties; lens free (fx = fy, cx, cy, k1, k2)",
               "data": meta.get("data"),
               "uncertainty_empirical_1sigma": {k: m.get(k) for k in ("rot_deg", "pos_mm", "along_axis_mm", "focal_px")},
               "uncertainty_note": "from two disjoint halves of the data (sigma of a half-data solve, conservative); "
                                   "the rig-to-LiDAR common offset of ~1-2 cm is the floor of the method; solver "
                                   "formal sigmas are 10-50x optimistic and are not quoted",
               "validation": {k: m.get(k) for k in ("track_reproj_px", "heldout_track_reproj_px", "lidar_edge_px",
                                                    "lidar_edge_parked_px", "vote")},
               "generated_by": meta.get("generated_by")}
        intr = {"camera": c, "session": meta.get("data"), "model": "equidistant",
                "model_note": "Kannala-Brandt = OpenCV fisheye = ROS distortion_model 'equidistant'; D = [k1, k2, k3, k4]",
                "image_width": W, "image_height": H, "camera_matrix": mat(K), "distortion_coefficients": fl(D),
                "horizontal_fov_deg": round(hf, 2), "vertical_fov_deg": round(vf, 2),
                "estimation": "targetless, jointly with the extrinsic (see extrinsic YAML); fy tied to fx (square pixels)"}
        _dump(ext, out / "extrinsic" / f"{c}.yaml"); _dump(intr, out / "intrinsic" / f"{c}.yaml")
        _dump(camera_info(c, W, H, K, D, "equidistant"), out / "camera_info" / f"{c}.yaml")
        cj = {"channel": c, "model": "kb", "width": W, "height": H, "intr": v["intr_kb"], "T_cam_lidar": T.tolist(),
              "T_lidar_cam": np.linalg.inv(T).tolist(), "T_ego_lidar": np.eye(4).tolist(),
              "T_ego_lidar_note": "identity placeholder (not estimated by nontarget_cal)", "dt_s": 0.0, "rs_s": 0.0,
              "position_V_m": fl(R_L_V.T @ C)}
        (out / "calib_json").mkdir(parents=True, exist_ok=True)
        (out / "calib_json" / f"{c}.json").write_text(json.dumps(cj, indent=1))
        files += [out / "extrinsic" / f"{c}.yaml", out / "intrinsic" / f"{c}.yaml", out / "camera_info" / f"{c}.yaml"]
    return files


def write_thermal(res, out: Path, R_L_V, metrics: dict, meta: dict, rig=None, W=640, H=480):
    files = []
    for c, v in res["cameras"].items():
        T = np.array(v["T_cam_lidar"]); R, t = T[:3, :3], T[:3, 3]; C = -R.T @ t
        fx, fy, cx, cy, k1, k2 = v["intr"]
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]]); D = np.array([k1, k2, 0.0, 0.0, 0.0])
        hf, vf = fov_pinhole(K, D, W, H)
        m = metrics.get(c, {})
        ext = {"camera": c, "parent_frame": "os_lidar", "child_frame": c, "description": EXT_DESC,
               "R": mat(R), "t": fl(t), "T_cam_lidar": mat(T),
               "rpy_deg_xyz": fl(Rot.from_matrix(R).as_euler("xyz", degrees=True)),
               "quaternion_xyzw": fl(Rot.from_matrix(R).as_quat()),
               "camera_position_in_os_lidar_m": fl(C),
               "camera_position_vehicle_m": {"value": [round(x, 4) for x in fl(R_L_V.T @ C)], "axes": POS_AXES},
               "time_offset_s": round(float(v["dt_s_mean"]), 5),
               "time_offset_meaning": "capture of the CENTRE row relative to t_frame (~header.stamp), PTP clock",
               "time_offset_per_bag_s": meta.get("thermal_dt_per_bag", {}).get(c),
               "time_offset_per_window_sd_ms": round(1e3 * float(v["dt_s_sd"]), 2),
               "row_readout_s": round(float(v["rs_s"]), 4),
               "row_readout_meaning": "capture time of row v = t_frame + time_offset_s + row_readout_s * (v/480 - 0.5); "
                                      "negative = bottom rows first (includes the bolometer lag)",
               "topic": meta.get("topics", {}).get(c),
               "method": "targetless (nontarget_cal): thermal KLT BA with the LiDAR-odometry trajectory fixed, "
                         "landmark-to-plane ties, near-range LiDAR-edge term (LO-accumulated), left-right stereo links",
               "data": meta.get("data"),
               "uncertainty_empirical_1sigma": {k: m.get(k) for k in ("rot_deg", "pos_mm", "along_axis_mm", "focal_px")},
               "validation": {k: m.get(k) for k in ("track_reproj_px", "heldout_track_reproj_px", "lidar_edge_px", "vote")},
               "generated_by": meta.get("generated_by")}
        if rig is not None:
            Tcr = T @ np.linalg.inv(rig)
            ext["camera_position_rig_m"] = {"value": fl(-Tcr[:3, :3].T @ Tcr[:3, 3]),
                                            "frame": "reference camera (camera_front5) optical frame"}
        intr = {"camera": c, "session": meta.get("data"), "model": "plumb_bob",
                "model_note": "pinhole + radial k1 k2 (OpenCV / ROS plumb_bob, D = [k1, k2, p1, p2, k3], p1 = p2 = k3 = 0)",
                "image_width": W, "image_height": H, "camera_matrix": mat(K), "distortion_coefficients": fl(D),
                "horizontal_fov_deg": round(hf, 2), "vertical_fov_deg": round(vf, 2),
                "estimation": "fx, cx, cy, k1, k2 from a rotation-only solve on far points; fx and fy/fx refined in the "
                              "final solve (fy/fx weakly determined)"}
        _dump(ext, out / "extrinsic" / f"{c}.yaml"); _dump(intr, out / "intrinsic" / f"{c}.yaml")
        _dump(camera_info(c, W, H, K, D, "plumb_bob"), out / "camera_info" / f"{c}.yaml")
        files += [out / "extrinsic" / f"{c}.yaml", out / "intrinsic" / f"{c}.yaml", out / "camera_info" / f"{c}.yaml"]
    return files


def write_rig(rgb_res, thermal_res, out: Path, R_L_V, ref_cam="camera_front5"):
    cams = {}
    for res in (rgb_res, thermal_res):
        if res:
            cams.update({c: np.array(v["T_cam_lidar"]) for c, v in res["cameras"].items()})
    if ref_cam not in cams:
        return None
    T_rig_lidar = cams[ref_cam]
    rig = {"schema": "nontarget_cal/rig/1", "units": "metres, degrees where named _deg",
           "frame_note": "T_a_b maps a point in frame b to frame a. rig = reference camera optical frame "
                         "(OpenCV: x right, y down, z forward). V = vehicle frame (x forward, y left, z up, "
                         "origin at the Ouster), axes measured from the LiDAR data.",
           "reference_camera": ref_cam, "R_lidar_V": np.round(R_L_V, 6).tolist(),
           "lidar": {"T_rig_lidar": np.round(T_rig_lidar, 9).tolist()}, "cameras": {}}
    for c, T in cams.items():
        Tcr = T @ np.linalg.inv(T_rig_lidar)
        C = -T[:3, :3].T @ T[:3, 3]
        rig["cameras"][c] = {"T_cam_lidar": np.round(T, 9).tolist(), "T_cam_rig": np.round(Tcr, 9).tolist(),
                             "position_in_rig_m": np.round(-Tcr[:3, :3].T @ Tcr[:3, 3], 5).tolist(),
                             "position_V_m": np.round(R_L_V.T @ C, 4).tolist()}
    _dump(rig, out / "rig.yaml")
    return out / "rig.yaml"


def make_zip(out: Path, files, name="nontarget_cal.zip"):
    z = out / name
    with zipfile.ZipFile(z, "w", zipfile.ZIP_DEFLATED) as zf:
        for f in files:
            f = Path(f)
            if f.exists():
                zf.write(f, f.relative_to(out))
    return z
