"""GNSS/INS of a DM bag: build the ENU trajectory file and read it back.

`build_ins` is /hdd/DM_calib/online/extract_dm.py:build_ins for one bag (same numbers);
`Trajectory` is the "dm" branch of online_calib/traj.py:Trajectory. The INS is used only for the
tracker's static-track test (the car's own body) and for body masks; every solve uses the LiDAR
odometry.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from .bag import header_ns
from .workspace import atomic_savez

WGS84_A, WGS84_F = 6378137.0, 1 / 298.257223563


def geodetic_to_ecef(lat_deg, lon_deg, h):
    la, lo = np.radians(lat_deg), np.radians(lon_deg)
    e2 = WGS84_F * (2 - WGS84_F)
    n = WGS84_A / np.sqrt(1 - e2 * np.sin(la) ** 2)
    return np.stack([(n + h) * np.cos(la) * np.cos(lo), (n + h) * np.cos(la) * np.sin(lo),
                     (n * (1 - e2) + h) * np.sin(la)], axis=-1)


def geodetic_to_enu(lat_deg, lon_deg, h, lat0, lon0, h0):
    la, lo = np.radians(lat0), np.radians(lon0)
    R = np.array([[-np.sin(lo), np.cos(lo), 0.0],
                  [-np.sin(la) * np.cos(lo), -np.sin(la) * np.sin(lo), np.cos(la)],
                  [np.cos(la) * np.cos(lo), np.cos(la) * np.sin(lo), np.sin(la)]])
    return (geodetic_to_ecef(lat_deg, lon_deg, h) - geodetic_to_ecef(lat0, lon0, h0)) @ R.T


def build_ins(fx: np.ndarray, vl: np.ndarray, im: np.ndarray, out_path: Path, log=print) -> dict:
    """fx (n,5) [t lat lon alt status]; vl (m,4) [t vx vy vz] ENU; im (k,11) [t qw qx qy qz wx wy wz ax ay az]."""
    tf = fx[:, 0].astype(np.int64)
    ti = im[:, 0].astype(np.int64)
    tv = vl[:, 0].astype(np.int64)
    if not np.array_equal(tf, tv):
        log(f"    /gps/vel has {len(tv)} stamps vs {len(tf)} for /gps/fix -> resampled")
        vl = np.column_stack([tf] + [np.interp(tf.astype(np.float64), tv.astype(np.float64), vl[:, 1 + k])
                                     for k in range(3)])
    rot = Rotation.from_quat(im[:, [2, 3, 4, 1]])
    tq = np.clip(tf.astype(np.float64), float(ti[0]), float(ti[-1]))
    q = Slerp(ti.astype(np.float64), rot)(tq).as_quat()[:, [3, 0, 1, 2]]
    w = np.stack([np.interp(tq, ti.astype(np.float64), im[:, 5 + k]) for k in range(3)], -1)
    acc = np.stack([np.interp(tq, ti.astype(np.float64), im[:, 8 + k]) for k in range(3)], -1)
    t = tf
    order = np.argsort(t, kind="stable")
    keep = np.r_[True, np.diff(t[order]) > 0]
    order = order[keep]
    t = t[order]
    llh = fx[:, 1:4][order]
    quat = q[order]
    venu = vl[:, 1:4][order]
    w = w[order]
    acc = acc[order]
    origin = llh[0].copy()
    pos = geodetic_to_enu(llh[:, 0], llh[:, 1], llh[:, 2], *origin)
    rot = Rotation.from_quat(quat[:, [1, 2, 3, 0]])
    vel_ego = rot.inv().apply(venu)
    atomic_savez(out_path, dm=np.array(True), t=t, pos=pos, origin=origin, quat=quat,
                 vel_ego=vel_ego, vel_enu=venu, llh=llh, imu_t=t, imu_rate=w, imu_accel=acc,
                 imu_hz=np.array(100.0), clip_id=np.zeros(len(t), int), fix_status=fx[:, 4][order])
    return {"n": int(len(t)), "origin": origin.tolist(), "duration_s": float((t[-1] - t[0]) / 1e9)}


def ins_rows_from_msgs(bag, rows_iter):
    """Collect /gps/fix, /gps/vel, /imu/data arrays from (topic, t, raw) rows."""
    fx, vl, im = [], [], []
    for name, _, data in rows_iter:
        m = bag.deserialize(name, data)
        if name.endswith("fix"):
            fx.append((header_ns(m), m.latitude, m.longitude, m.altitude, m.status.status))
        elif name.endswith("vel"):
            vl.append((header_ns(m), m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z))
        else:
            im.append((header_ns(m), m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z,
                       m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                       m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
    return np.array(fx), np.array(vl), np.array(im)


class Trajectory:
    """Ego poses at arbitrary times from the INS npz (online_calib traj.Trajectory, "dm" source)."""

    def __init__(self, ins_npz: Path):
        z = np.load(ins_npz)
        self.t = z["t"].astype(np.int64)
        self.origin = z["origin"].copy()
        self.pos = z["pos"]
        self.rot = Rotation.from_quat(z["quat"][:, [1, 2, 3, 0]])
        self.vel_ego = z["vel_ego"]
        self._slerp = Slerp(self.t.astype(np.float64), self.rot)
        self.imu_t = z["imu_t"].astype(np.int64)
        self.imu_rate = z["imu_rate"]
        self.imu_hz = float(z["imu_hz"])

    def R(self, t_ns) -> np.ndarray:
        return self._slerp(np.atleast_1d(t_ns).astype(np.float64)).as_matrix()

    def p(self, t_ns) -> np.ndarray:
        t = np.atleast_1d(t_ns).astype(np.float64)
        return np.stack([np.interp(t, self.t, self.pos[:, i]) for i in range(3)], axis=-1)

    def speed(self, t_ns) -> np.ndarray:
        t = np.atleast_1d(t_ns).astype(np.float64)
        return np.hypot(np.interp(t, self.t, self.vel_ego[:, 0]), np.interp(t, self.t, self.vel_ego[:, 1]))
