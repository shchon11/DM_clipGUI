"""Frame conversions and the OpenCV equidistant camera model (metres)."""
import cv2
import numpy as np


DEFAULT_R_LIDAR_V = np.diag([-1.0, -1.0, 1.0])


def frustum_corners(calibration, length):
    """Short K-dependent pinhole guide, not the distorted lens field boundary."""
    if not all(key in calibration for key in ("K", "width", "height")):
        return np.array([[-.63, -.43, 1], [.63, -.43, 1],
                         [.63, .43, 1], [-.63, .43, 1]]) * length
    w, h = calibration["width"], calibration["height"]
    pixels = np.array([[0., 0., 1.], [w, 0., 1.], [w, h, 1.], [0., h, 1.]])
    rays = np.linalg.solve(np.asarray(calibration["K"], float), pixels.T).T
    # Equal-length rays keep very wide-angle zero-shot intrinsics readable.
    return rays / np.linalg.norm(rays, axis=1, keepdims=True) * length


def vehicle_from_camera(T_cam_lidar, R_lidar_V=None):
    """Return T_V_cam, with V x forward, y left, z up at the Ouster.

    R_lidar_V maps vehicle coordinates into os_lidar. The calibrated matrix
    should be supplied; the fallback only accounts for the rearward LiDAR x.
    Camera axes are optical: x right, y down, z forward.
    """
    T = np.asarray(T_cam_lidar, dtype=float)
    R = np.asarray(DEFAULT_R_LIDAR_V if R_lidar_V is None else R_lidar_V,
                   dtype=float)
    result = np.eye(4)
    result[:3, :3] = R.T @ T[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ T[:3, 3]
    return result


def interpolate_pose(start, end, alpha):
    """Interpolate an SE(3) pose with shortest-path SO(3) rotation."""
    a, b = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
    u = float(np.clip(alpha, 0.0, 1.0))
    if u == 0:
        return a.copy()
    if u == 1:
        return b.copy()
    delta, _ = cv2.Rodrigues(a[:3, :3].T @ b[:3, :3])
    R, _ = cv2.Rodrigues(delta * u)
    result = np.eye(4)
    result[:3, :3] = a[:3, :3] @ R
    result[:3, 3] = (1 - u) * a[:3, 3] + u * b[:3, 3]
    return result


def project_fisheye(points_lidar, T_cam_lidar, K, D):
    """Return (uv, optical_depth_m, valid) for KB4 equidistant projection.

    Pixels use K's resolution. Invalid/behind-camera points have NaN uv;
    callers additionally clip to the image bounds and their camera mask.
    """
    points = np.asarray(points_lidar, dtype=float).reshape(-1, 3)
    T, K = np.asarray(T_cam_lidar, float), np.asarray(K, float)
    D = np.asarray(D, float).reshape(4)
    camera = points @ T[:3, :3].T + T[:3, 3]
    depth = camera[:, 2]
    valid = np.isfinite(camera).all(axis=1) & (depth > 0.05)
    uv = np.full((len(points), 2), np.nan)
    xy = camera[valid, :2] / depth[valid, None]
    radius = np.linalg.norm(xy, axis=1)
    theta = np.arctan(radius)
    theta2 = theta * theta
    distorted = theta * (1 + theta2 * (D[0] + theta2 * (
        D[1] + theta2 * (D[2] + theta2 * D[3]))))
    scale = np.ones_like(radius)
    np.divide(distorted, radius, out=scale, where=radius > 1e-12)
    normalized = xy * scale[:, None]
    uv[valid, 0] = K[0, 0] * normalized[:, 0] + K[0, 1] * normalized[:, 1] + K[0, 2]
    uv[valid, 1] = K[1, 1] * normalized[:, 1] + K[1, 2]
    valid &= np.isfinite(uv).all(axis=1)
    return uv, depth, valid


def project_camera(points_lidar, T_cam_lidar, K, D, model="equidistant"):
    """Dispatch native calibrated models; the recorded thermal pair is pinhole."""
    if model in ("equidistant", "kb", "fisheye"):
        return project_fisheye(points_lidar, T_cam_lidar, K, D)
    if model != "plumb_bob":
        raise ValueError("unsupported camera model: " + model)
    points = np.asarray(points_lidar, float).reshape(-1, 3)
    T, K, D = np.asarray(T_cam_lidar, float), np.asarray(K, float), np.asarray(D, float)
    camera = points @ T[:3, :3].T + T[:3, 3]
    depth = camera[:, 2]
    valid = np.isfinite(camera).all(axis=1) & (depth > 0.05)
    uv = np.full((len(points), 2), np.nan)
    if valid.any():
        projected, _ = cv2.projectPoints(camera[valid], np.zeros(3), np.zeros(3), K, D)
        uv[valid] = projected.reshape(-1, 2)
    valid &= np.isfinite(uv).all(axis=1)
    return uv, depth, valid
