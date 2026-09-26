"""Task functions run inside worker processes (see worker.py). Each is resumable: it returns early
when its output already exists (outputs are written atomically)."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import yaml

from .workspace import read_json, write_json


# ------------------------------------------------------------------ extraction
def t_scan_bag(ws, cfg, log, bag_path, bag_id, names):
    from .bag import Bag
    from .extract import scan_bag
    marker = ws.root / "extract" / f"{bag_id}_scan.json"
    if marker.exists():
        return read_json(marker)
    bag = Bag(bag_path)
    info = scan_bag(bag, bag_id, names, cfg, ws, log=log)
    bag.close()
    write_json(marker, info)
    return info


def t_extract_window(ws, cfg, log, bag_path, bag_id, win, names, zero_ns, rgb, thermal):
    from .bag import Bag
    from .extract import extract_standstill, extract_window, window_done
    if win.get("kind") == "standstill":
        if (ws.seg_dir(win["name"]) / "source.json").exists():
            return {"cached": True}
        bag = Bag(bag_path)
        out = extract_standstill(bag, win, names, cfg, ws, zero_ns, log=log)
        bag.close()
        return out
    if window_done(ws, win["name"], thermal):
        return {"cached": True}
    bag = Bag(bag_path)
    out = extract_window(bag, bag_id, win, names, cfg, ws, zero_ns, rgb=True, thermal=thermal, lidar=True, log=log)
    bag.close()
    return out


# ------------------------------------------------------------------ LiDAR odometry
def t_lo_chain(ws, cfg, log, win):
    """KISS-ICP -> continuous-time refinement -> map cache -> quality check, for one window."""
    from .lo.check import lo_check
    from .lo.kiss import run_kiss
    from .lo.maps import build_map
    from .lo.refine import refine
    from .lo.lotraj import LOTraj
    from scipy.spatial.transform import Rotation as Rot
    c = cfg["lo"]
    seg = ws.seg_dir(win)
    if not ws.lo("kiss", win).exists():
        run_kiss(seg, ws.lo("kiss", win), threads=cfg["resources"]["kiss_threads"], log=log, **c["kiss"])
    if not ws.lo("ref", win).exists():
        r = c["refine"]
        refine(seg, ws.lo("kiss", win), ws.lo("ref", win), iters=r["iters"], src_voxel=r["src_voxel"],
               tgt_voxel=r["tgt_voxel"], scale=r["scale"], cv_sigma=r["cv_sigma"], max_src=r["max_src"],
               offsets=r["offsets"], log=log)
    if not ws.lo("map", win).exists():
        build_map(seg, ws.lo("ref", win), ws.lo("map", win), log=log, **c["map"])
    if ws.lo_check(win).exists():
        return read_json(ws.lo_check(win))
    chk = lo_check(seg, ws.lo("ref", win), gaps=c["check"]["gaps"], n=c["check"]["n"], log=log)
    tr = LOTraj.load(ws.lo("ref", win))
    T = tr.T[1:]
    rot = Rot.from_matrix(np.einsum("nji,njk->nik", T[:-1, :3, :3], T[1:, :3, :3])).magnitude()
    path = np.linalg.norm(np.diff(T[:, :3, 3], axis=0), axis=1).sum()
    k = read_json(ws.lo("kiss", win).with_suffix(".json")) if ws.lo("kiss", win).with_suffix(".json").exists() else {}
    # KISS vs refined: the refinement should only polish (a large jump = a KISS failure)
    zk = np.load(ws.lo("kiss", win))
    Tk = zk["T_w_L"]
    n = min(len(Tk), len(T))
    dk = Rot.from_matrix(np.einsum("nji,njk->nik", Tk[:n, :3, :3], T[:n, :3, :3])).magnitude()
    chk.update({"rotation_deg": float(np.degrees(rot.sum())), "path_m": float(path),
                "kiss_vs_ref_rot_deg_max": float(np.degrees(dk.max())), **k})
    write_json(ws.lo_check(win), chk)
    return chk


def t_vehicle_axes(ws, cfg, log, wins):
    from .lo.axes import vehicle_axes
    out = ws.root / "lo" / "vehicle_axes.json"
    if out.exists():
        return read_json(out)
    ax = vehicle_axes([(ws.lo("ref", w), ws.seg_dir(w)) for w in wins], log=log)
    write_json(out, ax)
    return ax


# ------------------------------------------------------------------ tracks
def t_rgb_tracks(ws, cfg, log, win, cam, bag_id, mask):
    import os
    import cv2
    from .rgb.tracks import run_tracks
    # OpenCV otherwise uses every core in each of the parallel trackers (oversubscription); the
    # tracks are identical for any thread count
    cv2.setNumThreads(int(os.environ.get("OPENCV_NUM_THREADS", "2")))
    out = ws.tracks(win, cam)
    if out.exists():
        return {"cached": True}
    c = cfg["rgb"]["tracks"]
    return run_tracks(ws.seg_dir(win), cam, Path(mask), ws.ins(bag_id), out, grid=c["grid"], per_cell=c["per_cell"],
                      fb_max=c["fb_max"], min_len=c["min_len"], log=log)


def t_thermal_tracks(ws, cfg, log, win, cam):
    from .thermal.timemodel import Plan
    from .thermal.tracks import run_thermal_tracks
    if ws.thermal_tracks(win, cam).exists():
        return {"cached": True}
    c = cfg["thermal"]["tracks"]
    return run_thermal_tracks(ws, Plan(ws), win, cam, prep=c["prep"], log=log, grid=tuple(c["grid"]),
                              per_cell=c["per_cell"], fb_max=c["fb_max"], min_len=c["min_len"])


# ------------------------------------------------------------------ solves
def _rgb_start(spec, cams, design):
    from .rgb.solve import calib_cams, nominal_cams, result_calib
    if spec["kind"] == "nominal":
        return nominal_cams(cams, design)
    if spec["kind"] == "result":
        return calib_cams(cams, result_calib(read_json(spec["path"])))
    if spec["kind"] == "calib":
        cal = read_json(spec["path"])
        return calib_cams(cams, {c: (np.array(cal[c]["T_cam_lidar"]), np.array(cal[c]["intr"])) for c in cams})
    raise ValueError(spec)


def t_rgb_solve(ws, cfg, log, name, segs, cams, start, opts, held=None):
    from .rgb.solve import result_calib, run_ba
    out = ws.solve_dir("rgb", name)
    if (out / "result.json").exists():
        return {"cached": True, "path": str(out / "result.json")}
    design = yaml.safe_load(Path(cfg["paths"]["rig_design"]).read_text())
    calib_in = result_calib(read_json(held)) if held else None
    st = None if (held or start["kind"] == "nominal") else _rgb_start(start, cams, design)
    res = run_ba(ws, segs, cams, out, start=st, calib_in=calib_in, design=design, log=log,
                 threads=cfg["resources"]["solver_threads"], device=cfg["resources"].get("ba_device", "cpu"), **opts)
    return {"path": str(out / "result.json"), "reproj_median_px": res["reproj_median_px"], "n_obs": res["n_obs"]}


def _rig_from_rgb(path):
    """(T_rig_lidar, {cam: T_cam_lidar}, R_L_V) from an RGB result + vehicle axes."""
    if not path or not Path(path).exists():
        return None
    r = read_json(path)
    cams = {c: np.array(v["T_cam_lidar"]) for c, v in r["cameras"].items()}
    if "camera_front5" not in cams:
        return None
    ax = Path(path).parent.parent.parent / "lo" / "vehicle_axes.json"
    R_L_V = np.array(read_json(ax)["R_L_V"]) if ax.exists() else np.eye(3)
    return cams["camera_front5"], cams, R_L_V


def t_thermal_solve(ws, cfg, log, name, segs, start, opts, rgb_result=None, held=False):
    from .thermal.solve import run_tba
    from .thermal.timemodel import Plan
    out = ws.solve_dir("thermal", name)
    if (out / "result.json").exists():
        return {"cached": True, "path": str(out / "result.json")}
    st = read_json(start["path"])
    st = st["cameras"] if "cameras" in st else st
    if opts.get("stereo"):
        opts = dict(opts, stereo=str(ws.root / "thermal" / "links" / f"{opts['stereo']}.npz"))
    res = run_tba(ws, Plan(ws), segs, out, start=st, rig=_rig_from_rgb(rgb_result), calib_in=held, log=log,
                  threads=cfg["resources"]["solver_threads"], **opts)
    return {"path": str(out / "result.json"), "reproj_median_px": res["reproj_median_px"]}


def t_thermal_edges(ws, cfg, log, win, calib):
    from .thermal.edges import process
    from .thermal.timemodel import Plan
    cams = [c for c in cfg["cameras"]["thermal"] if not (ws.thermal_edges_dir() / f"{win}_{c}.npz").exists()]
    if not cams:
        return {"cached": True}
    e = cfg["thermal"]["edges"]
    process(ws, Plan(ws), win, read_json(calib), every=e["every"], hist=e["hist_s"], maxp=e["maxp"], cams=cams,
            time_kind=cfg["thermal"]["time_model"], log=log)
    return {"cams": cams}


def t_thermal_links(ws, cfg, log, name, segs, calib):
    from .thermal.stereo import link_stereo
    from .thermal.timemodel import Plan
    out = ws.root / "thermal" / "links" / f"{name}.npz"
    if out.exists():
        return {"cached": True}
    out.parent.mkdir(parents=True, exist_ok=True)
    s = cfg["thermal"]["stereo"]
    return link_stereo(ws, Plan(ws), segs, read_json(calib), out, every=s["every"], gate=s["gate"],
                       time_kind=cfg["thermal"]["time_model"], log=log)


# ------------------------------------------------------------------ validation
def t_rgb_edges(ws, cfg, log, calib, wins, parked, cams, out, vote=True):
    from .validate.edges_eval import rgb_edge_eval
    if Path(out).exists():
        return read_json(out)
    r = rgb_edge_eval(ws, cfg, read_json(calib), wins, parked, cams, frames=cfg["validation"]["edge_frames"],
                      vote=vote, log=log)
    write_json(out, r)
    return r


def t_thermal_edge_eval(ws, cfg, log, result, segs, out):
    from .validate.edges_eval import thermal_edge_eval
    if Path(out).exists():
        return read_json(out)
    r = thermal_edge_eval(ws, cfg, read_json(result), segs, log=log)
    write_json(out, r)
    return r


def t_images(ws, cfg, log, rgb_result, thermal_result, parked, moving, outdir, masks_dir):
    from .outputs.images import make_images
    return make_images(ws, cfg, rgb_result, thermal_result, parked, moving, Path(outdir), Path(masks_dir), log=log)


def t_crosstime_links(ws, cfg, log, solve):
    from .rgb.crosstime import associate_resumable
    out = ws.root / "rgb" / "crosstime" / "merges.npz"
    if out.exists():
        return {"cached": True}
    c = cfg["rgb"]["crosstime"]
    d = ws.root / "rgb" / solve
    merges, stats = associate_resumable(ws, read_json(d / "result.json"), d / "state.npz", ws.root / "rgb" / "crosstime" / "cache",
                                        gate_a=c["gate_a_m"], gate_b=c["gate_b"], max_px_med=c["max_px_med"],
                                        max_px_p90=c["max_px_p90"], log=log)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out, **{k: np.array(v) for k, v in merges.items()})
    write_json(out.with_suffix(".json"), stats)
    return {k: v for k, v in stats.items() if k != "per_cam_pair"}
