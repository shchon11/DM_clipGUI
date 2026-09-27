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
    """KISS-ICP -> continuous-time refinement -> map cache -> quality check, for one window. The raw
    sweeps are read from disk once and shared by the four steps (lotraj.sweep_cache)."""
    from .lo.lotraj import sweep_cache
    with sweep_cache():
        return _lo_chain(ws, cfg, log, win)


def _lo_chain(ws, cfg, log, win):
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
    _trim()          # return the refinement's freed heap to the OS before the map (glibc keeps it otherwise)
    if not ws.lo("map", win).exists():
        build_map(seg, ws.lo("ref", win), ws.lo("map", win), log=log, **c["map"])
    _trim()
    if ws.lo_check(win).exists():
        return read_json(ws.lo_check(win))
    if (ws.thermal16(win) / "source.json").exists() and not ws.lo("tedge", win).exists():
        # the calibration-free part of the thermal edge term, while the sweeps are in memory
        from .thermal.edges import save_sweep_edges
        save_sweep_edges(ws, win)
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


def _trim():
    try:
        import ctypes
        ctypes.CDLL("libc.so.6").malloc_trim(0)
    except Exception:  # noqa
        pass


def t_vehicle_axes(ws, cfg, log, wins):
    from .lo.axes import vehicle_axes
    out = ws.root / "lo" / "vehicle_axes.json"
    if out.exists():
        return read_json(out)
    ax = vehicle_axes([(ws.lo("ref", w), ws.seg_dir(w)) for w in wins], log=log)
    write_json(out, ax)
    return ax


# ------------------------------------------------------------------ tracks
def t_rgb_tracks(ws, cfg, log, win, cam, bag_id, mask, cams=None, masks=None):
    """KLT tracks of one camera of a window, or of several (`cams`, `masks`) tracked in parallel threads
    of this worker (OpenCV releases the GIL; every camera's tracks are the same as tracked alone)."""
    import os
    import cv2
    # OpenCV otherwise uses every core in each of the parallel trackers (oversubscription); the
    # tracks are identical for any thread count
    cv2.setNumThreads(int(os.environ.get("OPENCV_NUM_THREADS", "2")))
    if cams:
        from concurrent.futures import ThreadPoolExecutor
        with ThreadPoolExecutor(len(cams)) as ex:
            rs = list(ex.map(lambda cm: _rgb_tracks_one(ws, cfg, log, win, cm[0], bag_id, cm[1]), zip(cams, masks)))
        return dict(zip(cams, rs))
    return _rgb_tracks_one(ws, cfg, log, win, cam, bag_id, mask)


def _rgb_tracks_one(ws, cfg, log, win, cam, bag_id, mask):
    from .rgb.tracks import run_tracks
    out = ws.tracks(win, cam)
    if out.exists():
        return {"cached": True}
    c = cfg["rgb"]["tracks"]
    res = run_tracks(ws.seg_dir(win), cam, Path(mask), ws.ins(bag_id), out, grid=c["grid"], per_cell=c["per_cell"],
                     fb_max=c["fb_max"], min_len=c["min_len"], log=log, anchor=c.get("anchor"),
                     device=c.get("device", "cpu"), frame_step=c.get("frame_step", 1))
    keep = int(cfg["extract"].get("keep_rgb_every") or 0)
    if keep > 1 and not cfg["rgb"].get("crosstime", {}).get("enabled"):
        res["pruned_gb"] = prune_frames(ws.seg_dir(win) / "cam" / cam, keep)
    return res


def prune_frames(d: Path, keep: int) -> float:
    """After tracking, keep every `keep`-th RGB frame of a window (the later stages - LiDAR-edge validation,
    projection images - read a few frames; the tracker and the solves need no image). A marker stops the
    tracker from ever running on the thinned frames."""
    files = sorted(d.glob("*.jpg"))
    (d / "PRUNED").write_text(json.dumps({"keep_every": keep, "frames_before": len(files)}))
    gone = 0
    for i, f in enumerate(files):
        if i % keep:
            gone += f.stat().st_size
            f.unlink()
    return round(gone / 1e9, 3)


def t_thermal_tracks(ws, cfg, log, win, cam):
    from .thermal.timemodel import Plan
    from .thermal.tracks import run_thermal_tracks
    if ws.thermal_tracks(win, cam).exists():
        return {"cached": True}
    c = cfg["thermal"]["tracks"]
    return run_thermal_tracks(ws, Plan(ws), win, cam, prep=c["prep"], log=log, grid=tuple(c["grid"]),
                              per_cell=c["per_cell"], fb_max=c["fb_max"], min_len=c["min_len"],
                              device=c.get("device", "cpu"))


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
    from .rgb.solve import result_calib, result_time, run_ba
    out = ws.solve_dir("rgb", name)
    if (out / "result.json").exists():
        return {"cached": True, "path": str(out / "result.json")}
    design = yaml.safe_load(Path(cfg["paths"]["rig_design"]).read_text())
    calib_in = result_calib(read_json(held)) if held else None
    st = None if (held or start["kind"] == "nominal") else _rgb_start(start, cams, design)
    # camera time offsets travel with the calibration they were solved with (rgb.solve.time)
    src = held or (start.get("path") if start["kind"] == "result" else None)
    time_init = result_time(read_json(src)) if src else None
    res = run_ba(ws, segs, cams, out, start=st, calib_in=calib_in, design=design, log=log, time_init=time_init,
                 threads=cfg["resources"]["solver_threads"], device=cfg["resources"].get("ba_device", "cpu"),
                 kernel=cfg["resources"].get("ba_kernel"), **opts)
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
                  threads=cfg["resources"]["solver_threads"], kernel=cfg["resources"].get("ba_kernel"), **opts)
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


def t_thermal_links(ws, cfg, log, name, segs, calib, part=False):
    """part=True: the links of one window into links/parts/<name>.npz (merged by stereo.merge_links)."""
    from .thermal.stereo import link_stereo
    from .thermal.timemodel import Plan
    out = ws.root / "thermal" / "links" / ("parts" if part else "") / f"{name}.npz"
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
    matcher = c.get("matcher", "sift")
    kw = dict(gate_a=c["gate_a_m"], gate_b=c["gate_b"], max_px_med=c["max_px_med"], max_px_p90=c["max_px_p90"])
    cache = ws.root / "rgb" / "crosstime" / "cache"
    if matcher != "sift":
        # learned verification: its own candidate gates (wider: the appearance check is exact) and cache
        lc = dict(c.get("learned", {}))
        kw = dict(gate_a=lc.pop("gate_a_m", kw["gate_a"]), gate_b=lc.pop("gate_b", kw["gate_b"]),
                  max_px_med=lc.pop("max_px_med", kw["max_px_med"]), max_px_p90=lc.pop("max_px_p90", kw["max_px_p90"]),
                  matcher=matcher, learned=lc)
        cache = cache.with_name(f"cache_{matcher}")
    merges, stats = associate_resumable(ws, read_json(d / "result.json"), d / "state.npz", cache, log=log, **kw)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out, **{k: np.array(v) for k, v in merges.items()})
    write_json(out.with_suffix(".json"), stats)
    return {k: v for k, v in stats.items() if k != "per_cam_pair"}
