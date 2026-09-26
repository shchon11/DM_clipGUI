"""The pipeline: preflight -> names -> windows + extraction -> LiDAR odometry -> RGB solve -> thermal
solve -> validation -> outputs. Every stage is cached in the work directory (workspace.py); heavy work
runs in worker processes (worker.py / tasks.py), at most resources.max_procs at a time."""
from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml

from . import __version__
from . import errors as E
from .bag import Bag
from .events import Events, dir_size_gb
from .workspace import Workspace, read_json, write_json


def _key(*parts) -> str:
    return hashlib.sha1(json.dumps(parts, sort_keys=True, default=str).encode()).hexdigest()[:16]


class Pipeline:
    def __init__(self, args, cfg):
        self.a = args
        self.cfg = cfg
        self.ws = Workspace(Path(args.workdir))
        self.ev = Events(self.ws.root)
        self.sensors = [s.strip() for s in args.sensors.split(",") if s.strip()]
        self.bags = {f"b{i}": str(Path(b).resolve()) for i, b in enumerate(args.bags)}
        self.out = Path(args.out) if getattr(args, "out", None) else None
        snap = self.ws.root / "config_snapshot.json"
        snap.write_text(json.dumps(cfg, indent=1, default=str))
        self.snap = snap
        self.stage_times = []
        self.viz = None
        self.viz_run_id = None
        import threading
        # one budget of worker processes for the whole run: the RGB and thermal chains (and the RGB
        # tracker during the LiDAR odometry) run in parallel threads and share it
        self._sem = threading.BoundedSemaphore(self.cfg["resources"]["max_procs"])

    # ================================================================== task runner
    def run_tasks(self, stage: str, tasks, nproc: int | None = None, threads: int | None = None,
                  allow_fail: bool = False):
        """tasks: [(key, task_name, args)] -> {key: result dict}. Runs `python -m nontarget_cal.worker`."""
        nproc = min(nproc or self.cfg["resources"]["max_procs"], self.cfg["resources"]["max_procs"])
        threads = threads or self.cfg["resources"]["threads_per_proc"]
        tdir = self.ws.root / "tasks" / stage
        ldir = self.ws.root / "logs" / stage
        tdir.mkdir(parents=True, exist_ok=True); ldir.mkdir(parents=True, exist_ok=True)
        env = dict(os.environ, OMP_NUM_THREADS=str(threads), OPENBLAS_NUM_THREADS=str(threads),
                   MKL_NUM_THREADS=str(threads), OPENCV_NUM_THREADS=str(threads), PYTHONUNBUFFERED="1",
                   NONTARGET_LO_THREADS=str(self.cfg["resources"].get("lo_threads", threads)),
                   NONTARGET_TIE_THREADS=str(self.cfg["resources"].get("tie_threads", 3)))
        if self.cfg["resources"].get("gpu") == "off":
            env["CUDA_VISIBLE_DEVICES"] = ""
        results, pending, running = {}, list(tasks), {}
        total = len(tasks)
        done = 0
        for key, name, args in list(pending):
            rf = tdir / f"{key}.result.json"
            if rf.exists() and read_json(rf).get("ok"):
                results[key] = read_json(rf)
                pending.remove((key, name, args))
                done += 1
        if done:
            self.ev.progress(stage, done, total, "cached")
        while pending or running:
            while pending and len(running) < nproc and self._sem.acquire(blocking=False):
                key, name, args = pending.pop(0)
                spec = {"task": name, "workdir": str(self.ws.root), "config": str(self.snap), "args": args,
                        "result": str(tdir / f"{key}.result.json"), "log": str(ldir / f"{key}.log")}
                if self.viz is not None:
                    from .viz_pipeline import task_context
                    spec["viz_context"] = task_context(self, stage, key, args)
                sf = tdir / f"{key}.json"
                sf.write_text(json.dumps(spec, default=str))
                rf = tdir / f"{key}.result.json"
                if rf.exists():
                    rf.unlink()
                p = subprocess.Popen([sys.executable, "-m", "nontarget_cal.worker", str(sf)], env=env,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                running[key] = (p, name, time.time())
            time.sleep(1.0)
            for key in list(running):
                p, name, t0 = running[key]
                if p.poll() is None:
                    continue
                del running[key]
                self._sem.release()
                rf = tdir / f"{key}.result.json"
                r = read_json(rf) if rf.exists() else {"ok": False, "error": f"worker exited with {p.returncode}"}
                results[key] = r
                done += 1
                if not r.get("ok"):
                    tail = ""
                    lf = ldir / f"{key}.log"
                    if lf.exists():
                        tail = "\n".join(lf.read_text(errors="replace").splitlines()[-15:])
                    self.ev.log(f"task {stage}/{key} FAILED: {r.get('error')}\n{tail}")
                    self.ev.emit("task_failed", stage=stage, task=key, error=r.get("error"), log=str(lf))
                self.ev.progress(stage, done, total, f"{key} {'ok' if r.get('ok') else 'FAILED'} ({time.time() - t0:.0f} s)")
        failed = [k for k, r in results.items() if not r.get("ok")]
        if failed and not allow_fail:
            raise RuntimeError(f"{stage}: {len(failed)} task(s) failed: {failed[:5]} (logs in {ldir})")
        return results

    # ================================================================== helpers
    def plan(self):
        return read_json(self.ws.root / "windows" / "plan.json")

    def names(self):
        return read_json(self.ws.root / "names" / "mapping.json")

    def kept(self):
        return read_json(self.ws.root / "windows" / "lo_quality.json")

    def reference_rig(self):
        """Reference rotations for the name identification: --init if given, else the design file."""
        ref = {}
        if getattr(self.a, "init", None):
            try:
                from .init_calib import load_init
                ini = load_init(Path(self.a.init))
                ref = {c: np.linalg.inv(np.array(v["T_cam_lidar"])) for c, v in ini.get("rgb", {}).items()}
            except Exception as ex:  # noqa
                self.ev.warn("init_unreadable_for_names", f"--init not usable as name reference: {ex}")
        if not ref:
            d = yaml.safe_load(Path(self.cfg["paths"]["rig_design"]).read_text())
            ref = {c: np.array(v["reference_T_lidar_cam"]) for c, v in d["rgb"].items()}
        return ref

    # ================================================================== 1+2: preflight and names
    def stage_check(self, force=False):
        from . import namemap, preflight
        key = _key(self.bags, self.sensors, str(self.a.name_map), self.cfg.get("topics"), self.cfg.get("preflight"),
                   self.cfg["cameras"], getattr(self.a, "windows", None))
        d = self.ws.done("preflight", key)
        if d:
            self.ev.emit("stage_skip", stage="preflight")
            rep = read_json(self.ws.root / "preflight" / "report.json")
            return rep
        for b, p in self.bags.items():
            if not Path(p).exists():
                raise E.Refusal(E.NO_BAG, f"bag {p} not found (disk unplugged?)", f"bag {p}를 찾을 수 없음(디스크 연결 확인)")
        with self.ev.stage("names") as info:
            mapping = {}
            for b, p in self.bags.items():
                bag = Bag(p)
                nss = namemap.namespaces(bag, self.cfg)
                s = preflight.sample_series(bag, self.cfg, {"rgb": {n: n for n in nss["rgb"]}}, log=self.ev.log)
                m = namemap.resolve_names(bag, b, self.cfg, self.sensors,
                                          Path(self.a.name_map) if self.a.name_map else None, s,
                                          self.reference_rig(), self.ev, verify=not self.a.no_verify_names)
                mapping[b] = m
                bag.close()
                self.ev.log(f"{b}: names from {m['source']}")
            write_json(self.ws.root / "names" / "mapping.json", mapping)
            info["source"] = {b: m["source"] for b, m in mapping.items()}
        with self.ev.stage("preflight") as info:
            secs = None
            if getattr(self.a, "windows", None):
                from .windows import parse_explicit
                secs = sum(w["t1_s"] - w["t0_s"] for w in parse_explicit(self.a.windows, list(self.bags)))
            rep = preflight.run_preflight(self.bags, self.cfg, mapping, self.sensors, self.ws.root, self.ev, force=force,
                                          data_seconds=secs)
            rep["names"] = {b: {"rgb": m["rgb"], "thermal": m["thermal"], "source": m["source"]} for b, m in mapping.items()}
            write_json(self.ws.root / "preflight" / "report.json", rep)
            info.update({"ok": rep["ok"], "refusals": len(rep["refusals"]), "warnings": len(rep["warnings"]),
                         "estimate": rep["estimate"]})
        if not rep["ok"]:
            hard = [r for r in rep["refusals"] if not (force and r["forceable"])]
            r = hard[0]
            raise E.Refusal(r["code"], r["msg"] + (f" (+{len(hard) - 1} more, see preflight/report.json)" if len(hard) > 1 else ""),
                            r["msg_ko"], all=hard)
        for r in rep["refusals"]:
            self.ev.warn("forced_" + r["code"], "FORCED past: " + r["msg"], "강제 진행: " + r["msg_ko"])
        self.ws.mark("preflight", key)
        return rep

    # ================================================================== 3: windows + extraction
    def stage_extract(self):
        from .extract import bag_zero_ns
        from .windows import plan_windows
        names = self.names()
        key = _key(self.bags, self.sensors, self.cfg["windows"], getattr(self.a, "windows", None), names)
        if not self.ws.done("windows", key):
            with self.ev.stage("windows") as info:
                series, zeros = {}, {}
                for b, p in self.bags.items():
                    z = np.load(self.ws.root / "preflight" / f"series_{b}.npz")
                    series[b] = {k: z[k] for k in z.files}
                    bag = Bag(p)
                    zeros[b] = bag_zero_ns(bag, self.cfg["topics"])
                    bag.close()
                plan = plan_windows(series, zeros, self.cfg, self.sensors, explicit=getattr(self.a, "windows", None))
                thr = 0.8 * self.cfg["windows"]["thermal_min_rotation_deg"]
                for w in plan["windows"]:
                    w["thermal_extract"] = ("thermal" in self.sensors) and (w["rotation_deg"] >= thr or bool(getattr(self.a, "windows", None)))
                if not plan["windows"]:
                    raise E.Refusal(E.NO_GOOD_WINDOWS, "no moving windows found", "움직인 구간이 없음")
                write_json(self.ws.root / "windows" / "plan.json", plan)
                info.update({"windows": len(plan["windows"]), "seconds": round(sum(w["t1_s"] - w["t0_s"] for w in plan["windows"]), 1),
                             "thermal_candidates": sum(w["thermal_extract"] for w in plan["windows"])})
                self.ev.log("windows: " + ", ".join(f"{w['name']} {w['t0_s']:.0f}-{w['t1_s']:.0f}s {w['rotation_deg']:.0f}deg"
                                                    for w in plan["windows"]))
            self.ws.mark("windows", key)
            self.ws.invalidate("extract", "lo", "rgb", "thermal", "validation", "outputs")
        plan = self.plan()
        key = _key(plan, names)
        if self.ws.done("extract", key):
            self.ev.emit("stage_skip", stage="extract")
            return
        with self.ev.stage("extract", disk_path=self.ws.root / "extract") as info:
            zeros = {b["bag_id"]: b["zero_ns"] for b in plan["bags"]}
            scans = [(f"scan_{b}", "scan_bag", {"bag_path": p, "bag_id": b, "names": {k: names[b][k] for k in ("rgb", "thermal")}})
                     for b, p in self.bags.items()]
            self.run_tasks("extract_scan", scans, nproc=2, threads=2)
            tasks = []
            for w in plan["windows"] + plan["parked"]:
                b = w["bag_id"]
                tasks.append((w["name"], "extract_window", {"bag_path": self.bags[b], "bag_id": b, "win": w,
                              "names": {k: names[b][k] for k in ("rgb", "thermal")}, "zero_ns": zeros[b],
                              "rgb": "rgb" in self.sensors, "thermal": bool(w.get("thermal_extract"))}))
            self.run_tasks("extract", tasks, nproc=2, threads=2)
            info["thermal_disk_gb"] = round(dir_size_gb(self.ws.root / "thermal16"), 2)
        self.ws.mark("extract", key)

    # ================================================================== 4: LiDAR odometry
    def stage_lo(self):
        plan = self.plan()
        wins = [w["name"] for w in plan["windows"]]
        key = _key(wins, self.cfg["lo"])
        if self.ws.done("lo", key):
            self.ev.emit("stage_skip", stage="lo")
            return self.kept()
        with self.ev.stage("lo", disk_path=self.ws.root / "lo") as info:
            res = self.run_tasks("lo", [(w, "lo_chain", {"win": w}) for w in wins],
                                 threads=self.cfg["resources"]["kiss_threads"], allow_fail=True)
            c = self.cfg["lo"]["check"]
            q = {"windows": {}, "kept": [], "dropped": []}
            for w in wins:
                r = res.get(w, {})
                if not r.get("ok"):
                    q["windows"][w] = {"ok": False, "reason": r.get("error")}
                    q["dropped"].append(w)
                    continue
                o = r["out"]
                g = o.get("gap10", o.get(f"gap{c['gaps'][-1]}", {})).get("robust_std_mm", np.nan)
                th = o.get("thickness_11sweeps_median_cm", np.nan)
                reasons = []
                if not g <= c["max_gap10_mm"]:
                    reasons.append(f"sweep-to-sweep (1 s) {g:.1f} mm > {c['max_gap10_mm']}")
                if not th <= c["max_thickness_cm"]:
                    reasons.append(f"map thickness {th:.2f} cm > {c['max_thickness_cm']}")
                if o["path_m"] < c["min_path_m"]:
                    reasons.append(f"path {o['path_m']:.0f} m < {c['min_path_m']}")
                q["windows"][w] = {"ok": not reasons, "gap1s_mm": g, "thickness_cm": th, "rotation_deg": o["rotation_deg"],
                                   "path_m": o["path_m"], "reasons": reasons}
                (q["dropped"] if reasons else q["kept"]).append(w)
            for w in q["dropped"]:
                self.ev.warn("window_dropped", f"window {w} dropped by the LiDAR-odometry check: {q['windows'][w].get('reasons') or q['windows'][w].get('reason')}",
                             f"구간 {w}: LiDAR 오도메트리 품질 불량으로 제외")
            if not q["kept"]:
                raise E.Refusal(E.NO_GOOD_WINDOWS, "no window passed the LiDAR-odometry quality check",
                                "LiDAR 오도메트리 품질 검사를 통과한 구간이 없음")
            thr = self.cfg["windows"]["thermal_min_rotation_deg"]
            q["thermal"] = [w for w in q["kept"] if q["windows"][w]["rotation_deg"] >= thr and
                            (self.ws.thermal16(w) / "source.json").exists()]
            write_json(self.ws.root / "windows" / "lo_quality.json", q)
            self.run_tasks("axes", [("axes", "vehicle_axes", {"wins": q["kept"]})], nproc=1)
            info.update({"kept": len(q["kept"]), "dropped": len(q["dropped"]), "thermal_windows": len(q["thermal"])})
        self.ws.mark("lo", key)
        self.ws.invalidate("rgb", "thermal", "validation", "outputs")
        return self.kept()

    # ================================================================== 5: RGB
    def _rgb_opts(self, nwin, first=False):
        s = self.cfg["rgb"]["solve"]
        big = nwin > s["big_windows"]
        return {"free": s["free"], "ties": s["ties"], "tie_sigma": s["tie_sigma"], "pos_prior": s["pos_prior"],
                "f_prior": s["f_prior"], "pp_prior": s["pp_prior"], "k_prior": s["k_prior"], "huber": s["huber"],
                "iters": s["iters_first_pass"] if first else s["iters"], "min_len": s["min_len"],
                "min_parallax": s["min_parallax"], "max_dist": s["max_dist"],
                "max_tracks": s["max_tracks_big"] if big else s["max_tracks"], "step": s["step_big"] if big else s["step"],
                "init": "nominal" if first else "from"}

    def _win_seconds(self, wins):
        pl = {w["name"]: w for w in self.plan()["windows"]}
        return sum(pl[w]["t1_s"] - pl[w]["t0_s"] for w in wins)

    def stage_rgb(self):
        q = self.kept()
        plan = self.plan()
        wins = q["kept"]
        cams = list(self.cfg["cameras"]["rgb"])
        bag_of = {w["name"]: w["bag_id"] for w in plan["windows"]}
        key = _key(wins, self.cfg["rgb"], self.a.mode, str(self.a.init), self.a.force)
        if self.ws.done("rgb", key):
            self.ev.emit("stage_skip", stage="rgb")
            from .viz_pipeline import cached_result
            cached_result(self, "rgb")
            return read_json(self.ws.root / "rgb" / "summary.json")
        summ = {}
        self.stage_rgb_tracks(wins)
        with self.ev.stage("rgb_solve", disk_path=self.ws.root / "rgb") as info:
            # ---- start
            if self.a.mode == "warm":
                from .init_calib import load_init
                ini = load_init(Path(self.a.init))
                miss = [c for c in cams if c not in ini.get("rgb", {})]
                if miss:
                    raise E.Refusal(E.INIT, f"--init has no calibration for {miss}", f"--init에 {miss} 캘리브레이션이 없음")
                write_json(self.ws.root / "rgb" / "init_calib.json", ini["rgb"])
                start = {"kind": "calib", "path": str(self.ws.root / "rgb" / "init_calib.json")}
            else:
                # board-free two-pass start (lidar_odo V4n -> V4n2) on the most rotating windows
                n = self.cfg["rgb"]["solve"]["zeroshot_subset_windows"]
                sub = sorted(sorted(wins, key=lambda w: -q["windows"][w]["rotation_deg"])[:n], key=lambda w: wins.index(w))
                self.run_tasks("rgb_zeroshot1", [("pass1", "rgb_solve", {"name": "zeroshot_pass1", "segs": sub, "cams": cams,
                               "start": {"kind": "nominal"}, "opts": self._rgb_opts(len(sub), first=True)})], nproc=1)
                p1 = str(self.ws.root / "rgb" / "zeroshot_pass1" / "result.json")
                self.run_tasks("rgb_zeroshot2", [("pass2", "rgb_solve", {"name": "zeroshot_pass2", "segs": sub, "cams": cams,
                               "start": {"kind": "result", "path": p1}, "opts": self._rgb_opts(len(sub))})], nproc=1)
                start = {"kind": "result", "path": str(self.ws.root / "rgb" / "zeroshot_pass2" / "result.json")}
                summ["zeroshot_subset"] = sub
            # ---- per-bag solves and agreement
            bags = sorted({bag_of[w] for w in wins})
            use_bags = bags
            if len(bags) > 1:
                tasks = [(f"perbag_{b}", "rgb_solve", {"name": f"perbag_{b}", "segs": [w for w in wins if bag_of[w] == b],
                          "cams": cams, "start": start, "opts": self._rgb_opts(sum(bag_of[w] == b for w in wins))})
                         for b in bags]
                self.run_tasks("rgb_perbag", tasks, nproc=2)
                from .validate.compare import bag_agreement
                per = {b: read_json(self.ws.root / "rgb" / f"perbag_{b}" / "result.json") for b in bags}
                secs = {b: self._win_seconds([w for w in wins if bag_of[w] == b]) for b in bags}
                ag = bag_agreement(per, secs, self.cfg)
                write_json(self.ws.root / "rgb" / "bag_agreement.json", ag)
                summ["bag_agreement"] = ag
                if ag["disagreeing"]:
                    msg = (f"bag(s) {ag['disagreeing']} disagree with the others beyond the empirical repeatability "
                           f"(rig changed between recordings?): see rgb/bag_agreement.json")
                    ko = f"bag {ag['disagreeing']}의 결과가 다른 bag과 반복성 범위를 넘어 다름(리그가 바뀌었나?): rgb/bag_agreement.json 참고"
                    if not self.a.force:
                        if len(bags) - len(ag["disagreeing"]) < 1 or len(bags) == 2:
                            raise E.Refusal(E.BAG_DISAGREES, msg + ". With two bags it cannot be decided which one; "
                                            "run them separately or pass --force", ko + ". bag이 2개면 어느 쪽인지 판단 불가: 따로 돌리거나 --force", **ag)
                        use_bags = [b for b in bags if b not in ag["disagreeing"]]
                        self.ev.warn(E.BAG_DISAGREES, msg + "; excluded (use --force to keep)", ko + "; 제외함(--force로 포함 가능)")
                    else:
                        self.ev.warn(E.BAG_DISAGREES, msg + "; kept because of --force", ko + "; --force로 포함함")
            final_wins = [w for w in wins if bag_of[w] in use_bags]
            ct = self.cfg["rgb"].get("crosstime", {})
            if ct.get("enabled"):
                # links from a solve on the same windows (landmark keys), then the final solve uses them
                self.run_tasks("rgb_keys", [("keys", "rgb_solve", {"name": "keys", "segs": final_wins, "cams": cams,
                               "start": start, "opts": self._rgb_opts(len(final_wins))})], nproc=1)
                self.run_tasks("rgb_crosstime", [("links", "crosstime_links", {"solve": "keys"})], nproc=1)
                start = {"kind": "result", "path": str(self.ws.root / "rgb" / "keys" / "result.json")}
            if (self.cfg["rgb"]["solve"].get("reuse_pass2_as_final") and self.a.mode != "warm"
                    and summ.get("zeroshot_subset") == final_wins and not (self.ws.root / "rgb" / "final").exists()):
                # the second board-free pass already used exactly these windows with the same options
                import shutil
                shutil.copytree(self.ws.root / "rgb" / "zeroshot_pass2", self.ws.root / "rgb" / "final")
                self.ev.log("rgb: final = zero-shot pass 2 (same windows)")
            fo = self._rgb_opts(len(final_wins))
            if ct.get("enabled"):
                fo["cross"] = str(self.ws.root / "rgb" / "crosstime" / "merges.npz")
            self.run_tasks("rgb_final", [("final", "rgb_solve", {"name": "final", "segs": final_wins, "cams": cams,
                           "start": start, "opts": fo})], nproc=1)
            fin = read_json(self.ws.root / "rgb" / "final" / "result.json")
            summ.update({"final": str(self.ws.root / "rgb" / "final" / "result.json"), "windows": final_wins,
                         "bags": use_bags, "start": start, "reproj_median_px": fin["reproj_median_px"], "n_obs": fin["n_obs"]})
            write_json(self.ws.root / "rgb" / "summary.json", summ)
            info.update({"windows": len(final_wins), "n_obs": fin["n_obs"], "reproj_median_px": round(fin["reproj_median_px"], 3)})
        self.ws.mark("rgb", key)
        self.ws.invalidate("validation", "outputs")
        return summ

    def stage_rgb_tracks(self, wins=None):
        """KLT tracks need only the images and the INS, not the LiDAR odometry: run() starts this in a
        thread next to the LO stage (all planned windows); stage_rgb then finds them cached."""
        plan = self.plan()
        wins = wins if wins is not None else [w["name"] for w in plan["windows"]]
        cams = list(self.cfg["cameras"]["rgb"])
        bag_of = {w["name"]: w["bag_id"] for w in plan["windows"]}
        with self.ev.stage("rgb_tracks", disk_path=self.ws.root / "tracks"):
            masks = Path(self.cfg["paths"]["masks_dir"])
            self._ensure_masks(cams, wins, bag_of)
            tasks = [(f"{w}__{c}", "rgb_tracks", {"win": w, "cam": c, "bag_id": bag_of[w],
                                                  "mask": str(self.ws.root / "masks" / f"{c}.png") if not (masks / f"{c}.png").exists() else str(masks / f"{c}.png")})
                     for w in wins for c in cams]
            self.run_tasks("rgb_tracks", tasks, threads=2)

    def _ensure_masks(self, cams, wins, bag_of):
        """Ego-body masks: packaged ones for this vehicle; for a camera without one, build it from the
        extracted frames (online_calib/masks.py method) and warn (night masks are less reliable)."""
        masks = Path(self.cfg["paths"]["masks_dir"])
        miss = [c for c in cams if not (masks / f"{c}.png").exists()]
        if not miss:
            return
        from .rgb.masks import build_mask
        (self.ws.root / "masks").mkdir(exist_ok=True)
        for c in miss:
            out = self.ws.root / "masks" / f"{c}.png"
            if not out.exists():
                build_mask(self.ws, c, wins, bag_of, out)
            self.ev.warn("mask_built", f"no packaged ego mask for {c}: built one from the data ({out}); check it",
                         f"{c}의 차체 마스크가 없어 데이터로 만듦({out}); 확인 필요")

    # ================================================================== 6: thermal
    def _thermal_opts(self, part):
        t = self.cfg["thermal"]
        d = dict(t["solve_defaults"])
        p = t[part]
        o = {k: d[k] for k in ("tie_sigma", "min_len", "step", "max_tracks", "max_obs", "min_parallax", "max_dist",
                               "huber", "iters", "pos_prior", "f_prior", "pp_prior", "k_prior")}
        o.update({"free": p["free"], "ties": p.get("ties", False), "time": t["time_model"], "dt_mode": "seg",
                  "rounds": p.get("rounds", d["rounds"])})
        if part == "lens":
            o["min_dist"] = p["min_dist"]
        if p.get("edges"):
            o.update({"edges": True, "edge_weight": d["edge_weight"], "edge_sigma": d["edge_sigma"],
                      "edge_maxdepth": d["edge_maxdepth"], "edge_mindepth": d["edge_mindepth"]})
        return o

    def _win_start_ns(self, w):
        p = self.plan()
        z = {b["bag_id"]: b["zero_ns"] for b in p["bags"]}
        x = {v["name"]: v for v in p["windows"]}[w]
        return z[x["bag_id"]] + int(float(x["t0_s"]) * 1e9)

    def thermal_windows(self):
        q = self.kept()
        wins = list(q["thermal"])
        need = self.cfg["preflight"]["thermal"]["min_turning_windows"]
        if len(wins) < need:
            extra = sorted([w for w in q["kept"] if w not in wins and (self.ws.thermal16(w) / "source.json").exists()],
                           key=lambda w: -q["windows"][w]["rotation_deg"])
            thr = 0.8 * self.cfg["windows"]["thermal_min_rotation_deg"]
            extra = [w for w in extra if q["windows"][w]["rotation_deg"] >= thr][:need - len(wins)]
            if extra:
                self.ev.warn("thermal_windows_relaxed", f"thermal: only {len(wins)} windows >= "
                             f"{self.cfg['windows']['thermal_min_rotation_deg']} deg; added {extra}",
                             f"열화상: 기준 회전량 구간이 {len(wins)}개뿐이라 {extra} 추가")
            wins = sorted(wins + extra, key=lambda w: q["kept"].index(w))
        # chronological order (absolute time, across bags): thermal/tba.Trajs concatenates the windows'
        # knot times and looks poses up with ONE searchsorted, which needs them increasing. thermal_lo
        # listed its windows in time order; out-of-order windows get wrong poses (their time offsets
        # ran off by seconds on the first full run)
        wins = sorted(wins, key=lambda w: self._win_start_ns(w))
        if len(wins) < need:
            raise E.Refusal(E.NOT_ENOUGH_THERMAL, f"thermal: {len(wins)} usable turning windows after the LiDAR "
                            f"odometry (need >= {need})", f"열화상: LiDAR 오도메트리 후 쓸 수 있는 회전 구간 {len(wins)}개(최소 {need}개)")
        return wins

    def stage_thermal(self):
        cams = list(self.cfg["cameras"]["thermal"])
        wins = self.thermal_windows()
        rgb_res = str(self.ws.root / "rgb" / "final" / "result.json") if "rgb" in self.sensors else None
        key = _key(wins, self.cfg["thermal"], self.a.mode, str(self.a.init))
        if self.ws.done("thermal", key):
            self.ev.emit("stage_skip", stage="thermal")
            from .viz_pipeline import cached_result
            cached_result(self, "thermal")
            return read_json(self.ws.root / "thermal" / "summary.json")
        summ = {"windows": wins}
        with self.ev.stage("thermal_tracks", disk_path=self.ws.root / "thermal" / "tracks"):
            self.run_tasks("thermal_tracks", [(f"{w}__{c}", "thermal_tracks", {"win": w, "cam": c}) for w in wins for c in cams],
                           threads=1)
        with self.ev.stage("thermal_solve", disk_path=self.ws.root / "thermal") as info:
            start, from_design = self._thermal_start(cams, rgb_res)
            sp = str(self.ws.root / "thermal" / "start.json")
            write_json(sp, start)
            self.run_tasks("thermal_lens", [("lens", "thermal_solve", {"name": "lens", "segs": wins, "start": {"path": sp},
                           "opts": self._thermal_opts("lens"), "rgb_result": rgb_res})], nproc=1)
            lens = str(self.ws.root / "thermal" / "lens" / "result.json")
            self._check_thermal_dt(lens)
            self.run_tasks("thermal_edges", [(w, "thermal_edges", {"win": w, "calib": lens}) for w in wins], threads=1)
            self.run_tasks("thermal_pos", [("pos_e", "thermal_solve", {"name": "pos_e", "segs": wins, "start": {"path": lens},
                           "opts": dict(self._thermal_opts("pos"), edges=True, edge_weight=1.0, edge_sigma=1.0,
                                        edge_maxdepth=40.0, edge_mindepth=1.5), "rgb_result": rgb_res})], nproc=1)
            pos = str(self.ws.root / "thermal" / "pos_e" / "result.json")
            self.run_tasks("thermal_links", [("links", "thermal_links", {"name": "links", "segs": wins, "calib": pos})], nproc=1)
            fo = self._thermal_opts("final")
            if self.cfg["thermal"]["final"].get("stereo"):
                fo["stereo"] = "links"
            # the validation halves start from the same lens solve: run them next to the final
            halves = self._thermal_half_tasks(lens, rgb_res) if self.cfg["validation"]["halves"] else []
            self.run_tasks("thermal_final", [("final", "thermal_solve", {"name": "final", "segs": wins, "start": {"path": lens},
                           "opts": fo, "rgb_result": rgb_res})] + halves, nproc=3,
                           threads=self.cfg["resources"]["solver_threads"])
            fin = read_json(self.ws.root / "thermal" / "final" / "result.json")
            bag_of = {w["name"]: w["bag_id"] for w in self.plan()["windows"]}
            per_bag = {}
            for c in cams:
                d = np.array(fin["cameras"][c]["dt_s"])
                per_bag[c] = {b: round(float(np.mean([d[i] for i, w in enumerate(fin["segs"]) if bag_of[w] == b])), 5)
                              for b in sorted({bag_of[w] for w in fin["segs"]})}
            summ.update({"final": str(self.ws.root / "thermal" / "final" / "result.json"), "lens": lens, "pos_e": pos,
                         "dt_per_bag_s": per_bag, "baseline_mm": fin.get("stereo", {}).get("baseline_mm"),
                         "reproj_median_px": fin["reproj_median_px"]})
            write_json(self.ws.root / "thermal" / "summary.json", summ)
            info.update({"windows": len(wins), "baseline_mm": summ["baseline_mm"], "reproj_median_px": round(fin["reproj_median_px"], 3)})
        self.ws.mark("thermal", key)
        self.ws.invalidate("validation", "outputs")
        return summ

    def _check_thermal_dt(self, path, tol_s=0.03):
        """Every window's time offset must agree with the median to tol_s (per-window scatter is ~2 ms):
        a window that ran away would poison the next stages, which start from the mean."""
        r = read_json(path)
        bad = []
        for c, v in r["cameras"].items():
            d = np.array(v["dt_s"])
            off = np.abs(d - np.median(d))
            bad += [f"{c} {w}: {1e3 * d[i]:.0f} ms" for i, w in enumerate(r["segs"]) if off[i] > tol_s]
        if bad:
            raise E.Refusal("thermal_time_diverged", f"thermal lens solve: time offsets of some windows diverged: {bad}",
                            f"열화상 렌즈 풀이에서 일부 구간 시간 오프셋이 발산함: {bad}")

    def _thermal_start(self, cams, rgb_res):
        """warm: --init thermal calibration; zero-shot: the rig design (rotation rounded to 10 deg, centre
        rounded to 5 cm, nominal A70 lens and timing)."""
        if self.a.mode == "warm":
            from .init_calib import load_init
            ini = load_init(Path(self.a.init)).get("thermal", {})
            if all(c in ini for c in cams):
                return {c: ini[c] for c in cams}, False
            self.ev.warn("thermal_init_missing", f"--init has no thermal calibration for {[c for c in cams if c not in ini]}: "
                         "using the design start", "--init에 열화상 캘리브레이션이 없어 설계값에서 시작")
        from scipy.spatial.transform import Rotation as Rot
        d = yaml.safe_load(Path(self.cfg["paths"]["rig_design"]).read_text())["thermal"]
        out = {}
        for c in cams:
            Tlc = np.eye(4)
            Tlc[:3, :3] = Rot.from_euler("ZYX", d[c]["design_R_lidar_cam_euler_ZYX_deg"], degrees=True).as_matrix()
            Tlc[:3, 3] = d[c]["design_centre_lidar_m"]
            out[c] = {"T_cam_lidar": np.linalg.inv(Tlc).tolist(), "intr": d[c]["nominal_intr_pinhole"],
                      "rs_s": d[c]["nominal_row_readout_s"], "dt_s": d[c]["nominal_time_offset_s"]}
        return out, True

    # ================================================================== 7: validation
    def stage_validation(self):
        from .validate import compare, layout
        v = {"rgb": {}, "thermal": {}}
        key = _key(self.cfg["validation"], self.cfg["layout_rules"], self.ws.done("rgb"), self.ws.done("thermal"))
        if self.ws.done("validation", key):
            self.ev.emit("stage_skip", stage="validation")
            return read_json(self.ws.root / "validation" / "validation.json")
        ax = read_json(self.ws.root / "lo" / "vehicle_axes.json")
        R_L_V = np.array(ax["R_L_V"])
        with self.ev.stage("validation", disk_path=self.ws.root / "validation") as info:
            if "rgb" in self.sensors:
                self.run_val_rgb()
                rs = read_json(self.ws.root / "rgb" / "summary.json")
                wins = rs["windows"]
                cams = list(self.cfg["cameras"]["rgb"])
                A, B = wins[0::2], wins[1::2]
                fin = rs["final"]
            if "thermal" in self.sensors:
                self.run_val_thermal()
                ts = read_json(self.ws.root / "thermal" / "summary.json")
                twins = ts["windows"]
                TA, TB = twins[0::2], twins[1::2]
            # ---- assemble
            if "rgb" in self.sensors:
                fr = read_json(fin)
                vr = {"final": fin}
                if len(A) and len(B):
                    ra, rb = read_json(self.ws.root / "rgb" / "halfA" / "result.json"), read_json(self.ws.root / "rgb" / "halfB" / "result.json")
                    d = compare.diff(ra, rb, R_L_V)
                    vr["halves"] = {"A": A, "B": B, "diff": d, "summary": compare.summary(d),
                                    "sigma": compare.sigma_from_halves(d)}
                    h1 = read_json(self.ws.root / "rgb" / "heldout_AonB" / "result.json")
                    h2 = read_json(self.ws.root / "rgb" / "heldout_BonA" / "result.json")
                    vr["heldout"] = {c: 0.5 * (h1["cameras"][c]["reproj_median_px"] + h2["cameras"][c]["reproj_median_px"])
                                     for c in cams if h1["cameras"][c]["reproj_median_px"] is not None}
                    vr["heldout_lidar_check"] = {"AonB": h1["lidar_check"], "BonA": h2["lidar_check"]}
                vr["edges"] = read_json(self.ws.root / "validation" / "rgb_edges.json")
                vr["track_reproj"] = {c: fr["cameras"][c]["reproj_median_px"] for c in cams}
                vr["lidar_check"] = fr["lidar_check"]
                v["rgb"] = vr
            if "thermal" in self.sensors:
                tf = read_json(ts["final"])
                vt = {"final": ts["final"]}
                if len(TA) and len(TB):
                    ra, rb = read_json(self.ws.root / "thermal" / "halfA" / "result.json"), read_json(self.ws.root / "thermal" / "halfB" / "result.json")
                    d = compare.diff(ra, rb, R_L_V)
                    vt["halves"] = {"A": TA, "B": TB, "diff": d, "summary": compare.summary(d),
                                    "sigma": compare.sigma_from_halves(d),
                                    "baseline_mm": [ra.get("stereo", {}).get("baseline_mm"), rb.get("stereo", {}).get("baseline_mm")]}
                    h1 = read_json(self.ws.root / "thermal" / "heldout_AonB" / "result.json")
                    h2 = read_json(self.ws.root / "thermal" / "heldout_BonA" / "result.json")
                    vt["heldout"] = {c: 0.5 * (h1["cameras"][c]["reproj_median_px"] + h2["cameras"][c]["reproj_median_px"])
                                     for c in tf["cams"]}
                vt["edges"] = read_json(self.ws.root / "validation" / "thermal_edges.json")
                vt["track_reproj"] = {c: tf["cameras"][c]["reproj_median_px"] for c in tf["cams"]}
                vt["baseline_mm"] = tf.get("stereo", {}).get("baseline_mm")
                v["thermal"] = vt
            res_all = [read_json(fin)] if "rgb" in self.sensors else []
            if "thermal" in self.sensors:
                res_all.append(read_json(ts["final"]))
            P = layout.positions_V(res_all, R_L_V)
            v["layout"] = layout.check(self.cfg["layout_rules"].get("rules", []), P)
            v["gate"] = self._gate(v)
            write_json(self.ws.root / "validation" / "validation.json", v)
            info.update({"layout_pass": v["layout"]["pass"], "gate_failures": len(v["gate"]["failures"])})
        self.ws.mark("validation", key)
        return v

    def _val_rgb_tasks(self):
        rs = read_json(self.ws.root / "rgb" / "summary.json")
        wins, fin = rs["windows"], rs["final"]
        cams = list(self.cfg["cameras"]["rgb"])
        A, B = wins[0::2], wins[1::2]
        plan = self.plan()
        parked = [p["name"] for p in plan["parked"] if (self.ws.seg_dir(p["name"]) / "source.json").exists()]
        tasks, ho = [], []
        if len(A) and len(B):
            for h, ws_ in (("A", A), ("B", B)):
                tasks.append((f"rgb_half{h}", "rgb_solve", {"name": f"half{h}", "segs": ws_, "cams": cams,
                              "start": {"kind": "result", "path": fin}, "opts": self._rgb_opts(len(ws_))}))
            # held-out: the calibration of one half held on the other half's windows
            for h, other, segs in (("A", "B", B), ("B", "A", A)):
                ho.append((f"rgb_heldout_{h}on{other}", "rgb_solve", {"name": f"heldout_{h}on{other}", "segs": segs, "cams": cams,
                           "start": {"kind": "result", "path": fin}, "opts": dict(self._rgb_opts(len(segs)), ties=False),
                           "held": str(self.ws.root / "rgb" / f"half{h}" / "result.json")}))
        edge_wins = wins[:: max(1, len(wins) // 6)][:6]
        tasks.append(("rgb_edges", "rgb_edges", {"calib": fin, "wins": edge_wins, "parked": parked, "cams": cams,
                      "out": str(self.ws.root / "validation" / "rgb_edges.json"), "vote": self.cfg["validation"]["vote"]}))
        return tasks, ho

    def _val_thermal_tasks(self):
        ts = read_json(self.ws.root / "thermal" / "summary.json")
        twins = ts["windows"]
        TA, TB = twins[0::2], twins[1::2]
        fo = self._thermal_opts("final")
        if self.cfg["thermal"]["final"].get("stereo"):
            fo["stereo"] = "links"
        rgb_res = str(self.ws.root / "rgb" / "final" / "result.json") if "rgb" in self.sensors else None
        tasks, ho = [], []
        if len(TA) and len(TB):
            tasks += self._thermal_half_tasks(ts["lens"], rgb_res)
            for h, other, segs in (("A", "B", TB), ("B", "A", TA)):
                o = dict(self._thermal_opts("final"), free=["dt"], edges=False, ties=False)
                ho.append((f"thermal_heldout_{h}on{other}", "thermal_solve", {"name": f"heldout_{h}on{other}", "segs": segs,
                           "start": {"path": str(self.ws.root / "thermal" / f"half{h}" / "result.json")}, "opts": o,
                           "rgb_result": rgb_res, "held": True}))
        tasks.append(("thermal_edges", "thermal_edge_eval", {"result": ts["final"], "segs": twins,
                      "out": str(self.ws.root / "validation" / "thermal_edges.json")}))
        return tasks, ho

    def _thermal_half_tasks(self, lens, rgb_res):
        twins = self.thermal_windows()
        TA, TB = twins[0::2], twins[1::2]
        fo = self._thermal_opts("final")
        if self.cfg["thermal"]["final"].get("stereo"):
            fo["stereo"] = "links"
        if not (len(TA) and len(TB)):
            return []
        return [(f"thermal_half{h}", "thermal_solve", {"name": f"half{h}", "segs": ws_, "start": {"path": lens},
                 "opts": fo, "rgb_result": rgb_res}) for h, ws_ in (("A", TA), ("B", TB))]

    def run_val_rgb(self):
        tasks, ho = self._val_rgb_tasks()
        self.run_tasks("validation", tasks, nproc=3, threads=self.cfg["resources"]["solver_threads"])
        if ho:
            self.run_tasks("validation_heldout", ho, nproc=2, threads=self.cfg["resources"]["solver_threads"])

    def run_val_thermal(self):
        tasks, ho = self._val_thermal_tasks()
        self.run_tasks("validation", tasks, nproc=3, threads=self.cfg["resources"]["solver_threads"])
        if ho:
            self.run_tasks("validation_heldout", ho, nproc=2, threads=self.cfg["resources"]["solver_threads"])

    def _gate(self, v):
        g = self.cfg["validation"]["gate"]
        fails = []
        for sensor in ("rgb", "thermal"):
            s = v.get(sensor, {}).get("halves", {}).get("sigma", {})
            for c, x in s.items():
                if sensor == "rgb" and x["rot_deg"] > g["rgb_rot_deg"]:
                    fails.append(f"{c}: rotation 1-sigma {x['rot_deg']:.2f} deg > {g['rgb_rot_deg']}")
                lim = g["rgb_axis_mm"] if sensor == "rgb" else g["thermal_axis_mm"]
                if x["along_axis_mm"] > lim:
                    fails.append(f"{c}: along-axis 1-sigma {x['along_axis_mm']:.0f} mm > {lim}")
        for r in v.get("layout", {}).get("rules", []):
            if not r["pass"]:
                fails.append(f"layout rule {r['name']} failed: {r}")
        for c, e in v.get("thermal", {}).get("edges", {}).items():
            vt = e.get("vote", {})
            if vt.get("pass") is False:
                fails.append(f"{c}: voting gate failed (peak {vt.get('du')}/{vt.get('dv')} px, contrast {vt.get('contrast')})")
        return {"failures": fails, "pass": not fails}

    # ================================================================== 8: outputs
    def stage_outputs(self):
        from .outputs import report, writers
        out = self.out
        out.mkdir(parents=True, exist_ok=True)
        v = read_json(self.ws.root / "validation" / "validation.json")
        R_L_V = np.array(read_json(self.ws.root / "lo" / "vehicle_axes.json")["R_L_V"])
        names = self.names()
        plan = self.plan()
        q = self.kept()
        with self.ev.stage("outputs") as info:
            metrics = report.metrics_table(v, self.cfg)
            data = (f"{', '.join(Path(p).name for p in self.bags.values())}: {len(q['kept'])} windows, "
                    f"{sum(w['t1_s'] - w['t0_s'] for w in plan['windows'] if w['name'] in q['kept']):.0f} s")
            meta = {"data": data, "generated_by": f"nontarget_cal {__version__} ({self.a.mode})",
                    "topics": {c: names[b][k][c] for b in names for k in ("rgb", "thermal") for c in names[b].get(k, {})}}
            files = []
            rgb_res = therm_res = None
            if "rgb" in self.sensors:
                rgb_res = read_json(self.ws.root / "rgb" / "final" / "result.json")
                files += writers.write_rgb(rgb_res, out, R_L_V, metrics, meta, *self.cfg["cameras"]["rgb_size"])
            rig_T = np.array(rgb_res["cameras"][self.cfg["cameras"]["reference"]]["T_cam_lidar"]) if rgb_res else None
            if "thermal" in self.sensors:
                therm_res = read_json(self.ws.root / "thermal" / "final" / "result.json")
                ts = read_json(self.ws.root / "thermal" / "summary.json")
                meta["thermal_dt_per_bag"] = ts["dt_per_bag_s"]
                files += writers.write_thermal(therm_res, out, R_L_V, metrics, meta, rig=rig_T, W=self.cfg["cameras"]["thermal_size"][0],
                                               H=self.cfg["cameras"]["thermal_size"][1])
            rig = writers.write_rig(rgb_res, therm_res, out, R_L_V, self.cfg["cameras"]["reference"])
            if rig:
                files.append(rig)
            z = writers.make_zip(out, files, self.cfg["outputs"]["zip_name"])
            imgs = {}
            if self.cfg["outputs"]["images"]["parked"] or self.cfg["outputs"]["images"]["driving"]:
                parked = [p["name"] for p in plan["parked"] if (self.ws.seg_dir(p["name"]) / "source.json").exists()]
                r = self.run_tasks("images", [("images", "images", {
                    "rgb_result": str(self.ws.root / "rgb" / "final" / "result.json") if rgb_res else None,
                    "thermal_result": str(self.ws.root / "thermal" / "final" / "result.json") if therm_res else None,
                    "parked": parked if self.cfg["outputs"]["images"]["parked"] else [],
                    "moving": q["kept"] if self.cfg["outputs"]["images"]["driving"] else [],
                    "outdir": str(out), "masks_dir": self.cfg["paths"]["masks_dir"]})], nproc=1, allow_fail=True)
                imgs = r["images"].get("out", {}) if r["images"].get("ok") else {"error": r["images"].get("error")}
            summary = {"version": __version__, "mode": self.a.mode, "sensors": self.sensors, "bags": self.bags,
                       "workdir": str(self.ws.root), "out": str(out), "names": {b: {k: names[b][k] for k in ("rgb", "thermal", "source")} for b in names},
                       "windows": {"kept": q["kept"], "dropped": q["dropped"], "thermal": q.get("thermal")},
                       "preflight": read_json(self.ws.root / "preflight" / "report.json"),
                       "metrics": metrics, "validation": v, "stages": self._stage_times(),
                       "rgb": read_json(self.ws.root / "rgb" / "summary.json") if rgb_res else None,
                       "thermal": read_json(self.ws.root / "thermal" / "summary.json") if therm_res else None,
                       "images": imgs, "zip": str(z)}
            write_json(out / "summary.json", summary)
            report.write_all(out, summary, self.cfg)
            info.update({"files": len(files), "zip": str(z)})
        return summary

    def _parallel(self):
        """Context manager: par(fn) starts fn in a thread; on exit all are joined and the first
        exception (a Refusal first) is re-raised. Worker processes are limited by self._sem."""
        import threading
        from contextlib import contextmanager

        @contextmanager
        def cm():
            threads, errs = [], []

            def par(fn):
                def w():
                    try:
                        fn()
                    except BaseException as e:  # noqa
                        errs.append(e)
                t = threading.Thread(target=w, daemon=True)
                t.start()
                threads.append(t)
            try:
                yield par
            finally:
                for t in threads:
                    t.join()
            if errs:
                ref = [e for e in errs if isinstance(e, E.Refusal)]
                raise (ref or errs)[0]
        return cm()

    def _stage_times(self):
        ev = []
        p = self.ws.root / "events.jsonl"
        if p.exists():
            for line in p.read_text().splitlines():
                try:
                    r = json.loads(line)
                except Exception:  # noqa
                    continue
                if r.get("ev") == "stage_end":
                    ev.append({k: r.get(k) for k in ("stage", "wall_s", "disk_gb", "t")})
        last = {}
        for r in ev:
            last[r["stage"]] = r
        return list(last.values())

    # ================================================================== run
    def close_viz(self):
        if self.viz is not None:
            self.viz.close()

    def run(self):
        t0 = time.time()
        from .viz_pipeline import start, validation
        start(self)
        self.ev.emit("run_start", version=__version__, bags=self.bags, sensors=self.sensors, mode=self.a.mode,
                     workdir=str(self.ws.root), out=str(self.out), run_id=self.viz_run_id)
        write_json(self.ws.root / "run.json", {k: (str(v) if isinstance(v, Path) else v) for k, v in vars(self.a).items()})
        self.stage_check(force=self.a.force)
        self.stage_extract()
        # Three threads share the worker-process budget: the RGB tracker needs only the images, so it
        # runs next to the LiDAR odometry; the RGB chain (solves + validation solves) and the thermal
        # chain each start as soon as the LiDAR odometry is done.
        import threading
        lo_done, lo_ok = threading.Event(), []

        def after_lo(fn):
            def w():
                lo_done.wait()
                if lo_ok:
                    fn()
            return w
        with self._parallel() as par:
            if "rgb" in self.sensors:
                par(lambda: (self.stage_rgb_tracks(), after_lo(lambda: (self.stage_rgb(), self.run_val_rgb()))()))
            if "thermal" in self.sensors:
                par(after_lo(lambda: (self.stage_thermal(), self.run_val_thermal())))
            try:
                self.stage_lo()
                from .viz_pipeline import axes
                axes(self)
                if self.viz is not None:
                    self.viz.cached_map(self.ws, self.kept()["kept"][0])
                lo_ok.append(True)
            finally:
                lo_done.set()
        v = self.stage_validation()
        validation(self, v)
        s = self.stage_outputs()
        if self.viz is not None:
            self.viz.publish({"stage": "validation", "progress": 1.0, "status_text": "실제 계산 완료",
                              "complete": True}, force=True)
        self.ev.emit("run_end", ok=True, wall_s=round(time.time() - t0, 1), out=str(self.out),
                     gate_pass=s["validation"]["gate"]["pass"])
        return s
