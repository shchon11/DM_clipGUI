#!/usr/bin/env python3
# online_calib.py — 온라인(타깃 없는) 캘리브레이션 작업의 GUI 없는 부분.
#
# nontarget_cal(third_party/nontarget_cal, 차량에는 tools/setup_online_calib.sh 로 venv 에 설치)을
# 주행 bag 여러 개에 돌리는 "작업"을 다룬다: bag 읽기(metadata.yaml) · 디스크 추정 · 대기(pending) 판단 ·
# 백그라운드 실행(setsid, GUI 를 꺼도 계속) · 진행 JSON 줄 해석 · 중단 · 이어서 실행 · 결과 요약.
# Qt 도 ROS 도 import 하지 않는다 — test/test_online_calib.py 가 이 모듈을 그대로 시험한다.
#
# 도구와 약속된 것 (third_party/nontarget_cal/README.md §2):
#   stdout = 한 줄에 JSON 하나 (run_start, stage_start, stage_skip, stage_progress, stage_end, warning,
#            task_failed, refusal, error, run_end, check_done). 사람용 로그는 stderr 와 <workdir>/log.txt.
#   종료 코드 0 정상, 2 거절(마지막 JSON 줄의 code · msg · msg_ko), 1 오류.
#   같은 명령을 다시 돌리면 끝난 단계 · 창은 건너뛴다 (그래서 '이어서 실행' = 같은 명령 다시).

import json
import math
import os
import shlex
import shutil
import signal
import subprocess
import time
import uuid
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent

# ---------------------------------------------------------------- 도구 위치
DATA_HOME = Path(os.environ.get("XDG_DATA_HOME", Path.home() / ".local" / "share")) / "dm_clip_gui"
DEFAULT_VENV = DATA_HOME / "nontarget_cal" / "venv"
JOBS_ROOT = DATA_HOME / "calib" / "jobs"          # 작업마다 명령 · stdout · stderr · 종료 코드
APPLIED_ROOT = DATA_HOME / "calib" / "applied"    # 차량에 적용한 기록 (백업 · 결과 사본 · manifest)

# 소스 트리(scripts/)에서 돌면 리포의 사본, 설치본(install/lib/clip_recorder)이면 share 쪽은 없다 — 그때는 venv.
VENDORED_CANDIDATES = [
    HERE.parent / "third_party" / "nontarget_cal",
    Path.home() / "DM_clipGUI" / "third_party" / "nontarget_cal",
]


def find_tool(executable=None):
    """nontarget_cal 실행 파일과 버전 정보. {"exe", "version", "ok", "msg"}.

    executable 을 주면(설정의 calib.executable) 그것만 본다. 아니면 기본 venv.
    """
    exe = Path(os.path.expanduser(executable)) if executable else DEFAULT_VENV / "bin" / "nontarget_cal"
    info = {"exe": str(exe), "version": {}, "ok": False, "msg": ""}
    if not exe.is_file() or not os.access(exe, os.X_OK):
        info["msg"] = (f"nontarget_cal 이 설치되어 있지 않습니다 ({exe}). "
                       f"리포에서 한 번: bash tools/setup_online_calib.sh")
        return info
    for cand in (exe.parent.parent / "NONTARGET_CAL_VERSION",):
        try:
            info["version"] = yaml.safe_load(cand.read_text(encoding="utf-8")) or {}
        except Exception:
            pass
    info["ok"] = True
    return info


def vendored_info():
    """리포에 들어 있는 사본의 VENDORED_FROM (없으면 {})."""
    for d in VENDORED_CANDIDATES:
        f = d / "VENDORED_FROM"
        if f.is_file():
            try:
                return dict(yaml.safe_load(f.read_text(encoding="utf-8")) or {}, dir=str(d))
            except Exception:
                return {"dir": str(d)}
    return {}


# ---------------------------------------------------------------- 도구 설정값 (게이트 · 추정 기준)
# 도구의 config/default.yaml 과 같은 값. 사본이 있으면 거기서 읽고, 없으면 이 값을 쓴다.
TOOL_DEFAULTS = {
    "estimate": {"rgb_disk_gb": 4.3, "thermal_disk_gb": 0.8, "fixed_disk_gb": 10.0},
    "disk_margin_frac": 0.30, "disk_margin_gb": 20.0,
    "window_s": 40.0, "max_windows": 30,
    "gate": {"rgb_rot_deg": 0.5, "rgb_axis_mm": 60.0, "thermal_axis_mm": 60.0},
    "n_rgb_ref": 14, "n_thermal_ref": 2,
}


def tool_defaults():
    out = json.loads(json.dumps(TOOL_DEFAULTS))
    # 설치된 venv 의 것이 실제로 도는 값 — 먼저. 없으면 리포 사본.
    files = sorted(DEFAULT_VENV.glob("lib/python3*/site-packages/nontarget_cal/config/default.yaml"))
    files += [d / "nontarget_cal" / "config" / "default.yaml" for d in VENDORED_CANDIDATES]
    for f in files:
        if not f.is_file():
            continue
        try:
            c = yaml.safe_load(f.read_text(encoding="utf-8")) or {}
            pf = c.get("preflight", {})
            out["estimate"].update({k: float(v) for k, v in (pf.get("estimate") or {}).items()
                                    if k in out["estimate"]})
            out["disk_margin_frac"] = float(pf.get("disk_margin_frac", out["disk_margin_frac"]))
            out["disk_margin_gb"] = float(pf.get("disk_margin_gb", out["disk_margin_gb"]))
            w = c.get("windows", {})
            out["window_s"] = float(w.get("length_s", out["window_s"]))
            out["max_windows"] = int(w.get("max_windows", out["max_windows"]))
            g = (c.get("validation") or {}).get("gate") or {}
            out["gate"].update({k: float(g[k]) for k in out["gate"] if k in g})
        except Exception:
            pass
        break
    return out


# ---------------------------------------------------------------- bag
def bag_dir(path):
    """.db3/.mcap 파일을 주면 그 폴더."""
    p = Path(os.path.expanduser(str(path)))
    return p.parent if p.is_file() else p


def inspect_bag(path):
    """rosbag2 폴더의 metadata.yaml 만 읽는다 (bag 본문은 안 연다 — 수백 GB).

    {"path", "ok", "error", "duration_s", "size_gb", "n_rgb", "n_thermal", "has_lidar", "has_gnss",
     "storage", "start_ns", "label"}
    """
    d = bag_dir(path)
    info = {"path": str(d), "ok": False, "error": "", "duration_s": 0.0, "size_gb": 0.0,
            "n_rgb": 0, "n_thermal": 0, "has_lidar": False, "has_gnss": False, "storage": "",
            "start_ns": 0, "label": d.name}
    meta = d / "metadata.yaml"
    if not d.is_dir():
        info["error"] = "폴더가 없습니다 (디스크 연결 확인)"
        return info
    size = 0
    for f in d.iterdir():
        if f.suffix in (".db3", ".mcap") or f.name.endswith(".db3-wal"):
            try:
                size += f.stat().st_size
            except OSError:
                pass
    info["size_gb"] = size / 1e9
    if not meta.is_file():
        info["error"] = "metadata.yaml 이 없습니다 (녹화가 정상적으로 닫히지 않았을 수 있음)"
        return info
    try:
        m = (yaml.safe_load(meta.read_text(encoding="utf-8")) or {})["rosbag2_bagfile_information"]
    except Exception as e:
        info["error"] = f"metadata.yaml 을 읽을 수 없습니다: {e}"
        return info
    info["duration_s"] = float((m.get("duration") or {}).get("nanoseconds", 0)) / 1e9
    info["start_ns"] = int((m.get("starting_time") or {}).get("nanoseconds_since_epoch", 0))
    info["storage"] = m.get("storage_identifier", "")
    names = [((t or {}).get("topic_metadata") or {}).get("name", "")
             for t in m.get("topics_with_message_count") or []]
    info["n_rgb"] = sum(1 for n in names if n.endswith("/image_rgb/compressed"))
    info["n_thermal"] = sum(1 for n in names if n.startswith("/thermal") and n.endswith("/image_raw"))
    info["has_lidar"] = "/ouster/points" in names
    info["has_gnss"] = "/gps/fix" in names
    problems = []
    if not info["has_lidar"]:
        problems.append("/ouster/points 없음")
    if not info["has_gnss"]:
        problems.append("/gps/fix 없음")
    if info["n_rgb"] == 0 and info["n_thermal"] == 0:
        problems.append("카메라 영상 토픽 없음")
    if info["duration_s"] < 80:
        problems.append(f"너무 짧음 ({info['duration_s']:.0f} s, 최소 80 s 주행)")
    info["error"] = ", ".join(problems)
    info["ok"] = not problems
    return info


# ---------------------------------------------------------------- 디스크 추정
# 실측 (2026-09-24 야간 bag, 1000 s · 182 GB · RGB 14 + 열화상 2, 창 25개): 작업 폴더 최대 약 92–110 GB
# (추출 71 · LO 11 · 추적 6 · 열화상 3 GB …), 결과 폴더 46 MB. 도구 자체 추정은 10 + 창수 × 5.1 GB 에 30 % + 20 GB
# 여유를 두고 모자라면 거절한다. GUI 는 그보다 더 보수적으로 잡아 도구가 거절할 일이 없게 한다:
#   - 움직인 시간을 모르므로 bag 길이 전체를 쓴다 (도구는 움직인 시간만) — 창 수 상한(30 × 40 s)까지
#   - bag 크기로도 따로 추정해 (추출 = 창 구간의 영상 바이트 그대로) 둘 중 큰 쪽
#   - 여유 50 % + 20 GB, 결과 폴더 5 GB
GUI_MARGIN_FRAC = 0.50
GUI_MARGIN_GB = 20.0
SIZE_FRACTION = 0.65      # 작업 폴더 / (창 구간의 bag 바이트): 실측 110/182 = 0.60
OUT_GB = 5.0


def estimate_storage(bags, sensors=("rgb", "thermal"), defaults=None):
    """bags: inspect_bag 결과 목록. 필요한 여유 공간을 크게 잡은 추정.

    {"windows", "work_gb", "work_need_gb", "out_need_gb", "basis", "runtime_h": (lo, hi)}
    """
    d = defaults or tool_defaults()
    e = d["estimate"]
    total_s = sum(b.get("duration_s", 0.0) for b in bags)
    total_gb = sum(b.get("size_gb", 0.0) for b in bags)
    cap_s = d["max_windows"] * d["window_s"]
    used_s = min(total_s, cap_s)
    n40 = used_s / d["window_s"]
    n_rgb = max([b.get("n_rgb", 0) for b in bags] or [0])
    n_th = max([b.get("n_thermal", 0) for b in bags] or [0])
    per40 = 0.0
    if "rgb" in sensors:
        per40 += e["rgb_disk_gb"] * max(n_rgb, 1) / d["n_rgb_ref"]
    if "thermal" in sensors:
        per40 += e["thermal_disk_gb"] * max(n_th, 1) / d["n_thermal_ref"]
    by_time = e["fixed_disk_gb"] + n40 * per40
    by_size = SIZE_FRACTION * total_gb * (used_s / total_s if total_s > 0 else 1.0)
    work = max(by_time, by_size)
    tool_need = work * (1 + d["disk_margin_frac"]) + d["disk_margin_gb"]
    need = max(work * (1 + GUI_MARGIN_FRAC) + GUI_MARGIN_GB, tool_need)
    # 시간: 4창 46분, 25창 3–3.5시간(16코어 · HDD). 차량 PC(8코어면 약 1.8배)까지 범위로.
    lo_h = (20 + 7.5 * n40) / 60.0
    hi_h = lo_h * 2.0
    basis = (f"bag {len(bags)}개 · 길이 {total_s / 60:.1f}분 · {total_gb:.0f} GB → 40 s 창 약 {n40:.0f}개"
             f"{' (상한 ' + str(d['max_windows']) + '개)' if total_s > cap_s else ''}; "
             f"작업 폴더 추정 {work:.0f} GB (시간 기준 {by_time:.0f} · 크기 기준 {by_size:.0f}) "
             f"× {1 + GUI_MARGIN_FRAC:.1f} + {GUI_MARGIN_GB:.0f} GB")
    return {"windows": round(n40, 1), "work_gb": round(work, 1), "work_need_gb": round(need, 1),
            "out_need_gb": OUT_GB, "basis": basis, "runtime_h": (round(lo_h, 1), round(hi_h, 1)),
            "total_s": total_s, "total_gb": total_gb}


def existing_parent(p):
    p = Path(os.path.expanduser(str(p)))
    while not p.exists() and p != p.parent:
        p = p.parent
    return p


def free_gb(p):
    return shutil.disk_usage(existing_parent(p)).free / 1e9


def same_fs(a, b):
    try:
        return os.stat(existing_parent(a)).st_dev == os.stat(existing_parent(b)).st_dev
    except OSError:
        return False


def dir_size_gb(p):
    tot = 0
    for root, _, files in os.walk(p):
        for f in files:
            try:
                tot += os.lstat(os.path.join(root, f)).st_size
            except OSError:
                pass
    return tot / 1e9


def check_space(est, workdir, outdir, already_gb=0.0, free_fn=None, same_fs_fn=None):
    """(ok, 메시지, 자세히). already_gb = 이어서 실행할 때 작업 폴더에 이미 있는 양 (그만큼 덜 필요)."""
    free_fn = free_fn or free_gb
    same_fs_fn = same_fs_fn or same_fs
    work_need = max(est["work_need_gb"] - already_gb, GUI_MARGIN_GB)
    out_need = est["out_need_gb"]
    detail = {"work_need_gb": round(work_need, 1), "out_need_gb": out_need,
              "work_free_gb": round(free_fn(workdir), 1), "out_free_gb": round(free_fn(outdir), 1),
              "same_fs": bool(same_fs_fn(workdir, outdir))}
    if detail["same_fs"]:
        need = work_need + out_need
        ok = detail["work_free_gb"] >= need
        msg = (f"작업 폴더 디스크: 여유 {detail['work_free_gb']:.0f} GB / 필요 {need:.0f} GB"
               + ("" if ok else f" — {need - detail['work_free_gb']:.0f} GB 모자람"))
    else:
        ok_w = detail["work_free_gb"] >= work_need
        ok_o = detail["out_free_gb"] >= out_need
        ok = ok_w and ok_o
        msg = (f"작업 폴더: 여유 {detail['work_free_gb']:.0f} GB / 필요 {work_need:.0f} GB"
               + ("" if ok_w else " (모자람)")
               + f" · 결과 폴더: 여유 {detail['out_free_gb']:.0f} GB / 필요 {out_need:.0f} GB"
               + ("" if ok_o else " (모자람)"))
    if already_gb > 0:
        msg += f" (작업 폴더에 이미 {already_gb:.0f} GB — 이어서 실행)"
    return ok, msg, detail


# ---------------------------------------------------------------- 작업 (job)
PENDING, RUNNING, INTERRUPTED, CANCELLED, FAILED, REFUSED, DONE = (
    "pending", "running", "interrupted", "cancelled", "failed", "refused", "done")
STATE_LABEL = {PENDING: "대기 (공간 부족)", RUNNING: "실행 중", INTERRUPTED: "중단됨 (이어서 실행 가능)",
               CANCELLED: "취소됨 (이어서 실행 가능)", FAILED: "오류", REFUSED: "거절 (데이터 문제)",
               DONE: "완료"}
RESUMABLE = (PENDING, INTERRUPTED, CANCELLED, FAILED)


def new_job(bags, mode, workdir_root, out_root, sensors=("rgb", "thermal"), init=None, name_map=None,
            config=None, force=False, label=""):
    """작업 하나. workdir_root/out_root 아래에 작업 id 폴더를 만든다 (아직 디스크에는 안 만든다)."""
    if mode not in ("zeroshot", "warm"):
        raise ValueError(mode)
    if mode == "warm" and not init:
        raise ValueError("warm-start 는 이전 캘리브레이션(init)이 필요합니다")
    jid = time.strftime("calib_%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:4]
    return {
        "id": jid, "label": label, "created": time.time(), "state": PENDING, "state_msg": "",
        "bags": [str(bag_dir(b)) for b in bags], "mode": mode, "sensors": list(sensors),
        "init": init, "name_map": name_map, "config": config, "force": bool(force),
        "workdir": str(Path(os.path.expanduser(str(workdir_root))) / jid / "work"),
        "out": str(Path(os.path.expanduser(str(out_root))) / jid / "out"),
        "attempts": [], "pid": None, "result": None,
    }


def tool_args(job):
    """nontarget_cal run 인자 (실행 파일 뒤)."""
    a = ["run", "--bags", *job["bags"], "--out", job["out"], "--workdir", job["workdir"],
         "--mode", job["mode"], "--sensors", ",".join(job["sensors"])]
    if job["mode"] == "warm":
        a += ["--init", str(job["init"])]
    if job.get("name_map"):
        a += ["--name-map", str(job["name_map"])]
    if job.get("config"):
        a += ["--config", str(job["config"])]
    if job.get("force"):
        a += ["--force"]
    return a


def check_args(job, workdir=None):
    """nontarget_cal check (사전 점검만, 약 30초 · 가벼움). workdir: 점검 파일(작음)을 둘 곳 — 여유 공간도
    거기서 재므로 실제 작업 폴더와 같은 디스크여야 한다."""
    wd = workdir or str(Path(job["workdir"]).parent.parent / "_preflight" / time.strftime("%Y%m%d_%H%M%S"))
    a = ["check", "--bags", *job["bags"], "--workdir", wd,
         "--sensors", ",".join(job["sensors"])]
    if job.get("name_map"):
        a += ["--name-map", str(job["name_map"])]
    if job.get("config"):
        a += ["--config", str(job["config"])]
    if job["mode"] == "warm" and job.get("init"):
        a += ["--init", str(job["init"]), "--mode", "warm"]
    return a


def job_dir(job, root=None):
    return Path(root or JOBS_ROOT) / job["id"]


MARKER = "dm_online_calib"


def launch_detached(job, exe, args=None, root=None, nice=10, popen=subprocess.Popen):
    """setsid 로 따로 세션에 띄운다 — GUI 를 꺼도 계속 돈다. 시도마다 stdout/stderr/종료 코드 파일.

    bash 가 프로세스 그룹의 리더다 (pgid = pid) — 중단은 그룹 전체(도구의 작업 프로세스들 포함)에 신호.
    반환: attempt dict {"n", "pid", "stdout", "stderr", "exitcode", "cmd", "t0"}.
    """
    d = job_dir(job, root)
    d.mkdir(parents=True, exist_ok=True)
    n = len(job["attempts"]) + 1
    att = {"n": n, "t0": time.time(),
           "stdout": str(d / f"run{n}.stdout.jsonl"), "stderr": str(d / f"run{n}.stderr.log"),
           "exitcode": str(d / f"run{n}.exitcode")}
    args = tool_args(job) if args is None else args
    cmd = [str(exe), *args]
    att["cmd"] = cmd
    # $0 에 표식을 넣어 두면 /proc/<pid>/cmdline 으로 "그 작업의 bash 인지" 확인할 수 있다 (PID 재사용 대비)
    script = ('trap "" HUP; "$@" > "$DM_OUT" 2> "$DM_ERR"; rc=$?; echo $rc > "$DM_RC.tmp"; '
              'mv -f "$DM_RC.tmp" "$DM_RC"; exit $rc')
    prefix = ["nice", "-n", str(int(nice))] if nice else []
    env = dict(os.environ, DM_OUT=att["stdout"], DM_ERR=att["stderr"], DM_RC=att["exitcode"],
               PYTHONUNBUFFERED="1")
    (d / f"run{n}.cmd").write_text(" ".join(shlex.quote(c) for c in cmd) + "\n", encoding="utf-8")
    p = popen(prefix + ["bash", "-c", script, f"{MARKER}:{job['id']}", *cmd],
              stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
              start_new_session=True, env=env, close_fds=True)
    att["pid"] = p.pid
    _CHILDREN[p.pid] = p
    # fork 직후에는 /proc/<pid>/cmdline 이 아직 부모(파이썬) 것이다 — bash 로 바뀔 때까지 잠깐 기다린다
    for _ in range(100):
        if p.poll() is not None or pid_is_job(p.pid, job["id"]):
            break
        time.sleep(0.01)
    job["attempts"].append(att)
    job["pid"] = p.pid
    job["state"] = RUNNING
    job["state_msg"] = ""
    return att


_CHILDREN = {}     # 이 GUI 가 띄운 것 — 끝나면 거둬야(poll) 좀비로 남지 않는다


def pid_is_job(pid, job_id):
    """pid 가 살아 있고 그 작업의 bash 인가."""
    if not pid:
        return False
    child = _CHILDREN.get(pid)
    if child is not None and child.poll() is not None:
        _CHILDREN.pop(pid, None)
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        pass
    try:
        cmd = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
    except OSError:
        return False
    return f"{MARKER}:{job_id}".encode() in cmd


def read_exitcode(att):
    try:
        return int(Path(att["exitcode"]).read_text().strip())
    except (OSError, ValueError, KeyError):
        return None


def signal_job(job, sig=signal.SIGTERM):
    """작업의 프로세스 그룹 전체에 신호. 보낸 경우 True."""
    pid = job.get("pid")
    if not pid_is_job(pid, job["id"]):
        return False
    try:
        os.killpg(pid, sig)
        return True
    except (ProcessLookupError, PermissionError):
        return False


class JobStore:
    """작업 목록 (JSON 한 파일). GUI 를 다시 켜도 이어 보인다."""

    def __init__(self, path=None):
        self.path = Path(path or (JOBS_ROOT.parent / "jobs.json"))
        self.jobs = []
        self.load()

    def load(self):
        try:
            self.jobs = json.loads(self.path.read_text(encoding="utf-8")).get("jobs", [])
        except (OSError, ValueError):
            self.jobs = []
        return self.jobs

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps({"jobs": self.jobs}, ensure_ascii=False, indent=1), encoding="utf-8")
        os.replace(tmp, self.path)

    def add(self, job):
        self.jobs.append(job)
        self.save()
        return job

    def get(self, jid):
        return next((j for j in self.jobs if j["id"] == jid), None)

    def remove(self, jid):
        self.jobs = [j for j in self.jobs if j["id"] != jid]
        self.save()

    def running(self):
        return [j for j in self.jobs if j["state"] == RUNNING]


def settle_state(job, alive=None):
    """실행 중으로 적혀 있는 작업의 실제 상태를 정한다 (GUI 재시작 · 프로세스 종료 뒤).

    alive: 테스트용 — None 이면 pid_is_job. 바뀌었으면 True.
    """
    if job["state"] != RUNNING or not job["attempts"]:
        return False
    att = job["attempts"][-1]
    is_alive = pid_is_job(job.get("pid"), job["id"]) if alive is None else alive
    if is_alive:
        return False
    rc = read_exitcode(att)
    prog = Progress()
    prog.feed_file(att["stdout"])
    job["pid"] = None
    att["rc"] = rc
    if rc == 0 and prog.run_end:
        job["state"] = DONE
        job["state_msg"] = "통과" if prog.run_end.get("gate_pass") else "완료 — 판정 '확인 필요'"
    elif rc == 2 or prog.refusal:
        job["state"] = REFUSED
        r = prog.refusal or {}
        job["state_msg"] = r.get("msg_ko") or r.get("msg") or "거절"
        if r.get("code") == "insufficient_disk":
            job["state"] = PENDING          # 공간만 생기면 되는 거절은 대기로
    elif att.get("stop_reason"):
        job["state"] = CANCELLED
        job["state_msg"] = att["stop_reason"]
    elif rc is None:
        job["state"] = INTERRUPTED
        job["state_msg"] = "프로세스가 종료 코드 없이 끝났습니다 (전원 · 강제 종료) — 이어서 실행할 수 있습니다"
    else:
        job["state"] = FAILED
        e = prog.error or {}
        job["state_msg"] = (e.get("msg_ko") or "오류") + (f": {e.get('msg')}" if e.get("msg") else f" (exit {rc})")
    return True


# ---------------------------------------------------------------- 진행 이벤트
# 도구의 큰 단계 (stage_start/stage_end 이름) — 무게는 README §6 의 15분 주행 실측(분)
STAGES = [
    ("names", "카메라 이름 확인", 1), ("preflight", "사전 점검", 1), ("windows", "구간 선택", 0.5),
    ("extract", "데이터 추출", 27), ("lo", "LiDAR 오도메트리", 75), ("rgb_tracks", "RGB 특징점 추적", 10),
    ("thermal_tracks", "열화상 추적", 3), ("rgb_solve", "RGB 풀이", 40), ("thermal_solve", "열화상 풀이", 15),
    ("validation", "검증", 20), ("outputs", "결과 파일", 5),
]
STAGE_LABEL = {k: lbl for k, lbl, _ in STAGES}
SENSOR_STAGES = {"rgb": ("rgb_tracks", "rgb_solve"), "thermal": ("thermal_tracks", "thermal_solve")}
# stage_progress 의 세부 이름 → 큰 단계
SUBSTAGE = {"extract_scan": "extract", "extract": "extract", "lo": "lo", "axes": "lo",
            "rgb_tracks": "rgb_tracks", "thermal_tracks": "thermal_tracks",
            "images": "outputs", "validation": "validation", "validation_heldout": "validation"}
# stage_skip 이름 (캐시되어 건너뜀) → 큰 단계들
SKIP_ALIAS = {"rgb": ("rgb_solve",), "thermal": ("thermal_tracks", "thermal_solve")}


def main_stage(name):
    if name in SUBSTAGE:
        return SUBSTAGE[name]
    for pre, st in (("rgb_", "rgb_solve"), ("thermal_", "thermal_solve"), ("validation", "validation"),
                    ("extract", "extract")):
        if name.startswith(pre):
            return st
    return name


class Progress:
    """JSON 줄들을 먹여 진행 상태를 만든다."""

    def __init__(self, sensors=("rgb", "thermal")):
        self.sensors = tuple(sensors)
        self.stages = {k: {"state": "wait", "done": 0, "total": 0, "sub": "", "wall_s": None}
                       for k, _, _ in STAGES}
        self.warnings = []
        self.refusal = None
        self.error = None
        self.run_start = None
        self.run_end = None
        self.check = None
        self.task_failed = []
        self.last = None
        self.bad_lines = 0
        self._offset = 0
        self._partial = b""

    def feed_line(self, line):
        line = line.strip()
        if not line:
            return None
        try:
            ev = json.loads(line)
        except ValueError:
            self.bad_lines += 1
            return None
        if not isinstance(ev, dict) or "ev" not in ev:
            self.bad_lines += 1
            return None
        self.feed(ev)
        return ev

    def feed(self, ev):
        self.last = ev
        kind = ev.get("ev")
        st = ev.get("stage")
        if kind == "run_start":
            self.run_start = ev
        elif kind == "stage_start" and st in self.stages:
            self.stages[st].update(state="run", done=0, total=0)
        elif kind == "stage_skip":
            for s in SKIP_ALIAS.get(st, (st,)):
                if s in self.stages:
                    self.stages[s].update(state="skip")
        elif kind == "stage_progress" and st:
            m = main_stage(st)
            if m in self.stages:
                s = self.stages[m]
                if s["state"] == "wait":
                    s["state"] = "run"
                s.update(done=int(ev.get("done", 0)), total=int(ev.get("total", 0)), sub=st)
        elif kind == "stage_end" and st in self.stages:
            self.stages[st].update(state="ok" if ev.get("ok", True) else "fail", wall_s=ev.get("wall_s"))
        elif kind == "warning":
            self.warnings.append(ev)
        elif kind == "task_failed":
            self.task_failed.append(ev)
        elif kind == "refusal":
            self.refusal = ev
        elif kind == "error":
            self.error = ev
        elif kind == "run_end":
            self.run_end = ev
        elif kind == "check_done":
            self.check = ev

    def feed_file(self, path):
        """파일에서 새로 붙은 줄만 읽는다 (여러 번 불러도 된다). 반환: 새 이벤트 목록."""
        out = []
        try:
            with open(path, "rb") as f:
                f.seek(self._offset)
                data = f.read()
        except OSError:
            return out
        self._offset += len(data)
        data = self._partial + data
        lines = data.split(b"\n")
        self._partial = lines.pop()          # 마지막 줄은 아직 덜 써졌을 수 있다
        for raw in lines:
            ev = self.feed_line(raw.decode("utf-8", errors="replace"))
            if ev:
                out.append(ev)
        return out

    def expected(self):
        skip = set()
        for s, sts in SENSOR_STAGES.items():
            if s not in self.sensors:
                skip.update(sts)
        return [(k, lbl, w) for k, lbl, w in STAGES if k not in skip]

    def fraction(self):
        """대략의 전체 진행률 0..1 (단계 무게 = 실측 시간)."""
        tot = got = 0.0
        for k, _, w in self.expected():
            s = self.stages[k]
            tot += w
            if s["state"] in ("ok", "skip"):
                got += w
            elif s["state"] == "run" and s["total"] > 0:
                got += w * min(1.0, s["done"] / s["total"])
        if self.run_end:
            return 1.0
        return got / tot if tot else 0.0

    def current(self):
        """지금 도는 단계들 [(key, 한글, done, total, sub)]."""
        return [(k, STAGE_LABEL[k], s["done"], s["total"], s["sub"])
                for k, s in self.stages.items() if s["state"] == "run"]


# ---------------------------------------------------------------- 결과 요약
def load_result(out, defaults=None):
    """결과 폴더(summary.json) → 판정 · 카메라별 표. 없으면 None.

    도구가 기록한 카메라별 판정을 우선하고, 구형 결과만 pipeline._gate 기준으로 계산.
    배치 규칙은 전체 판정에만, 야간 RGB 에지/투표는 참고용으로 별도 표시한다.
    """
    out = Path(out)
    f = out / "summary.json"
    if not f.is_file():
        return None
    s = json.loads(f.read_text(encoding="utf-8"))
    from calib_viz.results import camera_gate

    v = s.get("validation") or {}
    metrics = s.get("metrics") or {}
    if not metrics and (out / "metrics.json").is_file():
        metrics = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
    # A complete solver gate is authoritative. Only legacy summaries need the
    # installed thresholds; caller-supplied thresholds remain an explicit review.
    legacy_defaults = defaults
    if not isinstance((v.get("gate") or {}).get("failures"), list) and defaults is None:
        legacy_defaults = tool_defaults()
    cams = []
    for c, m in metrics.items():
        decision = camera_gate(s, metrics, c, defaults=legacy_defaults)
        cams.append({"camera": c, "sensor": m.get("sensor", "rgb"),
                     "rot_deg": m.get("rot_deg"), "pos_mm": m.get("pos_mm"),
                     "along_axis_mm": m.get("along_axis_mm"), "focal_px": m.get("focal_px"),
                     "track_reproj_px": m.get("track_reproj_px"),
                     "heldout_px": m.get("heldout_track_reproj_px"), "lidar_edge_px": m.get("lidar_edge_px"),
                     "vote": m.get("vote") or {}, **decision})
    gate = v.get("gate") or {}
    layout = (v.get("layout") or {}).get("rules") or []
    return {"out": str(out), "mode": s.get("mode"), "version": s.get("version"), "bags": s.get("bags"),
            "names": s.get("names"), "gate_pass": bool(gate.get("pass")), "failures": gate.get("failures") or [],
            "layout": [{"name": r.get("name"), "pass": bool(r.get("pass"))} for r in layout],
            "cameras": cams, "warnings": (s.get("preflight") or {}).get("warnings") or [],
            "windows": len((s.get("windows") or {}).get("kept") or []),
            "report": str(out / "report.md") if (out / "report.md").is_file() else None}


def fmt_duration(sec):
    sec = max(0, int(sec))
    h, r = divmod(sec, 3600)
    m, s = divmod(r, 60)
    return f"{h}:{m:02d}:{s:02d}" if h else f"{m}:{s:02d}"


def pick_workdir_root(candidates, need_gb):
    """여유 공간이 가장 큰 후보 (필요량 이상이면 우선)."""
    best = None
    for c in candidates:
        try:
            f = free_gb(c)
        except OSError:
            continue
        key = (f >= need_gb, f)
        if best is None or key > best[0]:
            best = (key, c)
    return best[1] if best else None


def isfinite(x):
    try:
        return math.isfinite(float(x))
    except (TypeError, ValueError):
        return False
