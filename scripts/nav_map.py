#!/usr/bin/env python3
"""Data Machine 지도·내비 서버 — TCar-livingmap 포크(~/DataMachine-map)를 GUI 가 켜질 때 띄운다.

구역(하남·고덕·암사 …) 안에 루트를 정의하고, 휴대폰으로 차량 내비처럼 안내받는 웹앱이다. 서버는 127.0.0.1 에만
열리고, 휴대폰은 Tailscale Funnel 공개 주소(https://<이름>.<tailnet>.ts.net)로 들어온다.

- 서버는 GUI 와 따로 산다(start_new_session). GUI 를 다시 켜도 휴대폰 안내가 끊기지 않게 — 이미 떠 있으면 그대로 쓴다.
- 공개 주소는 `tailscale funnel status --json` 에서 이 포트로 프록시하는 항목을 찾는다. 없으면 이유를 돌려준다.

설정(선택): ~/.config/dm_clip_gui/navmap.yaml  {dir: ~/DataMachine-map, port: 8010, server: "", token: ""}
  - server 를 적으면(예: https://datamachine.tailc5057a.ts.net) 원격 모드 — 서버는 랩 PC 에서 상시로 돌고, 이 PC 는
    띄우지 않고 그 주소로 녹화 연동만 한다. token = 서버 .env 의 DM_TOKEN 과 같은 값 (파일은 600).
  - 비우면 로컬 모드 — 이 PC 의 dm-navmap 서비스(127.0.0.1:port), 토큰은 서버가 만든 data/.dm_token.

    python3 nav_map.py status   # 서버 · 공개 주소 확인
    python3 nav_map.py start    # 서버 띄우기 (떠 있으면 그대로)
"""
import json
import shutil
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

import yaml

CFG_PATH = Path.home() / ".config" / "dm_clip_gui" / "navmap.yaml"
DEFAULTS = {"dir": "~/DataMachine-map", "port": 8010, "server": "", "token": ""}


def config():
    cfg = dict(DEFAULTS)
    try:
        cfg.update(yaml.safe_load(CFG_PATH.read_text()) or {})
    except (OSError, yaml.YAMLError):
        pass
    cfg["dir"] = Path(str(cfg["dir"])).expanduser()
    cfg["port"] = int(cfg["port"])
    cfg["server"] = str(cfg.get("server") or "").strip().rstrip("/")
    cfg["token"] = str(cfg.get("token") or "").strip()
    return cfg


def remote(cfg):
    return bool(cfg["server"])


def base_url(cfg):
    return cfg["server"] or f"http://127.0.0.1:{cfg['port']}"


def alive(cfg, timeout=None):
    timeout = timeout or (5.0 if remote(cfg) else 1.5)
    try:
        with urllib.request.urlopen(f"{base_url(cfg)}/api/config", timeout=timeout) as r:
            return r.status == 200
    except OSError:
        return False


def start(cfg=None):
    """(ok, 문구). 이미 떠 있으면 그대로 쓴다. 원격 모드면 띄우지 않고 응답만 본다."""
    cfg = cfg or config()
    port, root = cfg["port"], cfg["dir"]
    if remote(cfg):
        if alive(cfg):
            return True, f"랩 PC 서버 사용 ({cfg['server']})"
        return False, f"랩 PC 서버({cfg['server']})가 응답하지 않음 — 이 PC 인터넷(핫스팟) · 랩 PC 확인"
    if alive(cfg):
        return True, f"이미 실행 중 (localhost:{port})"
    if (Path.home() / ".config/systemd/user/dm-navmap.service").exists():
        # 부팅 때 뜨는 서비스(dm-navmap) — GUI 가 따로 띄우면 포트를 두고 서비스와 다툰다
        subprocess.run(["systemctl", "--user", "restart", "dm-navmap"], capture_output=True, timeout=10)
        for _ in range(40):
            time.sleep(0.25)
            if alive(cfg):
                return True, f"서비스 dm-navmap 다시 시작함 (localhost:{port})"
        return False, "서비스 dm-navmap 이 응답하지 않습니다 — journalctl --user -u dm-navmap"
    uvicorn = root / ".venv" / "bin" / "uvicorn"
    if not (root / "server.py").exists():
        return False, f"{root} 에 지도 서버가 없습니다 (TCar-livingmap 포크를 받아 두세요)"
    if not uvicorn.exists():
        return False, f"{root}/.venv 가 없습니다 — python3 -m venv .venv && .venv/bin/pip install -r requirements.txt"
    log = open(root / "server.log", "ab")
    subprocess.Popen([str(uvicorn), "server:app", "--host", "127.0.0.1", "--port", str(port)],
                     cwd=root, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                     start_new_session=True)
    for _ in range(40):
        time.sleep(0.25)
        if alive(cfg):
            return True, f"시작함 (localhost:{port})"
    return False, f"서버가 {port} 에서 응답하지 않습니다 — {root}/server.log 확인"


def public_url(port):
    """(url | None, 이유). Tailscale Funnel 이 이 포트를 공개하고 있으면 그 https 주소."""
    ts = shutil.which("tailscale")
    if not ts:
        return None, "Tailscale 미설치 — 휴대폰에서 못 들어옴"
    try:
        out = subprocess.run([ts, "funnel", "status", "--json"], capture_output=True, text=True, timeout=4)
    except (OSError, subprocess.TimeoutExpired) as e:
        return None, f"tailscale 응답 없음 ({e})"
    if out.returncode != 0:
        return None, "tailscale 오류: " + (out.stderr.strip().splitlines() or ["?"])[-1][:120]
    online, why = tailscale_online(ts)
    try:
        st = json.loads(out.stdout or "{}")
    except ValueError:
        return None, "tailscale funnel status 를 읽지 못함"
    funnel = st.get("AllowFunnel") or {}
    for hostport, web in (st.get("Web") or {}).items():
        for handler in (web.get("Handlers") or {}).values():
            if str(handler.get("Proxy", "")).rstrip("/").endswith(f":{port}"):
                host = hostport[:-4] if hostport.endswith(":443") else hostport
                if funnel.get(hostport):
                    if not online:
                        return None, f"https://{host} 설정됨 · {why}"
                    return f"https://{host}", ""
                return None, f"https://{host} 는 테일넷 안에서만 열림 (Funnel 꺼짐)"
    return None, f"Funnel 미설정 — tailscale funnel --bg {port}"


def tailscale_online(ts="tailscale"):
    """(온라인?, 이유). 교내 Wi-Fi(IRCV_Basement_5G)는 Tailscale 서버 TLS 를 끊어서 Funnel 주소가 안 열린다 (2026-09-30)."""
    try:
        st = json.loads(subprocess.run([ts, "status", "--json"], capture_output=True, text=True,
                                       timeout=4).stdout or "{}")
    except (OSError, ValueError, subprocess.TimeoutExpired):
        return False, "tailscale 상태를 못 읽음"
    if (st.get("Self") or {}).get("Online"):
        return True, ""
    try:
        import ntrip_companion
        ssid = ntrip_companion.current_wifi()
        blocked = ssid and ssid in (ntrip_companion.load_config().get("blocked_wifi") or [])
    except Exception:
        ssid, blocked = "", False
    if blocked:
        return False, f"와이파이 '{ssid}'(교내망)가 Tailscale 을 막음 — 휴대폰 핫스팟에 연결하세요"
    try:
        synced = subprocess.run(["timedatectl", "show", "-p", "NTPSynchronized", "--value"], capture_output=True,
                                text=True, timeout=3).stdout.strip() == "yes"
    except (OSError, subprocess.TimeoutExpired):
        synced = True
    if not synced:
        # CMOS 배터리가 약해 부팅 때 시계가 틀리면 Tailscale 서버 인증서가 '아직 유효하지 않음' 이 되어 못 붙는다
        # (2026-10-01 재부팅 직후 3분간 공개 주소 NXDOMAIN). NTP 가 시계를 맞추면 알아서 붙는다.
        return False, "PC 시계가 아직 안 맞음 (CMOS 배터리) — NTP 가 맞출 때까지 Tailscale 이 못 붙음: sudo timedatectl set-ntp true"
    return False, "Tailscale 오프라인 (인터넷 · 핫스팟 연결 확인)"


_KAKAO_CACHE = {}          # 사이트 주소 -> (확인 시각, 문제 문구)
KAKAO_RECHECK_S = 600


def kakao_domain_problem(cfg, site):
    """카카오 지도 JS 키가 이 주소에서 열리는지. 등록 안 된 도메인이면 지도가 빈 화면이 된다 (401 domain mismatched)."""
    hit = _KAKAO_CACHE.get(site)
    if hit and time.time() - hit[0] < KAKAO_RECHECK_S:
        return hit[1]
    key = ""
    try:
        for line in (cfg["dir"] / ".env").read_text().splitlines():
            if line.startswith("KAKAO_JS_KEY="):
                key = line.split("=", 1)[1].strip()
    except OSError:
        pass
    problem = ""
    if not key and remote(cfg):
        return ""                                 # 키가 이 PC 에 없다 — 랩 PC 서버 쪽에서 확인할 일
    if not key:
        problem = "카카오 지도 키 없음 (~/DataMachine-map/.env KAKAO_JS_KEY)"
    else:
        req = urllib.request.Request(f"https://dapi.kakao.com/v2/maps/sdk.js?appkey={key}&autoload=false",
                                     headers={"Referer": site.rstrip("/") + "/"})
        try:
            urllib.request.urlopen(req, timeout=5).close()
        except urllib.error.HTTPError as e:
            if e.code in (401, 403):
                problem = f"카카오 콘솔에 {site} 도메인 미등록 — 지도가 안 뜸"
        except OSError:
            return hit[1] if hit else ""          # 인터넷 문제는 여기서 판단하지 않는다
    _KAKAO_CACHE[site] = (time.time(), problem)
    return problem


def status(cfg=None):
    cfg = cfg or config()
    running = alive(cfg)
    if remote(cfg):
        why = "" if running else "랩 PC 서버 응답 없음 — 이 PC 인터넷(핫스팟) 확인. 안 되면 녹화 연동이 안 됨"
        problem = kakao_domain_problem(cfg, cfg["server"]) if running else ""
        return {"running": running, "local": cfg["server"], "public": cfg["server"] if running else None,
                "note": why or problem, "kakao_problem": problem, "remote": True}
    url, why = public_url(cfg["port"])
    problem = kakao_domain_problem(cfg, url) if running and url else ""
    return {"running": running, "local": f"http://localhost:{cfg['port']}", "public": url,
            "note": why or problem, "kakao_problem": problem}


def _dm_token(cfg):
    """녹화 제어 토큰 — 원격이면 설정의 token(랩 PC .env DM_TOKEN 과 같은 값), 로컬이면 서버가 만든 data/.dm_token."""
    if cfg["token"]:
        return cfg["token"]
    try:
        return (cfg["dir"] / "data" / ".dm_token").read_text().strip()
    except OSError:
        return ""


def _post(cfg, path, body, timeout=None):
    timeout = timeout or (5.0 if remote(cfg) else 2.0)
    req = urllib.request.Request(f"{base_url(cfg)}{path}", data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json", "X-DM-Token": _dm_token(cfg)},
                                 method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read() or b"{}")


def dm_sync(cfg, enabled, recording, gnss, watch):
    """지도 연동 녹화: GUI 상태를 알리고 {active: [진행 중 런], watched: {run: 상태}} 를 받는다 (토큰 API)."""
    return _post(cfg, "/api/dm/sync", {"enabled": enabled, "recording": recording, "gnss": gnss,
                                       "watch": list(watch)})


def dm_run_state(cfg, rid, state, msg="", bag=""):
    """폰에 보일 이 런의 녹화 상태 — starting · recording · stopping · saved · failed · ignored."""
    return _post(cfg, f"/api/dm/runs/{urllib.parse.quote(rid)}", {"state": state, "msg": msg[:300], "bag": bag})


def aggregate_datasets(csv_path):
    """datasets.csv → {지도 루트: {n, sec, day, dusk, night, missing, last}} (서버의 /api/dm/datasets 와 같은 모양)."""
    import csv
    routes = {}

    def num(v):
        try:
            return float(v or 0)
        except ValueError:
            return 0.0
    with open(csv_path, encoding="utf-8-sig", newline="") as f:
        for row in csv.DictReader(f):
            rid = (row.get("map_route") or "").strip()
            if not rid:
                continue
            s = routes.setdefault(rid, {"n": 0, "sec": 0.0, "day": 0.0, "dusk": 0.0, "night": 0.0, "missing": 0,
                                        "last": ""})
            s["n"] += 1
            s["sec"] += num(row.get("duration_sec"))
            for k in ("day", "dusk", "night"):
                s[k] += num(row.get(k + "_sec"))
            if not str(row.get("on_disk", "")).startswith("있"):
                s["missing"] += 1
            s["last"] = max(s["last"], f"{row.get('date', '')} {row.get('start_time', '')}".strip())
    return routes


def upload_datasets(cfg, csv_path):
    """원격 모드에서 녹화가 끝날 때마다 루트별 녹화 합계를 랩 PC 서버에 올린다 (서버가 CSV 를 직접 못 읽으므로).
    로컬 모드면 서버가 CSV 를 직접 읽으니 아무것도 안 한다. → 올린 루트 수 (로컬이면 None)."""
    if not remote(cfg):
        return None
    routes = aggregate_datasets(csv_path)
    _post(cfg, "/api/dm/datasets", {"routes": routes, "csv": str(csv_path)}, timeout=10)
    return len(routes)


def distance_m(lat1, lon1, lat2, lon2):
    import math
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "status"
    if cmd == "start":
        ok, msg = start()
        print(msg)
        sys.exit(0 if ok else 1)
    if cmd == "upload":
        import dataset_catalog
        c = config()
        path = Path(sys.argv[2]) if len(sys.argv) > 2 else Path.home() / "DM_clipGUI" / "clips" / dataset_catalog.CSV_NAME
        print("올린 루트:", upload_datasets(c, path))
        sys.exit(0)
    print(json.dumps(status(), ensure_ascii=False, indent=1))
