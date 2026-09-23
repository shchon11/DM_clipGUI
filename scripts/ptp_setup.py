#!/usr/bin/env python3
# ptp_setup.py — GNSS(Orin) → PC → 카메라 PTP 체인을 한 번에 세우고 확인한다.   alias: ptp
#
#   GNSS PPS ─▶ Orin (grandmaster) ─▶ PC eno1 : ptp4l slave, 하드웨어 타임스탬프     ← 이 스크립트 (ptp-orin)
#                                       └▶ phc2sys: eno1 시계 → PC 시스템 시계         ← 이 스크립트 (phc2sys-orin)
#                                PC 시스템 시계 ─▶ enp3s0f1 : ptp4l, 소프트웨어 타임스탬프 ─▶ 카메라
#                                                   ↑ DM_clipGUI 가 센서 기동 때 띄운다 ('PTP grandmaster NIC' 비워 둠)
#
# 사용:
#   ptp            세우기 — 겹치는 ptp4l · phc2sys 정리, NTP 끔, 두 서비스 시작, 수렴할 때까지 기다린 뒤 점검 결과
#   ptp status     지금 상태만 (sudo 없이. GNSS 잠김(clockClass)은 sudo 가 필요해 물어본다)
#   ptp stop       두 서비스 멈춤
#
# 두 프로세스는 systemd 서비스로 띄운다 — 터미널을 닫아도 돌고, 로그는 journalctl -u ptp-orin / phc2sys-orin.
# config/systemd 의 유닛 파일을 설치해 두었으면(부팅 때 자동) 그걸 다시 시작하고, 아니면 systemd-run 으로 임시
# 서비스를 만든다 (재부팅하면 사라진다 — 그때 다시 ptp).
#
# 카메라 쪽 NIC(enp3s0f1)에는 하드웨어 타임스탬프 ptp4l 을 띄우지 않는다. 82599 는 받은 PTP 패킷을 한 번에 하나만
# 타임스탬프해서, 카메라 14대의 Delay_Req 가 겹치면 카메라 시계가 N×0.5 s 씩 튀었다 (2026-09-22).

import argparse
import re
import subprocess
import sys
import time
from pathlib import Path

ORIN_NIC = "eno1"
CAMERA_NIC = "enp3s0f1"
UNITS = ("ptp-orin", "phc2sys-orin")
CMDS = {"ptp-orin": ["/usr/sbin/ptp4l", "-i", ORIN_NIC, "-H", "-s", "-m"],
        "phc2sys-orin": ["/usr/sbin/phc2sys", "-a", "-r", "-m"]}
PTP_OFFSET_OK_NS = 10_000        # eno1 slave: 하드웨어 타임스탬프면 보통 수 µs 이하
PHC2SYS_OFFSET_OK_NS = 20_000    # PC 시계: 보통 수백 ns
SETTLE_SAMPLES = 5               # 이만큼 연달아 기준 안이면 수렴
TIMEOUT_S = 120
GUI_SESSION = Path.home() / ".config/dm_clip_gui/last_session.yaml"
CLOCK_CLASS = {6: "GNSS 에 잠김", 7: "GNSS 잃고 버티는 중 (holdover)", 13: "외부 시계에 잠김", 14: "holdover",
               52: "degraded", 187: "degraded", 248: "자유 시계 (GNSS 아님)", 255: "slave 전용"}

GREEN, YELLOW, RED, DIM, END = "\033[32m", "\033[33m", "\033[31m", "\033[2m", "\033[0m"


def ok(msg):
    print(f"  {GREEN}✓{END} {msg}")


def warn(msg):
    print(f"  {YELLOW}⚠{END} {msg}")


def bad(msg):
    print(f"  {RED}✗{END} {msg}")


def run(cmd, sudo=False, check=False, quiet=False):
    full = (["sudo"] + cmd) if sudo else cmd
    if not quiet:
        print(f"  {DIM}$ {' '.join(full)}{END}")
    return subprocess.run(full, text=True, capture_output=True, check=check)


def processes():
    """[(pid, argv)] — ptp4l · phc2sys 프로세스 (sudo 래퍼는 뺀다)."""
    out = []
    for pid_dir in Path("/proc").iterdir():
        if not pid_dir.name.isdigit():
            continue
        try:
            argv = [a.decode(errors="replace") for a in (pid_dir / "cmdline").read_bytes().split(b"\0") if a]
        except OSError:
            continue
        if argv and Path(argv[0]).name in ("ptp4l", "phc2sys"):
            out.append((int(pid_dir.name), argv))
    return out


def unit_of(pid):
    """그 프로세스가 속한 systemd 서비스 이름 (없으면 '')."""
    try:
        for line in Path(f"/proc/{pid}/cgroup").read_text().splitlines():
            m = re.search(r"/([^/]+)\.service$", line)
            if m:
                return m.group(1)
    except OSError:
        pass
    return ""


def unit_active(unit):
    return run(["systemctl", "is-active", "--quiet", unit], quiet=True).returncode == 0


def unit_installed(unit):
    return Path(f"/etc/systemd/system/{unit}.service").exists()


def journal(unit, since_s=30):
    """지금 돌고 있는 서비스 실행분의 최근 로그 (1줄/초 가정으로 since_s 줄).
    --since 는 못 쓴다 — 시계가 2126 년으로 튀었던 부트의 로그가 journal 에 남아 있어 '-15s' 에 늘 걸려 나온다."""
    inv = run(["systemctl", "show", "-p", "InvocationID", "--value", unit], quiet=True).stdout.strip()
    if not inv:
        return []
    r = run(["journalctl", f"_SYSTEMD_INVOCATION_ID={inv}", "-n", str(since_s * 2), "--no-pager", "-o", "cat"],
            quiet=True)
    return [l for l in r.stdout.splitlines() if not l.startswith("[")]   # -m 이 같은 줄을 '[t] …' 로 한 번 더 찍음


def offsets(lines, pattern):
    """(offset_ns, 상태 s0/s1/s2) 목록."""
    out = []
    for line in lines:
        m = re.search(pattern, line)
        if m:
            out.append((int(m.group(1)), m.group(2)))
    return out


def ptp_state(lines):
    state = ""
    for line in lines:
        m = re.search(r"port 1(?: \([^)]*\))?: \w+ to (\w+)", line)
        if m:
            state = m.group(1)
    return state


def gm_identity(lines):
    gm = ""
    for line in lines:
        m = re.search(r"selected best master clock (\S+)", line)
        if m:
            gm = m.group(1)
    return gm


def clock_class():
    """(clockClass, grandmasterIdentity) — pmc 로 ptp4l 에 물어본다 (root)."""
    r = run(["pmc", "-u", "-b", "0", "GET PARENT_DATA_SET"], sudo=True, quiet=True)
    cc = re.search(r"gm\.ClockClass\s+(\d+)|grandmasterClockClass\s+(\d+)", r.stdout)
    gm = re.search(r"grandmasterIdentity\s+(\S+)", r.stdout)
    value = int(next(g for g in cc.groups() if g)) if cc else None
    return value, (gm.group(1) if gm else "")


def gui_field():
    try:
        import yaml
        data = yaml.safe_load(GUI_SESSION.read_text()) or {}
        return str(((data.get("sensors") or {}).get("flir_cameras") or {}).get("launch_args", {})
                   .get("ptp_master_interface", "") or "").strip()
    except Exception:
        return ""


def camera_nodes_running():
    return any("flir_spinnaker_camera_node" in " ".join(argv) for _, argv in all_argv())


def all_argv():
    for pid_dir in Path("/proc").iterdir():
        if pid_dir.name.isdigit():
            try:
                yield int(pid_dir.name), (pid_dir / "cmdline").read_bytes().decode(errors="replace").split("\0")
            except OSError:
                continue


def report(ask_sudo=True):
    """체인 전체 점검. 문제 없으면 True."""
    good = True
    print("\n[1] Orin → PC (eno1, ptp-orin)")
    procs = processes()
    stray = [(pid, argv) for pid, argv in procs if unit_of(pid) not in UNITS and "/tmp/ptp4l-flir" not in argv]
    if unit_active("ptp-orin"):
        lines = journal("ptp-orin", 3600)
        state, gm = ptp_state(lines), gm_identity(lines)
        offs = offsets(journal("ptp-orin", 20), r"master offset\s+(-?\d+)\s+(s\d)")
        if offs and all(abs(o) < PTP_OFFSET_OK_NS and s == "s2" for o, s in offs[-SETTLE_SAMPLES:]):
            ok(f"SLAVE · grandmaster {gm or '?'} · 최근 offset {', '.join(str(o) for o, _ in offs[-3:])} ns")
        else:
            good = False
            bad(f"아직 안 맞음 (상태 {state or '?'}, 최근 offset {[o for o, _ in offs[-3:]] or '없음'}) — Orin 이 켜져 "
                "grandmaster 로 도는지, eno1 케이블 확인")
    else:
        good = False
        bad("ptp-orin 서비스가 안 돎 — ptp 로 세우세요")
    if ask_sudo and unit_active("ptp-orin"):
        cc, gm = clock_class()
        if cc is None:
            warn("GNSS 잠김 여부(clockClass)를 못 읽음")
        elif cc == 6:
            ok(f"grandmaster clockClass {cc} — {CLOCK_CLASS[cc]}")
        else:
            warn(f"grandmaster clockClass {cc} — {CLOCK_CLASS.get(cc, '?')}. 센서끼리는 맞지만 GNSS 시각은 아닙니다 "
                 "(Orin 의 GNSS · PPS 확인)")

    print("\n[2] PC 시스템 시계 (phc2sys-orin)")
    if unit_active("phc2sys-orin"):
        offs = offsets(journal("phc2sys-orin", 20), r"CLOCK_REALTIME phc offset\s+(-?\d+)\s+(s\d)")
        if offs and all(abs(o) < PHC2SYS_OFFSET_OK_NS and s == "s2" for o, s in offs[-SETTLE_SAMPLES:]):
            ok(f"eno1 시계에 맞춰짐 · 최근 offset {', '.join(str(o) for o, _ in offs[-3:])} ns")
        else:
            good = False
            bad(f"아직 안 맞음 (최근 offset {[o for o, _ in offs[-3:]] or '없음'})")
    else:
        good = False
        bad("phc2sys-orin 서비스가 안 돎 — ptp 로 세우세요")
    ntp = run(["timedatectl", "show", "-p", "NTP", "--value"], quiet=True).stdout.strip()
    if ntp == "yes":
        good = False
        bad("NTP 가 켜져 있음 — phc2sys 와 시스템 시계를 두고 싸웁니다 (ptp 가 끕니다)")
    else:
        ok("NTP 꺼짐")

    print("\n[3] 겹치는 PTP 프로세스")
    if stray:
        good = False
        for pid, argv in stray:
            bad(f"서비스 밖에서 도는 {' '.join(argv)} (pid {pid}) — ptp 가 정리합니다")
    else:
        ok("없음" + ("  (GUI 의 카메라용 ptp4l 은 따로 돎)" if any("/tmp/ptp4l-flir" in a for _, a in procs) else ""))

    print("\n[4] 카메라 쪽")
    nic = Path(f"/sys/class/net/{CAMERA_NIC}")
    mtu = (nic / "mtu").read_text().strip() if nic.exists() else "?"
    try:
        carrier = (nic / "carrier").read_text().strip() == "1"
    except OSError:                      # 링크가 내려가 있으면 carrier 를 읽을 수 없다 (EINVAL)
        carrier = False
    if not carrier:
        # 링크가 없으면 NetworkManager 가 FLIR-LAN 프로필(MTU 9000 · 192.168.1.100)을 아직 안 얹어 MTU 가 1500 으로
        # 보인다 — MTU 가 아니라 링크 문제다. 이 상태로 GUI 를 띄우면 카메라 NIC 가 '빈 유선 NIC' 으로 보여
        # 라이다 NIC 로 짐작된 적이 있다 (2026-09-22 16대 기동 실패).
        bad(f"{CAMERA_NIC} 링크 없음 — 카메라 스위치 전원 · 광 모듈(SFP+)을 확인하고, 링크가 올라온 뒤에 dm 으로 기동하세요")
    else:
        (ok if mtu == "9000" else bad)(f"{CAMERA_NIC} MTU {mtu}" + ("" if mtu == "9000" else " — 9000 이어야 영상 패킷이 안 버려집니다"))
    field = gui_field()
    if field and field.lower() not in ("none", "off", "false", "-"):
        warn(f"GUI 'PTP grandmaster NIC' 가 '{field}' — 비워 두세요 (자동으로 {CAMERA_NIC} 에 소프트웨어 타임스탬프 ptp4l)")
    else:
        ok(f"GUI 'PTP grandmaster NIC' 비어 있음 — 센서 기동 때 GUI 가 {CAMERA_NIC} 에 카메라용 ptp4l 을 띄웁니다")
    if any("/tmp/ptp4l-flir" in a for _, a in procs):
        ok("카메라용 ptp4l 이 돌고 있음 (센서 실행 중)")
    return good


def stop():
    print("PTP 서비스 멈춤")
    for unit in UNITS:
        if unit_active(unit):
            run(["systemctl", "stop", unit], sudo=True)
    print("멈췄습니다.")


def start(args):
    if camera_nodes_running() and not args.yes:
        print(f"{YELLOW}센서가 돌고 있습니다.{END} phc2sys 가 새로 뜨면 PC 시계를 한 번 크게 옮길 수 있고, 그러면 카메라 시계도"
              " 따라 튑니다.")
        if input("그래도 진행할까요? [y/N] ").strip().lower() != "y":
            return 1
    print("sudo 암호를 한 번 묻습니다.")
    if subprocess.run(["sudo", "-v"]).returncode != 0:
        return 1

    print("\n겹치는 PTP 정리")
    for unit in UNITS:
        if unit_active(unit):
            run(["systemctl", "stop", unit], sudo=True)
        run(["systemctl", "reset-failed", unit], sudo=True, quiet=True)
    for pid, argv in processes():
        if "/tmp/ptp4l-flir" in argv:          # GUI 가 띄운 카메라용 ptp4l — 건드리지 않는다
            continue
        print(f"  멈춤: {' '.join(argv)} (pid {pid})")
        run(["kill", str(pid)], sudo=True, quiet=True)
    time.sleep(1)

    if run(["timedatectl", "show", "-p", "NTP", "--value"], quiet=True).stdout.strip() == "yes":
        print("\nNTP 끔 (phc2sys 와 겹침)")
        run(["timedatectl", "set-ntp", "false"], sudo=True)

    print("\n서비스 시작")
    for unit in UNITS:
        if unit_installed(unit):
            run(["systemctl", "restart", unit], sudo=True)
        else:
            run(["systemd-run", f"--unit={unit}", "--collect", "--property=Restart=on-failure",
                 "--property=RestartSec=2"] + CMDS[unit], sudo=True)
        if unit == "ptp-orin":
            time.sleep(2)                          # phc2sys -a 가 ptp4l 소켓에 붙을 수 있게

    print(f"\n수렴 기다림 (최대 {TIMEOUT_S}초)", end="", flush=True)
    t0 = time.time()
    while time.time() - t0 < TIMEOUT_S:
        time.sleep(3)
        p = offsets(journal("ptp-orin", 15), r"master offset\s+(-?\d+)\s+(s\d)")
        c = offsets(journal("phc2sys-orin", 15), r"CLOCK_REALTIME phc offset\s+(-?\d+)\s+(s\d)")
        p_ok = len(p) >= SETTLE_SAMPLES and all(abs(o) < PTP_OFFSET_OK_NS and s == "s2" for o, s in p[-SETTLE_SAMPLES:])
        c_ok = len(c) >= SETTLE_SAMPLES and all(abs(o) < PHC2SYS_OFFSET_OK_NS and s == "s2" for o, s in c[-SETTLE_SAMPLES:])
        last_p = p[-1][0] if p else "—"
        last_c = c[-1][0] if c else "—"
        print(f"\r수렴 기다림 {time.time() - t0:4.0f}s   eno1 offset {last_p} ns   PC 시계 offset {last_c} ns      ",
              end="", flush=True)
        if p_ok and c_ok:
            break
    print()
    good = report(ask_sudo=True)
    print()
    if good:
        print(f"{GREEN}준비됐습니다.{END} 이제 dm 으로 센서를 기동하세요 (런치 로그 맨 위 "
              f"'[GUI] PTP grandmaster: … {CAMERA_NIC} 에서 ptp4l 을 띄웁니다' 확인).")
        return 0
    print(f"{RED}아직 준비 안 됨{END} — 위 ✗ 항목을 보세요. 로그: journalctl -u ptp-orin -f  /  journalctl -u phc2sys-orin -f")
    return 1


def main():
    parser = argparse.ArgumentParser(description="GNSS(Orin) → PC → 카메라 PTP 체인 세우기 · 점검")
    parser.add_argument("action", nargs="?", default="start", choices=["start", "status", "stop"])
    parser.add_argument("-y", "--yes", action="store_true", help="센서가 돌고 있어도 묻지 않고 진행")
    parser.add_argument("--no-sudo", action="store_true", help="status 에서 sudo 가 필요한 GNSS 잠김 확인을 건너뜀")
    args = parser.parse_args()
    if args.action == "stop":
        stop()
        return 0
    if args.action == "status":
        return 0 if report(ask_sudo=not args.no_sudo) else 1
    return start(args)


if __name__ == "__main__":
    sys.exit(main())
