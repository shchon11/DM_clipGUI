"""Camera name mapping: which topic namespace of a bag is which physical camera.

The vehicle has renamed its camera topics before (2026-09-24: same cameras, permuted names). The
calibration of camera X attached to camera Y's images would be silently wrong, so the mapping is
established and then VERIFIED geometrically; the tool never guesses silently.

Order of attempts per bag:
  1. `--name-map FILE` (either the camera_name_map_20260924.yaml format {cameras: {ns: {old: name}},
     thermal: {...}} or a plain {canonical_name: namespace});
  2. identity: every canonical name has its own topics;
  3. the known maps of the config (paths.known_name_maps) whose namespaces all exist in the bag;
  4. geometric identification (below) when names are unknown.
Every mapping (1-3) is then verified geometrically; a contradiction is a refusal. If the geometry is
inconclusive (too few matches, e.g. black frames) a mapping from 1-3 is kept with a warning; an
unknown bag is refused.

Geometric identification (the method behind camera_name_map_20260924.yaml):
  * simultaneous frames of all RGB cameras at a few moving moments; SIFT matches between every pair,
    rays from a nominal fisheye lens; one essential matrix per pair pooled over the moments -> the
    relative rotation R_ij and the baseline direction t_ij between the two cameras;
  * absolute rotations up to a global rotation by chaining R_ij over a maximum spanning tree
    (weights = inliers); for each hypothesis "the root is camera k" the rotations are placed in the
    LiDAR frame with the reference rig's R_k and matched to the reference rotations of the canonical
    names (Hungarian assignment); the hypothesis with the lowest cost wins;
  * cameras whose reference rotations are within `rot_ambiguity_deg` of each other (front2/5/8 all
    look straight ahead) are resolved by the SIGN of the measured baselines against the reference
    rig's baselines, over all permutations of the group;
  * accepted only if every camera matches within `rot_tol_deg` and every decision has a margin
    (second best clearly worse). Otherwise: refusal listing the candidates.
  * thermal pair: which of the two thermal namespaces is LEFT, from the baseline sign between them.
The reference rig is the --init calibration when given, else data/rig_design.yaml (the validated
2026-09-24 product of this vehicle).
"""
from __future__ import annotations

import itertools
import json
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation as Rot

from . import errors as E
from .bag import Bag, header_ns

ROT_TOL_DEG = 6.0
ROT_AMBIG_DEG = 6.0
MIN_INLIERS = 40


# ------------------------------------------------------------------ mapping files
def load_map_file(path: Path) -> dict:
    """-> {"rgb": {canonical: namespace}, "thermal": {canonical: namespace}}"""
    y = yaml.safe_load(Path(path).read_text()) or {}
    out = {"rgb": {}, "thermal": {}}
    if "cameras" in y or "thermal" in y:
        for ns, v in (y.get("cameras") or {}).items():
            out["rgb"][v["old"] if isinstance(v, dict) else v] = ns
        for ns, v in (y.get("thermal") or {}).items():
            out["thermal"][v["old"] if isinstance(v, dict) else v] = ns
    else:
        for k, v in y.items():
            (out["thermal"] if k.startswith("thermal") else out["rgb"])[k] = v
    return out


def _has(bag: Bag, cfg, kind: str, ns: str) -> bool:
    tp = cfg["topics"]
    if kind == "rgb":
        return tp["rgb_image"].format(name=ns) in bag.topics and tp["rgb_meta"].format(name=ns) in bag.topics
    return tp["thermal_image"].format(name=ns) in bag.topics


def namespaces(bag: Bag, cfg) -> dict:
    """All camera namespaces of a bag, by kind (RGB = has image_rgb/compressed; thermal = mono16 image_raw
    without an image_rgb twin)."""
    rgb, th = [], []
    for t, typ in bag.types.items():
        if t.endswith("/image_rgb/compressed"):
            rgb.append(t.split("/")[1])
    for t, typ in bag.types.items():
        if t.endswith("/image_raw") and typ.endswith("Image"):
            ns = t.split("/")[1]
            if ns not in rgb:
                th.append(ns)
    return {"rgb": sorted(rgb), "thermal": sorted(th)}


def candidate_mapping(bag: Bag, cfg, sensors, name_map: Path | None, ev):
    """Steps 1-3. Returns (mapping, source) or (None, reason)."""
    want = {"rgb": list(cfg["cameras"]["rgb"]) if "rgb" in sensors else [],
            "thermal": list(cfg["cameras"]["thermal"]) if "thermal" in sensors else []}
    if name_map is not None:
        m = load_map_file(name_map)
        return m, f"explicit {name_map}"
    ident = {k: {c: c for c in v} for k, v in want.items()}
    if all(_has(bag, cfg, k, c) for k, v in want.items() for c in v):
        return ident, "identity (topics carry the canonical names)"
    for p in cfg["paths"].get("known_name_maps", []):
        if not Path(p).exists():
            continue
        m = load_map_file(p)
        ok = all(c in m[k] and _has(bag, cfg, k, m[k][c]) for k, v in want.items() for c in v)
        if ok:
            return m, f"known map {p}"
    return None, "no explicit, identity or known map fits the bag's topics"


# ------------------------------------------------------------------ geometry
def _kb_unproject(uv, K):
    fx, fy, cx, cy, k1, k2 = K[:6]
    D = np.array([k1, k2, 0.0, 0.0])
    Km = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    n = cv2.fisheye.undistortPoints(uv.reshape(-1, 1, 2).astype(np.float64), Km, D).reshape(-1, 2)
    return n


def _pinhole_unproject(uv, K):
    fx, fy, cx, cy, k1, k2 = K[:6]
    Km = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    return cv2.undistortPoints(uv.reshape(-1, 1, 2).astype(np.float64), Km, np.array([k1, k2, 0, 0])).reshape(-1, 2)


def grab_frames(bag: Bag, cfg, kind: str, nss, times_ns, max_dt_ms=20.0):
    """For each moment: {ns: grey image} of frames within max_dt_ms of the moment (simultaneous)."""
    tp = cfg["topics"]
    topics = {(tp["rgb_image"] if kind == "rgb" else tp["thermal_image"]).format(name=ns): ns for ns in nss}
    out = []
    for t in times_ns:
        best = {}
        for name, _, data in bag.rows(list(topics), t, t + 150_000_000):
            ns = topics[name]
            if ns in best:
                continue
            m = bag.deserialize(name, data)
            best[ns] = (header_ns(m), m)
            if len(best) == len(topics):
                break
        if len(best) < 2:
            continue
        h0 = np.median([v[0] for v in best.values()])
        frame = {}
        for ns, (h, m) in best.items():
            if abs(h - h0) > max_dt_ms * 1e6:
                continue
            if kind == "rgb":
                img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_GRAYSCALE)
                if img.mean() < 60:
                    img = cv2.createCLAHE(3.0, (8, 8)).apply(img)
            else:
                raw = np.frombuffer(bytes(m.data), np.uint16).reshape(m.height, m.width).astype(np.float32)
                lo, hi = np.percentile(raw, [1, 99.5])
                img = cv2.createCLAHE(3.0, (8, 8)).apply(np.clip((raw - lo) / max(hi - lo, 1) * 255, 0, 255).astype(np.uint8))
            frame[ns] = img
        out.append(frame)
    return out


def pair_geometry(frames, K_of, unproject, min_inliers=MIN_INLIERS, thr=None):
    """Pooled essential matrix per camera pair -> {(a, b): (R_ab, t_ab, inliers)} with
    x_b = R_ab x_a + t_ab (unit t, direction of a's centre seen from b... in b's frame)."""
    sift = cv2.SIFT_create(3000)
    feats = []
    for fr in frames:
        f = {}
        for ns, img in fr.items():
            kp, des = sift.detectAndCompute(img, None)
            if des is not None and len(kp) > 20:
                f[ns] = (np.array([k.pt for k in kp], np.float64), des)
        feats.append(f)
    nss = sorted({ns for f in feats for ns in f})
    bf = cv2.BFMatcher(cv2.NORM_L2)
    out = {}
    for a, b in itertools.combinations(nss, 2):
        pa, pb = [], []
        for f in feats:
            if a not in f or b not in f:
                continue
            m = bf.knnMatch(f[a][1], f[b][1], k=2)
            good = [x[0] for x in m if len(x) == 2 and x[0].distance < 0.75 * x[1].distance]
            if len(good) < 8:
                continue
            pa.append(f[a][0][[g.queryIdx for g in good]]); pb.append(f[b][0][[g.trainIdx for g in good]])
        if not pa:
            continue
        pa, pb = np.concatenate(pa), np.concatenate(pb)
        if len(pa) < min_inliers:
            continue
        na, nb = unproject(pa, K_of(a)), unproject(pb, K_of(b))
        th = thr if thr is not None else 1.5 / K_of(a)[0]
        Em, msk = cv2.findEssentialMat(na, nb, np.eye(3), method=cv2.RANSAC, prob=0.999, threshold=th)
        if Em is None:
            continue
        if Em.shape[0] > 3:
            Em = Em[:3]
        n, R, t, msk2 = cv2.recoverPose(Em, na, nb, np.eye(3), mask=msk)
        if n < min_inliers:
            continue
        out[(a, b)] = (R, t.ravel() / np.linalg.norm(t), int(n))
    return out


def _angle(R):
    return float(np.degrees(np.linalg.norm(Rot.from_matrix(R).as_rotvec())))


def spanning_rotations(pairs, nodes):
    """Absolute rotations R_Ci (camera i -> root camera frame) by chaining over a maximum spanning
    tree of the pair graph (weights = inliers). Returns {ns: R_root_i} for the root's component."""
    edges = sorted(((v[2], a, b) for (a, b), v in pairs.items()), reverse=True)
    parent = {n: n for n in nodes}

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    adj = {n: [] for n in nodes}
    for w, a, b in edges:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb
            adj[a].append(b); adj[b].append(a)
    comps = {}
    for n in nodes:
        comps.setdefault(find(n), []).append(n)
    root = max(comps.values(), key=len)
    r0 = max(root, key=lambda n: len(adj[n]))
    R = {r0: np.eye(3)}
    stack = [r0]
    while stack:
        a = stack.pop()
        for b in adj[a]:
            if b in R:
                continue
            if (a, b) in pairs:
                Rab = pairs[(a, b)][0]              # x_b = Rab x_a  -> R_a_b (b -> a) = Rab^T
                R[b] = R[a] @ Rab.T
            else:
                Rba = pairs[(b, a)][0]              # x_a = Rba x_b
                R[b] = R[a] @ Rba
            stack.append(b)
    return R


def identify_rgb(pairs, nss, ref: dict, rot_tol=ROT_TOL_DEG, ambig=ROT_AMBIG_DEG):
    """ref: {canonical: T_lidar_cam (4x4)}. Returns (mapping {canonical: ns}, report)."""
    Rabs = spanning_rotations(pairs, nss)
    got = sorted(Rabs)
    names = sorted(ref)
    Rref = {c: np.asarray(ref[c])[:3, :3] for c in names}
    best = None
    for root in names:                     # hypothesis: the tree root is canonical `root`
        r0 = [n for n in got if np.allclose(Rabs[n], np.eye(3))][0]
        C = np.zeros((len(got), len(names)))
        for i, n in enumerate(got):
            R_L_n = Rref[root] @ Rabs[n]        # (n -> root) then (root -> L)
            for j, c in enumerate(names):
                C[i, j] = _angle(R_L_n.T @ Rref[c])
        ri, cj = linear_sum_assignment(C)
        cost = C[ri, cj].sum()
        if best is None or cost < best[0]:
            best = (cost, root, C, ri, cj, r0)
    cost, root, C, ri, cj, r0 = best
    assign = {got[i]: names[j] for i, j in zip(ri, cj)}
    rep = {"root_hypothesis": root, "per_camera": {}, "unresolved": [], "not_measured": sorted(set(nss) - set(got))}
    # ambiguity groups: canonical names whose reference rotations are within `ambig` of each other
    groups = []
    for c in names:
        g = sorted(d for d in names if _angle(Rref[c].T @ Rref[d]) < ambig)
        if len(g) > 1 and g not in groups:
            groups.append(g)
    for i, n in enumerate(got):
        row = C[i]
        j = names.index(assign[n])
        others = [row[k] for k in range(len(names)) if k != j and not any(assign[n] in g and names[k] in g for g in groups)]
        rep["per_camera"][n] = {"canonical": assign[n], "rot_err_deg": round(float(row[j]), 2),
                                "next_distinct_deg": round(float(min(others)), 2) if others else None}
        if row[j] > rot_tol:
            rep["unresolved"].append(f"{n}: best {assign[n]} at {row[j]:.1f} deg > {rot_tol} deg")
    # resolve groups by the baseline signs
    ref_c = {c: np.asarray(ref[c])[:3, 3] for c in names}
    for g in groups:
        members = [n for n, c in assign.items() if c in g]
        if len(members) < 2:
            continue
        scores = []
        for perm in itertools.permutations(g, len(members)):
            hyp = dict(assign)
            for n, c in zip(members, perm):
                hyp[n] = c
            angs = []
            for (a, b), (R, t, w) in pairs.items():
                if a not in hyp or b not in hyp or not ({a, b} & set(members)):
                    continue
                ca, cb = hyp[a], hyp[b]
                # x_b = R_b^T R_a x_a + R_b^T (c_a - c_b): the measured unit t is a's centre seen from b
                tb = Rref[cb].T @ (ref_c[ca] - ref_c[cb])
                if np.linalg.norm(tb) < 0.02:
                    continue
                tb = tb / np.linalg.norm(tb)
                angs.append(float(np.degrees(np.arccos(np.clip(t @ tb, -1, 1)))))
            # mean of angles clipped at 90 deg: pairs not involving a swapped member are common to all
            # hypotheses, so a median would hide the difference; the clip bounds single bad pairs
            scores.append((float(np.mean(np.minimum(angs, 90.0))) if angs else 180.0, perm, len(angs)))
        scores.sort()
        rep.setdefault("groups", []).append({"group": g, "members": members,
                                             "scores_deg": [(round(x[0], 1), list(x[1]), x[2]) for x in scores[:3]]})
        if len(scores) > 1 and not (scores[0][0] < 30 and scores[1][0] - scores[0][0] > 10 and scores[0][2] > 0):
            rep["unresolved"].append(f"group {g}: baseline signs do not separate {members} "
                                     f"(best {scores[0][0]:.0f} deg, next {scores[1][0]:.0f} deg)")
        for n, c in zip(members, scores[0][1]):
            assign[n] = c
    mapping = {c: n for n, c in assign.items()}
    return mapping, rep


def identify_thermal(pairs, nss, ref_left_right=("thermal_left", "thermal_right")):
    """Two thermal cameras looking forward: the LEFT one's centre lies at -x (image left) of the right one."""
    if len(nss) != 2 or not pairs:
        return None, {"unresolved": ["thermal pair: no geometry"]}
    (a, b), (R, t, w) = next(iter(pairs.items()))
    # x_b = R x_a + t: the centre of a (x_a = 0) is at t in b's frame, so a lies at image-x sign(t[0]) of b.
    a_left = t[0] < 0
    left, right = (a, b) if a_left else (b, a)
    rep = {"pair": [a, b], "t_a_in_b": np.round(t, 3).tolist(), "inliers": w,
           "unresolved": [] if abs(t[0]) > 0.5 else [f"thermal baseline mostly along z/y ({np.round(t, 2).tolist()})"]}
    return {ref_left_right[0]: left, ref_left_right[1]: right}, rep


def moments(bag: Bag, series_t, series_speed, n=6):
    """Moving moments spread over the bag."""
    t = np.asarray(series_t)
    s = np.nan_to_num(np.asarray(series_speed))
    mv = t[s > 3.0] if (s > 3.0).sum() > n else t
    if not len(mv):
        return []
    return [int(x) for x in mv[np.linspace(0, len(mv) - 1, n + 2).astype(int)[1:-1]]]


def resolve_names(bag: Bag, bag_id: str, cfg, sensors, name_map, series, reference: dict, ev,
                  verify: bool = True, workdir: Path | None = None) -> dict:
    """Returns {"rgb": {canonical: ns}, "thermal": {...}, "source": str, "verification": {...}}; raises Refusal."""
    cand, src = candidate_mapping(bag, cfg, sensors, name_map, ev)
    allns = namespaces(bag, cfg)
    design = yaml.safe_load(Path(cfg["paths"]["rig_design"]).read_text())
    seed = {c: design["rgb"][c]["seed_intr_kb"] for c in design["rgb"]}
    Kmean = np.mean([seed[c] for c in seed], 0)
    ver = {"source": src}
    times = moments(bag, series["t"], series["speed"], n=cfg.get("names", {}).get("moments", 6))
    need_geo = cand is None or verify
    mapping = {"rgb": {}, "thermal": {}}
    if "rgb" in sensors and need_geo:
        nss = allns["rgb"]
        frames = grab_frames(bag, cfg, "rgb", nss, times)
        pairs = pair_geometry(frames, lambda ns: Kmean, _kb_unproject)
        ref = {c: reference[c] for c in cfg["cameras"]["rgb"] if c in reference}
        geo, rep = identify_rgb(pairs, nss, ref)
        rep["pairs_measured"] = len(pairs)
        ver["rgb"] = rep
        if cand is None:
            missing = [c for c in cfg["cameras"]["rgb"] if c not in geo]
            if rep["unresolved"] or missing:
                raise E.Refusal(E.AMBIGUOUS_NAMES if not missing else E.UNKNOWN_NAMES,
                                f"{bag_id}: camera names are not known and the geometry does not identify them "
                                f"unambiguously: {rep['unresolved'] + [f'no geometry for {m}' for m in missing]}. "
                                "Give a mapping with --name-map.",
                                f"{bag_id}: 카메라 토픽 이름을 알 수 없고 영상 기하로도 확정할 수 없음: "
                                f"{rep['unresolved'] + [f'{m} 기하 없음' for m in missing]}. --name-map으로 대응표를 주세요.",
                                candidates=rep)
            mapping["rgb"] = {c: geo[c] for c in cfg["cameras"]["rgb"]}
            ev.warn("names_from_geometry", f"{bag_id}: camera names identified from image geometry "
                    f"(no map fitted): {mapping['rgb']}", f"{bag_id}: 카메라 이름을 영상 기하로 식별함")
        else:
            mapping["rgb"] = {c: cand["rgb"][c] for c in cfg["cameras"]["rgb"]}
            measured = [c for c in mapping["rgb"] if mapping["rgb"][c] in rep["per_camera"]]
            contra = [f"{c}: map says {mapping['rgb'][c]}, geometry says {geo.get(c)}"
                      for c in measured if geo.get(c) != mapping["rgb"][c]]
            ver["rgb"]["contradictions"] = contra
            if contra and not rep["unresolved"]:
                raise E.Refusal(E.AMBIGUOUS_NAMES, f"{bag_id}: the camera name map ({src}) contradicts the image "
                                f"geometry: {contra}. Was the vehicle's naming changed? Check and pass --name-map.",
                                f"{bag_id}: 카메라 이름 대응표({src})가 영상 기하와 모순됨: {contra}. 차량의 토픽 이름이 "
                                "바뀌었는지 확인하고 --name-map으로 지정하세요.", candidates=rep)
            if contra or rep["unresolved"] or len(measured) < len(mapping["rgb"]):
                ev.warn("names_unverified", f"{bag_id}: name map {src} only partly verified by geometry "
                        f"({len(measured)} cameras measured; unresolved: {rep['unresolved']}; contradictions: {contra})",
                        f"{bag_id}: 이름 대응표를 영상 기하로 일부만 확인함")
    elif "rgb" in sensors:
        mapping["rgb"] = {c: cand["rgb"][c] for c in cfg["cameras"]["rgb"]}
    if "thermal" in sensors:
        tns = allns["thermal"]
        if cand is not None and all(c in cand["thermal"] for c in cfg["cameras"]["thermal"]):
            mapping["thermal"] = {c: cand["thermal"][c] for c in cfg["cameras"]["thermal"]}
        if need_geo and len(tns) == 2:
            fr = grab_frames(bag, cfg, "thermal", tns, times + [t + 2_000_000_000 for t in times])
            pairs = pair_geometry(fr, lambda ns: np.array([680.0, 680.0, 320.0, 240.0, 0.0, 0.0]), _pinhole_unproject,
                                  min_inliers=20)
            geo, rep = identify_thermal(pairs, tns)
            ver["thermal"] = rep
            if not mapping["thermal"]:
                if geo is None or rep["unresolved"]:
                    raise E.Refusal(E.AMBIGUOUS_NAMES, f"{bag_id}: thermal camera names unknown and not resolvable "
                                    f"from geometry ({rep['unresolved']}); pass --name-map",
                                    f"{bag_id}: 열화상 카메라 이름을 알 수 없고 기하로도 확정 못함; --name-map 필요")
                mapping["thermal"] = geo
            elif geo is not None and not rep["unresolved"] and geo != mapping["thermal"]:
                raise E.Refusal(E.AMBIGUOUS_NAMES, f"{bag_id}: thermal map {mapping['thermal']} contradicts the "
                                f"geometry {geo}", f"{bag_id}: 열화상 이름 대응표가 기하와 모순됨: {mapping['thermal']} vs {geo}")
        if not mapping["thermal"]:
            raise E.Refusal(E.UNKNOWN_NAMES, f"{bag_id}: thermal cameras not found (namespaces {tns})",
                            f"{bag_id}: 열화상 카메라를 찾지 못함 ({tns})")
    missing = [(k, c, ns) for k in ("rgb", "thermal") for c, ns in mapping[k].items() if not _has(bag, cfg, k, ns)]
    if missing:
        raise E.Refusal(E.MISSING_TOPIC, f"{bag_id}: mapped topics missing: {missing}",
                        f"{bag_id}: 대응된 토픽이 bag에 없음: {missing}")
    mapping["source"] = src
    mapping["verification"] = ver
    return mapping
