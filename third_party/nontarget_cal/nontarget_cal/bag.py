"""Streaming reader for DM rosbag2 recordings (sqlite3 storage).

Adapted from /hdd/DM_calib/online/dmbag.py (same deserialisation, same PointCloud2 decoding).
One recording = a directory with `metadata.yaml` and one or more `*_N.db3` files (split bags are
read as one: every query runs over all files in order). Messages are read straight out of the
sqlite `messages` table; queries that can use the timestamp index are preferred, because a scan
by topic over a 180 GB recording touches every row.
"""
from __future__ import annotations

import sqlite3
from pathlib import Path

import numpy as np
import yaml
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

from .config import pkg_data

TS = get_typestore(Stores.ROS2_HUMBLE)
_FLIR = pkg_data("FlirMetadata.msg")
if _FLIR.exists():
    TS.register(get_types_from_msg(_FLIR.read_text(), "flir_spinnaker_camera/msg/FlirMetadata"))

_PC2_DT = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
           5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def header_ns(msg) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def bag_files(path: Path) -> list[Path]:
    path = Path(path)
    if path.suffix == ".db3":
        return [path]
    dbs = sorted(path.glob("*.db3"), key=lambda p: (len(p.name), p.name))
    if not dbs:
        raise FileNotFoundError(f"no .db3 under {path}")
    return dbs


class Bag:
    """A rosbag2 sqlite3 recording (possibly split). `topics`: name -> (id, type) of the FIRST file;
    ids are re-resolved per file."""

    def __init__(self, path):
        self.path = Path(path)
        self.files = bag_files(self.path)
        self.cons = [sqlite3.connect(f"file:{f}?mode=ro", uri=True) for f in self.files]
        self._topics = []
        for con in self.cons:
            self._topics.append({n: (i, t) for i, n, t in con.execute("select id,name,type from topics")})
        self.topics = {}
        for tp in self._topics:
            for n, v in tp.items():
                self.topics.setdefault(n, v)
        self.types = {n: v[1] for n, v in self.topics.items()}
        # compatibility with code written for one file
        self.con = self.cons[0]

    # ------------------------------------------------------------------ metadata
    def metadata(self) -> dict:
        m = self.path / "metadata.yaml" if self.path.is_dir() else self.path.parent / "metadata.yaml"
        if m.exists():
            return yaml.safe_load(m.read_text()).get("rosbag2_bagfile_information", {})
        return {}

    def counts(self) -> dict:
        """Message count per topic, from metadata.yaml when present (no table scan)."""
        md = self.metadata()
        out = {}
        for t in md.get("topics_with_message_count", []) or []:
            out[t["topic_metadata"]["name"]] = int(t["message_count"])
        return out

    def time_range(self) -> tuple[int, int]:
        lo, hi = None, None
        for con in self.cons:
            a, b = con.execute("select min(timestamp), max(timestamp) from messages").fetchone()
            if a is not None:
                lo = a if lo is None else min(lo, a)
                hi = b if hi is None else max(hi, b)
        return int(lo), int(hi)

    # ------------------------------------------------------------------ queries
    def _ids(self, k: int, names) -> dict:
        tp = self._topics[k]
        return {tp[n][0]: n for n in names if n in tp}

    def rows(self, names, t0: int | None = None, t1: int | None = None):
        """(topic name, bag time, raw cdr) of the given topics in [t0, t1], timestamp order per file."""
        for k, con in enumerate(self.cons):
            ids = self._ids(k, names)
            if not ids:
                continue
            q = "select topic_id, timestamp, data from messages where "
            args = []
            if t0 is not None:
                q += "timestamp >= ? and timestamp <= ? and "
                args += [int(t0), int(t1)]
            q += f"topic_id in ({','.join('?' * len(ids))}) order by timestamp"
            for tid, t, d in con.execute(q, (*args, *ids)):
                yield ids[tid], t, d

    def msgs(self, topic: str, limit: int = -1):
        """(bag time ns, deserialized message) in bag order."""
        n = 0
        typ = self.types[topic]
        for k, con in enumerate(self.cons):
            ids = self._ids(k, [topic])
            if not ids:
                continue
            tid = next(iter(ids))
            cur = con.execute("select timestamp,data from messages where topic_id=? order by timestamp limit ?",
                              (tid, -1 if limit < 0 else limit - n))
            for t, d in cur:
                yield t, TS.deserialize_cdr(d, typ)
                n += 1
                if 0 <= limit <= n:
                    return

    def first(self, topic: str):
        """First message of a topic, walking the timestamp index (fast even on huge bags)."""
        typ = self.types[topic]
        for k, con in enumerate(self.cons):
            ids = self._ids(k, [topic])
            if not ids:
                continue
            r = con.execute("select timestamp,data from messages where topic_id=? order by timestamp limit 1",
                            (next(iter(ids)),)).fetchone()
            if r:
                return r[0], TS.deserialize_cdr(r[1], typ)
        return None

    def deserialize(self, topic: str, data: bytes):
        return TS.deserialize_cdr(data, self.types[topic])

    def close(self):
        for c in self.cons:
            c.close()


def pointcloud(msg) -> np.ndarray:
    """PointCloud2 -> structured (height, width) array with the sensor's own fields."""
    dt = np.dtype({"names": [f.name for f in msg.fields],
                   "formats": [_PC2_DT[f.datatype] for f in msg.fields],
                   "offsets": [f.offset for f in msg.fields],
                   "itemsize": msg.point_step})
    return np.frombuffer(bytes(msg.data), dt).reshape(msg.height, msg.width)


SWEEP_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"),
                        ("ring", "<u2"), ("t", "<f4")])


def sweep_from_msg(msg) -> np.ndarray:
    """Ouster PointCloud2 -> the pipeline's sweep dtype (verbatim from online/extract_dm.py).
    `reflectivity` goes into the `intensity` slot; zero-range returns are dropped."""
    a = pointcloud(msg).reshape(-1)
    r2 = a["x"].astype(np.float64) ** 2 + a["y"].astype(np.float64) ** 2 + a["z"].astype(np.float64) ** 2
    keep = np.isfinite(r2) & (r2 > 0.25)
    a = a[keep]
    out = np.empty(len(a), dtype=SWEEP_DTYPE)
    out["x"], out["y"], out["z"] = a["x"], a["y"], a["z"]
    out["intensity"] = a["reflectivity"].astype(np.float32)
    out["ring"] = a["ring"].astype(np.uint16)
    out["t"] = a["t"].astype(np.float64) * 1e-9         # u32 ns from the sweep start
    return out
