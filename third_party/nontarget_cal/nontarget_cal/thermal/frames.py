"""Extracted 16-bit thermal frames of one window and camera.

Stored as ONE pack file per window and camera (thermal16/<win>/<cam>.pack: the PNG encodings back to back,
thermal16/<win>/<cam>.pack.idx.npy: header_ns, offset, length) instead of ~1200 small PNG files: the
tracker, the stereo links and the edge term each read most frames of a window, and on a disk shared by
several processes 1200 file opens + seeks cost more than the decoding. The frames are the same PNG
bytes (cv2.imdecode = cv2.imread); work directories with per-frame PNG files are still read.
"""
from __future__ import annotations

import os
from pathlib import Path

import cv2
import numpy as np


class PackWriter:
    def __init__(self, d: Path, cam: str, compression: int = 1):
        self.path = Path(d) / f"{cam}.pack"
        self.f = open(self.path, "wb")
        self.rows = []
        self.params = [cv2.IMWRITE_PNG_COMPRESSION, int(compression)]

    def add(self, header_ns: int, raw: np.ndarray):
        ok, buf = cv2.imencode(".png", raw, self.params)
        if not ok:
            raise RuntimeError("PNG encoding failed")
        b = buf.tobytes()
        self.rows.append((int(header_ns), self.f.tell(), len(b)))
        self.f.write(b)

    def close(self):
        self.f.close()
        idx = np.array(sorted(self.rows), np.int64).reshape(-1, 3)
        np.save(str(self.path) + ".idx.npy", idx)


class ThermalFrames:
    """headers (sorted int64) and read(h) / read_all(hs) -> uint16 images, from a pack or PNG files."""

    def __init__(self, win_dir: Path, cam: str):
        self.dir = Path(win_dir)
        self.cam = cam
        p = self.dir / f"{cam}.pack"
        if p.exists() and Path(str(p) + ".idx.npy").exists():
            self.pack = p
            self.idx = np.load(str(p) + ".idx.npy")
            self.headers = self.idx[:, 0]
            self._row = {int(h): i for i, h in enumerate(self.headers)}
            self._buf = None
        else:
            self.pack = None
            self.headers = np.array(sorted(int(f.stem) for f in (self.dir / cam).glob("*.png")), np.int64)

    def __contains__(self, h):
        return (int(h) in self._row) if self.pack is not None else (self.dir / self.cam / f"{int(h)}.png").exists()

    def _bytes(self, i):
        o, n = int(self.idx[i, 1]), int(self.idx[i, 2])
        if self._buf is not None:
            return self._buf[o:o + n]
        with open(self.pack, "rb") as f:
            f.seek(o)
            return np.frombuffer(f.read(n), np.uint8)

    def load_all(self):
        """Read the whole pack sequentially once (for consumers of most frames)."""
        if self.pack is not None and self._buf is None:
            self._buf = np.fromfile(self.pack, np.uint8)
        return self

    def read(self, h) -> np.ndarray:
        if self.pack is None:
            return cv2.imread(str(self.dir / self.cam / f"{int(h)}.png"), cv2.IMREAD_UNCHANGED)
        return cv2.imdecode(self._bytes(self._row[int(h)]), cv2.IMREAD_UNCHANGED)


def pack_existing(win_dir: Path, cam: str, remove: bool = False) -> bool:
    """Convert a window's per-frame PNG files of `cam` into a pack (same bytes)."""
    d = Path(win_dir)
    files = sorted((d / cam).glob("*.png"), key=lambda f: int(f.stem))
    if not files or (d / f"{cam}.pack").exists():
        return False
    tmp = d / f"{cam}.pack.tmp"
    rows = []
    with open(tmp, "wb") as f:
        for p in files:
            b = p.read_bytes()
            rows.append((int(p.stem), f.tell(), len(b)))
            f.write(b)
    np.save(str(d / f"{cam}.pack") + ".idx.npy", np.array(rows, np.int64))
    os.replace(tmp, d / f"{cam}.pack")
    if remove:
        for p in files:
            p.unlink()
    return True
