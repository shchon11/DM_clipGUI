"""Small exact helpers compiled with numba (optional: without numba the numpy versions run).

first_occurrence(keys): indices of the first occurrence of every distinct key, ascending. Same result
as `np.unique(keys, return_index=True)[1]` sorted (what lotraj.voxel_down does), with a hash table in
O(n) instead of a stable O(n log n) sort.
"""
from __future__ import annotations

import os

import numpy as np

os.environ.setdefault("NUMBA_THREADING_LAYER", "omp")
try:
    import numba as nb
except Exception:  # noqa  (numba missing or incompatible with this numpy)
    nb = None


def _first_occurrence_np(keys):
    _, idx = np.unique(keys, return_index=True)
    idx.sort()
    return idx


if nb is not None:
    @nb.njit(cache=True)
    def _first_occurrence_nb(keys):
        n = keys.shape[0]
        cap = 1
        bits = 0
        while cap < 2 * n + 1:
            cap *= 2
            bits += 1
        mask = cap - 1
        shift = np.uint64(64 - bits)
        mult = np.uint64(11400714819323198485)                   # 2^64 / golden ratio (Fibonacci hashing)
        table = np.empty(cap, np.int64)
        used = np.zeros(cap, np.bool_)
        out = np.empty(n, np.int64)
        m = 0
        for i in range(n):
            k = keys[i]
            h = np.int64((np.uint64(k) * mult) >> shift)          # high bits of the product
            while True:
                if not used[h]:
                    used[h] = True
                    table[h] = k
                    out[m] = i
                    m += 1
                    break
                if table[h] == k:
                    break
                h = (h + 1) & mask
        return out[:m]


if nb is not None:
    @nb.njit(cache=True)
    def _first_index_nb(keys):
        n = keys.shape[0]
        cap = 1
        bits = 0
        while cap < 2 * n + 1:
            cap *= 2
            bits += 1
        mask = cap - 1
        shift = np.uint64(64 - bits)
        mult = np.uint64(11400714819323198485)
        table = np.empty(cap, np.int64)
        first = np.empty(cap, np.int64)
        used = np.zeros(cap, np.bool_)
        rep = np.empty(n, np.int64)
        for i in range(n):
            k = keys[i]
            h = np.int64((np.uint64(k) * mult) >> shift)
            while True:
                if not used[h]:
                    used[h] = True
                    table[h] = k
                    first[h] = i
                    rep[i] = i
                    break
                if table[h] == k:
                    rep[i] = first[h]
                    break
                h = (h + 1) & mask
        return rep


def first_index(keys: np.ndarray) -> np.ndarray:
    """For every element, the index of the first element with the same key."""
    keys = np.ascontiguousarray(keys, dtype=np.int64)
    if nb is None or len(keys) < 2048:
        _, idx, inv = np.unique(keys, return_index=True, return_inverse=True)
        return idx[inv.ravel()]
    return _first_index_nb(keys)


def first_occurrence(keys: np.ndarray) -> np.ndarray:
    keys = np.ascontiguousarray(keys, dtype=np.int64)
    if nb is None or len(keys) < 2048:
        return _first_occurrence_np(keys)
    return _first_occurrence_nb(keys)


def load_npz_mmap(path, names=None) -> dict:
    """Members of an UNCOMPRESSED .npz (np.savez) as read-only memory maps: no private copy, the pages are
    shared through the OS page cache by every process reading the same LiDAR map (the RGB and thermal
    solves and their validation solves read each window's map 4 times each). Same values as np.load.
    Compressed members (or anything unusual) are read normally."""
    import zipfile
    out = {}
    with zipfile.ZipFile(path) as zf, open(path, "rb") as fh:
        for info in zf.infolist():
            name = info.filename[:-4] if info.filename.endswith(".npy") else info.filename
            if names is not None and name not in names:
                continue
            arr = None
            if info.compress_type == zipfile.ZIP_STORED:
                try:
                    fh.seek(info.header_offset)
                    head = fh.read(30)
                    nlen = int.from_bytes(head[26:28], "little")
                    xlen = int.from_bytes(head[28:30], "little")
                    start = info.header_offset + 30 + nlen + xlen
                    fh.seek(start)
                    ver = np.lib.format.read_magic(fh)
                    shape, fortran, dtype = np.lib.format._read_array_header(fh, ver)
                    if not dtype.hasobject and int(np.prod(shape)) > 0:
                        arr = np.memmap(path, dtype=dtype, mode="r", offset=fh.tell(), shape=shape,
                                        order="F" if fortran else "C")
                except Exception:  # noqa
                    arr = None
            if arr is None:
                with zf.open(info) as f:
                    arr = np.lib.format.read_array(f)
            out[name] = arr
    return out
