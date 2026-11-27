"""The drone's navigation map from a walk (docs/map-navigation.md, runner/fc/map_core.h).

A walk's clearance grid (clearance.py: unknown, free or occupied cells in metres, Z up, the walk's first camera at the
origin) coarsened to what the drone keeps, and written as the drone's map file, map.dmp; and the text lines that send
it to the drone's command port (the Pi's dfb_pi, UDP), with the guard's radius and, if asked, where the drone is.

The file, little-endian: "DFMP", u32 version 1, f32 cell size [m], f32 origin x y z [m], u32 cells nx ny nz, then
runs of cells, x fastest, then y, then z: a kind byte and the run's length (unsigned LEB128). Kinds: 0 unknown, 1
free, 2 occupied; 3 each as the cell a row back, 4 each as the cell a layer back. Then the CRC-32 (zlib's) of all
before it.
"""
import base64
import json
import socket
import struct
import time
import zlib

import numpy as np

UNKNOWN, FREE, OCCUPIED = 0, 1, 2
MAX_CELLS = 450_000          # the drone keeps 524 288 (MAP_MAX_CELLS)
MAX_BYTES = 32_000           # a file up the drone's link: 32 768 (FILE_RX_MAX)
PIECE = 36                   # bytes in one `fd` line (FILE_RX_PIECE)
PORT = 14560                 # dfb_pi's command port

PARAMS = (
    dict(key="cell_cm", label="Cell size", kind="number", unit="cm", default=8.0, min=2.0, max=50.0,
         help="The drone's cells: the clearance grid's are merged into these (coarser, if the map wouldn't fit the "
              "drone: 450 000 cells, 32 KB)."),
    dict(key="free_share", label="Free when at least", kind="number", unit="%", default=50.0, min=1.0, max=100.0,
         help="A merged cell is occupied if any of its cells is; free if this share of them is; unknown otherwise."),
)
SEND_PARAMS = (
    dict(key="radius_m", label="The drone's radius and margin", kind="number", unit="m", default=0.35, min=0.05, max=3.0,
         help="The guard keeps this far from what's occupied (the props' reach, and room to spare)."),
    dict(key="anchor", label="Where the drone is", kind="choice", default="start",
         options=[["start", "Where the walk started (it began with the drone at its take-off spot)"],
                  ["floor", "On the floor below where the walk started"],
                  ["none", "Not said (fixes will place it)"]],
         help="Without live fixes the drone needs telling once where it is. The walk's map starts at its first camera."),
    dict(key="heading_deg", label="Facing", kind="number", unit="°", default=90.0, min=-180.0, max=360.0,
         help="Which way the drone's nose points in the map, from its +X toward +Y. 90: the way the walk's camera first "
              "looked (the map's +Y)."),
)


def coarsen(cells, k, free_share=0.5):
    """Merge k×k×k blocks: occupied if any cell is, free if at least free_share of them are, else unknown."""
    if k == 1:
        return cells.astype("u1")
    pad = [(0, -n % k) for n in cells.shape]
    c = np.pad(cells, pad, constant_values=UNKNOWN)
    nx, ny, nz = (n // k for n in c.shape)
    blocks = c.reshape(nx, k, ny, k, nz, k)
    occ = (blocks == OCCUPIED).any(axis=(1, 3, 5))
    free = (blocks == FREE).mean(axis=(1, 3, 5)) >= free_share
    return np.where(occ, OCCUPIED, np.where(free, FREE, UNKNOWN)).astype("u1")


def _leb(v):
    out = bytearray()
    while True:
        c, v = v & 0x7F, v >> 7
        out.append(c | (0x80 if v else 0))
        if not v:
            return out


def encode(res, origin, cells):
    """map.dmp from cells indexed [x, y, z] (as map_core.c map_encode writes it)."""
    nx, ny, nz = cells.shape
    flat = np.ascontiguousarray(cells.transpose(2, 1, 0)).ravel()   # x fastest, then y, then z
    out = bytearray(b"DFMP" + struct.pack("<If3f3I", 1, res, *map(float, origin), nx, ny, nz))
    total, row, lay = flat.size, nx, nx * ny
    # the longest of: the same state; as a row back; as a layer back (numpy finds each run's end)
    same_end = _run_ends(flat[1:] != flat[:-1], total)
    row_end = _run_ends(flat[row:] != flat[:-row], total, row) if total > row else None
    lay_end = _run_ends(flat[lay:] != flat[:-lay], total, lay) if total > lay else None
    i = 0
    while i < total:
        best, kind = same_end[i] - i, int(flat[i])
        if row_end is not None and i >= row and row_end[i] - i > best:
            best, kind = row_end[i] - i, 3
        if lay_end is not None and i >= lay and lay_end[i] - i > best:
            best, kind = lay_end[i] - i, 4
        out.append(kind)
        out += _leb(int(best))
        i += int(best)
    return bytes(out + struct.pack("<I", zlib.crc32(out) & 0xFFFFFFFF))


def _run_ends(differs, total, back=0):
    """For each index i (from `back` on): where the run that starts there ends. differs[j]: cell back+j+1 vs ... breaks."""
    # breaks: positions p (≥ back) where cell p differs from what the run compares it with; a run from i ends at the
    # first break > i (or total)
    if back:
        breaks = np.flatnonzero(differs) + back
    else:
        breaks = np.flatnonzero(differs) + 1
    ends = np.full(total, total, np.int64)
    idx = np.searchsorted(breaks, np.arange(total), side="right" if not back else "left")
    has = idx < breaks.size
    ends[has] = breaks[idx[has]]
    return ends


def decode(data):
    """(res, origin, cells[x, y, z]) from map.dmp; ValueError on a bad file."""
    if len(data) < 40 or data[:4] != b"DFMP" or struct.unpack_from("<I", data, 4)[0] != 1:
        raise ValueError("not a map file")
    if zlib.crc32(data[:-4]) & 0xFFFFFFFF != struct.unpack_from("<I", data, len(data) - 4)[0]:
        raise ValueError("damaged map file")
    res, ox, oy, oz, nx, ny, nz = struct.unpack_from("<f3f3I", data, 8)
    total, flat, at, k, end = nx * ny * nz, np.zeros(nx * ny * nz, "u1"), 0, 36, len(data) - 4
    while k < end:
        kind = data[k]; k += 1
        run = sh = 0
        while True:
            c = data[k]; k += 1
            run |= (c & 0x7F) << sh; sh += 7
            if not c & 0x80:
                break
        back = nx if kind == 3 else nx * ny if kind == 4 else 0
        if back:
            for i in range(at, at + run):
                flat[i] = flat[i - back]
        else:
            flat[at:at + run] = kind
        at += run
    if at != total:
        raise ValueError("the cells don't add up")
    return res, (ox, oy, oz), flat.reshape(nz, ny, nx).transpose(2, 1, 0)


def from_clearance(grid_path, params=None):
    """The drone's map from a clearance grid.npz: dict(data, res, n, origin, k, counts)."""
    p = dict({q["key"]: q["default"] for q in PARAMS}, **(params or {}))
    g = np.load(grid_path)
    cells, origin, voxel = g["cells"], np.asarray(g["origin_m"], float), float(g["voxel_m"])
    k = max(1, round(p["cell_cm"] / 100 / voxel))
    while True:
        c = coarsen(cells, k, p["free_share"] / 100)
        if c.size <= MAX_CELLS:
            data = encode(k * voxel, origin, c)
            if len(data) <= MAX_BYTES:
                counts = np.bincount(c.ravel(), minlength=3)
                return dict(data=data, res=k * voxel, n=list(c.shape), origin=origin.tolist(), k=k,
                            unknown=int(counts[0]), free=int(counts[1]), occupied=int(counts[2]))
        k += 1


def floor_below(cells, res, origin, x=0.0, y=0.0, z=0.0):
    """The top of the first occupied cell below (x, y, z) in the map [m], or None."""
    i, j = int((x - origin[0]) / res), int((y - origin[1]) / res)
    kz = int((z - origin[2]) / res)
    if not (0 <= i < cells.shape[0] and 0 <= j < cells.shape[1]):
        return None
    for kk in range(min(kz, cells.shape[2] - 1), -1, -1):
        if cells[i, j, kk] == OCCUPIED:
            return origin[2] + (kk + 1) * res
    return None


def lines(data, radius, anchor=None):
    """The text lines that send a map to the drone (file_rx.h), its guard's radius, and where it is (mapanchor)."""
    out = [f"file map.dmp {len(data)} {zlib.crc32(data) & 0xFFFFFFFF:08x}"]
    out += [f"fd {o} {base64.b64encode(data[o:o + PIECE]).decode()}" for o in range(0, len(data), PIECE)]
    out += ["file end", f"mapguard {radius:.2f}"]
    if anchor:
        x, y, z, heading = anchor
        out.append(f"mapanchor {x:.3f} {y:.3f} {z:.3f} {heading:.1f}")
    out.append("map")
    return out


def send(host, lines_, port=PORT, timeout=2.0, gap=0.004):
    """Each line as a datagram to the drone's command port; its answers (dfb_pi answers the sender). Returns the
    answers to the lines that have them (file end, mapguard, mapanchor, map)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0.05)
    said = []
    try:
        for ln in lines_:
            s.sendto(ln.encode(), (host, port))
            time.sleep(gap)
            said += _drain(s)
        stop = time.time() + timeout
        while time.time() < stop and not any(a.startswith("map:") for a in said):
            said += _drain(s)
    finally:
        s.close()
    return said


def _drain(s):
    got = []
    while True:
        try:
            got.append(s.recv(2048).decode(errors="replace").strip())
        except (socket.timeout, BlockingIOError):
            return got
