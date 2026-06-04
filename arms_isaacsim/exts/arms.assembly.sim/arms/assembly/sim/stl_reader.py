"""
Lightweight STL reader that converts binary or ASCII STL files into
pxr-compatible vertex/index lists.  No external dependencies beyond
the Python standard library.

Output is in STL file units (mm for ARMS output).  The caller is
responsible for applying a scale factor when setting USD stage units.
"""
from __future__ import annotations

import struct
import pathlib
from typing import NamedTuple


class STLMesh(NamedTuple):
    points:  list[tuple[float, float, float]]   # world-frame vertices (mm)
    indices: list[int]                           # face vertex indices (3 per tri)
    counts:  list[int]                           # vertices per face (all 3)


def read(path: str | pathlib.Path) -> STLMesh:
    """Read a binary or ASCII STL file and return vertex/index arrays."""
    data = pathlib.Path(path).read_bytes()
    if _is_ascii(data):
        return _read_ascii(data.decode("utf-8", errors="replace"))
    return _read_binary(data)


# ---------------------------------------------------------------------------
def _is_ascii(data: bytes) -> bool:
    try:
        text = data[:256].decode("ascii")
        return text.strip().startswith("solid")
    except UnicodeDecodeError:
        return False


def _read_binary(data: bytes) -> STLMesh:
    n_tri = struct.unpack_from("<I", data, 80)[0]
    points:  list[tuple[float, float, float]] = []
    indices: list[int] = []
    offset = 84
    for _ in range(n_tri):
        offset += 12  # skip normal
        for v in range(3):
            x, y, z = struct.unpack_from("<fff", data, offset)
            indices.append(len(points))
            points.append((x, y, z))
            offset += 12
        offset += 2   # skip attribute byte count
    return STLMesh(points, indices, [3] * n_tri)


def _read_ascii(text: str) -> STLMesh:
    points:  list[tuple[float, float, float]] = []
    indices: list[int] = []
    n_tri = 0
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("vertex"):
            parts = line.split()
            coords = (float(parts[1]), float(parts[2]), float(parts[3]))
            indices.append(len(points))
            points.append(coords)
        elif line.startswith("facet normal"):
            n_tri += 1
    return STLMesh(points, indices, [3] * n_tri)
