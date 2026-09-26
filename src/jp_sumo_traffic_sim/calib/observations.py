"""センサス時間帯別交通量 → 方向別 Edge の観測交通量(docs/sumo-design.md §13.5)。

センサスの上り/下りは路線の起点・終点を基準にしていて、区間の形状の頂点順とは
対応しない(区間ごとにばらばら。C3 で確認)。そこで区間の**起点側・終点側の隣接区間**
から向きを決める:

- 下り = 起点側 → 終点側。区間の2つの端のうち、終点側の隣接区間に近い方が終点側の端
  (終点側が無ければ、起点側の隣接区間に近い方を起点側の端とする)
- 一方通行の区間(一方通行フラグ 1 = 下りのみ、2 = 上りのみ)で、層[2]の一方通行 Edge
  (JARTIC)の向きと 16 区間中 15 区間で一致した
- 向きが決まらない区間は上下の平均を両方向に使う

Edge の観測交通量 = その Edge が属するセンサス区間の、Edge の向きに対応する方向の
時間帯別交通量(小型 + 大型)。
"""

from __future__ import annotations

import math

import numpy as np
from shapely.geometry import Point


def far_ends(geom) -> tuple[tuple, tuple]:
    """区間の2つの端(全パートの端点のうち互いに最も離れた2点)。"""
    parts = list(geom.geoms) if geom.geom_type == "MultiLineString" else [geom]
    pts = [p for q in parts for p in (q.coords[0], q.coords[-1])]
    return max(((a, b) for a in pts for b in pts), key=lambda ab: math.dist(ab[0], ab[1]))


def down_vector(geom, start_geom=None, end_geom=None) -> tuple[np.ndarray | None, str]:
    """下り方向(起点側 → 終点側)の単位ベクトルと、決めた根拠('end' / 'start' / 'none')。

    start_geom / end_geom: 起点側・終点側の隣接区間の形状(無ければ None)。
    """
    a, b = far_ends(geom)
    v = np.subtract(b, a)
    if end_geom is not None:
        if Point(a).distance(end_geom) < Point(b).distance(end_geom):
            v = -v
        how = "end"
    elif start_geom is not None:
        if Point(b).distance(start_geom) < Point(a).distance(start_geom):
            v = -v
        how = "start"
    else:
        return None, "none"
    n = np.linalg.norm(v)
    return (v / n if n > 0 else None), how


def hourly_direction(unit: dict | None, direction: str) -> list[float | None]:
    """時間帯別交通量(24 時間)。s + l、欠測(null)は、その時刻の s・l が両方欠けたときだけ None。"""
    d = (unit or {}).get(direction) or {}
    out: list[float | None] = []
    for h in range(24):
        vals = [(d.get(k) or [None] * 24)[h] for k in ("s", "l")]
        vals = [v for v in vals if v is not None]
        out.append(float(sum(vals)) if vals else None)
    return out


def edge_counts(edge_vec: np.ndarray, dv: np.ndarray | None, unit: dict) -> tuple[list, str]:
    """Edge の向き edge_vec に対応する時間帯別交通量と方向('down' / 'up' / 'avg')。"""
    up, down = hourly_direction(unit, "up"), hourly_direction(unit, "down")
    if dv is None:
        avg = [
            None
            if (u is None and d is None)
            else ((u or 0.0) + (d or 0.0)) / (2 if (u is not None and d is not None) else 1)
            for u, d in zip(up, down, strict=True)
        ]
        return avg, "avg"
    if float(np.dot(edge_vec, dv)) >= 0:
        return down, "down"
    return up, "up"
