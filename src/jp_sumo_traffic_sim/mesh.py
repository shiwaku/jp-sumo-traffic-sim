"""地域メッシュ(JIS X 0410)の格子番号とコード。

格子番号は整数で扱う(緯度経度を小数で足し進めると取りこぼす)。
"""

from __future__ import annotations

import math

MESH6_LAT_DEG = 1.0 / 960  # 6 次メッシュ(125m)の南北 3.75 秒
MESH6_LON_DEG = 1.0 / 640  # 東西 5.625 秒


def mesh3_code(i: int, j: int) -> str:
    """3 次メッシュの格子番号 → 8 桁。i = floor(緯度 × 120)、j = floor(経度 × 80)。"""
    jj = j - 100 * 80
    return f"{i // 80:02d}{jj // 80:02d}{(i % 80) // 10}{(jj % 80) // 10}{i % 10}{jj % 10}"


def mesh6_code(i: int, j: int) -> str:
    """6 次メッシュ(125m)の格子番号 → 11 桁。i = floor(緯度 × 960)、j = floor(経度 × 640)。

    4〜6 次は 2 × 2 分割の番号(1 = 南西、2 = 南東、3 = 北西、4 = 北東)。
    """
    r, c = i % 8, j % 8
    q = [1 + (c >> k & 1) + 2 * (r >> k & 1) for k in (2, 1, 0)]
    return mesh3_code(i // 8, j // 8) + "".join(map(str, q))


def mesh6_index(lon: float, lat: float) -> tuple[int, int]:
    return math.floor(lat * 960 + 1e-9), math.floor(lon * 640 + 1e-9)


def mesh6_bounds(i: int, j: int) -> tuple[float, float, float, float]:
    """(経度の西端, 緯度の南端, 東端, 北端)。"""
    return j * MESH6_LON_DEG, i * MESH6_LAT_DEG, (j + 1) * MESH6_LON_DEG, (i + 1) * MESH6_LAT_DEG
