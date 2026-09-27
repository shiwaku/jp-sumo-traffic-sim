"""JARTIC 車両感知器(断面交通量 typeB)の位置の特定(docs/sumo-design.md §13.6、D2)。

感知器の地点は DRM リンク番号とリンク終端からの距離で示され、座標が無い。次の手がかりで
ネットワークの方向別 Edge に対応付ける:

1. 交差点制御情報(typeC)の定義: 感知器のリンクが交差点 Y の流入なら Y に入る道路、
   交差点 X の流出なら X から出る道路(交差点の座標は既知)
2. 地点名称の住所: 「北２３西１３」のような条・丁目は札幌の格子座標で位置に直す
3. 進行方向: 地点名称の末尾 2 文字の **2 文字目**(東西南北)。格子の向きで見た進行方向と
   8 割強が一致する(typeC で区間が確定する 114 地点で確認)。1 文字目の意味は未確認

候補の道路は、進行方向と道路の向き(格子の向き)の差が DIR_TOL_DEG 以内のものに絞る。
"""

from __future__ import annotations

import math
import re
import unicodedata

DIRS = {"東": 0.0, "北": 90.0, "西": 180.0, "南": 270.0}  # 格子の向きでの方位 [deg、反時計回り]
DIR_TOL_DEG = 45.0
GRID_ADDR = re.compile(r"^(北|南)(\d+)(西|東)(\d+)")


def normalize(name: str) -> str:
    """全角数字・全角スペースを半角にし、空白を除く。"""
    return re.sub(r"\s+", "", unicodedata.normalize("NFKC", name))


def travel_direction(name: str) -> str | None:
    """地点名称の末尾 2 文字(ともに東西南北)の 2 文字目 = 進行方向。当てはまらなければ None。"""
    s = normalize(name)
    if len(s) >= 2 and s[-1] in DIRS and s[-2] in DIRS:
        return s[-1]
    return None


def grid_address(name: str) -> tuple[str, int, str, int] | None:
    """「北２３西１３」→ ('北', 23, '西', 13)。条・丁目の住所でなければ None。"""
    s = normalize(name)
    body = s[:-2] if travel_direction(name) else s
    m = GRID_ADDR.match(body)
    if not m:
        return None
    return m.group(1), int(m.group(2)), m.group(3), int(m.group(4))


def angle_diff(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


def pick_edge(candidates: list[tuple[int, float]], travel: str | None) -> int | None:
    """(eid, 格子の向きでの方位) の候補から、進行方向に最も近いものを選ぶ。

    travel が無い(名称から方向が取れない)ときは、候補が 1 本だけなら選ぶ。
    """
    if not candidates:
        return None
    if travel is None:
        return candidates[0][0] if len(candidates) == 1 else None
    want = DIRS[travel]
    best = min(candidates, key=lambda c: angle_diff(c[1], want))
    return best[0] if angle_diff(best[1], want) <= DIR_TOL_DEG else None


def grid_bearing(p, q, grid_bearing_deg: float) -> float:
    """p → q の方位 [deg] を格子の向き(grid_bearing_deg だけ回した座標)で。"""
    th = math.radians(-grid_bearing_deg)
    dx, dy = q[0] - p[0], q[1] - p[1]
    gx = dx * math.cos(th) - dy * math.sin(th)
    gy = dx * math.sin(th) + dy * math.cos(th)
    return math.degrees(math.atan2(gy, gx)) % 360.0
