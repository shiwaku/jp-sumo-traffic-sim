"""方向別 Edge(層[2])→ netconvert の plain XML(docs/sumo-design.md §2)。

- node id = nid、edge id = eid。座標は投影座標(CRS_PROJ)のまま渡す
  (netconvert 側で --offset.disable-normalization を付けて正規化させない)
- 車線数・車線幅・速度・優先度は層[2]の値。一方通行は片方向の Edge しか無いので
  そのまま片方向になる
- 右折専用車線: 停止線手前 RT_ZONE_M で Edge を2本に分け(下流側の id は
  "{eid}.-{距離}")、下流側に中央側の1車線を足す。左側通行では最大 index の車線が
  中央側 = 右折車線になる
- 一時停止: 流入 Edge の priority を最低にし、ノードを priority_stop にする
- 長さ: 層[2]の実測長を length 属性で渡す。渡さないと netconvert は交差点の形状で
  削った後の形状長を使い、ノードが近接する所では長さ 0 の Edge ができる(C2 の試走で
  市全域の 35%)。長さ 0 の Edge はメソで1台も収容できず、そこで流れが止まる
"""

from __future__ import annotations

import math
from pathlib import Path
from xml.sax.saxutils import quoteattr

from shapely.geometry import LineString
from shapely.ops import substring

RT_ZONE_M = 60.0  # 停止線からこの距離で右折専用車線を足す(自前実装 RT_ZONE_M と同じ)
RT_MIN_SUBLANES = 4  # 専用車線が成立する最小サブレーン数(直進1車線分を残す。RT_MIN_SUB と同じ)
SPLIT_MIN_REMAIN_M = 10.0  # 分割後に上流側へ残す最小長。これが取れない短い Edge には足さない

# 優先度: 高速 > 国道 > 道道 > 市町村道(KSJ N13_003)。一時停止の流入は最低
CATEGORY_PRIORITY = {"4": 5, "1": 4, "2": 3, "3": 2}
STOP_PRIORITY = 1


def edge_priority(e: dict, stop_edges: set) -> int:
    if e["eid"] in stop_edges:
        return STOP_PRIORITY
    return CATEGORY_PRIORITY.get(str(e.get("category", "")), 2)


RIGHT_TURN_DEG = (45.0, 135.0)  # 時計回りにこの範囲の転回を右折とみなす


def _bearing(p, q) -> float:
    return math.degrees(math.atan2(q[1] - p[1], q[0] - p[0]))


def has_right_turn(e: dict, out_by_node: dict[int, list[dict]]) -> bool:
    """e の下流ノードに右折の行き先があるか(U ターン = e の上流ノードへ戻る Edge は除く)。

    行き先の無い流入に右折専用車線を足すと、netconvert がその車線を別の転回に割り当て、
    全車線を横切る接続ができて詰まる(C2 の試走で発覚)。
    """
    g = e["geometry"]
    h_in = _bearing(g[-2], g[-1])
    lo, hi = RIGHT_TURN_DEG
    for o in out_by_node.get(e["to"], []):
        if o["to"] == e["frm"]:
            continue
        og = o["geometry"]
        d = (_bearing(og[0], og[1]) - h_in + 180.0) % 360.0 - 180.0
        if lo <= -d <= hi:
            return True
    return False


def rt_split_pos(e: dict, out_by_node: dict[int, list[dict]] | None = None) -> float | None:
    """右折専用車線を足す分割位置(Edge 終端からの負の距離)。足さないなら None。"""
    if not e.get("right_turn_lane") or int(e.get("n_sublanes") or 0) < RT_MIN_SUBLANES:
        return None
    if out_by_node is not None and not has_right_turn(e, out_by_node):
        return None
    zone = min(RT_ZONE_M, float(e["length"]) - SPLIT_MIN_REMAIN_M)
    if zone < SPLIT_MIN_REMAIN_M:
        return None
    return -round(zone, 2)


def lane_width(e: dict, width_scale: float = 1.0) -> float:
    """車線幅 = 車道幅員 / 車線数。width_scale は冬季の幅員縮小(§5.4)。"""
    n = max(1, int(e.get("n_lanes") or 1))
    return round(float(e["carriageway_m"]) / n * width_scale, 2)


def write_plain_xml(
    nodes: list[dict],
    edges: list[dict],
    stop_edges: set,
    prefix: Path,
    width_scale: float = 1.0,
) -> dict:
    """{prefix}.nod.xml / {prefix}.edg.xml を書き、集計を返す。"""
    stop_nodes = {e["to"] for e in edges if e["eid"] in stop_edges}
    signal_nodes = {n["nid"] for n in nodes if n.get("has_signal")}
    out_by_node: dict[int, list[dict]] = {}
    for e in edges:
        out_by_node.setdefault(e["frm"], []).append(e)

    nod = ["<nodes>"]
    for n in nodes:
        nid = n["nid"]
        if nid in signal_nodes:
            typ = "traffic_light"
        elif nid in stop_nodes:
            typ = "priority_stop"
        else:
            typ = "priority"
        nod.append(f'  <node id="{nid}" x="{n["x"]:.2f}" y="{n["y"]:.2f}" type="{typ}"/>')
    nod.append("</nodes>")

    edg = ["<edges>"]
    n_split = 0
    for e in edges:
        n_lanes = max(1, int(e.get("n_lanes") or 1))
        common = (
            f'speed="{float(e["speed_kmh"]) / 3.6:.2f}" width="{lane_width(e, width_scale)}" '
            f'priority="{edge_priority(e, stop_edges)}"'
        )
        params = []
        for key in ("census_id", "ksj_ids"):
            if e.get(key):
                params.append(f"    <param key={quoteattr(key)} value={quoteattr(str(e[key]))}/>")
        if e.get("linked_edge", -1) not in (-1, None):
            params.append(f'    <param key="linked_edge" value="{e["linked_edge"]}"/>')

        pos = rt_split_pos(e, out_by_node)
        if pos is None:
            pieces = [(str(e["eid"]), e["frm"], e["to"], n_lanes, e["geometry"], e["length"])]
        else:
            # 停止線手前で2本に分け、下流側に右折専用車線を足す。netconvert の <split> は
            # length 属性と併用すると両側の長さが 0.1m になるため、自前で分割する
            line = LineString(e["geometry"])
            cut = line.length + pos  # pos は負(終端からの距離)
            up, down = substring(line, 0, cut), substring(line, cut, line.length)
            mid = f"{e['eid']}.{pos:g}"
            x, y = down.coords[0]
            nod.insert(-1, f'  <node id="{mid}" x="{x:.2f}" y="{y:.2f}" type="priority"/>')
            ratio = float(e["length"]) / line.length if line.length > 0 else 1.0
            pieces = [
                (str(e["eid"]), e["frm"], mid, n_lanes, list(up.coords), up.length * ratio),
                (mid, mid, e["to"], n_lanes + 1, list(down.coords), down.length * ratio),
            ]
            n_split += 1
        for eid, frm, to, nl, geom, length in pieces:
            shape = " ".join(f"{x:.2f},{y:.2f}" for x, y in geom)
            edg.append(
                f'  <edge id="{eid}" from="{frm}" to="{to}" numLanes="{nl}" {common} '
                f'length="{float(length):.2f}" shape="{shape}">'
            )
            edg += params
            edg.append("  </edge>")
    edg.append("</edges>")

    prefix.parent.mkdir(parents=True, exist_ok=True)
    Path(f"{prefix}.nod.xml").write_text("\n".join(nod) + "\n", encoding="utf-8")
    Path(f"{prefix}.edg.xml").write_text("\n".join(edg) + "\n", encoding="utf-8")
    return dict(
        n_nodes=len(nodes),
        n_edges=len(edges),
        n_signal_nodes=len(signal_nodes),
        n_priority_stop_nodes=len(stop_nodes - signal_nodes),
        n_rt_lane_splits=n_split,
    )


def base_eid(sumo_edge_id: str) -> int | None:
    """SUMO edge id → 層[2]の eid。右折車線の分割で付く '.-60' 等の接尾辞を外す。"""
    head = sumo_edge_id.split(".", 1)[0]
    return int(head) if head.lstrip("-").isdigit() else None
