"""方向別 Edge(層[2])→ netconvert の plain XML(docs/sumo-design.md §2)。

- node id = nid、edge id = eid。座標は投影座標(CRS_PROJ)のまま渡す
  (netconvert 側で --offset.disable-normalization を付けて正規化させない)
- 車線数・車線幅・速度・優先度は層[2]の値。一方通行は片方向の Edge しか無いので
  そのまま片方向になる
- 右折専用車線: 停止線手前 RT_ZONE_M で中央側に1車線を足す(<split>)。
  左側通行では最大 index の車線が中央側 = 右折車線になる
- 一時停止: 流入 Edge の priority を最低にし、ノードを priority_stop にする
"""

from __future__ import annotations

from pathlib import Path
from xml.sax.saxutils import quoteattr

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


def rt_split_pos(e: dict) -> float | None:
    """右折専用車線を足す分割位置(Edge 終端からの負の距離)。足さないなら None。"""
    if not e.get("right_turn_lane") or int(e.get("n_sublanes") or 0) < RT_MIN_SUBLANES:
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
        shape = " ".join(f"{x:.2f},{y:.2f}" for x, y in e["geometry"])
        n_lanes = max(1, int(e.get("n_lanes") or 1))
        attrs = (
            f'id="{e["eid"]}" from="{e["frm"]}" to="{e["to"]}" numLanes="{n_lanes}" '
            f'speed="{float(e["speed_kmh"]) / 3.6:.2f}" width="{lane_width(e, width_scale)}" '
            f'priority="{edge_priority(e, stop_edges)}" shape="{shape}"'
        )
        edg.append(f"  <edge {attrs}>")
        pos = rt_split_pos(e)
        if pos is not None:
            lanes = " ".join(str(i) for i in range(n_lanes + 1))
            edg.append(f'    <split pos="{pos}" lanes="{lanes}"/>')
            n_split += 1
        for key in ("census_id", "ksj_ids"):
            if e.get(key):
                edg.append(f"    <param key={quoteattr(key)} value={quoteattr(str(e[key]))}/>")
        if e.get("linked_edge", -1) not in (-1, None):
            edg.append(f'    <param key="linked_edge" value="{e["linked_edge"]}"/>')
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
