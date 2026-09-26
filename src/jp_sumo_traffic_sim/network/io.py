"""実ネットワーク入力の読み込み(architecture.md §2 の io 層)。

scripts/33(再生データ生成)と scripts/40(需要照合)で共用する。
GeoPackage から NetSim へ渡す素の dict 列を作る。
"""

from __future__ import annotations

import json

import geopandas as gpd

from jp_sumo_traffic_sim import config as C


def load_network_inputs():
    """(nodes, edges, signal_plans, stop_edges, census_hourly) を返す。"""
    nodes_g = gpd.read_file(C.PROCESSED / "network_conflated.gpkg", layer="nodes")
    edges_g = gpd.read_file(C.PROCESSED / "edges.gpkg", layer="edges")
    links_g = gpd.read_file(C.PROCESSED / "network_conflated.gpkg", layer="links")
    try:
        stops_g = gpd.read_file(C.PROCESSED / "network_conflated.gpkg", layer="stops")
    except Exception:
        stops_g = None
    f = C.PROCESSED / "signal_plans.json"
    plans = json.loads(f.read_text(encoding="utf-8")) if f.exists() else {}
    f = C.PROCESSED / "census_hourly.json"
    hourly = json.loads(f.read_text(encoding="utf-8")) if f.exists() else {}

    nodes = [
        dict(
            nid=int(r["nid"]),
            x=r.geometry.x,
            y=r.geometry.y,
            has_signal=bool(r["has_signal"]),
            signal_uid=r["signal_uid"] or "",
        )
        for _, r in nodes_g.iterrows()
    ]
    edges = [
        dict(
            eid=int(r["eid"]),
            frm=int(r["frm"]),
            to=int(r["to"]),
            geometry=list(r.geometry.coords),
            length=float(r["length_m"]),
            speed_kmh=int(r["speed_kmh"]),
            category=str(r["category"]),
            linked_edge=int(r["linked_edge"]),
            n_sublanes=int(r["n_sublanes"]),
            right_turn_lane=int(r["right_turn_lane"]),
            census_id=str(r["census_id"] or ""),
        )
        for _, r in edges_g.iterrows()
    ]

    # 一時停止: (ノード, 流入リンク) → 該当する方向別 Edge を特定
    stop_edges: set[int] = set()
    if stops_g is not None:
        link_ab = {int(i): (int(r["a"]), int(r["b"])) for i, r in links_g.iterrows()}
        by_pair: dict[tuple[int, int], list[int]] = {}
        for e in edges:
            by_pair.setdefault((e["frm"], e["to"]), []).append(e["eid"])
        for _, r in stops_g.iterrows():
            node, li = int(r["node"]), int(r["link_index"])
            a, b = link_ab[li]
            frm = b if node == a else a
            for eid in by_pair.get((frm, node), []):
                stop_edges.add(eid)
    return nodes, edges, plans, stop_edges, hourly
