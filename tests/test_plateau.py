"""PLATEAU の道路ポリゴンの利用(network/plateau.py・topology.contract_in_polygons)のテスト。"""

import sys
from pathlib import Path

import geopandas as gpd
import networkx as nx
from shapely import STRtree
from shapely.geometry import LineString, Polygon, box

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim.network import lanes as L
from jp_sumo_traffic_sim.network import plateau as P
from jp_sumo_traffic_sim.network import topology as T
from jp_sumo_traffic_sim.network.directed import build_edges


def test_road_width_measures_polygon_across_link():
    road = box(0, -8, 200, 8)  # 幅 16 m の道路区間
    geoms = [road]
    w = P.road_width(LineString([(0, 0), (200, 0)]), STRtree(geoms), geoms)
    assert abs(w - 16.0) < 0.01


def test_junction_polygons_are_small_faces_with_junction_nodes():
    polys = gpd.GeoDataFrame(
        geometry=[box(-10, -10, 10, 10), box(10, -8, 300, 8), box(-2000, -2000, 2000, 2000)]
    )
    idx = P.junction_polygons(polys, [(0, 0)])
    assert idx == [0]  # 区間の面(交差点ノードを含まない)と巨大な面(上限超え)は除く


def _edge(g, u, v, state="1", layer="0"):
    a, b = g.nodes[u], g.nodes[v]
    geom = [(a["x"], a["y"]), (b["x"], b["y"])]
    g.add_edge(u, v, length=LineString(geom).length, geometry=geom, state=state, layer=layer,
               category="3", width="3", ksj_ids=[])  # fmt: skip


def test_contract_in_polygons_merges_dual_carriageway_junction():
    """上下分離道路の交差点(面の中にノードが 2 つ)を 1 つにまとめ、面の中のリンクを消す。"""
    g = nx.MultiGraph()
    for n, (x, y) in {1: (0, 3), 2: (0, -3), 10: (-100, 3), 11: (100, 3), 20: (-100, -3),
                      21: (100, -3), 30: (0, 100), 40: (0, -100)}.items():  # fmt: skip
        g.add_node(n, x=x, y=y)
    for u, v in ((1, 2), (10, 1), (1, 11), (20, 2), (2, 21), (30, 1), (2, 40)):
        _edge(g, u, v)
    face = box(-10, -10, 10, 10)
    st = T.contract_in_polygons(g, [face])
    assert st["n_nodes_merged"] == 1 and st["n_internal_links"] == 1
    assert 2 not in g.nodes and g.degree(1) == 6
    assert Polygon(g.nodes[1]["jshape"]).equals(face)


def test_contract_in_polygons_keeps_grade_separated_nodes():
    """立体リンク(橋・トンネル)につながるノードはまとめない。"""
    g = nx.MultiGraph()
    for n, (x, y) in {1: (0, 3), 2: (0, -3), 10: (-100, 3), 11: (100, 3), 20: (-100, -3),
                      21: (100, -3)}.items():  # fmt: skip
        g.add_node(n, x=x, y=y)
    _edge(g, 1, 2)
    _edge(g, 10, 1)
    _edge(g, 1, 11)
    _edge(g, 20, 2, state="3")  # トンネル
    _edge(g, 2, 21, state="3")
    st = T.contract_in_polygons(g, [box(-10, -10, 10, 10)])
    assert st["n_nodes_merged"] == 0 and 2 in g.nodes


def test_lanes_from_plateau_width():
    """センサスが無ければ PLATEAU の実測幅(歩道込み)× 0.68 から車線数を決める。"""
    (e, _) = build_edges(
        [dict(a=1, b=2, geometry=[(0, 0), (100, 0)], length=100.0, oneway="", state="1",
              layer="0", category="3", width="2", speed_kmh=0, census_id="", ksj_ids="")]
    )  # fmt: skip
    L.assign_lanes(e, None, road_width_m=25.0)  # 片側 25 × 0.68 / 2 = 8.5 m ÷ 4.25 → 2 車線
    assert e.attrs["lane_source"] == "plateau_width" and e.attrs["n_lanes"] == 2
