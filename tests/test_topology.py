"""位相構築(network/topology.py)の単体テスト。合成ジオメトリのみでデータ非依存。"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim.network import topology as T


def rec(coords, **kw):
    return dict(geometry=coords, **kw)


def test_snap_merges_close_endpoints_only():
    pts = np.array([[0, 0], [0.3, 0], [5, 0], [5.2, 0.2]])
    labels, coords = T.snap_endpoints(pts, tol=0.5)
    assert labels[0] == labels[1]
    assert labels[2] == labels[3]
    assert labels[0] != labels[2]
    assert len(coords) == 2


def test_build_graph_snaps_and_drops_self_loops():
    recs = [
        rec([(0, 0), (100, 0)]),
        rec([(100.2, 0.1), (200, 0)]),  # 0.5m 以内 → 同一ノード
        rec([(50, 50), (60, 60), (50.1, 50.1)]),  # 周回 → 自己ループとして捨てる
    ]
    g = T.build_graph(recs)
    assert g.number_of_edges() == 2
    assert g.graph["n_self_loops_dropped"] == 1
    assert g.number_of_nodes() == 3 + 1  # (0,0) (100,0) (200,0) + 周回の両端1点


def test_contract_short_link_between_junctions():
    """次数3+ 同士に挟まれた短リンクは交差点1ノードに縮約される。"""
    recs = [
        # 十字の西・東・北・南の4本 + 中央に 8m の短リンク(交差点内部)
        rec([(-100, 0), (0, 0)]),
        rec([(8, 0), (100, 0)]),
        rec([(0, 0), (0, 100)]),
        rec([(8, 0), (8, -100)]),
        rec([(0, 0), (8, 0)]),  # 短リンク
    ]
    g = T.build_graph(recs)
    stats = T.contract_short_links(g)
    assert stats["n_contracted"] == 1
    assert g.number_of_nodes() == 5  # 縮約後: 中央1 + 端点4
    assert g.number_of_edges() == 4
    # 縮約ノードは中点(4, 0)
    center = [n for n in g.nodes if g.degree(n) == 4]
    assert len(center) == 1
    d = g.nodes[center[0]]
    assert abs(d["x"] - 4.0) < 1e-6 and abs(d["y"]) < 1e-6


def test_merge_short_link_through_degree2_node():
    """片端が次数2の短リンクは隣のエッジと結合され、延長が保存される。"""
    recs = [
        rec([(0, 0), (90, 0)]),
        rec([(90, 0), (98, 0)]),  # 8m の短リンク(次数2ノード経由)
        rec([(98, 0), (98, 80)]),
        rec([(98, 0), (98, -80)]),
    ]
    g = T.build_graph(recs)
    total_before = sum(d["length"] for _, _, d in g.edges(data=True))
    stats = T.contract_short_links(g)
    assert stats["n_merged"] == 1
    total_after = sum(d["length"] for _, _, d in g.edges(data=True))
    assert abs(total_before - total_after) < 1e-6
    # 結合エッジの形状は (0,0)→(90,0)→(98,0) をたどる
    merged = [d for _, _, d in g.edges(data=True) if len(d["ksj_ids"]) == 2]
    assert len(merged) == 1
    ys = {round(y, 1) for _, y in merged[0]["geometry"]}
    assert ys == {0.0}


def test_grade_separated_flag_and_interior_check():
    assert T.is_grade_separated("2")
    assert T.is_grade_separated("3")
    assert T.is_grade_separated("1", layer="1")  # 階層順でも立体
    assert not T.is_grade_separated("1")
    # トンネルの中間頂点に地表ノードが重なると違反として数える
    recs = [
        rec([(0, 0), (100, 0), (200, 0)], state="3"),  # トンネル
        rec([(100, 0), (100, 90)]),  # 地表(トンネル中間頂点と同座標)
        rec([(100, 0), (100, -90)]),
    ]
    g = T.build_graph(recs)
    chk = T.check_grade_separated(g)
    assert chk["n_grade_separated"] == 1
    assert chk["n_interior_contacts"] >= 1


def test_acceptance_reports_dead_ends():
    class FakeBoundary:
        def contains(self, x, y):
            return abs(x) <= 190  # x=±200 はコードン外扱い

        def boundary_dist(self, x, y):
            return 0.0 if abs(x) > 190 else 1e9

    recs = [
        rec([(-200, 0), (0, 0)]),
        rec([(0, 0), (200, 0)]),
        rec([(0, 0), (0, 100)]),  # 内部の行き止まり
    ]
    g = T.build_graph(recs)
    acc = T.acceptance(g, FakeBoundary())
    assert acc["n_components"] == 1
    assert acc["boundary_all_reachable"]
    assert acc["n_true_dead_ends"] == 1
    assert acc["true_dead_ends"][0]["xy"] == [0.0, 100.0]
