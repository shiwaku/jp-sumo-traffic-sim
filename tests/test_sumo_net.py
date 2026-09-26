"""SUMO ネットワーク変換のテスト(docs/sumo-design.md §2・§3・§10)。"""

from pathlib import Path

import pytest
import sumolib
from sumo_util import build, edge, node, two_way

from jp_sumo_traffic_sim import config as C
from jp_sumo_traffic_sim.cases import sapporo as S
from jp_sumo_traffic_sim.sumo import netgen


def _cross_nodes():
    return {
        0: node(0, 0, 0, signal=True),
        1: node(1, -400, 0),
        2: node(2, 400, 0),
        3: node(3, 0, 400),
        4: node(4, 0, -400),
    }


def _cross_edges(nodes, rt=0):
    return (
        two_way(10, 1, 0, nodes, n_lanes=2, category="1", rt=rt)
        + two_way(20, 0, 2, nodes, n_lanes=2, category="1")
        + two_way(30, 3, 0, nodes)
        + two_way(40, 0, 4, nodes)
    )


# --- plain XML の生成(netconvert 不要)---


def test_rt_split_only_when_wide_and_long_enough():
    nodes = _cross_nodes()
    wide = edge(1, 1, 0, nodes, n_lanes=2, rt=1)
    wide["n_sublanes"] = 4
    assert netgen.rt_split_pos(wide) == -60.0
    narrow = dict(wide, n_sublanes=3)
    assert netgen.rt_split_pos(narrow) is None
    short = dict(wide, length=50.0)
    assert netgen.rt_split_pos(short) == -40.0  # 上流側に 10m を残す
    tiny = dict(wide, length=15.0)
    assert netgen.rt_split_pos(tiny) is None
    assert netgen.rt_split_pos(dict(wide, right_turn_lane=0)) is None


def test_stop_edges_get_lowest_priority_and_priority_stop_node(tmp_path):
    nodes = _cross_nodes()
    nodes[0]["has_signal"] = False
    edges = _cross_edges(nodes)
    stop = {30}  # 北からの流入に一時停止
    netgen.write_plain_xml(list(nodes.values()), edges, stop, tmp_path / "t")
    nod = (tmp_path / "t.nod.xml").read_text(encoding="utf-8")
    edg = (tmp_path / "t.edg.xml").read_text(encoding="utf-8")
    assert '<node id="0" x="0.00" y="0.00" type="priority_stop"/>' in nod
    head = 'id="30" from="3" to="0" numLanes="1" speed="13.89" width="3.0" '
    assert head + f'priority="{netgen.STOP_PRIORITY}"' in edg
    assert 'priority="4"' in edg  # 国道


def test_base_eid_strips_split_suffix():
    assert netgen.base_eid("12") == 12
    assert netgen.base_eid("12.-60") == 12
    assert netgen.base_eid(":0_1_0") is None


# --- netconvert を通した結果 ---


def test_cross_net_lefthand_rt_lane_and_program(tmp_path):
    nodes = _cross_nodes()
    edges = _cross_edges(nodes, rt=1)
    for e in edges:
        e["n_sublanes"] = 4 if e["eid"] == 10 else e["n_sublanes"]
    stats = build(tmp_path, list(nodes.values()), edges)
    assert stats["n_rt_lane_splits"] == 1
    net = sumolib.net.readNet(stats["net"], withPrograms=True)

    # 右折専用車線: 分割後の流入は3車線。中央側(最大 index)は右折のみ
    appr = net.getEdge("10.-60")
    assert appr.getLaneNumber() == 3
    # 長さは層[2]の実測長(交差点の形状で削られない)。分割の上流側 + 下流側 = 元の長さ
    assert abs(appr.getLength() - 60.0) < 0.1
    assert abs(net.getEdge("10").getLength() + appr.getLength() - 400.0) < 0.1
    dirs = {}
    for conns in appr.getOutgoing().values():
        for c in conns:
            dirs.setdefault(c.getFromLane().getIndex(), set()).add(c.getDirection())
    assert dirs[2] == {"r"}, f"中央側車線の転回 {dirs}"
    assert "l" in dirs[0], "左折は路肩側(lane 0)から"

    # U ターン接続なし
    for e in net.getEdges():
        for conns in e.getOutgoing().values():
            assert all(c.getDirection() != "t" for c in conns)

    # 2現示: A(優先度の高い東西)青 → 黄 → 全赤 → B 青 → 黄 → 全赤、サイクル 120s
    prog = list(net.getTLS("0").getPrograms().values())[0]
    phases = prog.getPhases()
    assert [p.duration for p in phases] == [55, 3, 2, 55, 3, 2]
    a_green = phases[0].state
    idx = {}
    for conns in appr.getOutgoing().values():
        for c in conns:
            idx[c.getDirection()] = c.getTLLinkIndex()
    assert a_green[idx["s"]] == "G" and a_green[idx["r"]] == "g"


def test_sapporo_signal_axis_uses_grid():
    """札幌ケース: グリッドの東西方向の流入が A(先に青)。"""
    import math

    th = math.radians(S.GRID_BEARING_DEG)
    ox, oy = S.GRID_ORIGIN
    ew = ((ox, oy), (ox + 100 * math.cos(th), oy + 100 * math.sin(th)))
    ns = ((ox, oy), (ox - 100 * math.sin(th), oy + 100 * math.cos(th)))
    assert S.signal_axis(*ew) == "A"
    assert S.signal_axis(*ns) == "B"


# --- 実ネットワーク(data/sumo が無ければ skip)---

NET = C.ROOT / "data" / "sumo" / f"{C.CASE_NAME}.net.xml"
EDGES = C.PROCESSED / "edges.gpkg"
needs_net = pytest.mark.skipif(
    not (NET.exists() and EDGES.exists()), reason="make sumo-net で data/sumo を生成してから"
)


@needs_net
def test_real_net_grade_separation():
    """立体リンク(橋・トンネル)は途中で地表と接続しない。

    SUMO 側で1本の Edge のまま(途中に交差点が挿入されていない)で、端点は層[2]と同じ。
    近接ノードの縮約(junctions.join)をしていないことの確認にもなる。
    """
    import geopandas as gpd

    g = gpd.read_file(EDGES, layer="edges")
    grade = g[g["state"].astype(str).isin(["2", "3"])]
    assert len(grade) > 0
    net = sumolib.net.readNet(str(NET))
    for _, r in grade.iterrows():
        e = net.getEdge(str(int(r["eid"]))) if net.hasEdge(str(int(r["eid"]))) else None
        assert e is not None, f"立体リンク {r['eid']} が無い"
        assert e.getFromNode().getID() == str(int(r["frm"]))
        # 右折車線の分割があれば終点は分割ノード(実ノードは分割後の Edge の終点)
        to_id = e.getToNode().getID()
        assert to_id == str(int(r["to"])) or to_id.startswith(f"{int(r['eid'])}.")


@needs_net
def test_real_net_reachability_and_ids():
    import json

    rep = json.loads((C.REPORTS / "50_sumo_net.json").read_text(encoding="utf-8"))
    ck = rep["check"]
    assert ck["n_base_edges_found"] == ck["n_base_edges_expected"]
    assert ck["n_uturn_connections"] == 0
    assert ck["n_entries_reaching_exit"] == ck["n_entries"]
    assert Path(C.ROOT / rep["net"]).exists()


def test_no_rt_lane_without_right_turn_exit():
    """右折の行き先が無い流入には右折専用車線を足さない(左折・直進しか無い T 字路)。"""
    nodes = {0: node(0, 0, 0), 1: node(1, -400, 0), 2: node(2, 400, 0), 3: node(3, 0, 400)}
    # 西→東の流入(10)から見て、北(左折)と東(直進)しかない。南(右折)が無い
    edges = (
        two_way(10, 1, 0, nodes, n_lanes=2, rt=1)
        + two_way(20, 0, 2, nodes)
        + two_way(30, 0, 3, nodes)
    )
    appr = dict(edges[0], n_sublanes=4)
    out_by_node = {}
    for e in edges:
        out_by_node.setdefault(e["frm"], []).append(e)
    assert not netgen.has_right_turn(appr, out_by_node)
    assert netgen.rt_split_pos(appr, out_by_node) is None
    # 南への道があれば右折あり
    nodes[4] = node(4, 0, -400)
    out_by_node[0].append(edge(40, 0, 4, nodes))
    assert netgen.has_right_turn(appr, out_by_node)


def test_merge_keeps_one_priority_green_per_target_lane():
    """同じ流出車線へ向かう G は1本(直進)だけ残し、残りは従属青 g にする。"""
    from jp_sumo_traffic_sim.sumo.tls import _merge_to_minor

    state = list("GGG")
    conns = [(0, None, "l", "X_0"), (1, None, "s", "X_0"), (2, None, "s", "Y_0")]
    assert _merge_to_minor(state, conns) == 1
    assert state == ["g", "G", "G"]
