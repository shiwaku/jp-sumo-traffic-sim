"""SUMO 需要生成のテスト(docs/sumo-design.md §4)。"""

import sumolib
from sumo_util import build, node, two_way

from jp_sumo_traffic_sim.sumo import demand


def _t_net(tmp_path):
    """西から東への幹線に、南北の枝が付く十字路 + 上下分離なしの双方向道路。"""
    nodes = {
        0: node(0, 0, 0),
        1: node(1, -300, 0),
        2: node(2, 300, 0),
        3: node(3, 0, 300),
        4: node(4, 0, -300),
    }
    edges = (
        two_way(10, 1, 0, nodes, category="1")
        + two_way(20, 0, 2, nodes, category="1")
        + two_way(30, 3, 0, nodes)
        + two_way(40, 0, 4, nodes)
    )
    stats = build(tmp_path, list(nodes.values()), edges)
    return nodes, edges, sumolib.net.readNet(stats["net"]), stats


def test_turn_probabilities_renormalize_turn_classes(tmp_path):
    """直進 0.70 / 左折 0.15 / 右折 0.15 を存在する転回先で配り、U ターンは含めない。"""
    _, _, net, _ = _t_net(tmp_path)
    probs = demand.turn_probabilities(net)
    p = probs["10"]  # 西から交差点へ(東向き)
    assert set(p) == {"20", "31", "40"}, p  # 直進・左折(北)・右折(南)。対向の 11 は除外
    assert abs(sum(p.values()) - 1.0) < 1e-9
    assert abs(p["20"] - 0.70) < 1e-6
    assert abs(p["31"] - 0.15) < 1e-6 and abs(p["40"] - 0.15) < 1e-6
    assert "11" not in probs  # 流出(後続なし)は含めない


def test_entry_rates_use_census_then_default():
    nodes = {0: node(0, 0, 0), 1: node(1, -300, 0), 2: node(2, 300, 0)}
    edges = two_way(10, 1, 0, nodes, category="1") + two_way(20, 0, 2, nodes)
    edges[0]["census_id"] = "S1"
    hourly = {"S1": {"up": {"s": [0] * 8 + [900], "l": [0] * 8 + [100]}, "down": {}}}
    rates, st = demand.entry_rates(edges, hourly, 8)
    assert rates == {10: 1000.0, 21: 150.0}  # 21 は細街路の既定値
    assert st["n_census_entries"] == 1 and st["n_default_entries"] == 1


def test_cap_hops_redirects_to_exit():
    route = [str(i) for i in range(40)]
    paths = {"30": ["x1", "x2"]}
    capped, did = demand.cap_hops(route, paths, 30)
    assert did and capped == [str(i) for i in range(31)] + ["x1", "x2"]
    # 右折車線の分割(同じ eid の連続)は1交差点として数える
    split = ["5", "5.-60", "6"]
    assert demand.cap_hops(split, {}, 2) == (split, False)


def test_build_routes_end_at_exits(tmp_path):
    _, _, net, stats = _t_net(tmp_path)
    rs = demand.build_routes(net, stats["net"], {10: 600.0, 30: 300.0}, 0.0, 600.0, tmp_path / "t")
    assert rs["n_vehicles"] == 150
    assert rs["n_route_not_ending_at_exit"] == 0
