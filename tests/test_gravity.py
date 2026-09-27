"""初期 OD(od/gravity.py)と照合の指標(calib/estimation.py)の単体テスト。"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim.calib import estimation as E
from jp_sumo_traffic_sim.od import gravity as G


def test_edge_graph_and_zone_costs():
    # a → b → c、a → c(遠回りより直行が安い)
    edges = [
        dict(id="a", cost_s=60.0, succ=["b", "c"]),
        dict(id="b", cost_s=60.0, succ=["c"]),
        dict(id="c", cost_s=300.0, succ=[]),
    ]
    g, idx = G.edge_graph(edges)
    c = G.zone_costs(g, [idx["a"], None], [idx["b"], idx["c"]])
    assert c[0, 0] == 1.0  # a → b = b の 60 s
    assert c[0, 1] == 5.0  # a → c = c の 300 s(b を通ると 360 s)
    assert np.isinf(c[1]).all()  # 代表の Edge の無いゾーン


def test_routing_cost_priority_and_signal():
    base = G.routing_cost(100.0, 10.0, 4, (1, 4), False, 1.0, 10.0)
    low = G.routing_cost(100.0, 10.0, 1, (1, 4), True, 1.0, 10.0)
    assert base == 10.0
    assert low == 10.0 * 2 + 10.0  # 最低の優先度で 2 倍 + 信号の待ち


def test_gravity_is_production_constrained():
    prod = np.array([100.0, 50.0, 0.0])
    attr = np.array([1.0, 1.0, 2.0])
    cost = np.array([[0.0, 5.0, 5.0], [5.0, 0.0, 1.0], [5.0, 1.0, 0.0]])
    t = G.gravity(prod, attr, cost, gamma=0.1, min_cost=2.0)
    assert np.allclose(t.sum(axis=1), [100.0, 50.0, 0.0])
    assert t[1, 0] == 50.0  # ゾーン 2 は近すぎる(費用 1 < 2)ので、行 1 はすべてゾーン 0 へ
    assert np.all(np.diag(t) == 0)
    assert t[0, 2] > t[0, 1]  # 同じ費用なら集中の大きい方へ


def test_fit_cordon_recovers_totals():
    rng = np.random.default_rng(0)
    loads = rng.uniform(0, 1e-3, size=(50, 4))
    in_total, out_total = 10000.0, 9000.0
    truth = G.cordon_block_totals(20000.0, 0.3, in_total, out_total)
    obs = loads @ truth
    a, e = G.fit_cordon(loads, obs, in_total, out_total)
    assert abs(a - 20000.0) < 1.0 and abs(e - 0.3) < 1e-4
    ii, ie, ei, ee = G.cordon_block_totals(a, e, in_total, out_total)
    assert abs(ei + ee - in_total) < 1e-6 and abs(ie + ee - out_total) < 1e-6


def test_hourly_profile_and_taz_relation_xml():
    counts = [[None] * 5 + [10, 20, 40, 30], [None] * 5 + [None, 5, 10, 5]]
    p = G.hourly_profile(counts, [5, 6, 8], ref_hour=7)
    assert p == {5: 0.25, 6: 0.5, 8: 0.75}  # 5 時の欠ける 2 本目は使わない
    xml = G.taz_relation_xml([(0, 3600, {("a", "b"): 2.0, ("a", "c"): 0.0})])
    assert '<tazRelation from="a" to="b" count="2.000"/>' in xml and '"c"' not in xml


def test_observation_groups_and_fit_metrics():
    obs = E.merge_observations(
        {"1": dict(unit="U1", direction="up", counts=[100.0]),
         "2": dict(unit="U1", direction="up", counts=[300.0])},
        {"1": dict(unit="jartic:9", direction="北", counts=[180.0])},
    )  # fmt: skip
    gs = E.observation_groups(obs, 0)
    by = {(g["source"], g["unit"]): g for g in gs}
    assert by[("census", "U1")]["obs"] == 200.0 and by[("census", "U1")]["edges"] == ["1", "2"]
    assert by[("detector", "jartic:9")]["obs"] == 180.0
    m = E.fit_metrics([110, 190, 410], [100, 200, 400])
    assert m["n"] == 3 and m["r"] > 0.99 and abs(m["slope"] - 1.0) < 0.05
    assert m["geh_ok_share"] == 1.0
