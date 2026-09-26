"""単方向エッジ化(directed.py)と車線割り当て(lanes.py)の単体テスト。"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim.network import lanes as L
from jp_sumo_traffic_sim.network.directed import build_edges


def link(a, b, coords, oneway="", **kw):
    d = dict(a=a, b=b, geometry=coords, oneway=oneway, width="3", category="3")
    d.update(kw)
    return d


def test_twoway_makes_linked_pair():
    edges = build_edges([link(1, 2, [(0, 0), (100, 0)])])
    assert len(edges) == 2
    e1, e2 = edges
    assert (e1.frm, e1.to) == (1, 2)
    assert (e2.frm, e2.to) == (2, 1)
    assert e1.linked_edge == e2.eid and e2.linked_edge == e1.eid
    assert e2.geometry == [(100, 0), (0, 0)]  # 進行方向順
    assert e1.oneway_source == "assumed_twoway"


def test_oneway_f_and_r():
    ef = build_edges([link(1, 2, [(0, 0), (100, 0)], oneway="F")])
    assert len(ef) == 1 and (ef[0].frm, ef[0].to) == (1, 2)
    assert ef[0].linked_edge is None and ef[0].oneway_source == "jartic"
    er = build_edges([link(1, 2, [(0, 0), (100, 0)], oneway="R")])
    assert len(er) == 1 and (er[0].frm, er[0].to) == (2, 1)
    assert er[0].geometry == [(100, 0), (0, 0)]


def test_lanes_from_census_and_width():
    # センサス両方向8車線 → 片側4(実測はキャップしない)
    e1, e2 = build_edges([link(1, 2, [(0, 0), (100, 0)], census_id="X")])
    L.assign_lanes(e1, dict(n_lanes=8, w_carriageway=32.0, right_turn_lane=1))
    assert e1.attrs["n_lanes"] == 4
    assert e1.attrs["lane_source"] == "census"
    assert e1.attrs["right_turn_lane"] == 1
    # 右折専用車線のコードは 1 だけが「あり」(2=なし・3=右折禁止・4=調査路線が右折)
    for code, want in ((2, 0), (3, 0), (4, 0), (1.0, 1)):
        (e5, _) = build_edges([link(1, 2, [(0, 0), (100, 0)], census_id="X")])
        L.assign_lanes(e5, dict(n_lanes=4, w_carriageway=13.0, right_turn_lane=code))
        assert e5.attrs["right_turn_lane"] == want, code
    # 幅員区分5(19.5m+)の両方向 → 歩道分を引いて片側最大3にキャップ
    (e3, e4) = build_edges([link(1, 2, [(0, 0), (100, 0)], width="5")])
    L.assign_lanes(e3, None)
    assert 1 <= e3.attrs["n_lanes"] <= 3
    assert e3.attrs["lane_source"] == "width_assumed"
    # 幅員区分4(代表 16m)の双方向道路は片側 2 車線(四捨五入)
    (e6, _) = build_edges([link(1, 2, [(0, 0), (100, 0)], width="4")])
    L.assign_lanes(e6, None)
    assert e6.attrs["n_lanes"] == 2
    # サブレーンは 1.75m 分割
    assert e3.attrs["n_sublanes"] == int(e3.attrs["carriageway_m"] // 1.75)


def test_census_lane_guard_rejects_sideroad_mismatch():
    """一方通行の側道が断面8車線のセンサスを引き当てたら幅員推定へ落とす。"""
    (e,) = build_edges([link(1, 2, [(0, 0), (100, 0)], oneway="F", width="2")])
    L.assign_lanes(e, dict(n_lanes=8, w_carriageway=32.0))
    assert e.attrs["lane_source"] == "width_assumed"
    assert e.attrs["n_lanes"] <= 2


def test_speed_priority():
    (e,) = build_edges([link(1, 2, [(0, 0), (100, 0)], oneway="F", speed_kmh=50)])
    L.assign_speed(e, dict(speed_limit=40))
    assert (e.attrs["speed_kmh"], e.attrs["speed_source"]) == (50, "jartic")
    (e2,) = build_edges([link(1, 2, [(0, 0), (100, 0)], oneway="F", speed_kmh=0)])
    L.assign_speed(e2, dict(speed_limit=40))
    assert (e2.attrs["speed_kmh"], e2.attrs["speed_source"]) == (40, "census")
    (e3,) = build_edges([link(1, 2, [(0, 0), (100, 0)], oneway="F", speed_kmh=0)])
    L.assign_speed(e3, None)
    assert e3.attrs["speed_source"] == "assumed"
