"""実測需要(sim/demand.py と netsim の流入レート)の単体テスト。"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from sapporo_sim import config as C
from sapporo_sim.sim.demand import DEFAULT_RATE, hourly_rate
from sapporo_sim.sim.netsim import DT, NetSim

TH = math.radians(C.GRID_BEARING_DEG)


def rot(x, y):
    ox, oy = C.GRID_ORIGIN
    return (
        ox + x * math.cos(TH) - y * math.sin(TH),
        oy + x * math.sin(TH) + y * math.cos(TH),
    )


def section(up_s, up_l=None, down_s=None, down_l=None):
    def arr(v):
        if v is None:
            return None
        a = [None] * 24
        a[8] = v
        return a

    return {
        "up": {"s": arr(up_s), "l": arr(up_l)},
        "down": {"s": arr(down_s), "l": arr(down_l)},
    }


def test_hourly_rate_direction_mean():
    # 上下とも観測: (1000+100 + 500+50) / 2 = 825
    assert hourly_rate(section(1000, 100, 500, 50), 8) == 825.0
    # 片方向のみ観測: そのまま使う
    assert hourly_rate(section(1000, 100), 8) == 1100.0
    # 大型欠測は0として足す
    assert hourly_rate(section(1000, None, 500, None), 8) == 750.0
    # 観測なし(夜間など)は None
    assert hourly_rate(section(1000), 20) is None
    assert hourly_rate(None, 8) is None
    assert hourly_rate({}, 8) is None


def line_with_census(census_id=""):
    pts = {0: (0, 0), 1: (400, 0)}
    nodes = [dict(nid=k, x=rot(*v)[0], y=rot(*v)[1], has_signal=False) for k, v in pts.items()]
    g = [rot(*pts[0]), rot(*pts[1])]
    common = dict(
        geometry=g,
        length=400.0,
        speed_kmh=50,
        category="3",
        n_sublanes=4,
        right_turn_lane=0,
        census_id=census_id,
    )
    edges = [
        dict(eid=0, frm=0, to=1, linked_edge=1, **common),
        dict(eid=1, frm=1, to=0, linked_edge=0, **{**common, "geometry": g[::-1]}),
    ]
    return nodes, edges


def test_entry_rate_from_census():
    nodes, edges = line_with_census(census_id="X1")
    hourly = {"X1": section(1200, 0, 600, 0)}  # 方向平均 900台/時
    sim = NetSim(nodes, edges, entry_hourly=hourly, entry_hour=8)
    assert sim.n_census_entries == 2  # 両端とも流入 Edge
    assert abs(sim.entry_rate[0] * 3600 - 900.0) < 1e-6


def test_entry_rate_fallback_to_default():
    # 観測の無い時刻・裏付けの無い Edge は分類既定値
    nodes, edges = line_with_census(census_id="X1")
    hourly = {"X1": section(1200)}
    sim = NetSim(nodes, edges, entry_hourly=hourly, entry_hour=20)
    assert sim.n_census_entries == 0
    assert abs(sim.entry_rate[0] * 3600 - DEFAULT_RATE["minor"]) < 1e-6
    nodes, edges = line_with_census(census_id="")
    sim = NetSim(nodes, edges, entry_hourly=hourly, entry_hour=8)
    assert sim.n_census_entries == 0


def test_edge_flow_counts_crossings():
    """境界流出も含めて Edge 下流端の通過台数が数えられる。"""
    nodes, edges = line_with_census()
    sim = NetSim(nodes, edges, seed=3, entry_nodes={0})
    for _ in range(int(300 / DT)):
        sim.step()
    assert sim.edge_flow.get(0, 0) == sim.n_exited > 0  # 0→1 の末端で全数流出
