"""実ネットワークシミュレーション(sim/netsim.py)の単体テスト。合成ネットワークのみ。"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from sapporo_sim import config as C
from sapporo_sim.sim.netsim import DT, NetSim

# グリッド方位に合わせた十字ネットワークを作る(軸分類が E/W/N/S に揃うように)
TH = math.radians(C.GRID_BEARING_DEG)


def rot(x, y):
    ox, oy = C.GRID_ORIGIN
    return (
        ox + x * math.cos(TH) - y * math.sin(TH),
        oy + x * math.sin(TH) + y * math.cos(TH),
    )


def cross_network(has_signal=True):
    """中心(0,0)に4方向から100mの腕が伸びる十字。すべて双方向。"""
    pts = {0: (0, 0), 1: (-100, 0), 2: (100, 0), 3: (0, -100), 4: (0, 100)}
    nodes = [
        dict(nid=k, x=rot(*v)[0], y=rot(*v)[1], has_signal=(k == 0 and has_signal))
        for k, v in pts.items()
    ]
    edges = []
    eid = 0
    for a, b in [(1, 0), (0, 2), (3, 0), (0, 4)]:
        g = [(pts[a]), (pts[b])]
        g = [rot(*p) for p in g]
        e1 = dict(
            eid=eid,
            frm=a,
            to=b,
            geometry=g,
            length=100.0,
            speed_kmh=40,
            category="3",
            linked_edge=eid + 1,
        )
        e2 = dict(
            eid=eid + 1,
            frm=b,
            to=a,
            geometry=g[::-1],
            length=100.0,
            speed_kmh=40,
            category="3",
            linked_edge=eid,
        )
        edges += [e1, e2]
        eid += 2
    return nodes, edges


def test_turn_classification_and_no_uturn():
    nodes, edges = cross_network()
    sim = NetSim(nodes, edges, seed=1)
    east = sim.edges[0]  # 1→0 東行き
    # 直進 = 0→2(東)、左折 = 0→4(北)、右折 = 0→3(南)
    assert east.nexts[0].to.nid == 2
    assert east.nexts[1].to.nid == 4
    assert east.nexts[2].to.nid == 3
    # U ターン(0→1)は候補に無い
    assert all(e.to.nid != 1 for e in east.nexts.values())
    # 対向は 2→0 の西行き
    assert east.oncoming is not None and east.oncoming.frm.nid == 2


def test_signal_axes_and_phase_char():
    nodes, edges = cross_network()
    sim = NetSim(nodes, edges, seed=1)
    center = sim.nodes[0]
    center.offset, center.cycle, center.g_half = 0.0, 120.0, 57.0
    east = sim.edges[0]
    north = sim.edges[4]  # 3→0 北行き
    assert east.axis == "EW" and north.axis == "NS"
    assert east.green(10.0) and not north.green(10.0)
    assert not east.green(70.0) and north.green(70.0)
    assert center.phase_char(58.0) == "2"  # 全赤


def test_unsignalized_always_green():
    nodes, edges = cross_network(has_signal=False)
    sim = NetSim(nodes, edges, seed=1)
    assert sim.edges[0].green(5.0) and sim.edges[4].green(5.0)


def test_stop_sign_requires_full_stop():
    nodes, edges = cross_network(has_signal=False)
    sim = NetSim(nodes, edges, seed=1, stop_edges={0})
    e = sim.edges[0]
    veh = sim._make_vehicle(e)
    veh.pos, veh.speed, veh.turn = 40.0, 10.0, 0
    e.vehicles.append(veh)
    stop = e.length - 14.0
    # 一時停止するまで停止線を越えない
    crossed_without_stop = False
    stopped = False
    for _ in range(int(30 / DT)):
        sim.step()
        if not e.vehicles:
            break
        v = e.vehicles[0]
        if v.speed < 0.3:
            stopped = True
        if v.pos > stop + 0.5 and not stopped:
            crossed_without_stop = True
    assert stopped and not crossed_without_stop


def test_conservation_and_no_overlap():
    nodes, edges = cross_network()
    sim = NetSim(nodes, edges, seed=7, demand_scale=3.0)
    for _ in range(int(300 / DT)):
        sim.step()
        for e in sim.edges.values():
            for a, b in zip(e.vehicles, e.vehicles[1:], strict=False):
                assert a.pos - b.pos >= 4.5
    on_road = sum(len(e.vehicles) for e in sim.edges.values())
    assert sim.n_spawned == on_road + sim.n_exited
    assert sim.n_exited > 0
