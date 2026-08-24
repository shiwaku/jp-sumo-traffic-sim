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
    assert east.best(0).to.nid == 2
    assert east.best(1).to.nid == 4
    assert east.best(2).to.nid == 3
    # U ターン(0→1)は候補に無い
    assert all(c.to.nid != 1 for cands in east.nexts.values() for _, c in cands)
    # 対向は 2→0 の西行き
    assert east.oncoming is not None and east.oncoming.frm.nid == 2


def edge(eid, frm, to, geom, linked=-1):
    return dict(
        eid=eid,
        frm=frm,
        to=to,
        geometry=geom,
        length=100.0,
        speed_kmh=40,
        category="3",
        linked_edge=linked,
    )


def test_parallel_straight_candidates_both_fed():
    """並走する直進候補(上下分離など)は両方に交通が配分される。"""
    pts = {0: (-100, 0), 1: (0, 0), 2: (100, 18), 3: (100, -18)}
    nodes = [dict(nid=k, x=x, y=y, has_signal=False) for k, (x, y) in pts.items()]
    edges = [
        edge(0, 0, 1, [pts[0], pts[1]]),
        edge(1, 1, 2, [pts[1], pts[2]]),  # 直進 +10°
        edge(2, 1, 3, [pts[1], pts[3]]),  # 直進 -10°
    ]
    sim = NetSim(nodes, edges, seed=1, entry_nodes={0})
    e0 = sim.edges[0]
    assert len(e0.nexts[0]) == 2  # 両方が直進候補
    picks = {sim._sample_next(e0)[1].eid for _ in range(200)}
    assert picks == {1, 2}, f"片方に偏った: {picks}"


def test_hairpin_is_sink():
    """転回先が鋭角折返ししか無いノードは吸い込み口(流出)になる。

    創成トンネル坑口のような「ネットワークの外へ続く道」の内部端。
    折返しを許すと上下線を往復する閉回路がデッドロックを生む。
    """
    pts = {0: (-100, 0), 1: (0, 0), 2: (-95, -35)}
    nodes = [dict(nid=k, x=x, y=y, has_signal=False) for k, (x, y) in pts.items()]
    edges = [
        edge(0, 0, 1, [pts[0], pts[1]], linked=1),
        edge(1, 1, 0, [pts[1], pts[0]], linked=0),
        edge(2, 1, 2, [pts[1], pts[2]], linked=3),  # 約160° = U 扱いの折返し
        edge(3, 2, 1, [pts[2], pts[1]], linked=2),
    ]
    sim = NetSim(nodes, edges, seed=1, entry_nodes={0})
    e0 = sim.edges[0]
    assert not e0.nexts  # 転回先なし = 流出
    veh = sim._make_vehicle(e0)
    veh.pos, veh.speed = 99.0, 10.0
    e0.vehicles.append(veh)
    for _ in range(int(5 / DT)):
        sim.step()
    assert veh not in e0.vehicles and sim.n_exited == 1


def test_exit_next_covers_network():
    """全 Edge から最短ホップで流出点へ向かう経路が引ける(巡回上限用)。"""
    nodes, edges = cross_network()
    sim = NetSim(nodes, edges, seed=1)
    for e in sim.edges.values():
        assert (not e.nexts) or e.eid in sim.exit_next


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
