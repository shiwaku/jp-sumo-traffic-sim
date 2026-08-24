"""netsim のサブレーン動力学(Phase 2-②)の単体テスト。合成ネットワークのみ。"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from sapporo_sim import config as C
from sapporo_sim.sim.mobil import overlaps
from sapporo_sim.sim.netsim import DT, RT_ZONE_M, NetSim
from sapporo_sim.sim.simple import CAR_LEN, STOPLINE_M

TH = math.radians(C.GRID_BEARING_DEG)


def rot(x, y):
    ox, oy = C.GRID_ORIGIN
    return (
        ox + x * math.cos(TH) - y * math.sin(TH),
        oy + x * math.sin(TH) + y * math.cos(TH),
    )


def line_network(length=400.0, n_sublanes=4, right_turn_lane=0):
    """2ノードを結ぶ双方向1本のネットワーク(グリッド方位の東西)。"""
    pts = {0: (0, 0), 1: (length, 0)}
    nodes = [dict(nid=k, x=rot(*v)[0], y=rot(*v)[1], has_signal=False) for k, v in pts.items()]
    g = [rot(*pts[0]), rot(*pts[1])]
    edges = [
        dict(
            eid=0,
            frm=0,
            to=1,
            geometry=g,
            length=length,
            speed_kmh=50,
            category="3",
            linked_edge=1,
            n_sublanes=n_sublanes,
            right_turn_lane=right_turn_lane,
        ),
        dict(
            eid=1,
            frm=1,
            to=0,
            geometry=g[::-1],
            length=length,
            speed_kmh=50,
            category="3",
            linked_edge=0,
            n_sublanes=n_sublanes,
            right_turn_lane=0,
        ),
    ]
    return nodes, edges


def cross_network(n_sublanes=2, rt_west=0, has_signal=False):
    """中心(0,0)に4方向から150mの腕が伸びる十字。すべて双方向。"""
    pts = {0: (0, 0), 1: (-150, 0), 2: (150, 0), 3: (0, -150), 4: (0, 150)}
    nodes = [
        dict(nid=k, x=rot(*v)[0], y=rot(*v)[1], has_signal=(k == 0 and has_signal))
        for k, v in pts.items()
    ]
    edges = []
    eid = 0
    for a, b in [(1, 0), (0, 2), (3, 0), (0, 4)]:
        g = [rot(*pts[a]), rot(*pts[b])]
        rt = rt_west if (a, b) == (1, 0) else 0
        edges.append(
            dict(
                eid=eid,
                frm=a,
                to=b,
                geometry=g,
                length=150.0,
                speed_kmh=40,
                category="3",
                linked_edge=eid + 1,
                n_sublanes=n_sublanes,
                right_turn_lane=rt,
            )
        )
        edges.append(
            dict(
                eid=eid + 1,
                frm=b,
                to=a,
                geometry=g[::-1],
                length=150.0,
                speed_kmh=40,
                category="3",
                linked_edge=eid,
                n_sublanes=n_sublanes,
                right_turn_lane=0,
            )
        )
        eid += 2
    return nodes, edges


def put(sim, eid, pos, speed, turn, sublane=0, v0=None):
    e = sim.edges[eid]
    veh = sim._make_vehicle(e)
    veh.pos, veh.speed, veh.turn, veh.sublane = pos, speed, turn, sublane
    if v0 is not None:
        veh.v0 = v0
    e.vehicles.append(veh)
    return veh


def test_effective_sublanes_by_scenario():
    """冬季は実効サブレーンが減る(4 → 3 = 2車線が実質1.5車線)。"""
    nodes, edges = line_network(n_sublanes=4)
    sn = NetSim(nodes, edges, scenario="normal", entry_nodes=set())
    sw = NetSim(nodes, edges, scenario="winter", entry_nodes=set())
    assert sn.edges[0].n_sub_eff == 4
    assert sw.edges[0].n_sub_eff == 3
    # 1サブレーン(細街路)は 0 にはならない
    nodes, edges = line_network(n_sublanes=1)
    sw = NetSim(nodes, edges, scenario="winter", entry_nodes=set())
    assert sw.edges[0].n_sub_eff == 1


def test_overtake_on_edge():
    """実効4サブレーン(2車線)の Edge 上で速い車が遅い車を右から抜く。"""
    nodes, edges = line_network(length=400.0, n_sublanes=4)
    sim = NetSim(nodes, edges, seed=1, entry_nodes=set())
    slow = put(sim, 0, 150.0, 4.0, turn=0, v0=4.0)
    fast = put(sim, 0, 30.0, 12.0, turn=0)
    passed = False
    for _ in range(int(60 / DT)):
        sim.step()
        e = sim.edges[0]
        if slow not in e.vehicles or fast not in e.vehicles:
            break
        if abs(fast.pos - slow.pos) <= CAR_LEN:  # 併走中は必ず右側(重ならない)
            assert fast.sublane >= slow.sublane + slow.width, "左から抜いた(左側通行違反)"
        if fast.pos > slow.pos + CAR_LEN:
            passed = True
    assert passed, "Edge 上で追い越せていない"
    assert sim.n_lane_changes > 0


def test_single_file_when_narrow():
    """実効2サブレーン(乗用車1台分)では従来どおり単縦列で車線変更なし。"""
    nodes, edges = line_network(length=400.0, n_sublanes=2)
    sim = NetSim(nodes, edges, seed=1, entry_nodes=set())
    put(sim, 0, 150.0, 4.0, turn=0, v0=4.0)
    fast = put(sim, 0, 30.0, 12.0, turn=0)
    for _ in range(int(40 / DT)):
        sim.step()
    assert sim.n_lane_changes == 0
    assert fast.sublane == 0


def test_rt_vehicle_moves_to_rt_lane():
    """右折車は停止線手前で右折専用レーン(右端)へ寄る。"""
    nodes, edges = cross_network(n_sublanes=4, rt_west=1)
    sim = NetSim(nodes, edges, seed=1, entry_nodes=set())
    sim._oncoming_clear = lambda edge: False  # 対向が絶えない状況を固定
    rt = put(sim, 0, 150.0 - RT_ZONE_M - 20.0, 8.0, turn=2)
    for _ in range(int(30 / DT)):
        sim.step()
    e = sim.edges[0]
    assert rt in e.vehicles, "対向が塞がっているのに右折してしまった"
    assert rt.sublane + rt.width == e.n_sub_eff, f"専用レーンに寄っていない: {rt.sublane}"
    assert rt.speed < 0.5 and rt.pos > e.length - STOPLINE_M - 5.0, "停止線で待っていない"


def test_rt_lane_lets_straight_pass():
    """右折専用車線があれば、右折待ちの脇を直進車が通過できる。"""
    nodes, edges = cross_network(n_sublanes=4, rt_west=1)
    sim = NetSim(nodes, edges, seed=1, entry_nodes=set())
    sim._oncoming_clear = lambda edge: False
    rt = put(sim, 0, 100.0, 8.0, turn=2)
    st = put(sim, 0, 60.0, 8.0, turn=0)
    for _ in range(int(40 / DT)):
        sim.step()
    e = sim.edges[0]
    assert rt in e.vehicles, "右折車が交差点へ入ってしまった"
    assert st not in e.vehicles, "直進車が右折待ちの後ろから抜けられない"
    assert st in sim.edges[2].vehicles or sim.n_exited > 0  # 直進先(0→2)へ移った


def test_no_rt_lane_straight_blocked():
    """専用車線が無ければ右折待ちが後続の直進も止める(実態どおり)。"""
    nodes, edges = cross_network(n_sublanes=2, rt_west=0)
    sim = NetSim(nodes, edges, seed=1, entry_nodes=set())
    sim._oncoming_clear = lambda edge: False
    rt = put(sim, 0, 100.0, 8.0, turn=2)
    st = put(sim, 0, 60.0, 8.0, turn=0)
    for _ in range(int(40 / DT)):
        sim.step()
    e = sim.edges[0]
    assert rt in e.vehicles and st in e.vehicles, "直進車が右折待ちを追い越した"
    assert st.pos < rt.pos, "順序が入れ替わった"


def test_conservation_and_no_overlap_sublanes():
    """4サブレーン+信号+高需要でも保存則と(スパン内)最小車間を保つ。"""
    nodes, edges = cross_network(n_sublanes=4, has_signal=True)
    sim = NetSim(nodes, edges, seed=7, demand_scale=3.0)
    for _ in range(int(300 / DT)):
        sim.step()
        for e in sim.edges.values():
            vs = e.vehicles
            for i, a in enumerate(vs):
                for b in vs[i + 1 :]:
                    if overlaps(a.sublane, a.width, b.sublane, b.width):
                        assert abs(a.pos - b.pos) >= CAR_LEN, (
                            f"t={sim.t}: 車間割れ eid={e.eid} {a.pos:.2f}/{b.pos:.2f}"
                        )
                assert 0 <= a.sublane and a.sublane + a.width <= e.n_sub_eff, (
                    f"実効帯の外に出た: {a.sublane}"
                )
    on_road = sum(len(e.vehicles) for e in sim.edges.values())
    assert sim.n_spawned == on_road + sim.n_exited
    assert sim.n_exited > 0


def test_snapshot_lateral_offset():
    """描画座標は進行方向左(左側通行の自分の側)へオフセットされる。

    グリッド方位の東西道路: 東行きは北側、西行きは南側に出る。
    """
    nodes, edges = line_network(length=400.0, n_sublanes=4)
    sim = NetSim(nodes, edges, seed=1, entry_nodes=set())
    east = put(sim, 0, 200.0, 10.0, turn=0)  # 0→1 東行き
    west = put(sim, 1, 200.0, 10.0, turn=0)  # 1→0 西行き

    # グリッド座標系に戻して比較する
    def to_grid(x, y):
        ox, oy = C.GRID_ORIGIN
        dx, dy = x - ox, y - oy
        return (
            dx * math.cos(-TH) - dy * math.sin(-TH),
            dx * math.sin(-TH) + dy * math.cos(-TH),
        )

    e0, e1 = sim.edges[0], sim.edges[1]
    _, ye = to_grid(*e0.xy_at(east.pos, e0.lateral_of(east)))
    _, yw = to_grid(*e1.xy_at(west.pos, e1.lateral_of(west)))
    assert ye > 0.5, f"東行きが中心線の北側にいない: {ye:.2f}"
    assert yw < -0.5, f"西行きが中心線の南側にいない: {yw:.2f}"
