"""グリッドシミュレーション全体の健全性テスト。"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from sapporo_sim.sim.simple import GridSim


def run(scenario: str, seconds: float = 240.0) -> GridSim:
    sim = GridSim(scenario=scenario, seed=7)
    for _ in range(int(seconds / sim_dt())):
        sim.step()
    return sim


def sim_dt() -> float:
    from sapporo_sim.sim.simple import DT

    return DT


def test_network_shape():
    sim = GridSim()
    assert len(sim.nodes) == 13 * 12  # EW 13本 × NS 12本
    assert len(sim.links) == 574
    assert len(sim.entries) == 50  # 13*2 + 12*2


def test_conservation():
    """車両が消えない・湧かない: 発生 = 走行中 + 流出。"""
    sim = run("normal")
    on_road = sum(len(ln.vehicles) for ln in sim.links)
    assert sim.n_spawned == on_road + sim.n_exited


def test_no_overlap():
    """同一リンク上で車両が重ならない。"""
    sim = run("normal")
    for ln in sim.links:
        for a, b in zip(ln.vehicles, ln.vehicles[1:], strict=False):
            assert a.pos - b.pos >= 4.5, f"車両が重なった: {a.pos} vs {b.pos}"


def test_no_overlap_under_saturation():
    """飽和状態(冬季・需要3倍)でも最小車間と満杯リンク不進入を保証する。

    停止線通過後の車両が満杯リンクへめり込む不具合(issue #2)と、
    重なり補正が車間を保証しない不具合(issue #3)の回帰テスト。
    """
    sim = GridSim(scenario="winter", seed=7, demand_scale=3.0)
    for _ in range(int(600.0 / sim_dt())):
        sim.step()
        for ln in sim.links:
            vs = ln.vehicles
            for a, b in zip(vs, vs[1:], strict=False):
                assert a.pos - b.pos >= 4.5, (
                    f"t={sim.t}: 車間割れ link={ln.lid} {a.pos:.2f} vs {b.pos:.2f}"
                )
            if vs:
                assert vs[0].pos <= ln.length + 1e-6, f"t={sim.t}: リンク端を越えて滞留"
                assert vs[-1].pos >= 0.0, f"t={sim.t}: リンク始端より手前に配置"


def test_boundary_turn_exits():
    """境界で転回先が無い方向は強制直進せず、区域外への流出になる(issue #4)。"""
    sim = GridSim()
    jmax = len(sim.vs) - 1
    ln = sim.grid[(0, jmax)].out["E"]  # 北端(北5条通)の東行き
    veh = sim._make_vehicle(ln)
    veh.turn = 1  # 左折(北) → 区域外
    assert sim._next_link(ln, veh) is None
    veh.turn = 0  # 直進(東) → 区域内に残る
    assert sim._next_link(ln, veh) is ln.to.out["E"]
    veh.turn = 2  # 右折(南) → 区域内に残る
    assert sim._next_link(ln, veh) is ln.to.out["S"]


def test_winter_slower():
    """冬季は平常時より遅い(v0 低減と車間拡大の帰結)。"""
    vn = run("normal").stats()["mean_speed_ms"]
    vw = run("winter").stats()["mean_speed_ms"]
    assert vw < vn


def test_spillback_occurs_in_winter():
    """冬季は流入が塞がれる(スピルバックの前兆)が観測される。"""
    sim = run("winter", seconds=420.0)
    assert sim.n_blocked_spawn > 0
