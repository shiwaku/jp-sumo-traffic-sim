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
            assert a.pos - b.pos >= 4.5 - 0.2, f"車両が重なった: {a.pos} vs {b.pos}"


def test_winter_slower():
    """冬季は平常時より遅い(v0 低減と車間拡大の帰結)。"""
    vn = run("normal").stats()["mean_speed_ms"]
    vw = run("winter").stats()["mean_speed_ms"]
    assert vw < vn


def test_spillback_occurs_in_winter():
    """冬季は流入が塞がれる(スピルバックの前兆)が観測される。"""
    sim = run("winter", seconds=420.0)
    assert sim.n_blocked_spawn > 0
