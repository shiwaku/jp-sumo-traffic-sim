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


def test_oneway_blocks_reverse_links():
    """一方通行の逆走側リンクが除去され、転回・流入も選ばれない。"""
    base = GridSim()
    j = 6  # 南1条通
    v = base.vs[j]
    # 全長を東行き一方通行にする(通行方向順の頂点列・投影座標)
    line = [base.grid_to_proj(base.us[0] - 50, v), base.grid_to_proj(base.us[-1] + 50, v)]
    sim = GridSim(oneways=[line])

    n_seg = len(base.us) - 1
    assert sim.n_oneway_lines_matched == 1
    assert sim.n_oneway_blocked == n_seg  # 西行きが全区間消える
    assert len(sim.links) == len(base.links) - n_seg
    for i in range(1, len(sim.us)):
        assert "W" not in sim.grid[(i, j)].out
        assert "E" in sim.grid[(i - 1, j)].out
    # 東端の西行き流入が消えている
    imax = len(sim.us) - 1
    assert all(not (ln.frm is sim.grid[(imax, j)] and ln.heading == "W") for ln in sim.entries)
    # 内部ノードの転回サンプリングが封鎖方向(北向き到達時の左折=W)を選ばない
    ln = sim.grid[(5, j - 1)].out["N"]
    picks = {sim._sample_turn(ln) for _ in range(300)}
    assert 1 not in picks
    assert picks <= {0, 2}


def test_right_turn_yields_to_oncoming():
    """右折車は対向の直進車にギャップ受容で道を譲る(issue #5)。"""
    from sapporo_sim.sim.simple import STOPLINE_M

    sim = GridSim()
    n = sim.grid[(5, 5)]
    ln = sim.grid[(4, 5)].out["E"]  # (4,5)→(5,5) 東行き
    onc = sim._oncoming_link(ln)
    assert onc is sim.grid[(6, 5)].out["W"]
    assert onc.to is n

    # 信号を常時東西青にして信号待ちの影響を消す
    n.offset, n.cycle, n.g_ew = 0.0, 1e6, 1e6 - 10.0

    stop = ln.length - STOPLINE_M
    rt = sim._make_vehicle(ln)
    rt.pos, rt.speed, rt.turn = stop - 1.0, 0.0, 2  # 停止線1m手前の右折車
    ln.vehicles.append(rt)

    # 対向直進車: 停止線まで30m・10m/s(到達3秒 < 受容ギャップ5.5秒) → 待つ
    on = sim._make_vehicle(onc)
    on.pos, on.speed, on.turn, on.v0 = (onc.length - STOPLINE_M) - 30.0, 10.0, 0, 10.0
    onc.vehicles.append(on)
    assert not sim._oncoming_clear(ln)
    for _ in range(4):  # 2秒
        sim.step()
    assert rt.pos <= stop + 1e-6, "対向車が接近中なのに停止線を越えた"

    # 対向右折車は交錯しないので塞がない
    on.turn = 2
    assert sim._oncoming_clear(ln)

    # 対向が捌けたら発進して停止線を越える
    onc.vehicles.clear()
    for _ in range(20):  # 10秒
        sim.step()
    assert (not ln.vehicles) or ln.vehicles[0] is not rt or rt.pos > stop


def test_winter_slower():
    """冬季は平常時より遅い(v0 低減と車間拡大の帰結)。"""
    vn = run("normal").stats()["mean_speed_ms"]
    vw = run("winter").stats()["mean_speed_ms"]
    assert vw < vn


def test_spillback_occurs_in_winter():
    """冬季は流入が塞がれる(スピルバックの前兆)が観測される。"""
    sim = run("winter", seconds=420.0)
    assert sim.n_blocked_spawn > 0
