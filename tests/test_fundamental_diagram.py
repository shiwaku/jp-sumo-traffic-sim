"""基本図(fundamental diagram)の再現テスト。

論文の検証手順(docs/design.md Phase 2)を踏襲する:
環状路で車両数を段階的に増やし、平均速度が
自由流 → 減速 → 飽和(速度ほぼ0)と推移することを確認する。
ここが再現できなければ実装にバグがある。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from sapporo_sim.sim.simple import CAR_LEN, DT, Vehicle, idm_acc

RING_M = 1000.0
V0 = 13.9  # 50km/h


def ring_mean_speed(n_vehicles: int, sim_s: float = 300.0) -> float:
    """環状路に n 台を等間隔に置き、平均速度の定常値を返す。"""
    vehicles = [
        Vehicle(pos=i * RING_M / n_vehicles, speed=0.0, v0=V0, a=1.2, b=1.8, s0=2.5, T=1.3, turn=0)
        for i in range(n_vehicles)
    ]
    steps = int(sim_s / DT)
    tail = []
    for k in range(steps):
        order = sorted(range(n_vehicles), key=lambda i: -vehicles[i].pos)
        accs = [0.0] * n_vehicles
        for idx, i in enumerate(order):
            j = order[idx - 1]  # 先行車(先頭の先行は末尾 = 環状)
            gap = (vehicles[j].pos - vehicles[i].pos) % RING_M - CAR_LEN
            accs[i] = idm_acc(vehicles[i], gap, vehicles[i].speed - vehicles[j].speed)
        for i, veh in enumerate(vehicles):
            veh.speed = max(0.0, veh.speed + accs[i] * DT)
            veh.pos = (veh.pos + veh.speed * DT) % RING_M
        if k >= steps * 3 // 4:
            tail.append(sum(v.speed for v in vehicles) / n_vehicles)
    return sum(tail) / len(tail)


def test_fundamental_diagram():
    # 密度 [台/km]: 10 → 自由流 / 60 → 減速 / 130 → ほぼ飽和
    v_free = ring_mean_speed(10)
    v_mid = ring_mean_speed(60)
    v_jam = ring_mean_speed(130)
    assert v_free > 0.9 * V0, f"低密度で自由流にならない: {v_free:.1f}"
    assert v_free > v_mid > v_jam, (
        f"速度が密度で単調に落ちない: {v_free:.1f}/{v_mid:.1f}/{v_jam:.1f}"
    )
    assert v_mid < 0.7 * V0, f"中密度で減速しない: {v_mid:.1f}"
    assert v_jam < 2.0, f"高密度で飽和しない: {v_jam:.1f}"


def test_jam_density_consistency():
    """完全停止密度: 車頭間隔が car_len + s0 を下回ると動けない。"""
    # 1000m / (4.5 + 2.5) = 142.8台 が理論上の完全飽和
    v = ring_mean_speed(140, sim_s=120.0)
    assert v < 0.5
