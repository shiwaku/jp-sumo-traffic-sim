"""IDM 単体の受け入れテスト(docs/architecture.md §8)。"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim.sim.simple import DT, Vehicle, idm_acc


def make(v0=13.9, speed=0.0):
    return Vehicle(pos=0.0, speed=speed, v0=v0, a=1.2, b=1.8, s0=2.5, T=1.3, turn=0)


def test_idm_freeflow():
    """先行車なし単路: v -> v0 に単調収束し、オーバーシュートしない。"""
    veh = make(v0=13.9)
    prev = veh.speed
    for _ in range(2000):
        acc = idm_acc(veh, gap=1e9, dv=0.0)
        veh.speed = max(0.0, veh.speed + acc * DT)
        assert veh.speed <= veh.v0 + 1e-6
        assert veh.speed >= prev - 1e-9  # 単調非減少
        prev = veh.speed
    assert abs(veh.speed - veh.v0) < 0.05


def test_idm_car_following():
    """前方が定速: 車間が s* = s0 + v*T に収束する。"""
    lead_speed = 8.0
    veh = make(speed=8.0)
    gap = 60.0
    for _ in range(6000):
        acc = idm_acc(veh, gap=gap, dv=veh.speed - lead_speed)
        veh.speed = max(0.0, veh.speed + acc * DT)
        gap += (lead_speed - veh.speed) * DT
    expected = 2.5 + lead_speed * 1.3  # s0 + v*T
    assert abs(gap - expected) < 1.0
    assert abs(veh.speed - lead_speed) < 0.1


def test_idm_emergency():
    """前方が急停止: 衝突しない(車間 > 0 を維持)。"""
    veh = make(speed=13.9)
    gap = 40.0  # 50km/h で 40m 手前から
    for _ in range(400):
        acc = idm_acc(veh, gap=gap, dv=veh.speed - 0.0)
        veh.speed = max(0.0, veh.speed + acc * DT)
        gap -= veh.speed * DT
        assert gap > 0.0, "衝突した"
    assert veh.speed < 0.1


def test_idm_red_signal_stop():
    """赤信号(長さ0の停止車両): 停止線の手前 s0 付近で止まる。"""
    veh = make(speed=11.0)
    dist = 80.0  # 停止線まで
    for _ in range(600):
        acc = idm_acc(veh, gap=dist, dv=veh.speed)
        veh.speed = max(0.0, veh.speed + acc * DT)
        dist -= veh.speed * DT
    assert veh.speed < 0.05
    assert 0.5 < dist < 5.0  # 概ね s0=2.5 の車間で停止
