"""飽和交通流率の再現テスト(architecture.md §8 test_signal_as_stopped_vehicle)。

赤信号 = 長さ0の停止車両の後ろに待ち行列を作り、青開始後に停止線を
通過する車頭時間から飽和交通流率を測る。日本の基準値の帯
1800〜2000 台/時/車線に入ることを、シミュレーション本体と同じ
PARAMS["normal"] で確認する(パラメータ変更でここが割れたら要再調整)。

dt の影響は 0.5s → 0.2s で 1% 未満と確認済みのため DT のまま測る。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim.sim.simple import CAR_LEN, DT, PARAMS, Vehicle, idm_acc

V0 = 50 / 3.6  # 幹線の規制速度
N_QUEUE = 45


def discharge_crossing_times(params: dict) -> list[float]:
    """停止線(x=0)の後ろに詰めた待ち行列を発進させ、通過時刻を返す。"""
    spacing = CAR_LEN + params["s0"]
    vehicles = [
        Vehicle(
            pos=-(k + 1) * spacing,
            speed=0.0,
            v0=V0 * params["v0_factor"],
            a=params["a"],
            b=params["b"],
            s0=params["s0"],
            T=params["T"],
            turn=0,
        )
        for k in range(N_QUEUE)
    ]
    cross: dict[int, float] = {}
    t = 0.0
    for _ in range(int(240.0 / DT)):
        accs = []
        for k, v in enumerate(vehicles):
            if k == 0:
                accs.append(idm_acc(v, 1e9, 0.0))
            else:
                lead = vehicles[k - 1]
                accs.append(idm_acc(v, lead.pos - CAR_LEN - v.pos, v.speed - lead.speed))
        for k, v in enumerate(vehicles):
            old = v.pos
            v.speed = max(0.0, v.speed + accs[k] * DT)
            v.pos += v.speed * DT
            if k not in cross and old <= 0.0 < v.pos:
                cross[k] = t + (0.0 - old) / (v.pos - old) * DT  # 線形補間
        t += DT
    return [cross[k] for k in range(N_QUEUE) if k in cross]


def test_saturation_flow_normal():
    times = discharge_crossing_times(PARAMS["normal"])
    assert len(times) == N_QUEUE, "待ち行列が捌け切っていない"
    # 発進損失(先頭数台)を除いた 7〜41 台目の車頭時間で測る
    headways = [b - a for a, b in zip(times[6:40], times[7:41], strict=True)]
    flow = 3600.0 / (sum(headways) / len(headways))
    assert 1800.0 <= flow <= 2000.0, f"飽和交通流率 {flow:.0f} 台/時/車線が基準帯を外れた"


def test_startup_lost_time():
    """先頭数台の車頭時間は定常より長い(発進損失が存在する)。"""
    times = discharge_crossing_times(PARAMS["normal"])
    first = times[1] - times[0]
    steady = (times[40] - times[10]) / 30.0
    assert first > steady
