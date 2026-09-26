"""センサス時間帯別交通量の読み取り(sim/demand.py)の単体テスト。

流入レートの割り当ては SUMO 版(tests/test_sumo_demand.py)で検査する。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim.sim.demand import hourly_rate


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
