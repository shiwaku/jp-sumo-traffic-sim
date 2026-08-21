"""KSJ コードリストの向きを固定する。

幅員区分は「1 が最も狭い」。逆に読むと幹線と細街路が入れ替わり、
サブレーン数の算出が全部反転する。Phase 0 で一度間違えた箇所。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from sapporo_sim import ksj_codes as K


def test_width_order_is_narrow_to_wide():
    reps = [K.WIDTH_REPRESENTATIVE_M[str(i)] for i in range(1, 6)]
    assert reps == sorted(reps)
    assert K.ROAD_WIDTH["1"] == "3m未満"
    assert K.ROAD_WIDTH["5"] == "19.5m以上"
    assert K.ROAD_WIDTH["6"] == "不明"


def test_road_state_grade_separated_codes():
    assert K.ROAD_STATE["2"] == "橋・高架"
    assert K.ROAD_STATE["3"] == "トンネル"


def test_road_category():
    assert K.ROAD_CATEGORY["1"] == "国道"
    assert K.ROAD_CATEGORY["3"] == "市区町村道等"
