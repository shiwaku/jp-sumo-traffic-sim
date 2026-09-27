"""JARTIC 車両感知器の位置の特定(calib/detectors.py)の単体テスト。"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim.calib import detectors as D
from jp_sumo_traffic_sim.cases import sapporo as S


def test_travel_direction_is_second_char_of_suffix():
    assert D.travel_direction("北２３西１３\u3000\u3000東西") == "西"
    assert D.travel_direction("南１６西１１\u3000\u3000北南") == "南"
    assert D.travel_direction("月寒中央通\u3000４\u3000南南") == "南"
    assert D.travel_direction("張碓町１９\u3000\u3000札幌行") is None


def test_grid_address():
    assert D.grid_address("北 ７西２７\u3000\u3000北北") == ("北", 7, "西", 27)
    assert D.grid_address("北２４東 ８\u3000\u3000北北") == ("北", 24, "東", 8)
    assert D.grid_address("琴似 １－ ７ 北北") is None


def test_pick_edge_by_direction():
    cands = [(1, 5.0), (2, 92.0), (3, 180.0)]  # 東・北・西向き
    assert D.pick_edge(cands, "北") == 2
    assert D.pick_edge(cands, "南") is None  # 45 度以内の候補が無い
    assert D.pick_edge([(9, 200.0)], None) == 9  # 方向不明でも候補が1本なら選ぶ
    assert D.pick_edge(cands, None) is None


def test_grid_bearing_uses_rotated_grid():
    import math

    th = math.radians(S.GRID_BEARING_DEG)
    p, q = (0.0, 0.0), (100 * math.cos(th), 100 * math.sin(th))  # 格子の東向き
    assert D.angle_diff(D.grid_bearing(p, q, S.GRID_BEARING_DEG), 0.0) < 1e-6


def test_grid_address_xy_is_block_center():
    """北 1 条西 4 丁目の街区の中心は、北 1 条通と北 2 条通・西 4 丁目通と西 5 丁目通の中間。"""
    from shapely.geometry import Point

    x, y = S.grid_address_xy("北", 1, "西", 4)
    g = S.to_grid(Point(x, y))
    assert abs(g.y - (S.v_north(1) + S.v_north(2)) / 2) < 1e-6
    assert abs(g.x - (S.u_west(4) + S.u_west(5)) / 2) < 1e-6
