"""センサス観測交通量の方向割り当て(calib/observations.py)の単体テスト。"""

import sys
from pathlib import Path

import numpy as np
from shapely.geometry import LineString, MultiLineString

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim.calib import observations as O

# 区間は x=0〜100。頂点順は東向きだが、終点側の隣接区間は西(x<0)にある
SEC = LineString([(0, 0), (100, 0)])
WEST = LineString([(-100, 0), (0, 0)])
EAST = LineString([(100, 0), (200, 0)])


def test_down_vector_uses_end_neighbour_not_vertex_order():
    dv, how = O.down_vector(SEC, start_geom=EAST, end_geom=WEST)
    assert how == "end"
    assert np.allclose(dv, [-1, 0])  # 下り = 西向き(頂点順と逆)


def test_down_vector_falls_back_to_start_then_none():
    dv, how = O.down_vector(SEC, start_geom=WEST)
    assert how == "start" and np.allclose(dv, [1, 0])
    assert O.down_vector(SEC) == (None, "none")


def test_far_ends_of_multipart_section():
    g = MultiLineString([[(50, 0), (100, 0)], [(0, 0), (50, 0)]])
    a, b = O.far_ends(g)
    assert {a, b} == {(0.0, 0.0), (100.0, 0.0)}


def _unit(up, down):
    def arr(v):
        a = [None] * 24
        a[8] = v
        return a

    return {"up": {"s": arr(up), "l": arr(0)}, "down": {"s": arr(down), "l": arr(0)}}


def test_edge_counts_by_direction_and_average():
    unit = _unit(300, 900)
    counts, d = O.edge_counts(np.array([-5.0, 0.0]), np.array([-1.0, 0.0]), unit)
    assert d == "down" and counts[8] == 900
    counts, d = O.edge_counts(np.array([5.0, 0.0]), np.array([-1.0, 0.0]), unit)
    assert d == "up" and counts[8] == 300
    counts, d = O.edge_counts(np.array([5.0, 0.0]), None, unit)
    assert d == "avg" and counts[8] == 600
    assert counts[3] is None  # 観測の無い時刻
