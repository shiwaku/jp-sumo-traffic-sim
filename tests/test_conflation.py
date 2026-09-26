"""空間結合(network/conflation.py)の単体テスト。合成ジオメトリのみ。"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim.network import conflation as CF


def test_line_matcher_direction_and_coverage():
    # 規制線: x=0→200 を東向き(頂点順)。リンク1は同方向、リンク2は逆方向
    lm = CF.LineMatcher([[[(0, 0), (200, 0)]]], buf_m=15.0)
    got = lm.best([(10, 3), (190, 3)])
    assert got is not None and got[0] == 0 and got[1] > 0  # 同方向
    got = lm.best([(190, -3), (10, -3)])
    assert got is not None and got[1] < 0  # 逆方向
    # 直交リンクは方位角条件で弾く
    assert lm.best([(100, -50), (100, 50)]) is None
    # 離れた平行リンク(横断30m)はバッファで弾く
    assert lm.best([(10, 30), (190, 30)]) is None
    # 被覆50%未満(規制線の外に大半がはみ出す)は弾く
    assert lm.best([(150, 0), (500, 0)]) is None


def test_line_matcher_multipart_feature():
    """MultiLineString の2番目以降のパートも照合対象になる。"""
    lm = CF.LineMatcher([[[(0, 0), (100, 0)], [(300, 0), (500, 0)]]], buf_m=15.0)
    got = lm.best([(310, 2), (490, 2)])  # 2パート目に沿うリンク
    assert got is not None and got[0] == 0


def test_line_matcher_long_segment():
    """1セグメントが数百mの規制線(センサス等)でも近傍から漏れない。"""
    lm = CF.LineMatcher([[[(0, 0), (800, 0)]]], buf_m=15.0)
    got = lm.best([(350, 5), (450, 5)])  # 規制線の中点から離れた短いリンク
    assert got is not None and got[0] == 0


def test_point_matcher_nearest_and_inflow():
    node_xy = np.array([[0.0, 0.0], [130.0, 0.0]])
    pm = CF.PointMatcher([10, 20], node_xy)
    assert pm.nearest_node(5, 5) == 10
    assert pm.nearest_node(128, -3) == 20
    assert pm.nearest_node(60, 60) is None  # 30m 超は不採用

    # 交差点(0,0)の東側に立つ標識 → 東へ延びるリンクが流入リンク
    incident = [
        ("east", [(0, 0), (100, 0)]),
        ("north", [(0, 0), (0, 100)]),
    ]
    assert CF.PointMatcher.inflow_link(12, -4, incident) == "east"
    assert CF.PointMatcher.inflow_link(-3, 15, incident) == "north"
