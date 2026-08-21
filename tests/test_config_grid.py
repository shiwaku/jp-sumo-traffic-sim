"""グリッド座標系とコードン定義の不変条件。"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from sapporo_sim import config as C


def test_block_spacing_is_uniform():
    """条・丁目の格子は 130.0m 等間隔(Phase 0 実測)。"""
    assert C.v_south(2) - C.v_south(1) == -C.BLOCK_M
    assert C.v_north(2) - C.v_north(1) == C.BLOCK_M
    assert C.u_west(3) - C.u_west(4) == C.BLOCK_M


def test_ishiyama_dori_is_west_11():
    """石山通 = 西11丁目通、札幌駅前通 = 西4丁目通(Phase 0 で OSM と照合済み)。"""
    assert C.u_west(11) == C.U_W4 - C.BLOCK_M * 7
    assert C.u_west(4) == C.U_W4


def test_cordon_contains_boundary_streets():
    """コードンは境界の4本の通りを内側に含む(マージン分だけ外に張る)。"""
    g = C.CORDON_GRID
    for _, (name, ordinate) in C.CORDON_STREETS.items():
        assert g["west"] <= ordinate <= g["east"] or g["south"] <= ordinate <= g["north"], name


def test_cordon_size():
    g = C.CORDON_GRID
    assert round(g["east"] - g["west"], 1) == 1520.0
    assert round(g["north"] - g["south"], 1) == 1740.0


def test_grid_roundtrip():
    """to_grid / from_grid が往復で一致する。"""
    from shapely.geometry import Point

    p = Point(89500.0 + 731.0, -104000.0 - 512.0)
    q = C.from_grid(C.to_grid(p))
    assert abs(q.x - p.x) < 1e-6
    assert abs(q.y - p.y) < 1e-6


def test_cordon_polygon_is_rotated():
    """コードンは軸並行ではなくグリッドに整列した回転矩形。"""
    poly = C.cordon_polygon()
    minx, miny, maxx, maxy = poly.bounds
    # 回転しているぶん外接矩形は本体より大きい
    assert (maxx - minx) > (C.CORDON_GRID["east"] - C.CORDON_GRID["west"])
    assert round(poly.area / 1e6, 3) == 2.645
