"""札幌ケース: 碁盤目のグリッド座標とコードン(対象区域)。

札幌都心は 130.0m の等間隔格子なので、グリッド座標で街路が名指しできる。
この座標系・街路名・コードンは札幌でしか意味を持たないため、汎用の
config.py から切り出している(docs/sumo-design.md §14)。
"""

# --- グリッド座標系 -----------------------------------------------------------
# 札幌の碁盤目は EPSG:6679 の座標軸に対して約 10.9 度傾いている。
# GRID_ORIGIN を中心に -GRID_BEARING_DEG 回転させた座標を (u, v) と呼ぶ。
#   u = 東西方向(丁目の並び / 大きいほど東)
#   v = 南北方向(条の並び / 大きいほど北)
# この系では「同じ通り」が一定の横断座標を共有するため、街路の同定と
# コードンの定義が素直に書ける。値は cases/sapporo/scripts/02_grid_frame.py の実測。
GRID_BEARING_DEG = 10.906
GRID_ORIGIN = (89500.0, -104000.0)

# 条・丁目の格子間隔 [m](実測。南1条〜南3条 / 北1条〜北3条 / 西4〜西11丁目 で 130.0m)
BLOCK_M = 130.0
# 基準線の v / u(実測)
V_S1 = -104010.0  # 南1条通
V_N1 = -103660.0  # 北1条通
U_W4 = 89810.0  # 西4丁目通(札幌駅前通)


def v_south(n: int) -> float:
    """南 n 条通の v 座標."""
    return V_S1 - BLOCK_M * (n - 1)


def v_north(n: int) -> float:
    """北 n 条通の v 座標."""
    return V_N1 + BLOCK_M * (n - 1)


def u_west(n: int) -> float:
    """西 n 丁目通の u 座標."""
    return U_W4 - BLOCK_M * (n - 4)


def u_east(n: int) -> float:
    """東 n 丁目通の u 座標."""
    return U_W4 + BLOCK_M * (3 + n)


# --- 対象区域(コードン)-----------------------------------------------------
# 北=北5条通 / 南=南7条通 / 東=創成川通 / 西=石山通(国道230号)
# 石山通 = 西11丁目通、創成川通の東側車道 ≒ 東1丁目通 の位置にあることを実測で確認済み。
# 境界の幹線そのものを完全に含めるため、通りの中心線から外側にマージンを取る。
# これが無いと石山通・東1丁目通の一部が区域外にこぼれ、コードン流入点が欠ける。
CORDON_MARGIN_M = 45.0

CORDON_GRID = dict(
    west=u_west(11) - CORDON_MARGIN_M,  # 石山通(= 西11丁目通)
    east=u_east(1) + CORDON_MARGIN_M,  # 創成川通(東側車道 ≒ 東1丁目通)
    south=v_south(7) - CORDON_MARGIN_M,  # 南7条通
    north=v_north(5) + CORDON_MARGIN_M,  # 北5条通
)

# 境界の4本の通り(コードン流入点はこの上に載る)
CORDON_STREETS = dict(
    west=("西11丁目通(石山通)", u_west(11)),
    east=("東1丁目通(創成川通)", u_east(1)),
    south=("南7条通", v_south(7)),
    north=("北5条通", v_north(5)),
)


def cordon_polygon():
    """コードン矩形(EPSG:6679、グリッドに整列した回転矩形)を返す."""
    from shapely.affinity import rotate
    from shapely.geometry import box

    g = CORDON_GRID
    rect = box(g["west"], g["south"], g["east"], g["north"])
    return rotate(rect, GRID_BEARING_DEG, origin=GRID_ORIGIN, use_radians=False)


def region_polygon():
    """対象区域(ケース共通のインタフェース)。札幌はコードン矩形."""
    return cordon_polygon()


def clip_report_extras() -> dict:
    """reports/00_clip.json に載せるケース固有の値."""
    g = CORDON_GRID
    return dict(
        cordon_grid={k: round(v, 1) for k, v in g.items()},
        cordon_size_m=[
            round(g["east"] - g["west"], 1),
            round(g["north"] - g["south"], 1),
        ],
        grid_bearing_deg=GRID_BEARING_DEG,
    )


# --- 信号(SUMO 変換のフック、docs/sumo-design.md §3)---------------------------
# 自前実装(sim/netsim.py)と同じ条件: 2現示は東西青から始め、オフセットは
# 東行きの green wave(グリッド u 方向に GREEN_WAVE_KMH で進む)
GREEN_WAVE_KMH = 40.0  # 自前実装の PROGRESSION_MS と同じ


def _uv(x: float, y: float) -> tuple[float, float]:
    import math

    th = math.radians(-GRID_BEARING_DEG)
    ox, oy = GRID_ORIGIN
    dx, dy = x - ox, y - oy
    return ox + dx * math.cos(th) - dy * math.sin(th), oy + dx * math.sin(th) + dy * math.cos(th)


def signal_axis(p1: tuple, p2: tuple) -> str:
    """流入 Edge 終端の2点 → 'A'(東西)/ 'B'(南北)。"""
    (u1, v1), (u2, v2) = _uv(*p1), _uv(*p2)
    return "A" if abs(u2 - u1) >= abs(v2 - v1) else "B"


def signal_offsets(nodes: list[dict]) -> dict[int, float]:
    """信号ノード nid → オフセット [s](東行き green wave)。"""
    u_min = min(_uv(n["x"], n["y"])[0] for n in nodes)
    v = GREEN_WAVE_KMH / 3.6
    return {n["nid"]: (_uv(n["x"], n["y"])[0] - u_min) / v for n in nodes if n.get("has_signal")}


def to_grid(geom):
    """EPSG:6679 の図形をグリッド座標 (u, v) へ回転."""
    from shapely.affinity import rotate

    return rotate(geom, -GRID_BEARING_DEG, origin=GRID_ORIGIN, use_radians=False)


def from_grid(geom):
    """グリッド座標 (u, v) の図形を EPSG:6679 へ戻す."""
    from shapely.affinity import rotate

    return rotate(geom, GRID_BEARING_DEG, origin=GRID_ORIGIN, use_radians=False)
