"""共通設定(汎用部)とケースの読み込み.

地域に依存する値は cases/<name>/case.toml から読む(docs/sumo-design.md §14)。
ケースは環境変数 JPSUMO_CASE で選ぶ(既定: sapporo)。
区域ポリゴンなどコードで書く処理は case.toml の module(cases パッケージ)にある。
"""

import importlib
import os
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RAW = ROOT / "data" / "raw"
INTERIM = ROOT / "data" / "interim"
PROCESSED = ROOT / "data" / "processed"
REPORTS = ROOT / "reports"
CASES = ROOT / "cases"

# --- ケース -------------------------------------------------------------------
CASE_NAME = os.environ.get("JPSUMO_CASE", "sapporo")
CASE_DIR = CASES / CASE_NAME
CASE = tomllib.loads((CASE_DIR / "case.toml").read_text(encoding="utf-8"))

# --- 座標系 ---
CRS_KSJ = "EPSG:6668"  # JGD2011 地理座標(KSJ N13 の .prj 実測値。全国共通)
CRS_PROJ = CASE["crs"]["proj"]  # ケースの平面直角座標系。距離・速度計算はすべてこの系で行う

# --- 対象区域 ---
CLIP_BUFFER_M = CASE["region"]["clip_buffer_m"]  # 抽出は流入路を残すため外側にバッファを取る [m]

# --- 位相構築(札幌で実測した既定値。ケースで上書きできる)---
_topo = CASE.get("topology", {})
# 端点同一視の許容誤差。KSJ は端点が厳密一致しており 0.01〜1.0m で結果不変
SNAP_TOL_M = _topo.get("snap_tol_m", 0.5)
SHORT_LINK_M = _topo.get("short_link_m", 12.0)  # これ未満は交差点内部リンク候補

KSJ_MESHES = CASE["ksj"]["meshes"]


def case_module():
    """ケース固有処理のモジュール(case.toml の module)."""
    return importlib.import_module(CASE["module"])


def region_polygon():
    """対象区域のポリゴン(CRS_PROJ)。中身はケースが決める."""
    return case_module().region_polygon()


def core_polygon():
    """全道路を入れる詳細区域(ミクロの範囲)。ケースが持たなければ None(区域全体が詳細)."""
    fn = getattr(case_module(), "core_polygon", None)
    return fn() if fn else None
