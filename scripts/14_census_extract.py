"""道路交通センサス(令和3年度)から対象区域の区間を抽出し、属性の充足を確認する。

出典: 「全国道路・街路交通情勢調査」(国土交通省)を加工して作成
      公共データ利用規約 (PDL1.0)

センサスは国道・都道府県道・主要幹線市道のみが対象で、都心の細街路は含まれない。
そのため「区域内の道路をどれだけ覆えるか」も測る。

使う項目(docs/design.md 4.4):
  車線数                        → サブレーン換算
  幅員構成／車道部幅員(m)        → サブレーン数、冬季狭小の基準値
  昼間(非混雑時)旅行速度         → IDM の v0 推定
  朝夕(混雑時)旅行速度           → 検証ターゲット
  昼間12時間大型車混入率(%)       → 車両構成
  24時間/12時間交通量            → 流入発生量
  時間帯別交通量(jikantai/01.json) → コードン流入の時系列。
      キーは「交通量／都道府県指定市コード」_「交通量／調査単位区間番号」
  指定最高速度(km/h)             → IDM の v0
  代表信号交差点／右折専用車線の有無等 → 右折レーン(設計資料は「無い」としていたが在る)

出力: data/interim/census.gpkg (layer=sections)
      reports/14_census.json
"""

import json
import sys
from pathlib import Path

import geopandas as gpd
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim import config as C

SRC = "data/raw/census/traffic_census_2021_converted.parquet"
CRS_CENSUS = "EPSG:4612"  # R03 のCRS(docs/design.md 4.4)

KEEP = {
    "交通調査基本区間番号": "section_id",
    "交通量／都道府県指定市コード": "vol_pref_code",
    "交通量／調査単位区間番号": "vol_unit_no",
    "路線番号": "route_no",
    "路線名": "route",
    "市区町村コード": "muni",
    "区間種別": "section_type",
    "区間延長（ｋｍ）": "length_km",
    "車線数": "n_lanes",
    "幅員構成／道路部幅員（ｍ）": "w_road",
    "幅員構成／車道部幅員（ｍ）": "w_carriageway",
    "幅員構成／車道幅員（ｍ）": "w_lanes",
    "幅員構成／中央帯幅員（ｍ）": "w_median",
    "指定最高速度（ｋｍ／ｈ）": "speed_limit",
    "代表信号交差点／右折専用車線の有無等": "right_turn_lane",
    "バス路線／バス優先・専用レーンの有無": "bus_lane",
    "昼間１２時間大型車混入率（％）": "heavy_pct",
    "混雑度": "congestion",
    "昼間１２時間自動車類交通量（上下合計）／合計（台）": "v12h",
    "２４時間自動車類交通量（上下合計）／合計（台）": "v24h",
    "朝夕（混雑時）／上り／旅行速度／合計（ｋｍ／ｈ）": "v_peak_up",
    "朝夕（混雑時）／下り／旅行速度／合計（ｋｍ／ｈ）": "v_peak_dn",
    "昼間（非混雑時）／上り／旅行速度／合計（ｋｍ／ｈ）": "v_off_up",
    "昼間（非混雑時）／下り／旅行速度／合計（ｋｍ／ｈ）": "v_off_dn",
    "昼間１２時間ピーク比率（％）": "peak_pct",
}
# 充足を必ず見る列(欠損があるとモデルに直接効く)
CRITICAL = [
    "n_lanes",
    "w_carriageway",
    "speed_limit",
    "heavy_pct",
    "v_peak_up",
    "v_off_up",
    "v12h",
    "right_turn_lane",
]


def _peak_hour(hourly: dict) -> dict:
    """区域内の全区間を合計した上り交通量のピーク時刻。"""
    if not hourly:
        return {}
    total = np.zeros(24)
    for v in hourly.values():
        up = v.get("up") or {}
        for key in ("s", "l"):
            arr = up.get(key)
            if arr and len(arr) == 24:
                # 一部の区間は時間別の値が null。欠測は0として足す
                total += np.nan_to_num(np.asarray(arr, dtype=float))
    if not total.any():
        return {}
    h = int(np.argmax(total))
    return {"hour": h, "vehicles": int(total[h]), "profile": [int(x) for x in total]}


def main() -> None:
    src = C.ROOT / SRC
    if not src.exists():
        raise SystemExit(f"{SRC} が無い。scripts/13_fetch_census.py を先に実行する。")

    cordon = C.region_polygon()
    clip = cordon.buffer(C.CLIP_BUFFER_M, join_style=2)
    clip_ll = gpd.GeoSeries([clip], crs=C.CRS_PROJ).to_crs(CRS_CENSUS).iloc[0]
    cordon_ll = gpd.GeoSeries([cordon], crs=C.CRS_PROJ).to_crs(CRS_CENSUS).iloc[0]

    cols = [c for c in KEEP] + ["geometry"]
    g = gpd.read_parquet(src, columns=cols)
    n_all = len(g)
    if g.crs is None:
        g = g.set_crs(CRS_CENSUS)
    g = g[g.intersects(clip_ll)].rename(columns=KEEP).copy()
    g["in_cordon"] = g.intersects(cordon_ll)
    g = g.to_crs(C.CRS_PROJ)
    g["len_in_cordon_m"] = g.geometry.intersection(cordon).length.round(1)

    out = C.INTERIM / "census.gpkg"
    if out.exists():
        out.unlink()
    g.to_file(out, layer="sections", driver="GPKG")

    # --- 時間帯別交通量の結合 ---
    jikantai_path = C.RAW / "census" / "jikantai" / f"{C.CASE['census']['pref_code']}.json"
    hourly = {}
    if jikantai_path.exists():
        j = json.loads(jikantai_path.read_text(encoding="utf-8"))
        units = j.get("units", {})
        hours = j.get("hours", list(range(24)))

        def unit_key(row) -> str | None:
            a, b = row.get("vol_pref_code"), row.get("vol_unit_no")
            if a is None or b is None or a != a or b != b:
                return None

            def norm(v):
                s = str(v).strip()
                return s[:-2] if s.endswith(".0") else s

            return f"{norm(a)}_{norm(b)}"

        g["vol_key"] = g.apply(unit_key, axis=1)
        for _, r in g[g["in_cordon"]].iterrows():
            u = units.get(r["vol_key"])
            if not u:
                continue
            hourly[str(r["section_id"])] = {
                "vol_key": r["vol_key"],
                "hours": hours,
                "up": u.get("up"),
                "down": u.get("down"),
                "t12": u.get("t12"),
                "t24": u.get("t24"),
            }
        C.PROCESSED.mkdir(parents=True, exist_ok=True)
        (C.PROCESSED / "census_hourly.json").write_text(
            json.dumps(hourly, ensure_ascii=False, indent=1), encoding="utf-8"
        )

    inner = g[g["in_cordon"]]

    def fill(col: str) -> dict:
        if col not in inner.columns or not len(inner):
            return {"n": 0, "filled": 0, "rate": None}
        s = inner[col]
        ok = s.notna() & (s.astype(str).str.strip() != "")
        return {"n": int(len(s)), "filled": int(ok.sum()), "rate": round(float(ok.mean()), 3)}

    # 区域内の道路総延長に対する被覆(KSJ の格子街路と比較)
    ksj = gpd.read_file(C.INTERIM / "streets.gpkg", layer="roads")
    ksj_len = float(ksj["length_m"].sum())
    census_len = float(inner["len_in_cordon_m"].sum())

    def stats(col: str) -> dict:
        if col not in inner.columns:
            return {}
        v = gpd.pd.to_numeric(inner[col], errors="coerce").dropna()
        if not len(v):
            return {}
        return {
            "n": int(len(v)),
            "min": round(float(v.min()), 1),
            "median": round(float(v.median()), 1),
            "max": round(float(v.max()), 1),
        }

    rep = dict(
        source=SRC,
        crs=CRS_CENSUS,
        n_sections_nationwide=int(n_all),
        n_sections_clip=int(len(g)),
        n_sections_cordon=int(len(inner)),
        coverage=dict(
            census_len_in_cordon_m=round(census_len, 1),
            ksj_len_in_cordon_m=round(ksj_len, 1),
            ratio=round(census_len / ksj_len, 3) if ksj_len else None,
            note="センサスは国道・都道府県道・主要幹線市道のみ。細街路は含まれない",
        ),
        completeness={c: fill(c) for c in CRITICAL},
        distributions={
            c: stats(c)
            for c in [
                "n_lanes",
                "w_carriageway",
                "speed_limit",
                "heavy_pct",
                "v_peak_up",
                "v_off_up",
                "v12h",
                "v24h",
                "congestion",
            ]
        },
        right_turn_lane_values=(
            inner["right_turn_lane"].astype(str).value_counts().to_dict()
            if "right_turn_lane" in inner.columns and len(inner)
            else {}
        ),
        routes=sorted(set(inner["route"].dropna().astype(str))) if len(inner) else [],
        hourly=dict(
            n_sections_joined=len(hourly),
            join_rate=(round(len(hourly) / len(inner), 3) if len(inner) else None),
            peak_hour_up=_peak_hour(hourly),
            note="コードン流入の時系列と出発時刻分布に使う(docs/design.md 3.3)",
        ),
    )
    (C.REPORTS / "14_census.json").write_text(
        json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(rep, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
