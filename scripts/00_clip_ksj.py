"""KSJ N13 を対象区域(+バッファ)でクリップし、投影して保存する。

コードンはグリッド整列の回転矩形(config.cordon_polygon)。
出力:
  data/interim/ksj_clip.gpkg  layer=roads / cordon  (EPSG:6679)
  reports/00_clip.json
"""

import json
import sys
from pathlib import Path

import geopandas as gpd
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from sapporo_sim import config as C

ATTR_NAMES = {
    "N13_001": "登録日",
    "N13_002": "種別",
    "N13_003": "道路分類",
    "N13_004": "道路状態",
    "N13_005": "階層順",
    "N13_006": "幅員区分",
    "N13_007": "有料区分",
    "N13_008": "二次メッシュ",
}


def main() -> None:
    parts, src_crs = [], {}
    for mesh in C.KSJ_MESHES:
        shp = C.RAW / "ksj" / f"N13-24_{mesh}_SHP" / f"N13-24_{mesh}_SHP" / f"N13-24_{mesh}.shp"
        if not shp.exists():
            print(f"skip (missing): {shp}")
            continue
        g = gpd.read_file(shp)
        src_crs[mesh] = str(g.crs)
        g["mesh"] = mesh
        parts.append(g)
    if not parts:
        raise SystemExit("KSJ の SHP が無い。make fetch を先に実行する。")
    roads = gpd.GeoDataFrame(
        pd.concat(parts, ignore_index=True), geometry="geometry", crs=parts[0].crs
    )

    cordon = C.cordon_polygon()
    clip_poly = cordon.buffer(C.CLIP_BUFFER_M, join_style=2)
    clip_ll = gpd.GeoSeries([clip_poly], crs=C.CRS_PROJ).to_crs(roads.crs).iloc[0]

    n_all = len(roads)
    sel = roads[roads.intersects(clip_ll)].to_crs(C.CRS_PROJ).copy()
    sel["length_m"] = sel.length
    sel = sel.sort_values("length_m", ascending=False).reset_index(drop=True)
    sel["link_id"] = [f"K{i:05d}" for i in range(len(sel))]
    sel["in_cordon"] = sel.intersects(cordon)

    C.INTERIM.mkdir(parents=True, exist_ok=True)
    out = C.INTERIM / "ksj_clip.gpkg"
    sel.to_file(out, layer="roads", driver="GPKG")
    gpd.GeoDataFrame(
        {"kind": ["cordon", "clip"]}, geometry=[cordon, clip_poly], crs=C.CRS_PROJ
    ).to_file(out, layer="cordon", driver="GPKG")

    ll = gpd.GeoSeries([cordon], crs=C.CRS_PROJ).to_crs("EPSG:6668").total_bounds
    rep = dict(
        source_crs=src_crs,
        n_features_source=int(n_all),
        n_features_clipped=int(len(sel)),
        n_features_in_cordon=int(sel["in_cordon"].sum()),
        total_length_km=round(float(sel.length.sum()) / 1000, 2),
        cordon_grid={k: round(v, 1) for k, v in C.CORDON_GRID.items()},
        cordon_size_m=[
            round(C.CORDON_GRID["east"] - C.CORDON_GRID["west"], 1),
            round(C.CORDON_GRID["north"] - C.CORDON_GRID["south"], 1),
        ],
        cordon_area_km2=round(cordon.area / 1e6, 3),
        cordon_bounds_lonlat=[round(float(v), 5) for v in ll],
        grid_bearing_deg=C.GRID_BEARING_DEG,
        clip_buffer_m=C.CLIP_BUFFER_M,
        attributes=ATTR_NAMES,
    )
    C.REPORTS.mkdir(parents=True, exist_ok=True)
    (C.REPORTS / "00_clip.json").write_text(
        json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(rep, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
