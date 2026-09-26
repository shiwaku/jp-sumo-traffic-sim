"""KSJ N13 を対象区域(+バッファ)でクリップし、道路を選んで投影・保存する。

対象区域はケースが決める(config.region_polygon。札幌は市の行政界)。
道路の選択(case.toml の [network]、docs/sumo-design.md §13.4):
  - 詳細区域(config.core_polygon。札幌は都心コードン)に掛かる車道はすべて
  - それ以外は 指定の道路分類(国道・道道・高速)、幅員の広い市区町村道、
    センサス区間に沿って走る市区町村道だけ
  [network] が無いケースは区域内の車道をすべて入れる
出力:
  data/interim/ksj_clip.gpkg  layer=roads / paths / cordon  (EPSG:6679)
  (paths = 庭園路・徒歩道・石段。車道ネットワークからは除外し参照用に保持)
  reports/00_clip.json
"""

import json
import sys
from pathlib import Path

import geopandas as gpd
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim import config as C
from jp_sumo_traffic_sim import ksj_codes

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


def on_census_road(roads: gpd.GeoDataFrame, clip_poly, dist_m: float) -> pd.Series:
    """センサス区間に沿って走るリンク(両端点と中点がすべて中心線から dist_m 以内)。

    延長の割合で判定すると、センサス道路に直交する短い街路も拾ってしまうため、
    リンクの3点がすべて近いこと(= 沿って走っていること)を条件にする。
    roads・clip_poly は CRS_PROJ。
    """
    src = C.RAW / "census" / "traffic_census_2021_converted.parquet"
    out = pd.Series(False, index=roads.index)
    if not src.exists() or not len(roads):
        return out
    cen = gpd.read_parquet(src, columns=["geometry"])
    cen = cen[cen.intersects(gpd.GeoSeries([clip_poly], crs=C.CRS_PROJ).to_crs(cen.crs).iloc[0])]
    lines = cen.to_crs(C.CRS_PROJ).geometry.union_all()
    ok = pd.Series(True, index=roads.index)
    for frac in (0.0, 0.5, 1.0):
        pts = roads.geometry.interpolate(frac, normalized=True)
        ok &= pts.distance(lines) <= dist_m
    return out | ok


def select_roads(sel: gpd.GeoDataFrame, clip_poly) -> tuple[pd.Series, dict]:
    """シミュレーションに入れる車道の真偽値と内訳。"""
    net = C.CASE.get("network")
    if not net:
        return pd.Series(True, index=sel.index), dict(rule="all")
    cat = sel["N13_003"].astype(str)
    wid = sel["N13_006"].astype(str)
    by_core = sel["in_core"]
    by_cat = cat.isin(net["categories"])
    by_width = (cat == "3") & wid.isin(net["municipal_widths"])
    rest = ~(by_core | by_cat | by_width)
    by_census = pd.Series(False, index=sel.index)
    by_census.loc[sel.index[rest]] = on_census_road(
        sel[rest], clip_poly, float(net["census_dist_m"])
    )
    keep = by_core | by_cat | by_width | by_census
    return keep, dict(
        rule="core_all + categories + municipal_widths + on_census_road",
        categories=net["categories"],
        municipal_widths=net["municipal_widths"],
        n_core=int(by_core.sum()),
        n_category=int((by_cat & ~by_core).sum()),
        n_municipal_width=int((by_width & ~by_core).sum()),
        n_on_census_road=int(by_census.sum()),
        n_kept=int(keep.sum()),
        n_dropped=int((~keep).sum()),
    )


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

    cordon = C.region_polygon()
    clip_poly = cordon.buffer(C.CLIP_BUFFER_M, join_style=2)
    clip_ll = gpd.GeoSeries([clip_poly], crs=C.CRS_PROJ).to_crs(roads.crs).iloc[0]

    n_all = len(roads)
    sel = roads[roads.intersects(clip_ll)].to_crs(C.CRS_PROJ).copy()
    sel["length_m"] = sel.length
    sel["in_cordon"] = sel.intersects(cordon)  # 対象区域内(列名は Phase 0 からの互換)
    core = C.core_polygon()
    sel["in_core"] = sel.intersects(core) if core is not None else True

    # 種別(N13_002)で車道と歩行者系(庭園路・徒歩道・石段)を分ける。
    # 下流(probe/grid/names/inventory/viewer)が使うのは車道の roads レイヤのみ
    types = sel["N13_002"].astype(str)
    by_type = {
        code: dict(
            label=ksj_codes.ROAD_TYPE.get(code, "?"),
            n=int((types == code).sum()),
            n_in_cordon=int(((types == code) & sel["in_cordon"]).sum()),
            length_km=round(float(sel.loc[types == code, "length_m"].sum()) / 1000, 2),
        )
        for code in sorted(set(types))
    }
    is_path = types.isin(ksj_codes.NON_VEHICULAR_TYPES)
    paths = sel[is_path].sort_values("length_m", ascending=False).reset_index(drop=True)
    paths["link_id"] = [f"P{i:05d}" for i in range(len(paths))]
    sel = sel[~is_path]
    keep, selection = select_roads(sel, clip_poly)
    sel = sel[keep].sort_values("length_m", ascending=False).reset_index(drop=True)
    sel["link_id"] = [f"K{i:05d}" for i in range(len(sel))]

    C.INTERIM.mkdir(parents=True, exist_ok=True)
    out = C.INTERIM / "ksj_clip.gpkg"
    sel.to_file(out, layer="roads", driver="GPKG")
    if len(paths):
        paths.to_file(out, layer="paths", driver="GPKG")
    kinds, geoms = ["region", "clip"], [cordon, clip_poly]
    if core is not None:
        kinds.append("cordon")  # 詳細区域(ビューワの「コードン」表示)
        geoms.append(core)
    gpd.GeoDataFrame({"kind": kinds}, geometry=geoms, crs=C.CRS_PROJ).to_file(
        out, layer="cordon", driver="GPKG"
    )

    ll = gpd.GeoSeries([cordon], crs=C.CRS_PROJ).to_crs("EPSG:6668").total_bounds
    rep = dict(
        source_crs=src_crs,
        n_features_source=int(n_all),
        n_features_clipped=int(len(sel)),
        n_features_in_cordon=int(sel["in_cordon"].sum()),
        n_features_in_core=int(sel["in_core"].sum()),
        selection=selection,
        total_length_km=round(float(sel.length.sum()) / 1000, 2),
        by_road_type=by_type,
        n_paths_excluded=int(len(paths)),
        n_paths_excluded_in_cordon=int(paths["in_cordon"].sum()),
        cordon_area_km2=round(cordon.area / 1e6, 3),
        cordon_bounds_lonlat=[round(float(v), 5) for v in ll],
        clip_buffer_m=C.CLIP_BUFFER_M,
        attributes=ATTR_NAMES,
    )
    # ケース固有の値(札幌はコードンのグリッド座標・方位)
    extras = getattr(C.case_module(), "clip_report_extras", None)
    if extras:
        rep.update(extras())
    C.REPORTS.mkdir(parents=True, exist_ok=True)
    (C.REPORTS / "00_clip.json").write_text(
        json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(rep, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
