"""Phase 0 の成果をビューワ用データ(GeoJSON + meta.json)に書き出す。

出力先は viewer/public/data/。ビューワ(Vite + MapLibre)は
これを fetch して描画する。構成は
https://github.com/shiwaku/jartic-traffic-signal-cycle-converter/tree/main/viewer
をベースにしている。

出力:
  viewer/public/data/{network,cordon,lattice,reg_point,reg_line,signals,census}.geojson
  viewer/public/data/meta.json
"""

import json
import sys
from pathlib import Path

import geopandas as gpd
from shapely.geometry import LineString

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from sapporo_sim import config as C
from sapporo_sim.ksj_codes import ROAD_CATEGORY, ROAD_STATE, ROAD_WIDTH

PREC = 6  # 座標の小数桁。1e-6 度 ≒ 0.1m
OUT = C.ROOT / "viewer" / "public" / "data"


def round_coords(obj):
    if isinstance(obj, list):
        return [round_coords(v) for v in obj]
    if isinstance(obj, float):
        return round(obj, PREC)
    return obj


def to_fc(gdf: gpd.GeoDataFrame, props: list[str]) -> dict:
    g = gdf.to_crs("EPSG:4326")
    feats = []
    for _, r in g.iterrows():
        geom = json.loads(gpd.GeoSeries([r.geometry]).to_json())["features"][0]["geometry"]
        geom["coordinates"] = round_coords(geom["coordinates"])
        props_out = {}
        for k in props:
            v = r.get(k)
            if v is None or v != v:  # NaN
                continue
            props_out[k] = v.item() if hasattr(v, "item") else v
        feats.append({"type": "Feature", "geometry": geom, "properties": props_out})
    return {"type": "FeatureCollection", "features": feats}


def write(name: str, fc: dict) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{name}.geojson").write_text(json.dumps(fc, ensure_ascii=False), encoding="utf-8")
    size = (OUT / f"{name}.geojson").stat().st_size / 1e3
    print(f"  {name}.geojson  {len(fc['features']):5d} features  {size:8.1f} KB")


def optional(path: str, layer: str, props: list[str]) -> dict:
    f = C.INTERIM / path
    empty = {"type": "FeatureCollection", "features": []}
    if not f.exists():
        return empty
    try:
        g = gpd.read_file(f, layer=layer)
    except Exception:
        return empty
    for c in props:
        if c not in g.columns:
            g[c] = None
    return to_fc(g, props)


def report(name: str) -> dict:
    f = C.REPORTS / name
    return json.loads(f.read_text(encoding="utf-8")) if f.exists() else {}


def main() -> None:
    roads = gpd.read_file(C.INTERIM / "streets.gpkg", layer="roads")
    roads["road_category"] = roads["N13_003"].astype(str).map(ROAD_CATEGORY)
    roads["road_state"] = roads["N13_004"].astype(str).map(ROAD_STATE)
    roads["width_class"] = roads["N13_006"].astype(str).map(ROAD_WIDTH)
    roads["length_m"] = roads["length_m"].round(1)
    roads["street"] = roads["street"].fillna("")
    roads["street_kind"] = roads["street_kind"].fillna("未割り当て")
    inv = report("04_inventory.json")
    through = {s["street"] for s in inv.get("streets", []) if s["through"]}
    roads["through"] = roads["street"].isin(through)

    print("viewer/public/data/ に書き出す:")
    write(
        "network",
        to_fc(
            roads,
            [
                "link_id",
                "street",
                "street_kind",
                "through",
                "axis",
                "length_m",
                "road_category",
                "road_state",
                "width_class",
                "assign_err_m",
            ],
        ),
    )

    cordon = gpd.read_file(C.INTERIM / "ksj_clip.gpkg", layer="cordon")
    write("cordon", to_fc(cordon, ["kind"]))

    # 格子線
    g = C.CORDON_GRID
    lines, names = [], []
    for n in range(1, 12):
        v = C.v_south(n)
        if g["south"] <= v <= g["north"]:
            lines.append(LineString([(g["west"], v), (g["east"], v)]))
            names.append(f"南{n}条通")
    v = (C.v_south(1) + C.v_north(1)) / 2
    lines.append(LineString([(g["west"], v), (g["east"], v)]))
    names.append("大通")
    for n in range(1, 9):
        v = C.v_north(n)
        if g["south"] <= v <= g["north"]:
            lines.append(LineString([(g["west"], v), (g["east"], v)]))
            names.append(f"北{n}条通")
    for n in range(1, 16):
        u = C.u_west(n)
        if g["west"] <= u <= g["east"]:
            lines.append(LineString([(u, g["south"]), (u, g["north"])]))
            names.append(f"西{n}丁目通")
    for n in range(1, 5):
        u = C.u_east(n)
        if g["west"] <= u <= g["east"]:
            lines.append(LineString([(u, g["south"]), (u, g["north"])]))
            names.append(f"東{n}丁目通")
    lat = gpd.GeoDataFrame(
        {"name": names}, geometry=[C.from_grid(x) for x in lines], crs=C.CRS_PROJ
    )
    write("lattice", to_fc(lat, ["name"]))

    reg_props = [
        "code",
        "kind",
        "shape",
        "route",
        "crossing",
        "speed",
        "n_lanes",
        "length",
        "in_cordon",
        "dir_kind",
    ]
    write("reg_point", optional("regulations.gpkg", "point", reg_props))
    write("reg_line", optional("regulations.gpkg", "line", reg_props))
    write(
        "signals",
        optional(
            "signals.gpkg",
            "intersections",
            [
                "jartic_id",
                "intersection",
                "in_cordon",
                "n_phases",
                "n_in_links",
                "n_out_links",
                "cycle_min_s",
                "cycle_max_s",
                "n_hours",
            ],
        ),
    )
    write(
        "census",
        optional(
            "census.gpkg",
            "sections",
            [
                "section_id",
                "route",
                "n_lanes",
                "w_carriageway",
                "speed_limit",
                "heavy_pct",
                "v_peak_up",
                "v_off_up",
                "v12h",
                "congestion",
                "right_turn_lane",
                "in_cordon",
            ],
        ),
    )

    stats = report("01_probe.json")
    reg_rep, sig_rep, cen_rep = (
        report("11_regulations.json"),
        report("12_signals.json"),
        report("14_census.json"),
    )
    gr = C.CORDON_GRID
    meta = dict(
        center=[141.3501, 43.0591],
        bearing=-C.GRID_BEARING_DEG,
        grid_bearing=C.GRID_BEARING_DEG,
        cordon_size=[
            round(gr["east"] - gr["west"], 1),
            round(gr["north"] - gr["south"], 1),
        ],
        n_links=report("04_inventory.json").get("n_links"),
        n_nodes=stats["endpoint_snap"]["by_tolerance"][str(C.SNAP_TOL_M)]["n_nodes"],
        n_junctions=stats["junctions"]["n_deg3plus"],
        n_streets=report("04_inventory.json").get("n_streets"),
        n_through=report("04_inventory.json").get("n_through"),
        signals=dict(
            n_in_cordon=sig_rep.get("n_in_cordon"),
            cycle=sig_rep.get("cordon_cycle_s", {}),
            target_month=sig_rep.get("target_month"),
        ),
        regulations=dict(
            n_in_cordon=(
                (reg_rep.get("layers", {}).get("point", {}).get("n_in_cordon", 0) or 0)
                + (reg_rep.get("layers", {}).get("line", {}).get("n_in_cordon", 0) or 0)
            ),
            target_month=reg_rep.get("target_month"),
            not_provided=reg_rep.get("not_provided_by_hokkaido", []),
        ),
        census=dict(
            n_in_cordon=cen_rep.get("n_sections_cordon"),
            coverage=cen_rep.get("coverage", {}).get("ratio"),
        ),
    )
    (OUT / "meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=1), encoding="utf-8")
    print("  meta.json")


if __name__ == "__main__":
    main()
