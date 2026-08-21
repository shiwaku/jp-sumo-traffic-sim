"""Phase 0 の成果をブラウザで確認するビューワを書き出す。

街路インベントリ付きリンク・コードン・格子線・立体構造に加えて、
JARTIC 交通規制情報、JARTIC 交差点制御情報、道路交通センサスを重ねる。
GeoJSON は単一 HTML に埋め込む(file:// で開けるようにするため)。

出力: viewer/index.html, viewer/network.geojson
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
        feats.append(
            {
                "type": "Feature",
                "geometry": geom,
                "properties": {
                    k: (None if r.get(k) is None or r.get(k) != r.get(k) else r.get(k))
                    for k in props
                },
            }
        )
    return {"type": "FeatureCollection", "features": feats}


def main() -> None:
    roads = gpd.read_file(C.INTERIM / "streets.gpkg", layer="roads")
    roads["road_category"] = roads["N13_003"].astype(str).map(ROAD_CATEGORY)
    roads["road_state"] = roads["N13_004"].astype(str).map(ROAD_STATE)
    roads["width_class"] = roads["N13_006"].astype(str).map(ROAD_WIDTH)
    roads["length_m"] = roads["length_m"].round(1)
    roads["street"] = roads["street"].fillna("(未割り当て)")
    roads["street_kind"] = roads["street_kind"].fillna("未割り当て")

    inv = json.loads((C.REPORTS / "04_inventory.json").read_text(encoding="utf-8"))
    through = {s["street"] for s in inv["streets"] if s["through"]}
    roads["through"] = roads["street"].isin(through)

    net = to_fc(
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
    )

    cordon = gpd.read_file(C.INTERIM / "ksj_clip.gpkg", layer="cordon")
    cordon_fc = to_fc(cordon, ["kind"])

    # 格子線(コードン範囲に収まる部分)
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
    lattice_fc = to_fc(lat, ["name"])

    # --- 重ねるデータ(無ければ空で通す) ---
    def optional(path, layer, props):
        f = C.INTERIM / path
        if not f.exists():
            return {"type": "FeatureCollection", "features": []}
        try:
            g = gpd.read_file(f, layer=layer)
        except Exception:
            return {"type": "FeatureCollection", "features": []}
        for c in props:
            if c not in g.columns:
                g[c] = None
        return to_fc(g, props)

    REG_PROPS = [
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
    regulations = {
        "point": optional("regulations.gpkg", "point", REG_PROPS),
        "line": optional("regulations.gpkg", "line", REG_PROPS),
    }
    signals = optional(
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
    )
    census = optional(
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
    )

    out = C.ROOT / "viewer"
    out.mkdir(exist_ok=True)
    (out / "network.geojson").write_text(json.dumps(net, ensure_ascii=False), encoding="utf-8")

    center = gpd.GeoSeries([C.cordon_polygon()], crs=C.CRS_PROJ).to_crs("EPSG:4326")
    cx, cy = center.iloc[0].centroid.x, center.iloc[0].centroid.y

    stats = json.loads((C.REPORTS / "01_probe.json").read_text(encoding="utf-8"))

    def report(name):
        f = C.REPORTS / name
        return json.loads(f.read_text(encoding="utf-8")) if f.exists() else {}

    reg_rep, sig_rep, cen_rep = (
        report("11_regulations.json"),
        report("12_signals.json"),
        report("14_census.json"),
    )
    meta = dict(
        center=[round(cx, 6), round(cy, 6)],
        bearing=round(-C.GRID_BEARING_DEG, 3),
        n_links=inv["n_links"],
        n_streets=inv["n_streets"],
        n_through=inv["n_through"],
        n_through_lattice=inv["n_through_lattice"],
        n_through_backstreet=inv["n_through_backstreet"],
        n_nodes=stats["endpoint_snap"]["by_tolerance"][str(C.SNAP_TOL_M)]["n_nodes"],
        n_junctions=stats["junctions"]["n_deg3plus"],
        cordon_size=[
            round(g["east"] - g["west"], 1),
            round(g["north"] - g["south"], 1),
        ],
        grid_bearing=C.GRID_BEARING_DEG,
        width_class=stats["width_class_N13_006"],
        road_state=stats["road_state_N13_004"],
        link_length=stats["link_length_m"],
        regulations=dict(
            n=sum(len(v["features"]) for v in regulations.values()),
            n_in_cordon=sum(
                1
                for v in regulations.values()
                for f in v["features"]
                if f["properties"].get("in_cordon")
            ),
            target_month=reg_rep.get("target_month"),
            sim_relevant=reg_rep.get("simulation_relevant", []),
            not_provided=reg_rep.get("not_provided_by_hokkaido", []),
        ),
        signals=dict(
            n=len(signals["features"]),
            n_in_cordon=sig_rep.get("n_in_cordon"),
            join_rate=sig_rep.get("join_rate"),
            cycle=sig_rep.get("cordon_cycle_s", {}),
            target_month=sig_rep.get("target_month"),
        ),
        census=dict(
            n=len(census["features"]),
            n_in_cordon=cen_rep.get("n_sections_cordon"),
            coverage=cen_rep.get("coverage", {}).get("ratio"),
            v_peak=cen_rep.get("distributions", {}).get("v_peak_up", {}),
        ),
    )

    tpl = (C.ROOT / "viewer" / "_template.html").read_text(encoding="utf-8")
    html = (
        tpl.replace("__META__", json.dumps(meta, ensure_ascii=False))
        .replace("__NETWORK__", json.dumps(net, ensure_ascii=False))
        .replace("__CORDON__", json.dumps(cordon_fc, ensure_ascii=False))
        .replace("__LATTICE__", json.dumps(lattice_fc, ensure_ascii=False))
        .replace("__REG_POINT__", json.dumps(regulations["point"], ensure_ascii=False))
        .replace("__REG_LINE__", json.dumps(regulations["line"], ensure_ascii=False))
        .replace("__SIGNALS__", json.dumps(signals, ensure_ascii=False))
        .replace("__CENSUS__", json.dumps(census, ensure_ascii=False))
    )
    (out / "index.html").write_text(html, encoding="utf-8")
    size = (out / "index.html").stat().st_size / 1e6
    print(f"viewer/index.html を書き出した ({size:.2f} MB)")
    print(json.dumps(meta, ensure_ascii=False, indent=2)[:600])


if __name__ == "__main__":
    main()
