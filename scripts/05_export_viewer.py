"""Phase 0 の網をブラウザで確認するビューワを書き出す。

04 の街路インベントリ付きリンク、コードン、格子線、立体構造を GeoJSON にし、
単一 HTML に埋め込む(file:// で開けるようにするため)。

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

    out = C.ROOT / "viewer"
    out.mkdir(exist_ok=True)
    (out / "network.geojson").write_text(json.dumps(net, ensure_ascii=False), encoding="utf-8")

    center = gpd.GeoSeries([C.cordon_polygon()], crs=C.CRS_PROJ).to_crs("EPSG:4326")
    cx, cy = center.iloc[0].centroid.x, center.iloc[0].centroid.y

    stats = json.loads((C.REPORTS / "01_probe.json").read_text(encoding="utf-8"))
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
    )

    tpl = (C.ROOT / "viewer" / "_template.html").read_text(encoding="utf-8")
    html = (
        tpl.replace("__META__", json.dumps(meta, ensure_ascii=False))
        .replace("__NETWORK__", json.dumps(net, ensure_ascii=False))
        .replace("__CORDON__", json.dumps(cordon_fc, ensure_ascii=False))
        .replace("__LATTICE__", json.dumps(lattice_fc, ensure_ascii=False))
    )
    (out / "index.html").write_text(html, encoding="utf-8")
    size = (out / "index.html").stat().st_size / 1e6
    print(f"viewer/index.html を書き出した ({size:.2f} MB)")
    print(json.dumps(meta, ensure_ascii=False, indent=2)[:600])


if __name__ == "__main__":
    main()
