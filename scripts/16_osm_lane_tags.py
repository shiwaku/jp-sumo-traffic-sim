"""OpenStreetMap の車線・転回情報の充足を調べる(docs/sumo-design.md §13.4、D1c)。

車線数は センサス → OSM の lanes → PLATEAU の実測幅 → KSJ の幅員区分、交差点の転回は
OSM の turn:lanes・転回制約(restriction)を使う方針。区域内で OSM にどれだけ入っているかを数え、
取り込む価値があるかを判断する材料にする。

- 道路: highway が自動車の通る種別の way。延長で集計する(区域でクリップ)
- lanes / lanes:forward / lanes:backward、turn:lanes(:forward / :backward)
- 転回制約: type=restriction の relation(no_left_turn など)
- 信号交差点(層[2])の流入のうち、turn:lanes を持つ OSM の way が近くにある割合

入力:  OSM(Overpass API、1 回だけ問い合わせて data/interim/osm_lane_tags.json にキャッシュ)
       data/processed/network_conflated.gpkg・edges.gpkg
出力:  reports/16_osm_lane_tags.json
"""

import json
import sys
import urllib.parse
import urllib.request
from collections import Counter
from pathlib import Path

import geopandas as gpd
from shapely.geometry import LineString

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim import config as C

CACHE = C.INTERIM / "osm_lane_tags.json"
HIGHWAYS = (
    "motorway|trunk|primary|secondary|tertiary|unclassified|residential|living_street|"
    "motorway_link|trunk_link|primary_link|secondary_link|tertiary_link"
)
ENDPOINTS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass.osm.jp/api/interpreter",
]
NEAR_M = 20.0  # 信号交差点の流入と OSM の way の対応付けの距離


def fetch_osm() -> dict:
    if CACHE.exists():
        return json.loads(CACHE.read_text(encoding="utf-8"))
    reg = C.region_polygon()
    w, s, e, n = gpd.GeoSeries([reg], crs=C.CRS_PROJ).to_crs("EPSG:4326").total_bounds
    bbox = f"{s},{w},{n},{e}"
    q = (
        "[out:json][timeout:180];("
        f'way["highway"~"^({HIGHWAYS})$"]({bbox});'
        f'relation["type"="restriction"]({bbox});'
        ");out tags geom;"
    )
    last = None
    for ep in ENDPOINTS:
        try:
            req = urllib.request.Request(
                ep,
                data=urllib.parse.urlencode({"data": q}).encode(),
                headers={"User-Agent": "jp-sumo-traffic-sim/0.1"},
            )
            with urllib.request.urlopen(req, timeout=240) as r:  # noqa: S310
                d = json.loads(r.read().decode())
            CACHE.write_text(json.dumps(d, ensure_ascii=False), encoding="utf-8")
            return d
        except Exception as ex:  # 504 やタイムアウトは次のミラーへ
            last = ex
    raise SystemExit(f"Overpass から取得できなかった: {last}")


def main() -> None:
    d = fetch_osm()
    reg = C.region_polygon()
    ways, rels = [], []
    for el in d["elements"]:
        if el["type"] == "way" and el.get("geometry"):
            ways.append(
                dict(
                    tags=el.get("tags", {}),
                    geometry=LineString([(p["lon"], p["lat"]) for p in el["geometry"]]),
                )
            )
        elif el["type"] == "relation":
            rels.append(el.get("tags", {}))
    g = gpd.GeoDataFrame(ways, crs="EPSG:4326").to_crs(C.CRS_PROJ)
    g["geometry"] = g.geometry.intersection(reg)
    g = g[~g.geometry.is_empty]
    g["len"] = g.length
    g["highway"] = [t.get("highway") for t in g["tags"]]

    def has(t: dict, keys: tuple) -> bool:
        return any(k in t for k in keys)

    lane_keys = ("lanes", "lanes:forward", "lanes:backward")
    turn_keys = ("turn:lanes", "turn:lanes:forward", "turn:lanes:backward")
    g["has_lanes"] = [has(t, lane_keys) for t in g["tags"]]
    g["has_turn"] = [has(t, turn_keys) for t in g["tags"]]

    by_hw = {}
    for hw, sub in g.groupby("highway"):
        total = float(sub["len"].sum())
        by_hw[hw] = dict(
            km=round(total / 1000, 1),
            lanes_share=round(float(sub.loc[sub.has_lanes, "len"].sum()) / total, 3),
            turn_lanes_share=round(float(sub.loc[sub.has_turn, "len"].sum()) / total, 3),
        )
    total = float(g["len"].sum())

    # 信号交差点の流入(層[2]の Edge の終端 = 信号ノード)の近くに turn:lanes の way があるか
    nodes = gpd.read_file(C.PROCESSED / "network_conflated.gpkg", layer="nodes")
    edges = gpd.read_file(C.PROCESSED / "edges.gpkg", layer="edges")
    sig = set(nodes.loc[nodes["has_signal"], "nid"].astype(int))
    appr = edges[edges["to"].astype(int).isin(sig)].copy()
    tl = g[g.has_turn]
    appr["end_pt"] = [
        gpd.points_from_xy([c.coords[-1][0]], [c.coords[-1][1]])[0] for c in appr.geometry
    ]
    ends = gpd.GeoDataFrame(appr[["eid"]], geometry=appr["end_pt"], crs=C.CRS_PROJ)
    near = gpd.sjoin_nearest(ends, tl[["geometry"]], how="left", max_distance=NEAR_M)
    n_appr_turn = int(near.groupby(level=0)["index_right"].apply(lambda s: s.notna().any()).sum())

    rep = dict(
        source="OpenStreetMap(Overpass API、© OpenStreetMap contributors / ODbL)",
        n_ways=len(g),
        road_km=round(total / 1000, 1),
        lanes_share_by_length=round(float(g.loc[g.has_lanes, "len"].sum()) / total, 3),
        turn_lanes_share_by_length=round(float(g.loc[g.has_turn, "len"].sum()) / total, 3),
        n_ways_with_turn_lanes=int(g.has_turn.sum()),
        by_highway=by_hw,
        restrictions=dict(n=len(rels), by_type=dict(Counter(t.get("restriction") for t in rels))),
        signal_approaches=dict(
            n=len(appr),
            with_turn_lanes_nearby=n_appr_turn,
            share=round(n_appr_turn / len(appr), 3) if len(appr) else None,
            near_m=NEAR_M,
        ),
    )
    (C.REPORTS / "16_osm_lane_tags.json").write_text(
        json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(rep, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
