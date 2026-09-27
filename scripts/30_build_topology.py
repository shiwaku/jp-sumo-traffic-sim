"""KSJ クリップから実ネットワークの位相を構築する(Phase 1-①)。

入力:  data/interim/ksj_clip.gpkg  layer=roads(車道のみ。庭園路等は除外済み)
       data/raw/plateau/*/udx/tran/*.gml(PLATEAU の道路ポリゴン。あれば交差点の統合と形に使う)
出力:  data/processed/network.gpkg  layer=nodes / links(EPSG:6679)
       (nodes の jshape = 交差点の形の WKT)
       data/interim/plateau_roads.parquet(PLATEAU の道路ポリゴン、32 の幅員の実測でも使う)
       reports/30_topology.json     統計と受け入れ条件の検証結果
"""

import json
import sys
from pathlib import Path

import geopandas as gpd
from shapely.geometry import LineString, Point, Polygon

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim import config as C
from jp_sumo_traffic_sim.network import plateau as P
from jp_sumo_traffic_sim.network import topology as T


def load_records() -> list[dict]:
    g = gpd.read_file(C.INTERIM / "ksj_clip.gpkg", layer="roads")
    g = g[g["in_cordon"]]
    recs = []
    for _, row in g.iterrows():
        parts = (
            row.geometry.geoms if row.geometry.geom_type == "MultiLineString" else [row.geometry]
        )
        for p in parts:
            recs.append(
                dict(
                    geometry=list(p.coords),
                    link_id=row["link_id"],
                    state=str(row["N13_004"]),
                    layer=str(row["N13_005"]),
                    category=str(row["N13_003"]),
                    width=str(row["N13_006"]),
                )
            )
    return recs


def degree_hist(g) -> dict:
    hist: dict[int, int] = {}
    for _, d in g.degree():
        hist[d] = hist.get(d, 0) + 1
    return {str(k): hist[k] for k in sorted(hist)}


def length_stats(g) -> dict:
    ls = sorted(d["length"] for _, _, d in g.edges(data=True))
    if not ls:
        return {}
    q = lambda p: round(ls[min(len(ls) - 1, int(p * len(ls)))], 1)  # noqa: E731
    return dict(
        n=len(ls),
        min=round(ls[0], 2),
        median=q(0.5),
        p95=q(0.95),
        max=round(ls[-1], 1),
        total_km=round(sum(ls) / 1000, 2),
    )


def contract_with_plateau(g) -> dict:
    """PLATEAU の道路ポリゴンで交差点を統合し、交差点の形を持たせる(docs/sumo-design.md §13.4)。"""
    tran = sorted(C.RAW.glob("plateau/*/udx/tran"))
    if not tran:
        return dict(used=False)
    polys = P.load_road_polygons(tran[-1], C.CRS_PROJ)
    polys.to_parquet(C.INTERIM / "plateau_roads.parquet")
    jxy = [(d["x"], d["y"]) for n, d in g.nodes(data=True) if g.degree(n) >= 3]
    idx = P.junction_polygons(polys, jxy)
    st = T.contract_in_polygons(g, [polys.geometry.iloc[i] for i in idx])
    return dict(used=True, n_road_polygons=len(polys), n_junction_polygons=len(idx), **st)


def main() -> None:
    recs = load_records()
    g = T.build_graph(recs)
    stats_before = dict(n_nodes=g.number_of_nodes(), n_links=g.number_of_edges())
    n_short_before = sum(1 for _, _, d in g.edges(data=True) if d["length"] < T.SHORT_LINK_M)

    merge_stats = T.contract_short_links(g)
    plateau_stats = contract_with_plateau(g)
    comp_stats = T.drop_minor_components(g)
    gs = T.check_grade_separated(g)

    cordon = C.region_polygon()
    acc = T.acceptance(g, T.boundary_adapter(cordon))

    # --- 保存 ---
    C.PROCESSED.mkdir(parents=True, exist_ok=True)
    out = C.PROCESSED / "network.gpkg"
    nodes = gpd.GeoDataFrame(
        dict(
            nid=[n for n in g.nodes],
            degree=[g.degree(n) for n in g.nodes],
            jshape=[
                Polygon(d["jshape"]).wkt if d.get("jshape") else "" for _, d in g.nodes(data=True)
            ],
        ),
        geometry=[Point(d["x"], d["y"]) for _, d in g.nodes(data=True)],
        crs=C.CRS_PROJ,
    )
    links = gpd.GeoDataFrame(
        dict(
            a=[u for u, _, _ in g.edges(keys=True)],
            b=[v for _, v, _ in g.edges(keys=True)],
            length_m=[round(d["length"], 2) for _, _, d in g.edges(data=True)],
            state=[d["state"] for _, _, d in g.edges(data=True)],
            layer=[d["layer"] for _, _, d in g.edges(data=True)],
            category=[d["category"] for _, _, d in g.edges(data=True)],
            width=[d["width"] for _, _, d in g.edges(data=True)],
            ksj_ids=[",".join(d["ksj_ids"]) for _, _, d in g.edges(data=True)],
        ),
        geometry=[LineString(d["geometry"]) for _, _, d in g.edges(data=True)],
        crs=C.CRS_PROJ,
    )
    nodes.to_file(out, layer="nodes", driver="GPKG")
    links.to_file(out, layer="links", driver="GPKG")

    rep = dict(
        source="data/interim/ksj_clip.gpkg (roads, in_cordon)",
        snap_tol_m=T.SNAP_TOL_M,
        short_link_m=T.SHORT_LINK_M,
        n_source_records=len(recs),
        before=dict(**stats_before, n_short_links=n_short_before),
        short_link_merge=merge_stats,
        plateau_junctions=plateau_stats,
        minor_components=comp_stats,
        n_self_loops_dropped=g.graph.get("n_self_loops_dropped", 0),
        after=dict(n_nodes=g.number_of_nodes(), n_links=g.number_of_edges()),
        degree_hist=degree_hist(g),
        link_length_m=length_stats(g),
        grade_separated=gs,
        acceptance=acc,
    )
    C.REPORTS.mkdir(parents=True, exist_ok=True)
    (C.REPORTS / "30_topology.json").write_text(
        json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(rep, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
