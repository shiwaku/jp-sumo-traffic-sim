"""125m メッシュのゾーン(人口・従業者)と、SUMO の TAZ を作る(docs/sumo-design.md §13.5、D3)。

1. 対象区域に中心が入る 6 次メッシュ(125m)を並べる
2. 人口: 国勢調査 2020 の 125m メッシュ人口をメッシュコードで結合
3. 従業者: 経済センサス 2021 の小地域を国勢調査 2020 の小地域の境界に町丁名で対応付け
   (od/zones.py)、境界ごとの従業者を面積の比でメッシュに配る
4. 内部ゾーン = 人口か従業者のあるメッシュ。発着の Edge は、中点がメッシュに入る Edge
   (市区町村道を優先)。無ければメッシュの中心に近い Edge を NEAREST_K 本
5. 外部ゾーン = 区域の境界をまたぐ Edge を持つ、区域の外のノードごとに 1 つ。
   発 = そのノードから区域へ入る Edge、着 = 区域から出てくる Edge

入力:  case.toml [zones] の 125m メッシュ人口・小地域の境界・経済センサス
       data/sumo/{case}.net.xml、data/processed/edges.gpkg
出力:  data/processed/zones.gpkg(internal・external)、data/sumo/{case}.taz.xml
       reports/60_zones.json
"""

import json
import sys
from collections import Counter, defaultdict
from pathlib import Path
from xml.sax.saxutils import quoteattr

import geopandas as gpd
import numpy as np
import pandas as pd
import pyarrow.dataset as ds
import sumolib
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree
from shapely.geometry import Point, box

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim import config as C
from jp_sumo_traffic_sim.mesh import mesh6_bounds, mesh6_code, mesh6_index
from jp_sumo_traffic_sim.od.zones import match_towns, normalize_town
from jp_sumo_traffic_sim.sumo.netgen import base_eid

Z = C.CASE["zones"]
CRS_LL = "EPSG:6668"
NET = C.ROOT / "data" / "sumo" / f"{C.CASE_NAME}.net.xml"
EMP_COL = "T001168009"  # 従業者数 総数
LOCAL_CATEGORY = "3"  # KSJ N13_003 = 市区町村道。ゾーンの発着に優先する
NEAREST_K = 2
NEAREST_MAX_M = 300.0


def path_of(p: str) -> Path:
    q = Path(p)
    return q if q.is_absolute() else (C.ROOT / q).resolve()


def mesh_grid(poly_ll) -> gpd.GeoDataFrame:
    """poly_ll(緯度経度)の外接矩形を覆う 6 次メッシュ。"""
    x0, y0, x1, y1 = poly_ll.bounds
    i0, j0 = mesh6_index(x0, y0)
    i1, j1 = mesh6_index(x1, y1)
    rows = [
        dict(mesh=mesh6_code(i, j), geometry=box(*mesh6_bounds(i, j)))
        for i in range(i0, i1 + 1)
        for j in range(j0, j1 + 1)
    ]
    return gpd.GeoDataFrame(rows, crs=CRS_LL).to_crs(C.CRS_PROJ)


def load_population() -> dict[str, int]:
    t = (
        ds.dataset(path_of(Z["pop_mesh125"]))
        .to_table(
            filter=ds.field("MESH1_ID").isin(Z["mesh1"]), columns=["KEY_CODE", "人口（総数）"]
        )
        .to_pandas()
    )
    return dict(zip(t["KEY_CODE"], t["人口（総数）"].astype(int), strict=True))


def load_employees(rep: dict) -> gpd.GeoDataFrame:
    """小地域の境界(町丁名で統合)ごとの従業者数。"""
    pref = Z["city_prefix"]
    e = pd.read_csv(path_of(Z["econ_census"]), encoding="cp932", dtype=str, skiprows=[1])
    e = e[e["KEY_CODE"].str.startswith(pref)].copy()
    e["city"] = e["KEY_CODE"].str[:5]
    e["town"] = e["AZA_NAME"].map(normalize_town)
    e["emp"] = pd.to_numeric(e[EMP_COL], errors="coerce").fillna(0.0)  # 「-」= 0、秘匿「X」も 0

    b = gpd.read_file(f"zip://{path_of(Z['small_area_boundary'])}")
    b = b[b["KEY_CODE"].str.startswith(pref)].copy()
    b["city"] = b["KEY_CODE"].str[:5]
    b["town"] = b["S_NAME"].fillna("").map(normalize_town)
    b = b.to_crs(C.CRS_PROJ).dissolve(by=["city", "town"], as_index=False)[
        ["city", "town", "geometry"]
    ]

    econ_names = e.groupby("city")["town"].apply(list).to_dict()
    bound_names = b.groupby("city")["town"].apply(list).to_dict()
    m = match_towns(econ_names, bound_names)
    keys = list(zip(e["city"], e["town"], strict=True))
    e["bkey"] = [m.get(k) for k in keys]
    exact = sum(1 for k in keys if m.get(k) == k)
    matched = e["bkey"].notna()
    rep["employees"] = dict(
        n_econ_areas=len(e),
        n_boundary_areas=len(b),
        matched_exact=exact,
        matched_prefix=int(matched.sum()) - exact,
        unmatched=int((~matched).sum()),
        employees_total_city=int(e["emp"].sum()),
        employees_matched_share=round(float(e.loc[matched, "emp"].sum() / e["emp"].sum()), 4),
        unmatched_top=[
            dict(city=r.city, name=r.AZA_NAME, emp=int(r.emp))
            for r in e[~matched].nlargest(8, "emp").itertuples()
        ],
    )
    emp = e[matched].groupby("bkey")["emp"].sum()
    b["emp"] = [float(emp.get((c, t), 0.0)) for c, t in zip(b["city"], b["town"], strict=True)]
    return b[b["emp"] > 0]


def allocate_by_area(meshes: gpd.GeoDataFrame, areas: gpd.GeoDataFrame) -> pd.Series:
    """小地域の従業者を、小地域とメッシュの重なりの面積の比で配る(メッシュ → 従業者)。"""
    a = areas.copy()
    a["a_area"] = a.geometry.area
    ov = gpd.overlay(
        meshes[["mesh", "geometry"]], a[["emp", "a_area", "geometry"]], how="intersection"
    )
    ov["v"] = ov["emp"] * ov.geometry.area / ov["a_area"]
    return ov.groupby("mesh")["v"].sum()


def net_edges(net) -> list[dict]:
    cat = gpd.read_file(C.PROCESSED / "edges.gpkg", layer="edges", ignore_geometry=True)
    cat = dict(zip(cat["eid"].astype(int), cat["category"].astype(str), strict=True))
    out = []
    for e in net.getEdges():
        if e.getFunction() == "internal" or not e.allows("passenger"):
            continue
        shp = e.getShape()
        line = np.array(shp)
        seg = np.hypot(*np.diff(line, axis=0).T)
        half, acc, mid = seg.sum() / 2, 0.0, shp[0]
        for k, s in enumerate(seg):
            if acc + s >= half and s > 0:
                t = (half - acc) / s
                mid = tuple(line[k] + t * (line[k + 1] - line[k]))
                break
            acc += s
        out.append(
            dict(
                id=e.getID(),
                mid=mid,
                length=e.getLength(),
                category=cat.get(base_eid(e.getID())),
                frm=e.getFromNode().getID(),
                to=e.getToNode().getID(),
                succ=[t.getID() for t in e.getOutgoing() if t.allows("passenger")],
            )
        )
    return out


def main_component(edges: list[dict]) -> set[str]:
    """Edge の接続の最大の強連結成分(どの Edge からどの Edge へも行ける集合)。"""
    idx = {e["id"]: k for k, e in enumerate(edges)}
    src, dst = [], []
    for e in edges:
        for t in e["succ"]:
            if t in idx:
                src.append(idx[e["id"]])
                dst.append(idx[t])
    g = csr_matrix((np.ones(len(src)), (src, dst)), shape=(len(edges), len(edges)))
    _, label = connected_components(g, directed=True, connection="strong")
    big = np.bincount(label).argmax()
    return {e["id"] for e, lab in zip(edges, label, strict=True) if lab == big}


def zone_edges(zones: gpd.GeoDataFrame, edges: list[dict]) -> tuple[dict, Counter]:
    """ゾーン → [(edge id, 重み = 長さ)]。中点がメッシュに入る Edge(市区町村道を優先)、
    無ければ近い Edge。"""
    pts = gpd.GeoDataFrame(
        dict(i=range(len(edges))),
        geometry=[Point(e["mid"]) for e in edges],
        crs=C.CRS_PROJ,
    )
    hit = gpd.sjoin(pts, zones[["mesh", "geometry"]], predicate="within")
    by_zone = defaultdict(list)
    for i, z in zip(hit["i"], hit["mesh"], strict=True):
        by_zone[z].append(edges[i])
    local = [k for k, e in enumerate(edges) if e["category"] == LOCAL_CATEGORY]
    tree_local = cKDTree([edges[k]["mid"] for k in local])
    tree_all = cKDTree([e["mid"] for e in edges])
    out, how = {}, Counter()
    for z, geom in zip(zones["mesh"], zones.geometry, strict=True):
        es = by_zone.get(z, [])
        loc = [e for e in es if e["category"] == LOCAL_CATEGORY]
        if loc:
            how["local_inside"] += 1
            es = loc
        elif es:
            how["arterial_inside"] += 1
        else:
            c = geom.centroid
            d, k = tree_local.query((c.x, c.y), NEAREST_K)
            es = [edges[local[j]] for dd, j in zip(d, k, strict=True) if dd <= NEAREST_MAX_M]
            if not es:
                d, k = tree_all.query((c.x, c.y), NEAREST_K)
                es = [edges[j] for dd, j in zip(d, k, strict=True) if dd <= NEAREST_MAX_M]
            how["nearest" if es else "none"] += 1
        out[z] = [(e["id"], round(e["length"], 1)) for e in es]
    return out, how


def external_gates(net, edges: list[dict], region, main: set[str]) -> list[dict]:
    """区域の外のノードのうち、区域の境界をまたぐ Edge を持つもの(ネットワークは区域に
    掛かるリンクまでで切れているので、ここが区域の出入口になる)。

    発 = そのノードから区域内のノードへ入る Edge、着 = 区域内のノードから来る Edge。
    行き止まりの端だけでは足りない: 環状通のすぐ外を並走する街路が端のノードをつなぐため、
    放射方向の幹線の端の多くは隣が 2 つ以上ある。
    最大の強連結成分とつながらない Edge は除き、発も着も無いノードは落とす。
    """
    inside = {}

    def is_in(nid: str) -> bool:
        if nid not in inside:
            inside[nid] = region.contains(Point(net.getNode(nid).getCoord()))
        return inside[nid]

    from_main = {t for e in edges if e["id"] in main for t in e["succ"]}
    src, snk = defaultdict(list), defaultdict(list)
    for e in edges:
        fi, ti = is_in(e["frm"]), is_in(e["to"])
        if not fi and ti and (e["id"] in main or set(e["succ"]) & main):
            src[e["frm"]].append(e)
        elif fi and not ti and (e["id"] in main or e["id"] in from_main):
            snk[e["to"]].append(e)
    gates = []
    for nid in sorted(set(src) | set(snk)):
        es = src[nid] + snk[nid]
        gates.append(
            dict(
                zone=f"ext_{nid}",
                node=nid,
                category=min((e["category"] or "9") for e in es),
                sources=[(e["id"], 1.0) for e in src[nid]],
                sinks=[(e["id"], 1.0) for e in snk[nid]],
                geometry=Point(net.getNode(nid).getCoord()),
            )
        )
    return gates


def taz_xml(internal: dict, gates: list[dict]) -> str:
    lines = ['<?xml version="1.0" encoding="UTF-8"?>', "<additional>"]

    def taz(zid, sources, sinks):
        lines.append(f"    <taz id={quoteattr(zid)}>")
        for eid, w in sources:
            lines.append(f'        <tazSource id="{eid}" weight="{w}"/>')
        for eid, w in sinks:
            lines.append(f'        <tazSink id="{eid}" weight="{w}"/>')
        lines.append("    </taz>")

    for z, es in internal.items():
        taz(z, es, es)
    for g in gates:
        taz(g["zone"], g["sources"], g["sinks"])
    lines.append("</additional>")
    return "\n".join(lines) + "\n"


def main() -> None:
    rep: dict = dict(case=C.CASE_NAME)
    region = C.region_polygon()
    region_ll = gpd.GeoSeries([region], crs=C.CRS_PROJ).to_crs(CRS_LL).iloc[0]
    grid = mesh_grid(region_ll.buffer(0.02))  # 区域の外にはみ出す小地域の面積按分のため広めに
    grid["in_region"] = grid.geometry.centroid.within(region)

    pop = load_population()
    grid["pop"] = grid["mesh"].map(pop).fillna(0).astype(int)
    areas = load_employees(rep)
    emp = allocate_by_area(grid, areas)
    grid["emp"] = grid["mesh"].map(emp).fillna(0.0).round(1)

    zones = grid[grid["in_region"] & ((grid["pop"] > 0) | (grid["emp"] > 0))].copy()
    zones = zones.reset_index(drop=True)[["mesh", "pop", "emp", "geometry"]]

    net = sumolib.net.readNet(str(NET))
    edges = net_edges(net)
    main_ids = main_component(edges)
    ze, how = zone_edges(zones, [e for e in edges if e["id"] in main_ids])
    zones["n_edges"] = [len(ze[z]) for z in zones["mesh"]]
    zones = zones[zones["n_edges"] > 0]
    internal = {z: ze[z] for z in zones["mesh"]}
    gates = external_gates(net, edges, region, main_ids)

    out = C.PROCESSED / "zones.gpkg"
    zones.to_file(out, layer="internal", driver="GPKG")
    ext = gpd.GeoDataFrame(
        [
            dict(
                zone=g["zone"],
                node=g["node"],
                category=g["category"],
                sources=",".join(e for e, _ in g["sources"]),
                sinks=",".join(e for e, _ in g["sinks"]),
                geometry=g["geometry"],
            )
            for g in gates
        ],
        crs=C.CRS_PROJ,
    )
    ext.to_file(out, layer="external", driver="GPKG")
    (C.ROOT / "data" / "sumo" / f"{C.CASE_NAME}.taz.xml").write_text(
        taz_xml(internal, gates), encoding="utf-8"
    )

    in_reg = grid[grid["in_region"]]
    rep.update(
        region_km2=round(region.area / 1e6, 1),
        n_net_edges=len(edges),
        n_edges_main_component=len(main_ids),
        n_meshes_in_region=len(in_reg),
        n_internal_zones=len(zones),
        population_in_region=int(in_reg["pop"].sum()),
        population_in_zones=int(zones["pop"].sum()),
        employees_in_region=int(round(in_reg["emp"].sum())),
        employees_in_zones=int(round(zones["emp"].sum())),
        zone_edge_choice=dict(how),
        edges_per_zone_median=float(np.median(zones["n_edges"])),
        n_external_zones=len(gates),
        external_by_category=dict(Counter(g["category"] for g in gates)),
        top_employment_zones=[
            dict(mesh=r.mesh, emp=round(r.emp), pop=int(r.pop))
            for r in zones.nlargest(5, "emp").itertuples()
        ],
        note=(
            "人口 = 国勢調査 2020 の 125m メッシュ。従業者 = 経済センサス 2021 の小地域を"
            "町丁名で国勢調査 2020 の小地域の境界に対応付け、面積の比でメッシュに配ったもの"
        ),
    )
    (C.REPORTS / "60_zones.json").write_text(
        json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(rep, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
