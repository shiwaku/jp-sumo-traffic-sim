"""Phase 0 予備調査: KSJ 抽出網の素性を測る。

確認項目(資料 5.2):
  1. 端点一致状況(許容誤差ごとの接続率)
  2. 階層順(N13_005)の分布
  3. リンク長の分布(短リンクの数)
  4. 幹線が上下線分離か(スキャンラインで判定)
  5. 属性(道路分類・幅員区分)の充足状況

出力: reports/01_probe.json
"""

import json
import sys
from collections import Counter
from pathlib import Path

import geopandas as gpd
import numpy as np
from scipy.spatial import cKDTree
from shapely.geometry import LineString

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim import config as C
from jp_sumo_traffic_sim.ksj_codes import ROAD_CATEGORY as CODE_CATEGORY
from jp_sumo_traffic_sim.ksj_codes import ROAD_STATE, ROAD_WIDTH


def endpoints(gdf: gpd.GeoDataFrame) -> np.ndarray:
    pts = []
    for geom in gdf.geometry:
        lines = geom.geoms if geom.geom_type == "MultiLineString" else [geom]
        for ls in lines:
            c = list(ls.coords)
            pts.append(c[0][:2])
            pts.append(c[-1][:2])
    return np.asarray(pts, dtype=float)


def cluster_nodes(pts: np.ndarray, tol: float):
    """許容誤差 tol で端点を union-find クラスタリング。"""
    tree = cKDTree(pts)
    parent = np.arange(len(pts))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for i, j in tree.query_pairs(tol):
        ri, rj = find(i), find(j)
        if ri != rj:
            parent[max(ri, rj)] = min(ri, rj)
    labels = np.array([find(i) for i in range(len(pts))])
    return labels


def scanline_hits(gdf: gpd.GeoDataFrame, line: LineString) -> list[float]:
    """スキャンラインとの交点座標(線方向の距離)を返す。"""
    out = []
    for geom in gdf.geometry:
        if not geom.intersects(line):
            continue
        inter = geom.intersection(line)
        geoms = inter.geoms if hasattr(inter, "geoms") else [inter]
        for p in geoms:
            if p.is_empty:
                continue
            out.append(line.project(p.centroid))
    return sorted(out)


def group_gaps(vals: list[float], tol: float = 30.0):
    """近接する交点をまとめ、まとまり間の間隔を返す。"""
    if not vals:
        return [], []
    groups = [[vals[0]]]
    for v in vals[1:]:
        if v - groups[-1][-1] <= tol:
            groups[-1].append(v)
        else:
            groups.append([v])
    centers = [float(np.mean(g)) for g in groups]
    sizes = [len(g) for g in groups]
    gaps = [round(centers[i + 1] - centers[i], 1) for i in range(len(centers) - 1)]
    return list(zip([round(c, 1) for c in centers], sizes, strict=False)), gaps


def main() -> None:
    src = C.INTERIM / "ksj_clip.gpkg"
    roads = gpd.read_file(src, layer="roads")
    cordon = gpd.read_file(src, layer="cordon").query("kind == 'cordon'").geometry.iloc[0]
    inner = roads[roads["in_cordon"]].copy()

    rep: dict = {}

    # --- 1. 端点一致 ---
    pts = endpoints(inner)
    tol_report = {}
    for tol in (0.01, 0.1, 0.5, 1.0, 2.0, 5.0, 10.0):
        labels = cluster_nodes(pts, tol)
        n_nodes = len(set(labels))
        deg = Counter(labels)
        degdist = Counter(deg.values())
        tol_report[str(tol)] = dict(
            n_nodes=n_nodes,
            n_deg1=degdist.get(1, 0),
            n_deg2=degdist.get(2, 0),
            n_deg3plus=sum(v for k, v in degdist.items() if k >= 3),
            max_deg=max(deg.values()),
        )
    rep["endpoint_snap"] = dict(n_endpoints=len(pts), by_tolerance=tol_report)

    # 採用トレランスでの交差点(次数3以上)
    labels = cluster_nodes(pts, C.SNAP_TOL_M)
    deg = Counter(labels)
    node_xy = {}
    for lab in set(labels):
        m = labels == lab
        node_xy[lab] = pts[m].mean(axis=0)
    junctions = [lab for lab, d in deg.items() if d >= 3]
    rep["junctions"] = dict(tolerance_m=C.SNAP_TOL_M, n_deg3plus=len(junctions))

    # --- 2. 階層順 ---
    rep["hierarchy_N13_005"] = {
        str(k): int(v) for k, v in sorted(Counter(inner["N13_005"].astype(str)).items())
    }
    # 同一ノードに複数階層が集まる箇所(誤接続リスク)
    lab_iter = iter(labels)
    link_end_labels = []
    for _ in range(len(inner)):
        link_end_labels.append((next(lab_iter), next(lab_iter)))
    node_layers = {}
    for (a, b), lay in zip(link_end_labels, inner["N13_005"].astype(str), strict=False):
        node_layers.setdefault(a, set()).add(lay)
        node_layers.setdefault(b, set()).add(lay)
    mixed = {k: sorted(v) for k, v in node_layers.items() if len(v) > 1}
    rep["hierarchy_mixed_nodes"] = dict(
        n=len(mixed),
        examples=[
            dict(xy=[round(float(node_xy[k][0]), 1), round(float(node_xy[k][1]), 1)], layers=v)
            for k, v in list(mixed.items())[:10]
        ],
    )

    # --- 3. リンク長分布 ---
    L = inner["length_m"].to_numpy()
    rep["link_length_m"] = dict(
        n=len(L),
        min=round(float(L.min()), 2),
        p05=round(float(np.percentile(L, 5)), 1),
        p25=round(float(np.percentile(L, 25)), 1),
        median=round(float(np.median(L)), 1),
        p75=round(float(np.percentile(L, 75)), 1),
        p95=round(float(np.percentile(L, 95)), 1),
        max=round(float(L.max()), 1),
        n_under_5m=int((L < 5).sum()),
        n_under_12m=int((L < C.SHORT_LINK_M).sum()),
        n_under_30m=int((L < 30).sum()),
    )

    # --- 4. 上下線分離 / 街区間隔(スキャンライン) ---
    minx, miny, maxx, maxy = cordon.bounds
    scans = {}
    for frac in (0.35, 0.5, 0.65):
        y = miny + (maxy - miny) * frac
        hits = scanline_hits(inner, LineString([(minx, y), (maxx, y)]))
        groups, gaps = group_gaps(hits)
        scans[f"EW_scan_y{frac}"] = dict(
            n_hits=len(hits),
            n_streets=len(groups),
            gaps_m=gaps,
            median_gap_m=round(float(np.median(gaps)), 1) if gaps else None,
            multi_carriageway=[g for g in groups if g[1] > 1][:12],
        )
        x = minx + (maxx - minx) * frac
        hits = scanline_hits(inner, LineString([(x, miny), (x, maxy)]))
        groups, gaps = group_gaps(hits)
        scans[f"NS_scan_x{frac}"] = dict(
            n_hits=len(hits),
            n_streets=len(groups),
            gaps_m=gaps,
            median_gap_m=round(float(np.median(gaps)), 1) if gaps else None,
            multi_carriageway=[g for g in groups if g[1] > 1][:12],
        )
    rep["scanlines"] = scans

    # --- 5. 属性充足 ---
    rep["road_category_N13_003"] = {
        CODE_CATEGORY.get(str(k), str(k)): int(v)
        for k, v in sorted(Counter(inner["N13_003"].astype(str)).items())
    }
    rep["width_class_N13_006"] = {
        f"{k}:{ROAD_WIDTH.get(k, k)}": int(v)
        for k, v in sorted(Counter(inner["N13_006"].astype(str)).items())
    }
    rep["road_state_N13_004"] = {
        f"{k}:{ROAD_STATE.get(k, k)}": int(v)
        for k, v in sorted(Counter(inner["N13_004"].astype(str)).items())
    }

    (C.REPORTS / "01_probe.json").write_text(
        json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(rep, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
