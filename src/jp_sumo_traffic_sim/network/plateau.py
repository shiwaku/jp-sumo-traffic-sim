"""PLATEAU の交通(道路)モデル LOD1 の道路ポリゴン(docs/sumo-design.md §13.4、D1b)。

札幌市 2020 年度版の道路は LOD1(道路の面)まで。属性の区間種別に「交差部」は付いていないが、
LOD1 は交差点で道路を区切って整備する仕様なので、交差点の面は別のポリゴンになっている。
形で見分ける: **KSJ の交差点ノード(次数 3 以上)を含み、面積が小さい**ポリゴンを交差点の面とする
(交差点ノードを含むポリゴンの面積の中央値は 143 m²、次数 2 のノードを含む道路区間は 1,301 m²)。

- 交差点の面: ノードの統合と SUMO の交差点の形に使う(topology.contract_in_polygons)
- 道路区間の面: リンクに沿って幅を測り、車線数の推定に使う(road_width)
"""

from __future__ import annotations

import math
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
from shapely import STRtree
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

GML = "{http://www.opengis.net/gml}"
TRAN = "{http://www.opengis.net/citygml/transportation/2.0}"
URO = "{https://www.geospatial.jp/iur/uro/3.1}"

JUNCTION_MAX_AREA_M2 = 5000.0  # 交差点の面とみなす上限(駅前広場のような巨大な面は除く)
WIDTH_STEP_M = 10.0  # 幅を測る間隔
WIDTH_END_MARGIN_M = 8.0  # リンクの両端(交差点)付近は測らない
WIDTH_HALF_M = 40.0  # 幅を測る横断線の片側の長さ


def load_road_polygons(tran_dir: Path, crs: str):
    """udx/tran/*.gml の道路ポリゴン → GeoDataFrame(crs に投影)。"""
    import geopandas as gpd

    recs = []
    for p in sorted(tran_dir.glob("*.gml")):
        for road in ET.parse(p).getroot().iter(TRAN + "Road"):
            st = road.find(".//" + URO + "sectionType")
            fn = road.find(TRAN + "function")
            for pl in road.iter(GML + "posList"):
                v = np.array(pl.text.split(), float).reshape(-1, 3)
                if len(v) < 4:
                    continue
                recs.append(
                    dict(
                        gid=road.get(GML + "id"),
                        section_type=st.text if st is not None else None,
                        function=fn.text if fn is not None else None,
                        geometry=Polygon(v[:, [1, 0]]),  # posList は 緯度 経度 高さ
                    )
                )
    g = gpd.GeoDataFrame(recs, crs="EPSG:6668")
    return g.to_crs(crs)


def junction_polygons(polys, junction_xy: list[tuple[float, float]]) -> list[int]:
    """交差点の面とみなすポリゴンの行番号(交差点ノードを含み、面積が上限以下)。"""
    tree = STRtree(list(polys.geometry))
    hit = set()
    for x, y in junction_xy:
        for i in tree.query(Point(x, y), predicate="within"):
            if polys.geometry.iloc[i].area <= JUNCTION_MAX_AREA_M2:
                hit.add(int(i))
    return sorted(hit)


def road_width(line: LineString, tree: STRtree, geoms: list) -> float | None:
    """リンクに沿って道路区間の面の幅を測り、中央値を返す(面が無ければ None)。

    WIDTH_STEP_M ごとに、リンクに直交する横断線を引き、その点を含む面の区間の長さを幅とする。
    """
    L = line.length
    if L <= 2 * WIDTH_END_MARGIN_M:
        ds = [L / 2]
    else:
        n = max(1, int((L - 2 * WIDTH_END_MARGIN_M) // WIDTH_STEP_M))
        ds = [WIDTH_END_MARGIN_M + (L - 2 * WIDTH_END_MARGIN_M) * (k + 0.5) / n for k in range(n)]
    widths = []
    for d in ds:
        p = line.interpolate(d)
        a, b = line.interpolate(max(0.0, d - 1.0)), line.interpolate(min(L, d + 1.0))
        dx, dy = b.x - a.x, b.y - a.y
        nrm = math.hypot(dx, dy)
        if nrm == 0:
            continue
        ux, uy = -dy / nrm, dx / nrm
        seg = LineString(
            [
                (p.x - ux * WIDTH_HALF_M, p.y - uy * WIDTH_HALF_M),
                (p.x + ux * WIDTH_HALF_M, p.y + uy * WIDTH_HALF_M),
            ]
        )
        cand = [geoms[i] for i in tree.query(seg, predicate="intersects")]
        if not cand:
            continue
        inter = seg.intersection(unary_union(cand))
        parts = list(getattr(inter, "geoms", [inter]))
        for part in parts:
            if part.length > 0 and part.distance(p) < 0.5:
                widths.append(part.length)
                break
    if not widths:
        return None
    return float(np.median(widths))
