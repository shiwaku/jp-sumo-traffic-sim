"""KSJ リンクから実ネットワークの位相を構築する(Phase 1、architecture.md §4.1)。

パイプライン:
  KSJ リンク → 端点抽出 → SNAP_TOL_M でクラスタリング(union-find + KDTree)
  → 立体リンクの接続検証 → 短リンク統合 → networkx.MultiGraph

グラフの規約:
  - ノード属性: x, y (EPSG:6679)
  - エッジ属性: length [m], geometry (座標列), ksj_ids (元リンクIDのリスト),
    state (N13_004 道路状態。結合時は延長最大のものを代表にする),
    category / width (同上)
  - 双方向 Edge 化・一方通行は Phase 1-③ (directed.py) で行う。ここでは無向
"""

from __future__ import annotations

import math

import networkx as nx
import numpy as np
from scipy.spatial import cKDTree

SNAP_TOL_M = 0.5  # Phase 0 実測: 0.01〜1.0m で結果不変、2m 超で誤結合
SHORT_LINK_M = 12.0  # これ未満は交差点内部として扱う(Phase 0 実測 60本)
GRADE_SEPARATED_STATES = ("2", "3")  # 橋・高架 / トンネル
BOUNDARY_TOL_M = 30.0  # コードン境界ノードの判定距離


def is_grade_separated(road_state: str, layer: str | int = "0") -> bool:
    """立体判定。階層順だけでは不十分(創成トンネルは階層順0)なので道路状態も見る。"""
    try:
        lyr = int(layer)
    except (TypeError, ValueError):
        lyr = 0
    return str(road_state) in GRADE_SEPARATED_STATES or lyr != 0


def snap_endpoints(points: np.ndarray, tol: float = SNAP_TOL_M) -> tuple[np.ndarray, np.ndarray]:
    """端点を tol でクラスタリングし、(所属クラスタid配列, クラスタ代表座標) を返す。"""
    parent = np.arange(len(points))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    tree = cKDTree(points)
    for i, j in tree.query_pairs(tol):
        ri, rj = find(i), find(j)
        if ri != rj:
            parent[rj] = ri

    roots = np.array([find(i) for i in range(len(points))])
    uniq, labels = np.unique(roots, return_inverse=True)
    coords = np.zeros((len(uniq), 2))
    for k in range(len(uniq)):
        coords[k] = points[roots == uniq[k]].mean(axis=0)
    return labels, coords


def _line_length(coords: list[tuple[float, float]]) -> float:
    return sum(math.dist(a, b) for a, b in zip(coords, coords[1:], strict=False))


def build_graph(records: list[dict], snap_tol: float = SNAP_TOL_M) -> nx.MultiGraph:
    """レコード(geometry=座標列 + 属性)から無向マルチグラフを作る。

    自己ループ(スナップ後に両端が同一ノードになる周回リンク)は捨てて数える。
    """
    pts = []
    for r in records:
        pts.append(r["geometry"][0])
        pts.append(r["geometry"][-1])
    labels, coords = snap_endpoints(np.asarray(pts, dtype=float), snap_tol)

    g = nx.MultiGraph()
    g.graph["n_self_loops_dropped"] = 0
    for nid, (x, y) in enumerate(coords):
        g.add_node(int(nid), x=float(x), y=float(y))
    for k, r in enumerate(records):
        a, b = int(labels[2 * k]), int(labels[2 * k + 1])
        if a == b:
            g.graph["n_self_loops_dropped"] += 1
            continue
        g.add_edge(
            a,
            b,
            length=_line_length(r["geometry"]),
            geometry=list(r["geometry"]),
            ksj_ids=[r.get("link_id", str(k))],
            state=str(r.get("state", "1")),
            layer=str(r.get("layer", "0")),
            category=str(r.get("category", "")),
            width=str(r.get("width", "")),
        )
    return g


def _merge_attrs(d1: dict, d2: dict, geometry: list) -> dict:
    """次数2ノード除去でエッジを結合するときの属性マージ(延長最大を代表)。"""
    rep = d1 if d1["length"] >= d2["length"] else d2
    return dict(
        length=d1["length"] + d2["length"],
        geometry=geometry,
        ksj_ids=d1["ksj_ids"] + d2["ksj_ids"],
        state=rep["state"],
        layer=rep["layer"],
        category=rep["category"],
        width=rep["width"],
    )


def _join_geoms(g1: list, g2: list, via_xy: tuple[float, float]) -> list:
    """共有ノード via で2本の座標列をつなぐ。向きを揃えてから結合する。"""

    def orient(coords: list, end_at_via: bool) -> list:
        d_head = math.dist(coords[0], via_xy)
        d_tail = math.dist(coords[-1], via_xy)
        at_tail = d_tail <= d_head
        if end_at_via:
            return coords if at_tail else coords[::-1]
        return coords[::-1] if at_tail else coords

    return orient(g1, True) + orient(g2, False)[1:]


def contract_short_links(g: nx.MultiGraph, short_m: float = SHORT_LINK_M) -> dict:
    """短リンクを統合する。

    - 次数3以上のノードに挟まれた短リンク → 交差点を1つのノードに縮約
    - 片端が次数2の短リンク → その次数2ノードを消して隣のエッジと結合
    - 片端が次数1(行き止まりの短い突起) → そのまま残す(レポートで列挙)

    戻り値は統計(縮約数・結合数)。
    """
    n_contracted = n_merged = 0
    changed = True
    while changed:
        changed = False
        for u, v, key, d in list(g.edges(keys=True, data=True)):
            if d["length"] >= short_m or not g.has_edge(u, v, key):
                continue
            deg_u, deg_v = g.degree(u), g.degree(v)
            if deg_u >= 3 and deg_v >= 3:
                # 交差点内部: v を u に縮約し、u を中点へ動かす
                ux, uy = g.nodes[u]["x"], g.nodes[u]["y"]
                vx, vy = g.nodes[v]["x"], g.nodes[v]["y"]
                g.remove_edge(u, v, key)
                for _, w, dd in list(g.edges(v, data=True)):
                    if w == v:
                        continue
                    g.add_edge(u, w, **dd)
                g.remove_node(v)
                g.nodes[u]["x"], g.nodes[u]["y"] = (ux + vx) / 2, (uy + vy) / 2
                n_contracted += 1
            elif deg_u == 2 or deg_v == 2:
                mid = u if deg_u == 2 else v
                other_edges = [
                    (a, b, k2, dd)
                    for a, b, k2, dd in g.edges(mid, keys=True, data=True)
                    if (a, b, k2) != (u, v, key) and (b, a, k2) != (u, v, key)
                ]
                if len(other_edges) != 1:
                    continue
                a, b, k2, dd = other_edges[0]
                far1 = v if mid == u else u  # 短リンクの反対側
                far2 = b if a == mid else a  # 隣エッジの反対側
                if far1 == far2:  # 結合すると自己ループになる(三角形) → 触らない
                    continue
                via = (g.nodes[mid]["x"], g.nodes[mid]["y"])
                merged = _merge_attrs(d, dd, _join_geoms(d["geometry"], dd["geometry"], via))
                g.remove_node(mid)
                g.add_edge(far1, far2, **merged)
                n_merged += 1
            else:
                continue
            changed = True
            break
    return dict(n_contracted=n_contracted, n_merged=n_merged)


def contract_in_polygons(g: nx.MultiGraph, polygons: list, tol: float = 1.0) -> dict:
    """交差点の面(PLATEAU の道路ポリゴン)ごとに、面に入る交差点ノードを1つにまとめる。

    上下分離道路の交差点のように、中心線では複数ノードに分かれる交差点を1つの交差点にする
    (長さ 0 の Edge やおかしな車線の接続の根本対策、docs/sumo-design.md §13.4)。

    - 対象は次数 3 以上で、接続するリンクがすべて地表(立体でない)かつ階層順が同じノード
      (立体交差は面が重なるだけなので、階層の違うノードはまとめない)
    - まとめたノードは面の中の元ノードの重心に置く。面の中のリンク(両端が同じ面)は消す
    - 面を持つノード(1 つだけのものも含む)には面の外周を ``jshape`` として持たせる
      (SUMO の交差点の形に使う)
    """
    from shapely import STRtree
    from shapely.geometry import Point

    if not polygons:
        return dict(n_polygons=0, n_nodes_merged=0, n_internal_links=0, n_shaped=0)
    tree = STRtree(polygons)
    groups: dict[int, list] = {}
    for n, d in g.nodes(data=True):
        if g.degree(n) < 3:
            continue
        eds = list(g.edges(n, data=True))
        if any(is_grade_separated(dd["state"], dd.get("layer", "0")) for _, _, dd in eds):
            continue
        if len({str(dd.get("layer", "0")) for _, _, dd in eds}) != 1:
            continue
        idx = tree.query(Point(d["x"], d["y"]).buffer(tol), predicate="intersects")
        if len(idx):
            best = min(idx, key=lambda i: polygons[i].area)  # 重なるなら小さい方(交差点の面)
            groups.setdefault(int(best), []).append(n)

    n_merged = n_internal = n_shaped = 0
    for pi, members in groups.items():
        shape = [tuple(c) for c in polygons[pi].exterior.coords]
        rep = members[0]
        if len(members) >= 2:
            xs = [g.nodes[m]["x"] for m in members]
            ys = [g.nodes[m]["y"] for m in members]
            mset = set(members)
            # 面の中のリンク(両端がどちらも面の中のノード)は消える
            n_internal += sum(1 for u, v in g.subgraph(members).edges() if u in mset and v in mset)
            for m in members[1:]:
                for _, w, dd in list(g.edges(m, data=True)):
                    if w in mset:
                        continue
                    g.add_edge(rep, w, **dd)
                g.remove_node(m)
                n_merged += 1
            for u, v, k in list(g.edges(rep, keys=True)):
                if u == v:
                    g.remove_edge(u, v, k)
            g.nodes[rep]["x"], g.nodes[rep]["y"] = sum(xs) / len(xs), sum(ys) / len(ys)
        g.nodes[rep]["jshape"] = shape
        n_shaped += 1
    return dict(
        n_polygons=len(polygons),
        n_nodes_merged=n_merged,
        n_internal_links=n_internal,
        n_shaped=n_shaped,
    )


def check_grade_separated(g: nx.MultiGraph, tol: float = SNAP_TOL_M) -> dict:
    """立体リンクが端点以外で地表ノードと接していないか検証する。

    立体リンクの中間頂点に tol 以内のノードがあれば違反として列挙する。
    """
    node_xy = np.array([(d["x"], d["y"]) for _, d in g.nodes(data=True)])
    node_ids = list(g.nodes)
    tree = cKDTree(node_xy)
    violations = []
    portals = set()
    n_gs = 0
    for u, v, d in g.edges(data=True):
        if not is_grade_separated(d["state"], d.get("layer", "0")):
            continue
        n_gs += 1
        portals.update((u, v))
        interior = d["geometry"][1:-1]
        for x, y in interior:
            for idx in tree.query_ball_point((x, y), tol):
                violations.append(dict(node=int(node_ids[idx]), xy=[round(x, 1), round(y, 1)]))
    return dict(
        n_grade_separated=n_gs,
        portal_nodes=sorted(int(p) for p in portals),
        n_interior_contacts=len(violations),
        interior_contacts=violations[:20],
    )


def drop_minor_components(g: nx.MultiGraph) -> dict:
    """主成分以外(コードン角をかすめるだけの断片など)を取り除き、統計を返す。"""
    comps = sorted(nx.connected_components(g), key=len, reverse=True)
    dropped = []
    for c in comps[1:]:
        dropped.append(len(c))
        g.remove_nodes_from(c)
    return dict(n_dropped_components=len(dropped), dropped_sizes=dropped)


def acceptance(g: nx.MultiGraph, cordon, boundary_tol: float = BOUNDARY_TOL_M) -> dict:
    """受け入れ条件(architecture.md §4.1)の検証結果を返す。

    cordon: contains(x, y) と boundary_dist(x, y) を持つアダプタ
    (in_cordon 選定は intersects なのでリンクはコードン外へはみ出す。
    コードン外・縁 boundary_tol 以内のノードは境界ノードとして扱い、
    行き止まりはコードン内部の次数1ノードだけを数える)。
    """
    comps = sorted(nx.connected_components(g), key=len, reverse=True)
    main = comps[0] if comps else set()

    boundary_nodes, dead_ends = [], []
    for n, d in g.nodes(data=True):
        x, y = d["x"], d["y"]
        on_boundary = not cordon.contains(x, y) or cordon.boundary_dist(x, y) <= boundary_tol
        if on_boundary:
            boundary_nodes.append(n)
        if g.degree(n) == 1 and not on_boundary:
            dead_ends.append(dict(node=int(n), xy=[round(x, 1), round(y, 1)]))

    return dict(
        n_components=len(comps),
        main_component_nodes=len(main),
        component_sizes=[len(c) for c in comps[:10]],
        n_boundary_nodes=len(boundary_nodes),
        boundary_all_reachable=all(n in main for n in boundary_nodes),
        n_true_dead_ends=len(dead_ends),
        true_dead_ends=dead_ends[:30],
    )


class _CordonAdapter:
    """shapely 依存を acceptance() から切り離すための小さなアダプタ。"""

    def __init__(self, shapely_polygon):
        self._poly = shapely_polygon

    def contains(self, x: float, y: float) -> bool:
        from shapely.geometry import Point

        return self._poly.contains(Point(x, y))

    def boundary_dist(self, x: float, y: float) -> float:
        from shapely.geometry import Point

        return self._poly.exterior.distance(Point(x, y))


def boundary_adapter(shapely_polygon) -> _CordonAdapter:
    return _CordonAdapter(shapely_polygon)
