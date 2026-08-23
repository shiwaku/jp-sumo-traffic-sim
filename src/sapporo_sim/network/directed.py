"""無向リンク → 方向別 Edge 化(Phase 1-③、architecture.md §4.3)。

- 双方向道路は2本の Edge にし、互いを linked_edge で指す
- 一方通行(conflation の oneway="F"/"R")は1本のみ
- Edge の geometry は常に進行方向順(frm → to)
- oneway_source が assumed のものはレポートで数え上げる
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Edge:
    eid: int
    frm: int  # ノードid
    to: int
    length: float
    geometry: list  # 進行方向順の座標列
    linked_edge: int | None = None  # 対向 Edge の eid(一方通行は None)
    oneway_source: str = "assumed_twoway"
    attrs: dict = field(default_factory=dict)


def build_edges(links: list[dict]) -> list[Edge]:
    """conflation 済みリンクから方向別 Edge を作る。

    links の各要素: a, b, geometry(a→b順), oneway("F"/"R"/""), その他属性
    """
    edges: list[Edge] = []
    for ln in links:
        a, b = ln["a"], ln["b"]
        geom = list(ln["geometry"])
        attrs = {k: v for k, v in ln.items() if k not in ("a", "b", "geometry", "oneway")}
        length = ln.get("length") or _length(geom)
        attrs["length"] = length
        ow = ln.get("oneway", "")
        if ow == "F":
            edges.append(Edge(len(edges), a, b, length, geom, None, "jartic", dict(attrs)))
        elif ow == "R":
            edges.append(Edge(len(edges), b, a, length, geom[::-1], None, "jartic", dict(attrs)))
        else:
            e1 = Edge(len(edges), a, b, length, geom, None, "assumed_twoway", dict(attrs))
            e2 = Edge(len(edges) + 1, b, a, length, geom[::-1], None, "assumed_twoway", dict(attrs))
            e1.linked_edge, e2.linked_edge = e2.eid, e1.eid
            edges.extend((e1, e2))
    return edges


def _length(coords: list) -> float:
    import math

    return sum(math.dist(p, q) for p, q in zip(coords, coords[1:], strict=False))
