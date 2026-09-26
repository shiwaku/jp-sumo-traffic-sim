"""信号現示計画の生成(docs/sumo-design.md §3)。

netconvert が作った信号(接続と linkIndex)を読み、2現示の計画で上書きする:

    [A 青] → [A 黄] → [全赤] → [B 青] → [B 黄] → [全赤]

- 流入 Edge を2つの軸(A/B)に分ける。軸の判定はケースのフック(札幌はグリッドの
  東西/南北)、無ければ優先度が最も高い流入の向きを A とする
- 青中の右折(左側通行で対向と交錯する側)は従属青 `g`。対向直進・左折に譲る
- サイクル長は JARTIC 計画(時間帯別の中央値。自前実装と同じ)、無ければ既定値。
  青時間は等分
- オフセット = A 青の開始時刻
"""

from __future__ import annotations

import math
from collections.abc import Callable

DEFAULT_CYCLE_S = 120.0  # JARTIC 計画の無い信号(自前実装と同じ既定値)
DEFAULT_YELLOW_S = 3.0
DEFAULT_ALLRED_S = 2.0
AXIS_TOL_DEG = 45.0  # 基準軸からこの角度以内の流入を同じ軸とみなす
YIELD_DIRS = {"r", "R", "t"}  # 左側通行で対向と交錯する転回(従属青)


def plan_timing(uid: str, plans: dict) -> tuple[float, float, float]:
    """(サイクル長, 黄, 全赤)。JARTIC 計画が無ければ既定値。"""
    plan = plans.get(uid) if uid else None
    if not plan:
        return DEFAULT_CYCLE_S, DEFAULT_YELLOW_S, DEFAULT_ALLRED_S
    cycles = sorted(h["cycle_s"] for h in plan["by_hour"].values())
    return (
        float(cycles[len(cycles) // 2]),
        float(plan.get("yellow_s") or DEFAULT_YELLOW_S),
        float(plan.get("all_red_s") or DEFAULT_ALLRED_S),
    )


def _heading(edge) -> float:
    """Edge 終端の進行方位 [deg](x 軸から反時計回り)。"""
    (x1, y1), (x2, y2) = edge.getShape()[-2:]
    return math.degrees(math.atan2(y2 - y1, x2 - x1))


def _undirected_diff(a: float, b: float) -> float:
    d = abs(a - b) % 180.0
    return min(d, 180.0 - d)


def default_axis(in_edges: list) -> dict:
    """優先度が最も高い流入の向きを A 軸として、流入 Edge id → 'A'/'B'。"""
    ref = max(in_edges, key=lambda e: (e.getPriority(), e.getLaneNumber()))
    h0 = _heading(ref)
    return {
        e.getID(): "A" if _undirected_diff(_heading(e), h0) <= AXIS_TOL_DEG else "B"
        for e in in_edges
    }


MERGE_PRIORITY = {"s": 0, "l": 1, "L": 1, "r": 2, "R": 2, "t": 3}  # 合流で G を残す順


def _merge_to_minor(state: list[str], conns: list) -> int:
    """同じ流出車線へ向かう G が複数あれば、1本(直進優先)を残して g(従属青)にする。

    上下分離道路の合流部などで、別々の流入から同じ車線へ G が2本入ると譲り合いが
    無くなる(SUMO の "Unsafe green phase" 警告。C2 の試走で 217 交差点)。
    """
    by_lane: dict[str, list] = {}
    for i, _e, d, to_lane in conns:
        if state[i] == "G":
            by_lane.setdefault(to_lane, []).append((MERGE_PRIORITY.get(d, 9), i))
    n = 0
    for cands in by_lane.values():
        for _, i in sorted(cands)[1:]:
            state[i] = "g"
            n += 1
    return n


def build_programs(
    net,
    timing: dict[str, tuple[float, float, float]],
    offsets: dict[str, float] | None = None,
    axis_fn: Callable[[tuple, tuple], str] | None = None,
) -> tuple[str, dict]:
    """tll 追加ファイル(XML 文字列)と集計を返す。

    net: sumolib.net.Net(withPrograms 不要)。timing: TLS id → (cycle, yellow, allred)。
    axis_fn: 流入 Edge 終端の2点 → 'A'/'B'(ケースのフック)。
    """
    offsets = offsets or {}
    links: dict[str, list] = {}  # tls id → [(linkIndex, from edge, direction, to lane id)]
    for e in net.getEdges():
        for conns in e.getOutgoing().values():
            for c in conns:
                tl = c.getTLSID()
                if tl:
                    links.setdefault(tl, []).append(
                        (c.getTLLinkIndex(), e, c.getDirection(), c.getToLane().getID())
                    )

    out = ["<additional>"]
    stats = dict(n_tls=0, n_single_axis=0, n_merge_yield=0)
    for tl_id in sorted(links, key=str):
        conns = links[tl_id]
        in_edges = list({e.getID(): e for _, e, _, _ in conns}.values())
        if axis_fn is not None:
            axis = {e.getID(): axis_fn(*e.getShape()[-2:]) for e in in_edges}
        else:
            axis = default_axis(in_edges)
        if len(set(axis.values())) == 1:
            stats["n_single_axis"] += 1  # B 側の流入なし(B 青は全赤 = 歩行者現示相当)
        cycle, yellow, allred = timing.get(
            tl_id, (DEFAULT_CYCLE_S, DEFAULT_YELLOW_S, DEFAULT_ALLRED_S)
        )
        green = max(1.0, (cycle - 2 * (yellow + allred)) / 2)

        n = max(i for i, _, _, _ in conns) + 1
        phases = []
        for ax in ("A", "B"):
            g, y = ["r"] * n, ["r"] * n
            for i, e, d, _ in conns:
                if axis[e.getID()] == ax:
                    g[i] = "g" if d in YIELD_DIRS else "G"
                    y[i] = "y"
            stats["n_merge_yield"] += _merge_to_minor(g, conns)
            phases += [(green, "".join(g)), (yellow, "".join(y)), (allred, "r" * n)]
        off = round(offsets.get(tl_id, 0.0) % cycle, 2)
        out.append(f'  <tlLogic id="{tl_id}" type="static" programID="0" offset="{off}">')
        for dur, st in phases:
            out.append(f'    <phase duration="{round(dur, 2)}" state="{st}"/>')
        out.append("  </tlLogic>")
        stats["n_tls"] += 1
    out.append("</additional>")
    return "\n".join(out) + "\n", stats
