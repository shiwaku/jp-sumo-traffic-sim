"""初期 OD(重力モデル)の部品(docs/sumo-design.md §13.5 ②、D4)。

T_ij = s_b · k(t) · P_i · A_j · exp(-γ c_ij) / Σ_k A_k exp(-γ c_ik)   (発生制約型)

- OD は 4 つのブロックに分ける: 内→内・内→外・外→内・外→外(通過)。ブロックごとに
  発生・集中の指標が違う。外部ゾーンの発生・集中はコードン(出入口)の観測交通量、
  残る水準(内→内の台数と通過の割合)は観測交通量への当てはめで決める(fit_cordon)
- c_ij: ゾーン間の自由流の経路の費用 [分](SUMO の Edge の接続のグラフで最短経路。右左折の
  可否は接続に入っている)。費用は duarouter と同じ考え方(routing_cost): 所要時間を優先度で
  割り増し、信号交差点の待ちを足す
- k(t): 時間帯の形。観測交通量の合計の時間変動から取る
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import lsq_linear
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra


def routing_cost(
    length_m: float,
    speed_ms: float,
    priority: int,
    prio_range: tuple[int, int],
    to_signal: bool,
    priority_factor: float,
    tls_penalty_s: float,
) -> float:
    """Edge の経路の費用 [s]。duarouter の --weights.priority-factor・--weights.tls-penalty に倣う:
    所要時間 × (1 + 係数 × (最高の優先度 - 優先度) / (最高 - 最低)) + 信号の待ち。"""
    lo, hi = prio_range
    rel = (hi - priority) / (hi - lo) if hi > lo else 0.0
    t = length_m / max(speed_ms, 0.1) * (1.0 + priority_factor * rel)
    return t + (tls_penalty_s if to_signal else 0.0)


def edge_graph(edges: list[dict]) -> tuple[csr_matrix, dict[str, int]]:
    """Edge を点、接続を辺とするグラフ。辺 e → f の重み = f の費用 [s]。

    edges: {id, cost_s, succ} の一覧。
    """
    idx = {e["id"]: k for k, e in enumerate(edges)}
    rows, cols, w = [], [], []
    for e in edges:
        for f in e["succ"]:
            if f in idx:
                rows.append(idx[e["id"]])
                cols.append(idx[f])
                w.append(max(edges[idx[f]]["cost_s"], 0.01))
    n = len(edges)
    return csr_matrix((w, (rows, cols)), shape=(n, n)), idx


def zone_costs(
    graph: csr_matrix, origins: list[int | None], dests: list[int | None], chunk: int = 256
) -> np.ndarray:
    """ゾーン間の経路の費用 [分]。origins・dests はゾーンの代表の Edge の番号(無ければ None)。

    到達できない組・代表の無いゾーンは inf。
    """
    nz_o = [k for k, o in enumerate(origins) if o is not None]
    d_idx = np.array([d if d is not None else 0 for d in dests])
    d_ok = np.array([d is not None for d in dests])
    out = np.full((len(origins), len(dests)), np.inf)
    for a in range(0, len(nz_o), chunk):
        ks = nz_o[a : a + chunk]
        dist = dijkstra(graph, directed=True, indices=[origins[k] for k in ks])
        sub = dist[:, d_idx]
        sub[:, ~d_ok] = np.inf
        out[ks] = sub / 60.0
    return out


def gravity(
    prod: np.ndarray,
    attr: np.ndarray,
    cost: np.ndarray,
    gamma: float,
    min_cost: float = 0.0,
) -> np.ndarray:
    """発生制約型の重力モデル。行の合計 = prod(集中先が無い行は 0)。

    cost < min_cost の組(徒歩の距離)と同じゾーンどうしは 0。
    """
    f = np.exp(-gamma * np.where(np.isfinite(cost), cost, np.inf))
    f[cost < min_cost] = 0.0
    if f.shape[0] == f.shape[1]:
        np.fill_diagonal(f, 0.0)
    w = f * attr[None, :]
    s = w.sum(axis=1, keepdims=True)
    with np.errstate(invalid="ignore", divide="ignore"):
        t = np.where(s > 0, w / s, 0.0) * prod[:, None]
    return t


def fit_cordon(
    loads: np.ndarray, obs: np.ndarray, in_total: float, out_total: float
) -> tuple[float, float]:
    """コードンの交通量で縛った水準の当てはめ。

    ブロック(内→内・内→外・外→内・外→外)の台数を
      内→内 = a、外→外 = e · 入口の合計、外→内 = (1 - e) · 入口の合計、内→外 = 出口の合計 - 外→外
    とし、loads [地点 × ブロック](1 トリップあたりの配分交通量)から作る交通量を obs に
    最小二乗で合わせて (a, e) を返す。0 ≤ e ≤ min(1, 出口 / 入口)。
    """
    u_ii, u_ie, u_ei, u_ee = loads.T
    base = in_total * u_ei + out_total * u_ie
    design = np.c_[u_ii, in_total * (u_ee - u_ei - u_ie)]
    e_max = min(1.0, out_total / in_total) if in_total > 0 else 0.0
    res = lsq_linear(design, obs - base, bounds=([0.0, 0.0], [np.inf, e_max]))
    return float(res.x[0]), float(res.x[1])


def cordon_block_totals(n_ii: float, ee_share: float, in_total: float, out_total: float):
    """fit_cordon の結果 → ブロック(内→内・内→外・外→内・外→外)の台数。"""
    ee = ee_share * in_total
    return np.array([n_ii, max(out_total - ee, 0.0), in_total - ee, ee])


def hourly_profile(counts: list[dict], hours: list[int], ref_hour: int) -> dict[int, float]:
    """観測地点の交通量の合計の時間変動(ref_hour = 1)。全時間で値のある地点だけを使う。"""
    use = [c for c in counts if all(c[h] is not None for h in [*hours, ref_hour])]
    ref = sum(c[ref_hour] for c in use)
    return {h: sum(c[h] for c in use) / ref for h in hours}


def taz_relation_xml(intervals: list[tuple[float, float, dict[tuple[str, str], float]]]) -> str:
    """tazRelation(OD 表)の XML。intervals: (begin, end, {(発, 着): 台数})。"""
    lines = ['<?xml version="1.0" encoding="UTF-8"?>', "<data>"]
    for b, e, od in intervals:
        lines.append(f'    <interval id="{int(b)}" begin="{b:g}" end="{e:g}">')
        for (o, d), v in od.items():
            if v > 0:
                lines.append(f'        <tazRelation from="{o}" to="{d}" count="{v:.3f}"/>')
        lines.append("    </interval>")
    lines.append("</data>")
    return "\n".join(lines) + "\n"
