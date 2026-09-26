"""コードン流入 + 転回率 → SUMO の経路(docs/sumo-design.md §4)。

方針は design.md §3.2 のまま(OD は使わない)。自前実装(sim/netsim.py)と同じ規則:

- 流入 Edge = 無向次数1ノードから出る Edge。レートはセンサス時間帯別交通量
  (方向平均)、裏付けが無ければ分類の既定値(sim/demand.py)
- 転回クラス(直進/左折/右折 = TURN_P)を存在する転回先で再正規化し、クラス内の
  複数候補(上下分離・並走車道)は角度適合の重み 1/(1+ずれ)² で配る
- 通過交差点数が MAX_HOPS を超えたら最寄りの流出点へ向ける

経路は jtrrouter(流入 flows + 転回率 turns)で作り、巡回上限は後処理で掛ける
(jtrrouter の打ち切りは車両が区域の途中で消えるため使わない)。
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path

from jp_sumo_traffic_sim.sim.demand import DEFAULT_RATE, hourly_rate
from jp_sumo_traffic_sim.sumo.netgen import base_eid
from jp_sumo_traffic_sim.sumo.runner import run_tool

TURN_P = (0.70, 0.15, 0.15)  # 直進 / 左折 / 右折(自前実装と同じ初期値。Phase 3-② の調整対象)
STRAIGHT_DEG = 45.0  # 方位角差がこの範囲なら直進
UTURN_DEG = 135.0  # これを超える転回は U ターンとして除外
MAX_HOPS = 30  # 巡回上限(通過交差点数)。コードンの対角は交差点 20〜25 個分
ROUTER_SAFETY_FACTOR = 0.5  # jtrrouter の経路長の安全上限(ネットワーク Edge 数に対する比)


def _bearing(p, q) -> float:
    return math.degrees(math.atan2(q[1] - p[1], q[0] - p[0]))


def _angdiff(a: float, b: float) -> float:
    """a - b を (-180, 180] に。正 = 反時計回り(左折)。"""
    return (a - b + 180.0) % 360.0 - 180.0


def turn_probabilities(net) -> dict[str, dict[str, float]]:
    """非内部 Edge → {後続 Edge: 確率}。後続の無い Edge(流出)は含めない。"""
    out: dict[str, dict[str, float]] = {}
    for e in net.getEdges():
        if e.getFunction() == "internal":
            continue
        succ = [s for s in e.getOutgoing() if s.getFunction() != "internal"]
        if not succ:
            continue
        rs = e.getRawShape()
        h_in = _bearing(rs[-2], rs[-1])
        cand: dict[int, list] = {0: [], 1: [], 2: []}
        for s in succ:
            ss = s.getRawShape()
            d = _angdiff(_bearing(ss[0], ss[1]), h_in)
            if abs(d) <= STRAIGHT_DEG:
                turn, score = 0, abs(d)
            elif 0 < d < UTURN_DEG:
                turn, score = 1, abs(d - 90.0)
            elif -UTURN_DEG < d < 0:
                turn, score = 2, abs(-d - 90.0)
            else:
                continue
            cand[turn].append((1.0 / (1.0 + score) ** 2, s.getID()))
        classes = [t for t in range(3) if cand[t]]
        if not classes:
            # 鋭角の折返ししか無い(右折車線の分割点などで起きうる)→ 角度で最も近いものへ
            out[e.getID()] = {succ[0].getID(): 1.0}
            continue
        z = sum(TURN_P[t] for t in classes)
        probs: dict[str, float] = {}
        for t in classes:
            wz = sum(w for w, _ in cand[t])
            for w, sid in cand[t]:
                probs[sid] = probs.get(sid, 0.0) + TURN_P[t] / z * w / wz
        out[e.getID()] = probs
    return out


def degree1_nodes(edges: list[dict]) -> set:
    """無向次数1のノード(コードン境界の流出入口。自前実装と同じ規則)。"""
    deg: dict[int, int] = {}
    seen: set = set()
    for e in edges:
        le = e.get("linked_edge", -1)
        key = e["eid"] if le in (-1, None) else min(e["eid"], le)
        if key in seen:
            continue
        seen.add(key)
        for nid in (e["frm"], e["to"]):
            deg[nid] = deg.get(nid, 0) + 1
    return {nid for nid, d in deg.items() if d == 1}


def entry_rates(
    edges: list[dict], hourly: dict, hour: int, scale: float = 1.0
) -> tuple[dict[int, float], dict]:
    """流入 Edge eid → 流入レート [台/時] と集計。"""
    d1 = degree1_nodes(edges)
    rates: dict[int, float] = {}
    n_census = 0
    vol_default = 0.0
    for e in edges:
        if e["frm"] not in d1:
            continue
        rate = hourly_rate(hourly.get(e["census_id"]), hour) if e.get("census_id") else None
        if rate is None:
            klass = "arterial" if str(e.get("category", "")) in ("1", "2") else "minor"
            rate = DEFAULT_RATE[klass]
            vol_default += rate
        else:
            n_census += 1
        rates[e["eid"]] = rate * scale
    stats = dict(
        n_entries=len(rates),
        n_census_entries=n_census,
        n_default_entries=len(rates) - n_census,
        total_entry_vph=round(sum(rates.values()), 1),
        default_entry_vph=round(vol_default * scale, 1),
    )
    return rates, stats


def write_flows(rates: dict[int, float], begin: float, end: float, path: Path) -> None:
    """流入 flow(台数固定・出発時刻はランダム = 条件付きポアソン)。"""
    dur_h = (end - begin) / 3600.0
    lines = ["<routes>"]
    for eid, vph in sorted(rates.items()):
        n = round(vph * dur_h)
        if n <= 0:
            continue
        lines.append(
            f'  <flow id="f{eid}" from="{eid}" begin="{begin}" end="{end}" number="{n}" '
            'type="car" departLane="best" departSpeed="max"/>'
        )
    lines.append("</routes>")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_turns(probs: dict[str, dict[str, float]], begin: float, end: float, path: Path) -> None:
    lines = ["<edgeRelations>", f'  <interval begin="{begin}" end="{end}">']
    for frm in sorted(probs):
        for to, p in sorted(probs[frm].items()):
            lines.append(f'    <edgeRelation from="{frm}" to="{to}" probability="{p:.6f}"/>')
    lines += ["  </interval>", "</edgeRelations>"]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def exit_paths(net) -> dict[str, list[str]]:
    """各 Edge → 最寄りの流出 Edge までの後続列(逆 BFS。自身は含まない)。"""
    succ: dict[str, list[str]] = {}
    preds: dict[str, list[str]] = {}
    for e in net.getEdges():
        if e.getFunction() == "internal":
            continue
        ss = [s.getID() for s in e.getOutgoing() if s.getFunction() != "internal"]
        succ[e.getID()] = ss
        for s in ss:
            preds.setdefault(s, []).append(e.getID())
    nxt: dict[str, str | None] = {}
    dq = deque()
    for eid, ss in succ.items():
        if not ss:
            nxt[eid] = None
            dq.append(eid)
    while dq:
        cur = dq.popleft()
        for p in preds.get(cur, []):
            if p not in nxt:
                nxt[p] = cur
                dq.append(p)
    paths: dict[str, list[str]] = {}
    for eid in succ:
        path, cur = [], nxt.get(eid)
        while cur is not None and len(path) < len(succ):
            path.append(cur)
            cur = nxt.get(cur)
        paths[eid] = path
    return paths


def cap_hops(
    route: list[str], paths: dict[str, list[str]], max_hops: int
) -> tuple[list[str], bool]:
    """通過交差点数(層[2]の Edge 単位)が max_hops を超えたら最寄りの流出へつなぎ替える。"""
    hops, last = 0, None
    for i, sid in enumerate(route):
        b = base_eid(sid)
        if b != last:
            hops += 1
            last = b
        if hops > max_hops:
            return route[: i + 1] + paths.get(sid, []), True
    return route, False


def build_routes(
    net,
    net_path: Path,
    rates: dict[int, float],
    begin: float,
    end: float,
    out_prefix: Path,
    seed: int = 42,
    probs: dict | None = None,
) -> dict:
    """flows/turns を書き、jtrrouter で経路を作って巡回上限を掛ける。集計を返す。"""
    probs = probs if probs is not None else turn_probabilities(net)
    flows = Path(f"{out_prefix}.flows.xml")
    turns = Path(f"{out_prefix}.turns.xml")
    raw = Path(f"{out_prefix}.jtr.rou.xml")
    out = Path(f"{out_prefix}.rou.xml")
    write_flows(rates, begin, end, flows)
    n_expected = sum(round(v * (end - begin) / 3600.0) for v in rates.values())
    write_turns(probs, begin, end, turns)
    sinks = sorted(
        e.getID()
        for e in net.getEdges()
        if e.getFunction() != "internal"
        and not any(s.getFunction() != "internal" for s in e.getOutgoing())
    )
    n_net_edges = sum(1 for e in net.getEdges() if e.getFunction() != "internal")
    run_tool(
        "jtrrouter",
        [
            "-n", str(net_path), "-r", str(flows), "-t", str(turns),
            "--sink-edges", ",".join(sinks),
            "--allow-loops", "true",
            "--max-edges-factor", str(ROUTER_SAFETY_FACTOR),
            "--randomize-flows", "true",
            "--seed", str(seed),
            "--ignore-errors", "true",
            "--no-step-log", "true",
            "--no-warnings", "true",
            "-o", str(raw),
        ],
    )  # fmt: skip

    paths = exit_paths(net)
    tree = ET.parse(raw)
    root = tree.getroot()
    n_veh = n_capped = n_unfinished = 0
    for veh in root.iter("vehicle"):
        r = veh.find("route")
        if r is None:
            continue
        edges = r.get("edges", "").split()
        edges, capped = cap_hops(edges, paths, MAX_HOPS)
        n_capped += capped
        n_unfinished += bool(edges) and edges[-1] not in sinks
        r.set("edges", " ".join(edges))
        n_veh += 1
    tree.write(out, encoding="utf-8", xml_declaration=True)
    if n_veh != n_expected:
        # jtrrouter は存在しない流入 Edge の flow を黙って捨てる(--ignore-errors)
        raise RuntimeError(f"経路の台数 {n_veh} が流入の合計 {n_expected} と合わない")
    return dict(
        n_vehicles=n_veh,
        n_hop_capped=n_capped,
        n_route_not_ending_at_exit=n_unfinished,
        n_sinks=len(sinks),
        n_net_edges=n_net_edges,
        routes=str(out),
    )
