"""層[2]の方向別 Edge から SUMO ネットワークを作る(docs/sumo-design.md §2・§3)。

入力:  data/processed/*(edges / network_conflated / signal_plans)
出力:  data/sumo/{case}.net.xml(平常時)・{case}_winter.net.xml(冬季。車線幅 × 0.75)ほか中間 XML
       reports/50_sumo_net.json

検査(受け入れ条件 §2.8):
- 層[2]の Edge がすべて SUMO に1対1で入っている(右折車線の分割で増えた分は除く)
- U ターン接続が無い
- 全流入 Edge から流出 Edge へ到達可能(sumolib で再検査)
"""

import json
import sys
from collections import deque
from pathlib import Path

import sumolib

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim import config as C
from jp_sumo_traffic_sim.network.io import load_network_inputs
from jp_sumo_traffic_sim.sumo.build import build_network
from jp_sumo_traffic_sim.sumo.netgen import base_eid
from jp_sumo_traffic_sim.sumo.vtypes import WIDTH_SCALE

OUT = C.ROOT / "data" / "sumo"


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


def check_net(net_path: Path, edges: list[dict]) -> dict:
    net = sumolib.net.readNet(str(net_path))
    sumo_edges = [e for e in net.getEdges() if e.getFunction() != "internal"]
    ids = {e.getID() for e in sumo_edges}
    base = {base_eid(i) for i in ids}
    want = {e["eid"] for e in edges}

    n_uturn = 0
    succ: dict[str, set] = {}
    for e in sumo_edges:
        for to, conns in e.getOutgoing().items():
            succ.setdefault(e.getID(), set()).add(to.getID())
            n_uturn += sum(1 for c in conns if c.getDirection() == "t")

    # 流入 = 次数1ノードから出る Edge、流出 = 後続の無い Edge
    d1 = {str(n) for n in degree1_nodes(edges)}
    entries = [e.getID() for e in sumo_edges if e.getFromNode().getID() in d1]
    exits = {i for i in ids if not succ.get(i)}
    reach_all: set = set()
    n_entry_ok = 0
    for s in entries:
        seen, dq = {s}, deque([s])
        while dq:
            for nx in succ.get(dq.popleft(), ()):
                if nx not in seen:
                    seen.add(nx)
                    dq.append(nx)
        reach_all |= seen
        n_entry_ok += bool(seen & exits)
    return dict(
        n_sumo_edges=len(ids),
        n_base_edges_found=len(base & want),
        n_base_edges_expected=len(want),
        missing_eids=sorted(want - base)[:50],
        n_uturn_connections=n_uturn,
        n_entries=len(entries),
        n_entries_reaching_exit=n_entry_ok,
        n_exits=len(exits),
        n_edges_reachable_from_entries=len(reach_all),
        n_tls_in_net=len(net.getTrafficLights()),
    )


def main() -> None:
    nodes, edges, plans, stop_edges, _ = load_network_inputs()
    case = C.case_module()
    axis_fn = getattr(case, "signal_axis", None)
    offsets = case.signal_offsets(nodes) if hasattr(case, "signal_offsets") else None

    stats = build_network(
        nodes, edges, stop_edges, plans, OUT, C.CASE_NAME, axis_fn=axis_fn, offsets=offsets
    )
    # 冬季: 同じ位相・信号で車線幅だけ縮める(雪堤、docs/sumo-design.md §5.4)
    winter = build_network(
        nodes,
        edges,
        stop_edges,
        plans,
        OUT,
        f"{C.CASE_NAME}_winter",
        axis_fn=axis_fn,
        offsets=offsets,
        width_scale=WIDTH_SCALE["winter"],
    )
    stats["winter_net"] = str(Path(winter["net"]).relative_to(C.ROOT)).replace("\\", "/")
    stats["check"] = check_net(Path(stats["net"]), edges)
    stats["net"] = str(Path(stats["net"]).relative_to(C.ROOT)).replace("\\", "/")
    stats["case"] = C.CASE_NAME
    stats["signal_axis"] = "case" if axis_fn else "default(priority)"
    stats["signal_offsets"] = "case" if offsets else "zero"

    C.REPORTS.mkdir(parents=True, exist_ok=True)
    (C.REPORTS / "50_sumo_net.json").write_text(
        json.dumps(stats, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    ck = stats["check"]
    print(
        f"edges {ck['n_base_edges_found']}/{ck['n_base_edges_expected']} "
        f"(SUMO {ck['n_sumo_edges']}、右折車線分割 {stats['n_rt_lane_splits']})、"
        f"信号 {stats['n_tls']}(JARTIC {stats['n_jartic_timed']})、"
        f"Uターン {ck['n_uturn_connections']}、"
        f"流入→流出 {ck['n_entries_reaching_exit']}/{ck['n_entries']}、"
        f"警告 {stats['netconvert_warnings']}"
    )


if __name__ == "__main__":
    main()
