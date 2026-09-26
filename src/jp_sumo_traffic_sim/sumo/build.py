"""SUMO ネットワークの生成(docs/sumo-design.md §2・§3)。

1. 方向別 Edge → plain XML(netgen)
2. netconvert(1回目): 接続・信号の linkIndex を確定させる
3. 信号計画を2現示で作り直し(tls)、netconvert(2回目)で上書きする
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

import sumolib

from jp_sumo_traffic_sim.sumo import netgen, tls
from jp_sumo_traffic_sim.sumo.runner import run_tool, warnings_of

# netconvert の方針(§2.8)。既定値でも意図を明示するために書くものを含む
NETCONVERT_OPTIONS = [
    "--lefthand", "true",  # 左側通行。誤ると右折が対向と交錯しない
    "--offset.disable-normalization", "true",  # SUMO 座標 = 投影座標
    "--no-turnarounds", "true",  # U ターンは作らない(自前実装と同じ)
    "--junctions.join", "false",  # 近接ノードの縮約は立体交差を誤接続する
    "--geometry.remove", "false",  # 次数2ノードを消すと edge id が eid と対応しなくなる
    "--tls.guess", "false",  # 信号は層[2]で確定済み
    "--tls.left-green.time", "0",  # 右折矢印は作らない(JARTIC 現示の投入は Step 4)
]  # fmt: skip


def build_network(
    nodes: list[dict],
    edges: list[dict],
    stop_edges: set,
    plans: dict,
    out_dir: Path,
    name: str,
    axis_fn: Callable[[tuple, tuple], str] | None = None,
    offsets: dict[int, float] | None = None,
    width_scale: float = 1.0,
) -> dict:
    """{out_dir}/{name}.net.xml を作り、集計を返す。

    offsets: 信号ノード nid → オフセット [s](A 青の開始時刻)。
    width_scale: 車線幅の倍率(冬季の幅員縮小 §5.4)。
    """
    prefix = out_dir / name
    stats = netgen.write_plain_xml(nodes, edges, stop_edges, prefix, width_scale)

    net0 = Path(f"{prefix}.pre.net.xml")
    p1 = run_tool(
        "netconvert",
        [
            "-n",
            f"{prefix}.nod.xml",
            "-e",
            f"{prefix}.edg.xml",
            *NETCONVERT_OPTIONS,
            "-o",
            str(net0),
        ],
    )

    uid_of = {str(n["nid"]): n.get("signal_uid") or "" for n in nodes}
    net = sumolib.net.readNet(str(net0))
    timing = {
        t.getID(): tls.plan_timing(uid_of.get(t.getID(), ""), plans) for t in net.getTrafficLights()
    }
    tll, tls_stats = tls.build_programs(
        net,
        timing,
        offsets={str(k): v for k, v in (offsets or {}).items()},
        axis_fn=axis_fn,
    )
    tll_path = Path(f"{prefix}.tll.xml")
    tll_path.write_text(tll, encoding="utf-8")

    out = Path(f"{prefix}.net.xml")
    p2 = run_tool(
        "netconvert",
        ["-s", str(net0), "-i", str(tll_path), *NETCONVERT_OPTIONS, "-o", str(out)],
    )
    warns = warnings_of(p1) + warnings_of(p2)
    stats.update(tls_stats)
    stats.update(
        net=str(out),
        n_jartic_timed=sum(1 for t in timing if uid_of.get(t) and uid_of[t] in plans),
        netconvert_warnings=len(warns),
        netconvert_warning_samples=warns[:20],
    )
    return stats
