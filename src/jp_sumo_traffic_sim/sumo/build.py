"""SUMO ネットワークの生成(docs/sumo-design.md §2・§3・§13.3)。

1. 方向別 Edge → plain XML(netgen)
2. netconvert(1回目): 接続・信号の linkIndex を確定させる
3. 信号計画を2現示で作り直し(tls)、netconvert(2回目)で上書きする

都心区域のミクロ用ネットワークは、市全域のネットワークから都心区域に掛かる Edge を
切り出して作る(cut_subnet)。edge id が共通なので、メソの経路をそのまま渡せる。
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

import sumolib
from shapely.geometry import LineString

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
    "--tls.left-green.time", "0",  # 右折矢印は作らない(JARTIC 現示の投入は C5)
]  # fmt: skip


def retime_signals(
    net_in: Path,
    net_out: Path,
    nodes: list[dict],
    plans: dict,
    axis_fn: Callable[[tuple, tuple], str] | None = None,
    offsets: dict[int, float] | None = None,
) -> dict:
    """net_in の信号を2現示の計画で作り直して net_out に書く。集計を返す。"""
    uid_of = {str(n["nid"]): n.get("signal_uid") or "" for n in nodes}
    net = sumolib.net.readNet(str(net_in))
    timing = {
        t.getID(): tls.plan_timing(uid_of.get(t.getID(), ""), plans) for t in net.getTrafficLights()
    }
    tll, stats = tls.build_programs(
        net,
        timing,
        offsets={str(k): v for k, v in (offsets or {}).items()},
        axis_fn=axis_fn,
    )
    tll_path = net_out.with_name(net_out.name.replace(".net.xml", ".tll.xml"))
    tll_path.write_text(tll, encoding="utf-8")
    proc = run_tool(
        "netconvert",
        ["-s", str(net_in), "-i", str(tll_path), *NETCONVERT_OPTIONS, "-o", str(net_out)],
    )
    stats["n_jartic_timed"] = sum(1 for t in timing if uid_of.get(t) and uid_of[t] in plans)
    stats["warnings"] = warnings_of(proc)
    return stats


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
    out = Path(f"{prefix}.net.xml")
    ts = retime_signals(net0, out, nodes, plans, axis_fn, offsets)
    warns = warnings_of(p1) + ts.pop("warnings")
    stats.update(ts)
    stats.update(
        net=str(out),
        netconvert_warnings=len(warns),
        netconvert_warning_samples=warns[:20],
    )
    return stats


def cut_subnet(
    net_in: Path,
    net_out: Path,
    polygon,
    nodes: list[dict],
    plans: dict,
    axis_fn: Callable[[tuple, tuple], str] | None = None,
    offsets: dict[int, float] | None = None,
    keep_ids: list[str] | None = None,
) -> dict:
    """net_in から polygon に掛かる Edge だけを残した部分網を net_out に作る。

    edge id は net_in と同じ。境界の信号は接続が減るので、同じ規則で計画を作り直す。
    keep_ids を渡すとその Edge を残す(冬季を平常時と同じ Edge 集合にするため。
    冬季は車線幅が違い、形状が少しずれて polygon との交差判定が変わる)。
    """
    if keep_ids is not None:
        keep = list(keep_ids)
    else:
        net = sumolib.net.readNet(str(net_in))
        keep = [
            e.getID()
            for e in net.getEdges()
            if e.getFunction() != "internal" and LineString(e.getShape()).intersects(polygon)
        ]
    ids = net_out.with_name(net_out.name.replace(".net.xml", ".edges.txt"))
    ids.write_text("\n".join(keep) + "\n", encoding="utf-8")
    tmp = net_out.with_name(net_out.name.replace(".net.xml", ".pre.net.xml"))
    p1 = run_tool(
        "netconvert",
        [
            "-s",
            str(net_in),
            "--keep-edges.input-file",
            str(ids),
            *NETCONVERT_OPTIONS,
            "-o",
            str(tmp),
        ],
    )
    ts = retime_signals(tmp, net_out, nodes, plans, axis_fn, offsets)
    warns = warnings_of(p1) + ts.pop("warnings")
    return dict(
        net=str(net_out),
        keep_ids=keep,
        n_edges=len(keep),
        n_tls=ts["n_tls"],
        n_jartic_timed=ts["n_jartic_timed"],
        netconvert_warnings=len(warns),
    )
