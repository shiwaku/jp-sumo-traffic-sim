"""SUMO テスト用の小さな検証ネットワーク(docs/sumo-design.md §10)。

ネットワークは本番と同じ経路(sumo.build.build_network)で作る。
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import traci

from jp_sumo_traffic_sim.sumo.build import build_network
from jp_sumo_traffic_sim.sumo.runner import SIM_OPTIONS, tool
from jp_sumo_traffic_sim.sumo.vtypes import vtype_xml


def node(nid, x, y, signal=False):
    return dict(nid=nid, x=float(x), y=float(y), has_signal=signal, signal_uid="")


def edge(eid, frm, to, nodes, n_lanes=1, speed_kmh=50, category="3", linked=-1, **kw):
    a, b = nodes[frm], nodes[to]
    geom = [(a["x"], a["y"]), (b["x"], b["y"])]
    length = ((b["x"] - a["x"]) ** 2 + (b["y"] - a["y"]) ** 2) ** 0.5
    return dict(
        eid=eid,
        frm=frm,
        to=to,
        geometry=geom,
        length=length,
        speed_kmh=speed_kmh,
        category=category,
        linked_edge=linked,
        n_lanes=n_lanes,
        carriageway_m=3.0 * n_lanes,
        n_sublanes=int(3.0 * n_lanes // 1.75),
        right_turn_lane=kw.get("rt", 0),
        census_id="",
        ksj_ids="",
    )


def two_way(eid, a, b, nodes, **kw):
    """a→b を eid、b→a を eid+1 とする双方向道路。"""
    return [
        edge(eid, a, b, nodes, linked=eid + 1, **kw),
        edge(eid + 1, b, a, nodes, linked=eid, **kw),
    ]


def build(tmp: Path, nodes: list, edges: list, **kw) -> dict:
    return build_network(nodes, edges, set(), {}, tmp, "t", **kw)


def write_routes(tmp: Path, body: str, scenario: str = "normal", speed_dev: bool = False) -> Path:
    p = tmp / "t.rou.xml"
    p.write_text(
        f"<routes>\n  {vtype_xml(scenario, speed_dev=speed_dev)}\n{body}\n</routes>\n",
        encoding="utf-8",
    )
    return p


def start(net: str, routes: Path, extra: list[str] | None = None, label: str = "t") -> None:
    traci.start(
        [
            tool("sumo"),
            "-n",
            net,
            "-r",
            str(routes),
            *SIM_OPTIONS,
            "--seed",
            "42",
            "--collision.action",
            "warn",
            "--no-warnings",
            "true",
            *(extra or []),
        ],
        label=label,
    )


def close() -> None:
    """現在の TraCI 接続を閉じる(SUMO が先に落ちていても接続を残さない)。"""
    try:
        traci.close()
    except traci.exceptions.FatalTraCIError:
        pass
