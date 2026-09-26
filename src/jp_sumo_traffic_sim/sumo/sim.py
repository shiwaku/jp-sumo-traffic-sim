"""SUMO の実行と出力の読み取り(docs/sumo-design.md §6)。

- 断面交通量・区間速度: edgeData(meandata)。記録区間だけを1区間として集計する
- 全体統計: statistic-output(挿入・到着・テレポート)
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from jp_sumo_traffic_sim.sumo.runner import SIM_OPTIONS, run_tool
from jp_sumo_traffic_sim.sumo.vtypes import vtype_xml


def run(
    net: Path,
    routes: Path,
    out_prefix: Path,
    record_begin: float,
    end: float,
    scenario: str = "normal",
    seed: int = 42,
) -> dict:
    """SUMO を1回実行し、出力ファイルのパスを返す。"""
    vtypes = Path(f"{out_prefix}.vtypes.add.xml")
    vtypes.write_text(f"<additional>\n  {vtype_xml(scenario)}\n</additional>\n", encoding="utf-8")
    edgedata = Path(f"{out_prefix}.edgedata.xml")
    meandata = Path(f"{out_prefix}.meandata.add.xml")
    meandata.write_text(
        f'<additional>\n  <edgeData id="rec" file="{edgedata.name}" begin="{record_begin}" '
        f'end="{end}" period="{end - record_begin}"/>\n</additional>\n',
        encoding="utf-8",
    )
    stats = Path(f"{out_prefix}.stats.xml")
    run_tool(
        "sumo",
        [
            "-n", str(net),
            # vType は経路より先に読む必要がある
            "-a", f"{vtypes},{meandata}",
            "-r", str(routes),
            *SIM_OPTIONS,
            "--begin", "0", "--end", str(end),
            "--seed", str(seed),
            "--statistic-output", str(stats),
            "--no-warnings", "true",
        ],
    )  # fmt: skip
    return dict(edgedata=edgedata, stats=stats)


def read_edgedata(path: Path) -> dict[str, dict]:
    """edge id → {entered, left, speed, sampledSeconds}(記録区間の1区間分)。"""
    out: dict[str, dict] = {}
    for iv in ET.parse(path).getroot().iter("interval"):
        for e in iv.iter("edge"):
            out[e.get("id")] = dict(
                entered=float(e.get("entered", 0)),
                left=float(e.get("left", 0)),
                speed=float(e.get("speed", 0) or 0),
                sampledSeconds=float(e.get("sampledSeconds", 0)),
            )
    return out


def read_stats(path: Path) -> dict:
    """statistic-output の主要項目。"""
    root = ET.parse(path).getroot()
    v = root.find("vehicles")
    t = root.find("teleports")
    ts = root.find("vehicleTripStatistics")
    return dict(
        loaded=int(v.get("loaded")),
        inserted=int(v.get("inserted")),
        running=int(v.get("running")),
        waiting=int(v.get("waiting")),
        teleports=int(t.get("total")) if t is not None else 0,
        teleports_jam=int(t.get("jam", 0)) if t is not None else 0,
        trip_speed_kmh=round(float(ts.get("speed")) * 3.6, 1) if ts is not None else None,
    )
