"""市全域のメソの試走(docs/sumo-design.md §13.3・§13.6 C2)。

需要推定(C3)の前に、市全域のネットワークでメソが最後まで回ることと、1 試行の
所要時間を確かめる。需要はランダム(randomTrips.py)で、実測とは無関係。

- 需要: 1 時間に TRIPS_PER_HOUR 本。市外との出入口(ネットワークの端)を起終点に
  選びやすくする(--fringe-factor)。経路は duarouter の最短時間経路
- メソ: 信号・交差点の制御を有効にする(--meso-junction-control)

入力:  data/sumo/{case}.net.xml(make sumo-net)
出力:  reports/54_meso_smoke.json
"""

import json
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim import config as C
from jp_sumo_traffic_sim.sumo import sim as sumo_sim
from jp_sumo_traffic_sim.sumo.runner import SUMO_HOME, env, run_tool

SUMO_DIR = C.ROOT / "data" / "sumo"
DEMAND_S = 3600.0
TRIPS_PER_HOUR = 7200
FRINGE_FACTOR = 10.0  # 出入口を起終点に選ぶ重み
MIN_DISTANCE_M = 1000.0  # 短すぎるトリップを避ける
END_S = DEMAND_S + 3 * 3600.0  # 需要終了後も捌け切るまで回す
MESO_OPTIONS = ["--mesosim", "true", "--meso-junction-control", "true"]


def main() -> None:
    net = SUMO_DIR / f"{C.CASE_NAME}.net.xml"
    if not net.exists():
        raise SystemExit(f"{net} が無い。先に make sumo-net を実行する")
    prefix = SUMO_DIR / f"{C.CASE_NAME}_smoke"
    trips, routes = Path(f"{prefix}.trips.xml"), Path(f"{prefix}.rou.xml")

    t0 = time.time()
    subprocess.run(
        [
            sys.executable, str(SUMO_HOME / "tools" / "randomTrips.py"),
            "-n", str(net), "-o", str(trips), "-r", str(routes),
            "-b", "0", "-e", str(DEMAND_S),
            "-p", str(3600.0 / TRIPS_PER_HOUR),
            "--fringe-factor", str(FRINGE_FACTOR),
            "--min-distance", str(MIN_DISTANCE_M),
            "--seed", "42",
        ],
        check=True, capture_output=True, text=True, env=env(),
    )  # fmt: skip
    t_route = time.time() - t0

    stats_path = Path(f"{prefix}.stats.xml")
    edgedata = Path(f"{prefix}.edgedata.xml")
    t1 = time.time()
    run_tool(
        "sumo",
        [
            "-n", str(net), "-r", str(routes), *MESO_OPTIONS,
            "--begin", "0", "--end", str(END_S), "--seed", "42",
            "--statistic-output", str(stats_path),
            "--edgedata-output", str(edgedata),
            "--no-step-log", "true", "--no-warnings", "true",
        ],
    )  # fmt: skip
    t_sim = time.time() - t1
    st = sumo_sim.read_stats(stats_path)
    # 待ち時間の大きい Edge(ボトルネックの候補)
    rows = []
    for e in ET.parse(edgedata).getroot().iter("edge"):
        rows.append(
            dict(
                edge=e.get("id"),
                waiting_s=round(float(e.get("waitingTime", 0))),
                speed_ms=round(float(e.get("speed", 0) or 0), 2),
                entered=int(float(e.get("entered", 0))),
            )
        )
    rows.sort(key=lambda r: -r["waiting_s"])

    rep = dict(
        case=C.CASE_NAME,
        net=str(net.relative_to(C.ROOT)).replace("\\", "/"),
        demand=dict(
            kind="randomTrips(実測とは無関係)",
            demand_s=DEMAND_S,
            trips_per_hour=TRIPS_PER_HOUR,
            fringe_factor=FRINGE_FACTOR,
            min_distance_m=MIN_DISTANCE_M,
        ),
        meso_options=MESO_OPTIONS,
        end_s=END_S,
        sumo=st,
        wall_clock_s=dict(routing=round(t_route, 1), simulation=round(t_sim, 1)),
        finished=st["running"] == 0 and st["waiting"] == 0,
        top_waiting_edges=rows[:10],
    )
    (C.REPORTS / "54_meso_smoke.json").write_text(
        json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(
        f"loaded {st['loaded']} inserted {st['inserted']} running {st['running']} "
        f"waiting {st['waiting']} teleports {st['teleports']}  "
        f"経路 {t_route:.0f}s / メソ {t_sim:.0f}s"
    )


if __name__ == "__main__":
    main()
