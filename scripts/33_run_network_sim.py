"""実ネットワーク上のミクロシミュレーションを実行し、再生データを書き出す。

入力:
  data/processed/edges.gpkg              方向別 Edge(32_directed_lanes)
  data/processed/network_conflated.gpkg  ノード(信号)・stops(一時停止)
  data/processed/signal_plans.json       JARTIC サイクル長
出力:
  viewer/public/data/sim_net_{scenario}.json  1秒刻みの再生データ
  reports/33_netsim.json                      シナリオ別の集計
"""

import json
import sys
import time
from pathlib import Path

from pyproj import Transformer

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim import config as C
from jp_sumo_traffic_sim.network.io import load_network_inputs
from jp_sumo_traffic_sim.sim.netsim import DT, NetSim

WARMUP_S = 300.0
RECORD_S = 300.0
FRAME_S = 1.0
ENTRY_HOUR = 8  # 朝ピーク(reports/14_census.json の peak_hour_up)
OUT = C.ROOT / "viewer" / "public" / "data"


def run(scenario, nodes, edges, plans, stop_edges, hourly):
    sim = NetSim(
        nodes,
        edges,
        scenario=scenario,
        seed=42,
        signal_plans=plans,
        stop_edges=stop_edges,
        entry_hourly=hourly,
        entry_hour=ENTRY_HOUR,
    )

    cx = sum(n["x"] for n in nodes) / len(nodes)
    cy = sum(n["y"] for n in nodes) / len(nodes)
    tf = Transformer.from_crs(C.CRS_PROJ, "EPSG:6668", always_xy=True)
    lon0, lat0 = tf.transform(cx, cy)
    sig_nodes = [n for n in sim.nodes.values() if n.has_signal]
    nodes_dm = [[round((n.x - cx) * 10), round((n.y - cy) * 10)] for n in sig_nodes]

    t0 = time.time()
    steps_per_frame = round(FRAME_S / DT)
    frames, series, spill_max = [], [], 0
    for k in range(int((WARMUP_S + RECORD_S) / DT)):
        sim.step()
        if sim.t <= WARMUP_S or k % steps_per_frame:
            continue
        snap = sim.snapshot()
        flat = []
        for x, y, v in snap["pts"]:
            flat += [round((x - cx) * 10), round((y - cy) * 10), round(v * 10)]
        frames.append({"t": snap["t"], "p": flat, "sig": snap["sig"]})
        st = sim.stats()
        series.append(st)
        spill_max = max(spill_max, st["n_links_backed_up"])

    n_series = [s["n_vehicles"] for s in series]
    v_series = [s["mean_speed_ms"] for s in series]
    stats = dict(
        scenario=scenario,
        network="phase1_real",
        n_nodes=len(sim.nodes),
        n_signalized=sim.n_signalized,
        n_edges=len(sim.edges),
        n_entries=len(sim.entries),
        n_census_entries=sim.n_census_entries,
        entry_hour=ENTRY_HOUR,
        n_stop_sign_edges=len(stop_edges),
        wall_clock_s=round(time.time() - t0, 1),
        spawned=sim.n_spawned,
        exited=sim.n_exited,
        blocked_spawn=sim.n_blocked_spawn,
        lane_changes=sim.n_lane_changes,
        vehicles=dict(
            min=min(n_series), mean=round(sum(n_series) / len(n_series), 1), max=max(n_series)
        ),
        mean_speed_kmh=round(sum(v_series) / len(v_series) * 3.6, 1),
        max_links_backed_up=spill_max,
    )
    data = dict(
        scenario=f"net_{scenario}",
        origin=[round(lon0, 6), round(lat0, 6)],
        frame_s=FRAME_S,
        unit="0.1m / 0.1m/s",
        nodes=nodes_dm,
        frames=frames,
    )
    return data, stats


def main() -> None:
    nodes, edges, plans, stop_edges, hourly = load_network_inputs()
    OUT.mkdir(parents=True, exist_ok=True)
    report = {}
    for scenario in ("normal", "winter"):
        data, stats = run(scenario, nodes, edges, plans, stop_edges, hourly)
        f = OUT / f"sim_net_{scenario}.json"
        f.write_text(json.dumps(data, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
        stats["file_mb"] = round(f.stat().st_size / 1e6, 2)
        report[scenario] = stats
        print(
            f"net_{scenario}: {stats['vehicles']['mean']}台(平均) "
            f"{stats['mean_speed_kmh']}km/h  {stats['file_mb']}MB ({stats['wall_clock_s']}s)"
        )
    (C.REPORTS / "33_netsim.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
