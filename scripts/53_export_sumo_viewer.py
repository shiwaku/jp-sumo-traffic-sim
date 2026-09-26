"""SUMO を平常時・冬季で回し、ビューワの再生データを書き出す(docs/sumo-design.md §8)。

撤去した自前実装(scripts/33)と同じ形式・同じ条件(朝8時の需要、ウォームアップ
300 秒 + 記録 300 秒、1 秒刻み)で書き出すので、ビューワ側の変更は要らない。

- 車両: TraCI の購読で位置(サブレーンの横位置込み)と速度を取る
- 信号: 現示の状態文字列を、生成した計画(tll.xml)の A 青 / B 青と照らして
  '1'(A = 札幌では東西)/ '0'(B)/ '2'(黄・全赤)にする

入力:  data/sumo/{case}.net.xml・{case}_winter.net.xml・{case}.tll.xml(make sumo-net)
       data/processed/*(edges / census_hourly)
出力:  viewer/public/data/sim_net_{normal,winter}.json
       reports/53_sumo_viewer.json
"""

import json
import re
import sys
import time
from pathlib import Path

import sumolib
import traci
from pyproj import Transformer

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim import config as C
from jp_sumo_traffic_sim.network.io import load_network_inputs
from jp_sumo_traffic_sim.sumo import demand
from jp_sumo_traffic_sim.sumo.runner import SIM_OPTIONS, STEP_LENGTH_S, tool
from jp_sumo_traffic_sim.sumo.vtypes import vtype_xml

WARMUP_S = 300.0
RECORD_S = 300.0
FRAME_S = 1.0
ENTRY_HOUR = 8  # 朝ピーク(reports/14_census.json の peak_hour_up)
SUMO_DIR = C.ROOT / "data" / "sumo"
OUT = C.ROOT / "viewer" / "public" / "data"
V = traci.constants


def green_states(tll: Path) -> dict[str, tuple[str, str]]:
    """TLS id → (A 青の状態文字列, B 青の状態文字列)。生成時の6現示の 1・4 番目。"""
    out = {}
    text = tll.read_text(encoding="utf-8")
    for m in re.finditer(r'<tlLogic id="([^"]+)"(.*?)</tlLogic>', text, re.S):
        states = re.findall(r'state="([^"]+)"', m.group(2))
        out[m.group(1)] = (states[0], states[3])
    return out


def run(scenario: str, routes: Path, nodes_xy: dict, greens: dict, cx: float, cy: float):
    net = SUMO_DIR / (
        f"{C.CASE_NAME}.net.xml" if scenario == "normal" else f"{C.CASE_NAME}_winter.net.xml"
    )
    vt = SUMO_DIR / f"{C.CASE_NAME}_viewer_{scenario}.vtypes.add.xml"
    vt.write_text(f"<additional>\n  {vtype_xml(scenario)}\n</additional>\n", encoding="utf-8")
    traci.start(
        [
            tool("sumo"), "-n", str(net), "-a", str(vt), "-r", str(routes), *SIM_OPTIONS,
            "--begin", "0", "--end", str(WARMUP_S + RECORD_S), "--seed", "42",
            "--no-warnings", "true",
        ]
    )  # fmt: skip
    t0 = time.time()
    tls_ids = [t for t in traci.trafficlight.getIDList() if t in greens and t in nodes_xy]
    for t in tls_ids:
        traci.trafficlight.subscribe(t, [V.TL_RED_YELLOW_GREEN_STATE])
    frames, n_series, v_series, teleports = [], [], [], 0
    steps_per_frame = round(FRAME_S / STEP_LENGTH_S)
    k = 0
    try:
        while traci.simulation.getTime() < WARMUP_S + RECORD_S:
            traci.simulationStep()
            k += 1
            for vid in traci.simulation.getDepartedIDList():
                traci.vehicle.subscribe(vid, [V.VAR_POSITION, V.VAR_SPEED])
            teleports += traci.simulation.getStartingTeleportNumber()
            t = traci.simulation.getTime()
            if t <= WARMUP_S or k % steps_per_frame:
                continue
            flat, sp = [], []
            for res in traci.vehicle.getAllSubscriptionResults().values():
                (x, y), v = res[V.VAR_POSITION], res[V.VAR_SPEED]
                if v < 0 or x == V.INVALID_DOUBLE_VALUE:
                    continue  # テレポート中(道路上にいない)
                flat += [round((x - cx) * 10), round((y - cy) * 10), round(v * 10)]
                sp.append(v)
            tl = traci.trafficlight.getAllSubscriptionResults()
            sig = ""
            for tid in tls_ids:
                st = tl[tid][V.TL_RED_YELLOW_GREEN_STATE]
                a, b = greens[tid]
                sig += "1" if st == a else ("0" if st == b and "G" in b.upper() else "2")
            frames.append({"t": round(t, 1), "p": flat, "sig": sig})
            n_series.append(len(sp))
            v_series.append(sum(sp) / len(sp) if sp else 0.0)
    finally:
        traci.close()
    nodes_dm = [
        [round((nodes_xy[t][0] - cx) * 10), round((nodes_xy[t][1] - cy) * 10)] for t in tls_ids
    ]
    stats = dict(
        scenario=scenario,
        engine="sumo",
        n_tls=len(tls_ids),
        entry_hour=ENTRY_HOUR,
        wall_clock_s=round(time.time() - t0, 1),
        vehicles=dict(
            min=min(n_series), mean=round(sum(n_series) / len(n_series), 1), max=max(n_series)
        ),
        mean_speed_kmh=round(sum(v_series) / len(v_series) * 3.6, 1),
        teleports=teleports,
    )
    return frames, nodes_dm, stats


def main() -> None:
    nodes, edges, _, _, hourly = load_network_inputs()
    net_path = SUMO_DIR / f"{C.CASE_NAME}.net.xml"
    if not net_path.exists():
        raise SystemExit(f"{net_path} が無い。先に make sumo-net を実行する")
    net = sumolib.net.readNet(str(net_path))
    rates, _ = demand.entry_rates(edges, hourly, ENTRY_HOUR)
    rs = demand.build_routes(
        net, net_path, rates, 0.0, WARMUP_S + RECORD_S, SUMO_DIR / f"{C.CASE_NAME}_viewer"
    )
    greens = green_states(SUMO_DIR / f"{C.CASE_NAME}.tll.xml")

    nodes_xy = {str(n["nid"]): (n["x"], n["y"]) for n in nodes}
    cx = sum(n["x"] for n in nodes) / len(nodes)
    cy = sum(n["y"] for n in nodes) / len(nodes)
    lon0, lat0 = Transformer.from_crs(C.CRS_PROJ, "EPSG:6668", always_xy=True).transform(cx, cy)

    OUT.mkdir(parents=True, exist_ok=True)
    report = {}
    for scenario in ("normal", "winter"):
        frames, nodes_dm, stats = run(scenario, Path(rs["routes"]), nodes_xy, greens, cx, cy)
        data = dict(
            scenario=f"net_{scenario}",
            origin=[round(lon0, 6), round(lat0, 6)],
            frame_s=FRAME_S,
            unit="0.1m / 0.1m/s",
            nodes=nodes_dm,
            frames=frames,
        )
        f = OUT / f"sim_net_{scenario}.json"
        f.write_text(json.dumps(data, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
        stats["file_mb"] = round(f.stat().st_size / 1e6, 2)
        stats["n_vehicles_routed"] = rs["n_vehicles"]
        report[scenario] = stats
        print(
            f"net_{scenario}: {stats['vehicles']['mean']}台(平均) {stats['mean_speed_kmh']}km/h "
            f"テレポート {stats['teleports']}  {stats['file_mb']}MB ({stats['wall_clock_s']}s)"
        )
    (C.REPORTS / "53_sumo_viewer.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )


if __name__ == "__main__":
    main()
