"""Phase 3-①: 実測需要でシミュレーションを回し、センサスと突き合わせる。

コードン流入をセンサス時間帯別交通量(方向平均)にした上で、区域内の
センサス17区間について 断面交通量(上下合計)と区間旅行速度を照合する。
この表が転回率調整(Phase 3-②)とオフセット推定(③)の目的関数の土台。

- 断面交通量: 区間に属する Edge を対向ペア(=1断面)にまとめ、
  ペア合計(上下合計)の断面平均を取る
- 旅行速度: 記録時間中の区間上の全車両の速度平均(空間平均)
- 照合時刻: 朝ピーク(8時、v_peak = 朝夕混雑時旅行速度)と
  昼オフピーク(13時、v_off = 昼間非混雑時旅行速度)

入力:  data/processed/*(edges / conflated / signal_plans / census_hourly)
       data/interim/census.gpkg(v_peak・v_off・路線名)
出力:  reports/40_demand_check.json
"""

import json
import sys
from pathlib import Path

import geopandas as gpd

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim import config as C
from jp_sumo_traffic_sim.network.io import load_network_inputs
from jp_sumo_traffic_sim.sim.demand import hourly_rate
from jp_sumo_traffic_sim.sim.netsim import DT, NetSim

WARMUP_S = 600.0
RECORD_S = 1800.0
RUNS = [
    dict(name="peak", hour=8, v_cols=("v_peak_up", "v_peak_dn")),
    dict(name="offpeak", hour=13, v_cols=("v_off_up", "v_off_dn")),
]


def load_census_sections() -> dict:
    g = gpd.read_file(C.INTERIM / "census.gpkg", layer="sections")
    out = {}
    for _, r in g.iterrows():
        vals = {}
        for c in ("v_peak_up", "v_peak_dn", "v_off_up", "v_off_dn"):
            v = gpd.pd.to_numeric(gpd.pd.Series([r.get(c)]), errors="coerce").iloc[0]
            vals[c] = None if gpd.pd.isna(v) else float(v)
        out[str(r["section_id"])] = dict(route=str(r.get("route") or ""), **vals)
    return out


def run_and_measure(nodes, edges, plans, stop_edges, hourly, hour):
    sim = NetSim(
        nodes,
        edges,
        scenario="normal",
        seed=42,
        signal_plans=plans,
        stop_edges=stop_edges,
        entry_hourly=hourly,
        entry_hour=hour,
    )
    # 区間 → Edge 群、断面(対向ペア)グループ
    sec_edges: dict[str, list] = {}
    for e in sim.edges.values():
        if e.census_id:
            sec_edges.setdefault(e.census_id, []).append(e)

    for _ in range(int(WARMUP_S / DT)):
        sim.step()
    flow0 = dict(sim.edge_flow)
    sp_sum: dict[str, float] = {}
    sp_n: dict[str, int] = {}
    steps = int(RECORD_S / DT)
    for k in range(steps):
        sim.step()
        if k % 2:  # 1秒ごとに区間速度を標本化
            continue
        for sid, es in sec_edges.items():
            for e in es:
                for v in e.vehicles:
                    sp_sum[sid] = sp_sum.get(sid, 0.0) + v.speed
                    sp_n[sid] = sp_n.get(sid, 0) + 1

    result = {}
    for sid, es in sec_edges.items():
        # 断面 = 対向ペア(一方通行は単独)。ペア合計の平均が上下合計の断面交通量
        groups: dict[int, int] = {}
        for e in es:
            key = e.eid if e.linked is None else min(e.eid, e.linked)
            dflow = sim.edge_flow.get(e.eid, 0) - flow0.get(e.eid, 0)
            groups[key] = groups.get(key, 0) + dflow
        sim_vol = sum(groups.values()) / len(groups) * (3600.0 / RECORD_S)
        sim_v = (sp_sum.get(sid, 0.0) / sp_n[sid] * 3.6) if sp_n.get(sid) else None
        result[sid] = dict(
            sim_vol=round(sim_vol, 0), sim_v_kmh=None if sim_v is None else round(sim_v, 1)
        )
    return sim, result


def main() -> None:
    nodes, edges, plans, stop_edges, hourly = load_network_inputs()
    sections = load_census_sections()
    report = {}
    for run in RUNS:
        sim, measured = run_and_measure(nodes, edges, plans, stop_edges, hourly, run["hour"])
        rows = []
        for sid, m in sorted(measured.items()):
            sec = sections.get(sid, {})
            obs_vol = hourly_rate(hourly.get(sid), run["hour"])
            obs_vol = None if obs_vol is None else obs_vol * 2  # 方向平均 → 上下合計
            vs = [sec.get(c) for c in run["v_cols"] if sec.get(c) is not None]
            obs_v = round(sum(vs) / len(vs), 1) if vs else None
            rows.append(
                dict(
                    section=sid,
                    route=sec.get("route", ""),
                    obs_vol=None if obs_vol is None else round(obs_vol, 0),
                    sim_vol=m["sim_vol"],
                    vol_ratio=(round(m["sim_vol"] / obs_vol, 2) if obs_vol else None),
                    obs_v_kmh=obs_v,
                    sim_v_kmh=m["sim_v_kmh"],
                )
            )
        # 集計: 交通量比(観測量重み)と速度誤差
        vr = [(r["vol_ratio"], r["obs_vol"]) for r in rows if r["vol_ratio"] is not None]
        dv = [
            r["sim_v_kmh"] - r["obs_v_kmh"]
            for r in rows
            if r["sim_v_kmh"] is not None and r["obs_v_kmh"] is not None
        ]
        report[run["name"]] = dict(
            hour=run["hour"],
            warmup_s=WARMUP_S,
            record_s=RECORD_S,
            n_entries=len(sim.entries),
            n_census_entries=sim.n_census_entries,
            spawned=sim.n_spawned,
            exited=sim.n_exited,
            blocked_spawn=sim.n_blocked_spawn,
            mean_speed_kmh=sim.stats()["mean_speed_ms"] * 3.6,
            sections=rows,
            summary=dict(
                vol_ratio_weighted=(
                    round(sum(r * w for r, w in vr) / sum(w for _, w in vr), 2) if vr else None
                ),
                vol_ratio_median=(round(sorted(r for r, _ in vr)[len(vr) // 2], 2) if vr else None),
                speed_mae_kmh=(round(sum(abs(x) for x in dv) / len(dv), 1) if dv else None),
                speed_bias_kmh=(round(sum(dv) / len(dv), 1) if dv else None),
                n_sections=len(rows),
            ),
        )
        s = report[run["name"]]["summary"]
        print(
            f"{run['name']}(hour={run['hour']}): vol_ratio(加重)={s['vol_ratio_weighted']} "
            f"中央値={s['vol_ratio_median']} 速度MAE={s['speed_mae_kmh']}km/h "
            f"バイアス={s['speed_bias_kmh']}km/h"
        )
    (C.REPORTS / "40_demand_check.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )


if __name__ == "__main__":
    main()
