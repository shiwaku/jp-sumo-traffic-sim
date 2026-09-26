"""Phase 3-①: 実測需要でシミュレーションを回し、センサスと突き合わせる。

コードン流入をセンサス時間帯別交通量(方向平均)にした上で、区域内の
センサス17区間について 断面交通量(上下合計)と区間旅行速度を照合する。
この表が転回率調整(Phase 3-②)とオフセット推定(③)の目的関数の土台。

- 断面交通量: 区間に属する Edge を対向ペア(=1断面)にまとめ、
  ペア合計(上下合計)の断面平均を取る
- 旅行速度: 記録時間中の区間上の全車両の速度平均(空間平均)
- 照合時刻: 朝ピーク(8時、v_peak = 朝夕混雑時旅行速度)と
  昼オフピーク(13時、v_off = 昼間非混雑時旅行速度)

シミュレーションは SUMO(docs/sumo-design.md §4・§6)。需要はセンサス流入 + 既定値、
転回率 0.70/0.15/0.15、巡回上限 30。自前実装(撤去済み)での結果は sumo-design.md §11.1。

入力:  data/processed/*(edges / census_hourly)
       data/interim/census.gpkg(v_peak・v_off・路線名)
       data/sumo/{case}.net.xml(make sumo-net で生成)
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
from jp_sumo_traffic_sim.sumo import demand as sumo_demand
from jp_sumo_traffic_sim.sumo import sim as sumo_sim
from jp_sumo_traffic_sim.sumo.netgen import base_eid

SUMO_DIR = C.ROOT / "data" / "sumo"

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


def run_and_measure(edges, hourly, hour):
    """SUMO で回し、区間ごとの計測値と実行情報を返す。"""
    import sumolib

    net_path = SUMO_DIR / f"{C.CASE_NAME}.net.xml"
    if not net_path.exists():
        raise SystemExit(f"{net_path} が無い。先に make sumo-net を実行する")
    net = sumolib.net.readNet(str(net_path))
    end = WARMUP_S + RECORD_S
    prefix = SUMO_DIR / f"{C.CASE_NAME}_h{hour:02d}"
    rates, dstats = sumo_demand.entry_rates(edges, hourly, hour)
    rstats = sumo_demand.build_routes(net, net_path, rates, 0.0, end, prefix)
    out = sumo_sim.run(net_path, Path(rstats["routes"]), prefix, WARMUP_S, end)
    ed = sumo_sim.read_edgedata(out["edgedata"])
    st = sumo_sim.read_stats(out["stats"])

    # 層[2]の Edge ごと: 下流端(停止線側)の区間の left = 断面通過台数(自前実装の edge_flow)。
    # 右折車線の分割がある Edge は分割後の区間が下流端
    seg_of: dict[int, list[str]] = {}
    for sid in ed:
        b = base_eid(sid)
        if b is not None:
            seg_of.setdefault(b, []).append(sid)
    flow = {b: ed[max(ss, key=len)]["left"] for b, ss in seg_of.items()}

    result = {}
    by_sec: dict[str, list[dict]] = {}
    for e in edges:
        if e.get("census_id"):
            by_sec.setdefault(e["census_id"], []).append(e)
    for sid, es in by_sec.items():
        groups: dict[int, float] = {}
        sp, w = 0.0, 0.0
        for e in es:
            le = e.get("linked_edge", -1)
            key = e["eid"] if le in (-1, None) else min(e["eid"], le)
            groups[key] = groups.get(key, 0.0) + flow.get(e["eid"], 0.0)
            for seg in seg_of.get(e["eid"], []):
                sp += ed[seg]["speed"] * ed[seg]["sampledSeconds"]
                w += ed[seg]["sampledSeconds"]
        sim_vol = sum(groups.values()) / len(groups) * (3600.0 / RECORD_S)
        result[sid] = dict(
            sim_vol=round(sim_vol, 0), sim_v_kmh=round(sp / w * 3.6, 1) if w > 0 else None
        )
    tot_sp = sum(v["speed"] * v["sampledSeconds"] for v in ed.values())
    tot_w = sum(v["sampledSeconds"] for v in ed.values())
    info = dict(
        engine="sumo",
        n_entries=dstats["n_entries"],
        n_census_entries=dstats["n_census_entries"],
        n_default_entries=dstats["n_default_entries"],
        total_entry_vph=dstats["total_entry_vph"],
        default_entry_vph=dstats["default_entry_vph"],
        routes=dict(
            n_vehicles=rstats["n_vehicles"],
            n_hop_capped=rstats["n_hop_capped"],
            n_route_not_ending_at_exit=rstats["n_route_not_ending_at_exit"],
        ),
        sumo=st,
        mean_speed_kmh=round(tot_sp / tot_w * 3.6, 1) if tot_w > 0 else None,
    )
    return info, result


def main() -> None:
    _, edges, _, _, hourly = load_network_inputs()
    sections = load_census_sections()
    report = {}
    for run in RUNS:
        info, measured = run_and_measure(edges, hourly, run["hour"])
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
            **info,
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
