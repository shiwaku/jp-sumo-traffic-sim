"""初期 OD(重力モデル)を作り、水準を観測交通量に当てはめる(docs/sumo-design.md §13.5 ②、D4)。

1. ゾーン間の自由流の経路の費用(SUMO の Edge の接続のグラフ、od/gravity.py。duarouter と同じ
   優先度の割り増しと信号の待ち)
2. 出入口(外部ゾーン)の交通量(コードンの断面交通量): 出入口の Edge から直進で
   GATE_OBS_STEPS 本先までに推定側の観測があれば基準時(REF_HOUR)の値。無ければ車線数 ×
   1 車線あたりの交通量(幅員 13m 以上は観測のある出入口の中央値、それ未満は
   NARROW_GATE_PER_LANE。細街路の観測はほとんど無く、観測のある細街路の出入口は幹線並みの
   例外なので中央値を使わない)
3. 4 つのブロックの OD の形(合計 1):
   - 内→内: 発生 = 人口、集中 = 従業者 + ATTR_POP_SHARE × 人口(通勤・通学 + その他)
   - 内→外: 発生 = 人口、集中 = 出口の交通量
   - 外→内: 発生 = 入口の交通量、集中 = 内→内と同じ
   - 外→外(通過): 発生 = 入口、集中 = 出口の交通量(近すぎる組は除く)
4. 水準: 入口の合計 = 外→内 + 外→外、出口の合計 = 内→外 + 外→外 とし、残る 2 つ
   (内→内の台数と、入口のうち通過の割合)を、ブロックごとの単位需要(AON_TRIPS 本)の
   自由流の配分(duarouter)から、基準時の観測交通量(推定側)への有界の最小二乗で決める
   (od/gravity.py fit_cordon)。照合は (観測の単位, 方向) ごとの平均で行う
5. 時間帯: 観測(感知器)の合計の時間変動 k(t) で各時間の台数にする
6. 書き出し: トリップ(発着の Edge はゾーンの TAZ の重みで選ぶ)と、routeSampler の OD 制約用の
   粗いゾーン(内部は 1 km メッシュ、外部は出入口ごと)の TAZ と OD 表

観測の単位の HOLDOUT_FRAC を検証用に外す(data/processed/holdout_units.json に保存し、
D4 の後段でも使う)。

入力:  data/sumo/{case}.net.xml・{case}.taz.xml、data/processed/zones.gpkg
       data/processed/census_counts.json・detector_counts.json
出力:  data/sumo/{case}_d4.trips.xml、{case}_d4_coarse.taz.xml、{case}_d4_coarse_od.xml
       data/processed/holdout_units.json、reports/61_initial_od.json
"""

import json
import sys
import time
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import geopandas as gpd
import numpy as np
import sumolib

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim import config as C
from jp_sumo_traffic_sim.calib import estimation as E
from jp_sumo_traffic_sim.od import HOURS, REF_HOUR, START_HOUR
from jp_sumo_traffic_sim.od import gravity as G
from jp_sumo_traffic_sim.sumo.netgen import base_eid
from jp_sumo_traffic_sim.sumo.runner import (
    ROUTING_OPTIONS,
    ROUTING_PRIORITY_FACTOR,
    ROUTING_TLS_PENALTY_S,
    run_tool,
)

SUMO_DIR = C.ROOT / "data" / "sumo"
PREFIX = SUMO_DIR / f"{C.CASE_NAME}_d4"
HOLDOUT_FRAC = 0.2
SEED = 42

GAMMA_PER_MIN = 0.10  # 所要時間の抵抗 exp(-γ c)
MIN_COST_MIN = 2.0  # これ未満の組(徒歩の距離)は車のトリップにしない
EE_MIN_COST_MIN = 2.0  # 通過交通の最短の所要時間。環状通を少しだけ走って隣の出入口へ抜ける
# 交通は実在するので、ごく近い組だけを除く
ATTR_POP_SHARE = 0.3
# 出入口の交通量を探す直進の本数。直進に限らないと、出入口の細街路が環状通の観測を拾う
GATE_OBS_STEPS = 2
WIDE_WIDTHS = {"4", "5"}  # 幅員 13m 以上(1 車線あたりの交通量を分ける)
NARROW_GATE_PER_LANE = 50.0  # 幅員 13m 未満の出入口の 1 車線あたりの交通量 [台/時](仮定)
# 出入口の 1 車線あたりの交通量の上限 [台/時]。細い出入口が直進先の幹線の観測を拾うと、
# 1 車線の一時停止の流入から 1 時間 1,200 台が出発して詰まる(D4 で発覚)
GATE_CAP_PER_LANE = {True: 800.0, False: 300.0}
AON_TRIPS = 40000  # 水準の当てはめに使う、ブロックごとの単位需要のトリップ数
BLOCKS = ("II", "IE", "EI", "EE")


def read_taz(path: Path) -> dict[str, dict]:
    out = {}
    for t in ET.parse(path).getroot().iter("taz"):
        out[t.get("id")] = dict(
            sources=[(s.get("id"), float(s.get("weight"))) for s in t.iter("tazSource")],
            sinks=[(s.get("id"), float(s.get("weight"))) for s in t.iter("tazSink")],
        )
    return out


def net_edges(net) -> list[dict]:
    lg = gpd.read_file(C.PROCESSED / "edges.gpkg", layer="edges", ignore_geometry=True)
    width_of = dict(zip(lg["eid"].astype(int), lg["width"].astype(str), strict=True))
    es = [e for e in net.getEdges() if e.getFunction() != "internal" and e.allows("passenger")]
    prios = [e.getPriority() for e in es]
    out = []
    for e in es:
        out.append(
            dict(
                id=e.getID(),
                cost_s=G.routing_cost(
                    e.getLength(),
                    e.getSpeed(),
                    e.getPriority(),
                    (min(prios), max(prios)),
                    e.getToNode().getType() == "traffic_light",
                    ROUTING_PRIORITY_FACTOR,
                    ROUTING_TLS_PENALTY_S,
                ),
                lanes=e.getLaneNumber(),
                width=width_of.get(base_eid(e.getID()), ""),
                succ=[f.getID() for f in e.getOutgoing() if f.allows("passenger")],
                straight=[
                    f.getID()
                    for f, cs in e.getOutgoing().items()
                    if f.allows("passenger") and any(c.getDirection() == "s" for c in cs)
                ],
            )
        )
    return out


def sample_edges(rng, cands: list[tuple[str, float]], n: int) -> list[str]:
    ids = [c[0] for c in cands]
    w = np.array([c[1] for c in cands], float)
    return list(rng.choice(ids, size=n, p=w / w.sum()))


def write_trips(path: Path, trips: list[tuple[float, str, str, str, str]]) -> None:
    """trips: (出発時刻, 発ゾーン, 着ゾーン, 発 Edge, 着 Edge)。"""
    trips = sorted(trips)
    with open(path, "w", encoding="utf-8") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n<routes>\n')
        for k, (t, o, d, fe, te) in enumerate(trips):
            f.write(
                f'    <trip id="{k}" depart="{t:.1f}" from="{fe}" to="{te}" '
                f'fromTaz="{o}" toTaz="{d}" departLane="best" departSpeed="max"/>\n'
            )
        f.write("</routes>\n")


def sample_od(rng, od: np.ndarray, n: int) -> list[tuple[int, int]]:
    """OD の形 od(合計 1)から n 本の (発, 着) を引く。"""
    flat = od.ravel()
    k = rng.choice(flat.size, size=n, p=flat / flat.sum())
    return list(zip(*np.unravel_index(k, od.shape), strict=True))


def route_loads(trips_path: Path, out: Path) -> dict[str, dict[str, int]]:
    """duarouter の自由流の経路 → ブロック → Edge → 通過本数。trip id の接頭辞がブロック。"""
    run_tool(
        "duarouter",
        [
            "-n", str(SUMO_DIR / f"{C.CASE_NAME}.net.xml"), "--route-files", str(trips_path),
            "-o", str(out), "--ignore-errors", "true", "--no-warnings", "true",
            "--no-step-log", "true", "--routing-threads", "8", *ROUTING_OPTIONS,
        ],
    )  # fmt: skip
    loads: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
    for _, el in ET.iterparse(out):
        if el.tag == "vehicle":
            b = el.get("id").split("_")[0]
            r = el.find("route")
            for eid in r.get("edges").split():
                loads[b][eid] += 1
            el.clear()
    return loads


def main() -> None:
    t0 = time.time()
    rng = np.random.default_rng(SEED)
    rep: dict = dict(case=C.CASE_NAME, hours=HOURS, start_hour=START_HOUR, ref_hour=REF_HOUR)

    # --- 観測とホールドアウト ---
    census = json.loads((C.PROCESSED / "census_counts.json").read_text(encoding="utf-8"))["edges"]
    dets = json.loads((C.PROCESSED / "detector_counts.json").read_text(encoding="utf-8"))["edges"]
    units = sorted({v["unit"] for v in census.values()} | {v["unit"] for v in dets.values()})
    holdout = E.split_holdout(units, HOLDOUT_FRAC, SEED)
    (C.PROCESSED / "holdout_units.json").write_text(
        json.dumps(dict(frac=HOLDOUT_FRAC, seed=SEED, units=sorted(holdout)), indent=1),
        encoding="utf-8",
    )
    obs = E.merge_observations(census, dets)

    # --- ゾーン ---
    taz = read_taz(SUMO_DIR / f"{C.CASE_NAME}.taz.xml")
    zi = gpd.read_file(C.PROCESSED / "zones.gpkg", layer="internal")
    ze = gpd.read_file(C.PROCESSED / "zones.gpkg", layer="external")
    zi = zi[zi["mesh"].isin(taz)].reset_index(drop=True)
    ze = ze[ze["zone"].isin(taz)].reset_index(drop=True)
    net = sumolib.net.readNet(str(SUMO_DIR / f"{C.CASE_NAME}.net.xml"))
    edges = net_edges(net)
    graph, idx = G.edge_graph(edges)
    by_id = {e["id"]: e for e in edges}
    pred_s = defaultdict(list)  # 直進の上流
    for e in edges:
        for f in e["straight"]:
            pred_s[f].append(e["id"])
    obs_ref: dict[str, float] = {}
    for o in obs:
        c = o["counts"][REF_HOUR]
        if c is not None and o["unit"] not in holdout:
            obs_ref[o["eid"]] = max(obs_ref.get(o["eid"], 0.0), c)

    def near_obs(eid: str, nxt) -> float | None:
        """eid か、直進で GATE_OBS_STEPS 本先までの最初の観測。出入口より車線の多い Edge の
        観測は別の道路(幹線)とみなして使わない。"""
        front, seen = [eid], {eid}
        lanes0 = by_id[eid]["lanes"]
        for _ in range(GATE_OBS_STEPS + 1):
            vals = [obs_ref[x] for x in front if x in obs_ref and by_id[x]["lanes"] <= lanes0]
            if vals:
                return max(vals)
            front = [y for x in front for y in nxt(x) if y not in seen]
            seen.update(front)
        return None

    def gate_edges(z: str, side: str) -> list[tuple[str, float | None]]:
        nxt = (lambda x: by_id[x]["straight"]) if side == "sources" else (lambda x: pred_s[x])
        return [(eid, near_obs(eid, nxt)) for eid, _ in taz[z][side]]

    def rep_edge(z: str, side: str) -> int | None:
        c = taz[z][side]
        return idx.get(max(c, key=lambda s: s[1])[0]) if c else None

    ZI, X = list(zi["mesh"]), list(ze["zone"])
    pop, emp = zi["pop"].to_numpy(float), zi["emp"].to_numpy(float)
    attr_i = emp + ATTR_POP_SHARE * pop
    ge = {(z, side): gate_edges(z, side) for z in X for side in ("sources", "sinks")}
    per_lane = defaultdict(list)
    for es in ge.values():
        for eid, v in es:
            if v is not None:
                e = by_id[eid]
                per_lane[e["width"] in WIDE_WIDTHS].append(v / e["lanes"])
    per_lane_med = {True: float(np.median(per_lane[True])), False: NARROW_GATE_PER_LANE}

    def gate_volume(z: str, side: str) -> float:
        total = 0.0
        for eid, v in ge[(z, side)]:
            e = by_id[eid]
            wide = e["width"] in WIDE_WIDTHS
            v = v if v is not None else e["lanes"] * per_lane_med[wide]
            total += min(v, e["lanes"] * GATE_CAP_PER_LANE[wide])
        return total

    g_in = np.array([gate_volume(z, "sources") for z in X])
    g_out = np.array([gate_volume(z, "sinks") for z in X])
    n_gate_obs = {
        side: sum(v is not None for (z, sd), es in ge.items() if sd == side for _, v in es)
        for side in ("sources", "sinks")
    }

    t1 = time.time()
    o_i = [rep_edge(z, "sources") for z in ZI]
    d_i = [rep_edge(z, "sinks") for z in ZI]
    o_x = [rep_edge(z, "sources") for z in X]
    d_x = [rep_edge(z, "sinks") for z in X]
    c_all = G.zone_costs(graph, o_i + o_x, d_i + d_x)
    ni = len(ZI)
    cost = dict(II=c_all[:ni, :ni], IE=c_all[:ni, ni:], EI=c_all[ni:, :ni], EE=c_all[ni:, ni:])
    t_cost = round(time.time() - t1, 1)

    shape = dict(
        II=G.gravity(pop, attr_i, cost["II"], GAMMA_PER_MIN, MIN_COST_MIN),
        IE=G.gravity(pop, g_out, cost["IE"], GAMMA_PER_MIN, MIN_COST_MIN),
        EI=G.gravity(g_in, attr_i, cost["EI"], GAMMA_PER_MIN, MIN_COST_MIN),
        EE=G.gravity(g_in, g_out, cost["EE"], GAMMA_PER_MIN, EE_MIN_COST_MIN),
    )
    zone_o = dict(II=ZI, IE=ZI, EI=X, EE=X)
    zone_d = dict(II=ZI, IE=X, EI=ZI, EE=X)
    mean_cost = {}
    for b in BLOCKS:
        s = shape[b].sum()
        shape[b] = shape[b] / s
        c = np.where(np.isfinite(cost[b]), cost[b], 0.0)
        mean_cost[b] = round(float((shape[b] * c).sum()), 1)

    # --- 水準: 単位需要を自由流で配分し、観測に当てはめる ---
    aon_trips = []
    for b in BLOCKS:
        for k, (i, j) in enumerate(sample_od(rng, shape[b], AON_TRIPS)):
            o, d = zone_o[b][i], zone_d[b][j]
            aon_trips.append(
                (
                    0.0,
                    f"{b}_{k}",
                    sample_edges(rng, taz[o]["sources"], 1)[0],
                    sample_edges(rng, taz[d]["sinks"], 1)[0],
                )
            )
    aon_path = Path(f"{PREFIX}_aon.trips.xml")
    with open(aon_path, "w", encoding="utf-8") as f:
        f.write("<routes>\n")
        for t, vid, fe, te in aon_trips:
            f.write(f'    <trip id="{vid}" depart="{t}" from="{fe}" to="{te}"/>\n')
        f.write("</routes>\n")
    t1 = time.time()
    loads = route_loads(aon_path, Path(f"{PREFIX}_aon.rou.xml"))
    t_aon = round(time.time() - t1, 1)

    groups = E.observation_groups(obs, REF_HOUR)
    fit_rows = []
    for g in groups:
        x = [np.mean([loads[b].get(e, 0) / AON_TRIPS for e in g["edges"]]) for b in BLOCKS]
        fit_rows.append((g, np.array(x)))
    cal = [(g, x) for g, x in fit_rows if g["unit"] not in holdout]
    in_total, out_total = float(g_in.sum()), float(g_out.sum())
    n_ii, ee_share = G.fit_cordon(
        np.array([x for _, x in cal]), np.array([g["obs"] for g, _ in cal]), in_total, out_total
    )
    scales = G.cordon_block_totals(n_ii, ee_share, in_total, out_total)
    aon_fit = {}
    for name, rows in (("calibration", cal), ("holdout", [r for r in fit_rows if r not in cal])):
        o = np.array([g["obs"] for g, _ in rows])
        s = np.array([x @ scales for _, x in rows])
        aon_fit[name] = E.fit_metrics(s, o)

    # --- 時間帯と書き出し ---
    prof = G.hourly_profile([v["counts"] for v in dets.values()], HOURS, REF_HOUR)
    trips = []
    coarse_od: dict[int, dict] = defaultdict(lambda: defaultdict(float))
    coarse = {z: f"c{z[:8]}" for z in ZI} | {z: z for z in X}
    n_by_block_hour = defaultdict(dict)
    for h in HOURS:
        b0 = (h - START_HOUR) * 3600.0
        for b in BLOCKS:
            mat = shape[b] * scales[BLOCKS.index(b)] * prof[h]
            n = rng.poisson(mat)
            n_by_block_hour[b][h] = int(n.sum())
            for i, j in zip(*np.nonzero(n), strict=True):
                o, d = zone_o[b][i], zone_d[b][j]
                m = int(n[i, j])
                coarse_od[h][(coarse[o], coarse[d])] += m
                fes = sample_edges(rng, taz[o]["sources"], m)
                tes = sample_edges(rng, taz[d]["sinks"], m)
                for fe, te in zip(fes, tes, strict=True):
                    trips.append((b0 + rng.uniform(0, 3600), o, d, fe, te))
    write_trips(Path(f"{PREFIX}.trips.xml"), trips)

    # 粗いゾーンの TAZ(Edge は細かいゾーンの和集合)と OD 表
    ctaz: dict[str, tuple[set, set]] = defaultdict(lambda: (set(), set()))
    for z, c in coarse.items():
        ctaz[c][0].update(e for e, _ in taz[z]["sources"])
        ctaz[c][1].update(e for e, _ in taz[z]["sinks"])
    lines = ["<additional>"]
    for c, (src, snk) in sorted(ctaz.items()):
        lines.append(f'    <taz id="{c}">')
        lines += [f'        <tazSource id="{e}" weight="1"/>' for e in sorted(src)]
        lines += [f'        <tazSink id="{e}" weight="1"/>' for e in sorted(snk)]
        lines.append("    </taz>")
    lines.append("</additional>")
    Path(f"{PREFIX}_coarse.taz.xml").write_text("\n".join(lines) + "\n", encoding="utf-8")
    ivs = [
        ((h - START_HOUR) * 3600.0, (h - START_HOUR + 1) * 3600.0, dict(coarse_od[h]))
        for h in HOURS
    ]
    Path(f"{PREFIX}_coarse_od.xml").write_text(G.taz_relation_xml(ivs), encoding="utf-8")

    rep.update(
        holdout=dict(frac=HOLDOUT_FRAC, n_units=len(holdout), n_units_total=len(units)),
        n_internal_zones=ni,
        n_gate_edges_with_obs=n_gate_obs,
        gate_per_lane_median={
            ("wide" if k else "narrow"): round(v) for k, v in per_lane_med.items()
        },
        cordon_ref_hour=dict(inbound=round(in_total), outbound=round(out_total)),
        fitted=dict(ii_trips=round(n_ii), ee_share_of_inbound=round(ee_share, 3)),
        n_external_zones=len(X),
        params=dict(
            gamma_per_min=GAMMA_PER_MIN,
            min_cost_min=MIN_COST_MIN,
            ee_min_cost_min=EE_MIN_COST_MIN,
            attr_pop_share=ATTR_POP_SHARE,
            gate_obs_steps=GATE_OBS_STEPS,
            narrow_gate_per_lane=NARROW_GATE_PER_LANE,
            gate_cap_per_lane={
                ("wide" if k else "narrow"): v for k, v in GATE_CAP_PER_LANE.items()
            },
            aon_trips_per_block=AON_TRIPS,
        ),
        mean_trip_time_free_flow_min=mean_cost,
        block_trips_ref_hour={b: round(float(s)) for b, s in zip(BLOCKS, scales, strict=True)},
        hourly_profile={h: round(v, 3) for h, v in prof.items()},
        trips_by_block_hour={b: v for b, v in n_by_block_hour.items()},
        n_trips=len(trips),
        n_observation_groups=len(groups),
        aon_fit_ref_hour=aon_fit,
        n_coarse_zones=len(ctaz),
        n_coarse_od_pairs={h: len(coarse_od[h]) for h in HOURS},
        wall_clock_s=dict(costs=t_cost, aon=t_aon, total=round(time.time() - t0, 1)),
    )
    (C.REPORTS / "61_initial_od.json").write_text(
        json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(rep, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
