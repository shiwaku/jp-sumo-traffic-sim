"""初期 OD を配分し、観測交通量に補正して照合する(docs/sumo-design.md §13.5 ③④・§13.7、D4)。

1. 配分: 61 のトリップを duaIterate.py(メソ)で DUA_STEPS 回繰り返し、混雑を反映した経路にする
   (経路探索は ROUTING_OPTIONS。61 の費用と同じ考え方)
2. 補正: routeSampler.py が、duaIterate の最後の経路を候補に、推定側の観測交通量(1 時間ごと)に
   合う経路と台数を選ぶ。2 通り比べる:
   - counts: 観測交通量だけ
   - counts_od: 観測交通量 + 初期 OD(粗いゾーンの OD 表)
3. 照合: それぞれをメソで回し、時間帯ごとに観測とシミュレーションの交通量を比べる
   (§13.7: r・R²・回帰の傾き・RMSE%・GEH < 5 の割合)。推定側 / ホールドアウト、
   センサス / 感知器 に分けて出す。配分だけ(補正なし)の結果も比べる
4. 散布図: reports/figures/62_scatter_{方式}.png

観測は (出典, 観測の単位, 方向) ごとに平均して比べる(calib/estimation.py)。
シミュレーション時刻 0 = START_HOUR 時(od/__init__.py)。最初の時間は助走で、照合は EVAL_HOURS。

入力:  data/sumo/{case}.net.xml、{case}_d4.trips.xml・{case}_d4_coarse.taz.xml・
       {case}_d4_coarse_od.xml
       data/processed/census_counts.json・detector_counts.json・holdout_units.json
出力:  data/sumo/d4/(duaIterate の作業・補正後の経路・メソの出力)
       reports/62_assign_calibrate.json、reports/figures/62_scatter_*.png
"""

import json
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim import config as C
from jp_sumo_traffic_sim.calib import estimation as E
from jp_sumo_traffic_sim.od import HOURS, START_HOUR
from jp_sumo_traffic_sim.sumo import sim as sumo_sim
from jp_sumo_traffic_sim.sumo.netgen import base_eid
from jp_sumo_traffic_sim.sumo.runner import ROUTING_OPTIONS, SUMO_HOME, env, run_tool

SUMO_DIR = C.ROOT / "data" / "sumo"
WORK = SUMO_DIR / "d4"
NET = SUMO_DIR / f"{C.CASE_NAME}.net.xml"
EVAL_HOURS = HOURS[1:]
END_S = len(HOURS) * 3600
DUA_STEPS = 8
SEED = 42
MESO_OPTIONS = ["--mesosim", "true", "--meso-junction-control", "true"]


def py_tool(rel: str, args: list[str], cwd: Path | None = None) -> subprocess.CompletedProcess:
    proc = subprocess.run(
        [sys.executable, str(SUMO_HOME / "tools" / rel), *args],
        capture_output=True, text=True, env=env(), cwd=cwd, check=False,
    )  # fmt: skip
    if proc.returncode != 0:
        raise RuntimeError(f"{rel} failed ({proc.returncode}):\n{proc.stderr[-4000:]}")
    return proc


def dua_iterate(trips: Path) -> tuple[Path, Path]:
    """duaIterate(メソ)。最後の反復の経路と、その反復の Edge の時間帯別の値を返す。"""
    d = WORK / "dua"
    if d.exists():
        shutil.rmtree(d)
    d.mkdir(parents=True)
    dua_args = [f"duarouter--{a.lstrip('-')}" if a.startswith("--") else a for a in ROUTING_OPTIONS]
    py_tool(
        "assign/duaIterate.py",
        [
            "-n", str(NET), "-t", str(trips), "-b", "0", "-e", str(END_S),
            "-a", "3600", "-m", "-j", "-l", str(DUA_STEPS), "--no-gzip", "-T",
            "--time-to-teleport", "300",
            "duarouter--routing-threads", "8",
            *dua_args, "sumo--seed", str(SEED),
        ],
        cwd=d,
    )  # fmt: skip
    last = DUA_STEPS - 1
    stem = trips.name.removesuffix(".trips.xml")
    routes = d / f"{last:03d}" / f"{stem}_{last:03d}.rou.xml"
    dump = d / f"{last:03d}" / "dump_3600.xml"
    return routes, dump


def obs_xml(obs: list[dict], holdout: set[str], path: Path) -> int:
    """routeSampler の観測値(推定側。同じ Edge に両方あるときはセンサス)。"""
    per_edge: dict[str, dict] = {}
    for o in sorted(obs, key=lambda o: o["source"] != "census"):
        if o["unit"] not in holdout and o["eid"] not in per_edge:
            per_edge[o["eid"]] = o
    counts = {eid: dict(unit=o["unit"], counts=o["counts"]) for eid, o in per_edge.items()}
    xml, n = E.obs_edgedata_xml(counts, HOURS, START_HOUR, exclude_units=set())
    path.write_text(xml, encoding="utf-8")
    return n


def route_sample(cand: Path, obs: Path, out: Path, od: bool) -> dict:
    mismatch = Path(str(out).replace(".rou.xml", ".mismatch.xml"))
    args = [
        "-r", str(cand), "-d", str(obs), "-o", str(out), "--mismatch-output", str(mismatch),
        "-a", 'departLane="best" departSpeed="max"', "--seed", str(SEED), "--threads", "8",
    ]  # fmt: skip
    if od:
        args += [
            "--od-files",
            str(SUMO_DIR / f"{C.CASE_NAME}_d4_coarse_od.xml"),
            "--taz-files",
            str(SUMO_DIR / f"{C.CASE_NAME}_d4_coarse.taz.xml"),
        ]
    py_tool("routeSampler.py", args)
    return dict(n_vehicles=sum(1 for _ in ET.parse(out).getroot().iter("vehicle")))


def run_meso(routes: Path, prefix: Path) -> tuple[Path, dict]:
    edgedata, stats = Path(f"{prefix}.edgedata.xml"), Path(f"{prefix}.stats.xml")
    add = Path(f"{prefix}.meandata.add.xml")
    add.write_text(
        f'<additional>\n  <edgeData id="hourly" file="{edgedata.name}" begin="0" end="{END_S}" '
        'period="3600"/>\n</additional>\n',
        encoding="utf-8",
    )
    run_tool(
        "sumo",
        [
            "-n", str(NET), "-r", str(routes), "-a", str(add), *MESO_OPTIONS,
            "--begin", "0", "--end", str(END_S + 1800), "--seed", str(SEED),
            "--time-to-teleport", "300",
            "--statistic-output", str(stats), "--no-step-log", "true", "--no-warnings", "true",
        ],
    )  # fmt: skip
    return edgedata, sumo_sim.read_stats(stats)


def hourly_flows(edgedata: Path) -> dict[int, dict[str, float]]:
    """時 → eid → 通過台数(右折車線の分割の上流側 = 層[2]の Edge の入口で数える)。"""
    out: dict[int, dict[str, float]] = {}
    for iv in ET.parse(edgedata).getroot().iter("interval"):
        h = START_HOUR + round(float(iv.get("begin")) / 3600)
        f = out.setdefault(h, {})
        for e in iv.iter("edge"):
            sid = e.get("id")
            if base_eid(sid) is not None and sid == str(base_eid(sid)):
                f[sid] = float(e.get("entered", 0))
    return out


def evaluate(obs: list[dict], flows: dict[int, dict[str, float]], holdout: set[str]) -> dict:
    """時 → {全体・推定側・ホールドアウト × 全出典・センサス・感知器} の指標と、散布図用の点。"""
    res, points = {}, {}
    for h in EVAL_HOURS:
        rows = []
        for g in E.observation_groups(obs, h):
            sims = [flows.get(h, {}).get(e) for e in g["edges"]]
            sims = [s for s in sims if s is not None]
            if sims:
                rows.append((g, float(np.mean(sims))))
        out = {}
        for part, sel in (
            ("all", lambda g: True),
            ("calibration", lambda g: g["unit"] not in holdout),
            ("holdout", lambda g: g["unit"] in holdout),
        ):
            for src in ("all", "census", "detector"):
                rr = [(g, s) for g, s in rows if sel(g) and src in ("all", g["source"])]
                out[f"{part}/{src}"] = E.fit_metrics([s for _, s in rr], [g["obs"] for g, _ in rr])
        res[h] = out
        points[h] = [(g["obs"], s, g["source"], g["unit"] in holdout) for g, s in rows]
    return dict(metrics=res, points=points)


def site_temporal_r(obs: list[dict], flows: dict[int, dict[str, float]]) -> dict:
    """地点ごとの時刻推移(EVAL_HOURS)の相関の中央値(出典別)。"""
    out = {}
    for src in ("census", "detector"):
        rs = []
        for o in obs:
            if o["source"] != src:
                continue
            ov = [o["counts"][h] for h in EVAL_HOURS]
            sv = [flows.get(h, {}).get(o["eid"]) for h in EVAL_HOURS]
            if any(v is None for v in ov + sv) or np.std(ov) == 0 or np.std(sv) == 0:
                continue
            rs.append(float(np.corrcoef(ov, sv)[0, 1]))
        out[src] = dict(n=len(rs), median_r=round(float(np.median(rs)), 3) if rs else None)
    return out


def scatter(points: dict, title: str, path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, len(points), figsize=(5 * len(points), 5))
    for ax, (h, pts) in zip(np.atleast_1d(axes), points.items(), strict=True):
        vmax = max([max(p[0], p[1]) for p in pts] + [1.0]) * 1.05
        for src, mk in (("census", "o"), ("detector", "^")):
            for hold, col in ((False, "tab:blue"), (True, "tab:red")):
                pp = [p for p in pts if p[2] == src and p[3] == hold]
                if pp:
                    ax.scatter(
                        [p[0] for p in pp], [p[1] for p in pp], s=10, marker=mk, c=col,
                        alpha=0.6, label=f"{src}{' (holdout)' if hold else ''}",
                    )  # fmt: skip
        ax.plot([0, vmax], [0, vmax], "k--", lw=0.8)
        ax.set_xlim(0, vmax)
        ax.set_ylim(0, vmax)
        ax.set_aspect("equal")
        ax.set_title(f"{title}  {h}:00-{h + 1}:00")
        ax.set_xlabel("observed [veh/h]")
        ax.set_ylabel("simulated [veh/h]")
        ax.legend(fontsize=7, loc="upper left")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main() -> None:
    WORK.mkdir(parents=True, exist_ok=True)
    census = json.loads((C.PROCESSED / "census_counts.json").read_text(encoding="utf-8"))["edges"]
    dets = json.loads((C.PROCESSED / "detector_counts.json").read_text(encoding="utf-8"))["edges"]
    obs = E.merge_observations(census, dets)
    holdout = set(
        json.loads((C.PROCESSED / "holdout_units.json").read_text(encoding="utf-8"))["units"]
    )
    rep: dict = dict(case=C.CASE_NAME, hours=HOURS, eval_hours=EVAL_HOURS, dua_steps=DUA_STEPS)
    t = {}

    t0 = time.time()
    routes, dump = dua_iterate(SUMO_DIR / f"{C.CASE_NAME}_d4.trips.xml")
    t["dua_iterate"] = round(time.time() - t0, 1)
    obs_path = WORK / "obs.xml"
    rep["n_count_constraints"] = obs_xml(obs, holdout, obs_path)

    results = {}
    # 配分だけ(補正なし): duaIterate の最後の反復のメソの値
    ev = evaluate(obs, hourly_flows(dump), holdout)
    results["assigned"] = dict(
        metrics=ev["metrics"], temporal=site_temporal_r(obs, hourly_flows(dump))
    )
    scatter(
        ev["points"], "assigned (initial OD)", C.REPORTS / "figures" / "62_scatter_assigned.png"
    )

    for name, od in (("counts", False), ("counts_od", True)):
        t0 = time.time()
        out = WORK / f"{name}.rou.xml"
        rs = route_sample(routes, obs_path, out, od)
        t[f"route_sampler_{name}"] = round(time.time() - t0, 1)
        t0 = time.time()
        edgedata, stats = run_meso(out, WORK / name)
        t[f"meso_{name}"] = round(time.time() - t0, 1)
        flows = hourly_flows(edgedata)
        ev = evaluate(obs, flows, holdout)
        results[name] = dict(
            route_sampler=rs,
            sumo=stats,
            metrics=ev["metrics"],
            temporal=site_temporal_r(obs, flows),
        )
        scatter(ev["points"], name, C.REPORTS / "figures" / f"62_scatter_{name}.png")
        for h in EVAL_HOURS:
            m = ev["metrics"][h]
            c, ho = m["calibration/all"], m["holdout/all"]
            print(
                f"[{name}] {h}時 推定側 r={c.get('r')} 傾き={c.get('slope')} "
                f"GEH<5={c.get('geh_ok_share')} | ホールドアウト r={ho.get('r')} "
                f"傾き={ho.get('slope')}",
                flush=True,
            )
    rep["results"] = results
    rep["wall_clock_s"] = t
    (C.REPORTS / "62_assign_calibrate.json").write_text(
        json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8"
    )


if __name__ == "__main__":
    main()
