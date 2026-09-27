"""現況再現の照合: シミュレーションの時間帯別交通量を観測と比べる(docs/sumo-design.md §13.7)。

ネットワークや需要の作り方と切り離した物差し。SUMO の edgeData(1 時間ごとの interval)を
入力にし、どの試行も同じ観測・同じホールドアウト・同じ指標で比べる(calib/estimation.py)。

- 観測: センサス(census_counts.json)+ 感知器(detector_counts.json)
- ホールドアウト: 観測の単位の 2 割(単位の ID のハッシュで固定)
- 時間帯ごとに、全体・推定側・ホールドアウト × 全出典・センサス・感知器 の指標
  (r・R²・回帰の傾き・RMSE%・GEH < 5 の割合・交通量比)と、地点ごとの時刻推移の相関
- 受け入れ条件(§13.7)の判定: --peak-hour の時間で
- 散布図: reports/figures/63_scatter_{label}.png

使い方:
  uv run python scripts/63_validate.py --edgedata data/sumo/x.edgedata.xml --start-hour 6 \\
      --hours 7 8 9 --label baseline

出力:  reports/63_validate_{label}.json、reports/figures/63_scatter_{label}.png
"""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim import config as C
from jp_sumo_traffic_sim.calib import estimation as E
from jp_sumo_traffic_sim.sumo.sim import read_hourly_flows


def scatter(points: dict, title: str, path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    hours = [h for h, pts in points.items() if pts]
    fig, axes = plt.subplots(1, len(hours), figsize=(5 * len(hours), 5), squeeze=False)
    for ax, h in zip(axes[0], hours, strict=True):
        pts = points[h]
        vmax = max(max(p[0], p[1]) for p in pts) * 1.05
        for src, mk in (("census", "o"), ("detector", "^")):
            for hold, col in ((False, "tab:blue"), (True, "tab:red")):
                pp = [p for p in pts if p[2] == src and p[3] == hold]
                if pp:
                    ax.scatter(
                        [p[0] for p in pp], [p[1] for p in pp], s=10, marker=mk, c=col,
                        alpha=0.6, label=f"{src}{' (holdout)' if hold else ''} n={len(pp)}",
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
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--edgedata", required=True, type=Path)
    ap.add_argument("--start-hour", required=True, type=int, help="シミュレーション時刻 0 の時")
    ap.add_argument("--hours", nargs="+", type=int, default=[7, 8, 9], help="照合する時")
    ap.add_argument("--peak-hour", type=int, default=8, help="受け入れ条件を判定する時")
    ap.add_argument("--label", default=None)
    args = ap.parse_args()
    label = args.label or args.edgedata.name.split(".")[0]

    census = json.loads((C.PROCESSED / "census_counts.json").read_text(encoding="utf-8"))["edges"]
    dets = json.loads((C.PROCESSED / "detector_counts.json").read_text(encoding="utf-8"))["edges"]
    obs = E.merge_observations(census, dets)
    flows = read_hourly_flows(args.edgedata, args.start_hour)
    hours = [h for h in args.hours if h in flows]
    if not hours:
        raise SystemExit(f"{args.edgedata} に照合する時間帯({args.hours})の 1 時間の区間が無い")

    ev = E.evaluate(obs, flows, hours)
    units = {o["unit"] for o in obs}
    rep = dict(
        case=C.CASE_NAME,
        label=label,
        edgedata=str(args.edgedata),
        start_hour=args.start_hour,
        hours=hours,
        observations=dict(
            n_edges=dict(census=len(census), detector=len(dets)),
            n_units=len(units),
            n_holdout_units=sum(E.is_holdout(u) for u in units),
            holdout_frac=E.HOLDOUT_FRAC,
        ),
        metrics=ev["metrics"],
        temporal=E.site_temporal_r(obs, flows, hours),
        acceptance=(
            dict(hour=args.peak_hour, **E.acceptance(ev["metrics"][args.peak_hour]))
            if args.peak_hour in ev["metrics"]
            else None
        ),
        criteria=dict(
            r=E.ACCEPT_R,
            slope=E.ACCEPT_SLOPE,
            geh_ok_share=E.ACCEPT_GEH_SHARE,
            holdout_r=E.ACCEPT_HOLDOUT_R,
        ),
    )
    C.REPORTS.mkdir(parents=True, exist_ok=True)
    (C.REPORTS / f"63_validate_{label}.json").write_text(
        json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    scatter(ev["points"], label, C.REPORTS / "figures" / f"63_scatter_{label}.png")

    for h in hours:
        m = ev["metrics"][h]
        a, c, d, ho = m["all/all"], m["all/census"], m["all/detector"], m["holdout/all"]
        print(
            f"{h}時 n={a.get('n')} r={a.get('r')} 傾き={a.get('slope')} "
            f"GEH<5={a.get('geh_ok_share')} RMSE%={a.get('rmse_pct')} | "
            f"センサス r={c.get('r')} 感知器 r={d.get('r')} | ホールドアウト r={ho.get('r')}"
        )
    if rep["acceptance"]:
        acc = rep["acceptance"]
        verdict = "合格" if acc["passed"] else "不合格"
        detail = "、".join(
            f"{k}={v['value']}{'○' if v['ok'] else '×'}" for k, v in acc["checks"].items()
        )
        print(f"受け入れ条件({acc['hour']}時): {verdict}({detail})")


if __name__ == "__main__":
    main()
