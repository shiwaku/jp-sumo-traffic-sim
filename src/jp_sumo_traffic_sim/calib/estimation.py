"""センサス断面交通量からの需要推定と照合の部品(docs/sumo-design.md §13.5、C3b)。

- ホールドアウトは観測の単位(調査単位区間)で分ける。1つの観測を複数の区間が共有するため、
  区間で分けると同じ観測が推定側と検証側に入る
- 照合は (単位, 方向) ごとに Edge の値を平均してから比べる(長い区間ほど Edge が多く、
  Edge 単位だと重みが偏るため)
- 指標: GEH(交通量の照合で標準的な指標。5 未満を良好とする)、交通量比、速度誤差
"""

from __future__ import annotations

import math
import random
from collections import defaultdict

GEH_OK = 5.0


def split_holdout(units: list[str], frac: float, seed: int = 42) -> set[str]:
    """観測の単位からホールドアウト(検証用)を frac の割合で選ぶ。"""
    us = sorted(set(units))
    rng = random.Random(seed)
    rng.shuffle(us)
    return set(us[: round(len(us) * frac)])


def geh(model: float, obs: float) -> float:
    """GEH 統計量(時間交通量どうし)。"""
    if model + obs <= 0:
        return 0.0
    return math.sqrt(2.0 * (model - obs) ** 2 / (model + obs))


def obs_edgedata_xml(
    counts: dict[str, dict], hours: list[int], start_hour: int, exclude_units: set[str]
) -> tuple[str, int]:
    """routeSampler 用の観測値(edgeData 形式、1 時間の interval ごと)と Edge 数。

    counts: census_counts.json の edges(eid → {unit, counts[24], ...})。
    シミュレーション時刻 0 = start_hour 時。
    """
    lines = ["<data>"]
    n = 0
    for h in hours:
        b = (h - start_hour) * 3600
        lines.append(f'  <interval id="h{h:02d}" begin="{b}" end="{b + 3600}">')
        for eid, v in sorted(counts.items(), key=lambda kv: int(kv[0])):
            if v["unit"] in exclude_units:
                continue
            c = v["counts"][h]
            if c is None:
                continue
            lines.append(f'    <edge id="{eid}" entered="{c:.0f}"/>')
            n += 1
        lines.append("  </interval>")
    lines.append("</data>")
    return "\n".join(lines) + "\n", n


def compare(
    counts: dict[str, dict],
    sim_flow: dict[str, float],
    hour: int,
    holdout: set[str],
    sim_speed: dict[str, float] | None = None,
    obs_speed: dict[tuple[str, str], float] | None = None,
) -> dict:
    """(単位, 方向) ごとに観測とシミュレーションを比べ、推定側・検証側の指標を返す。

    sim_flow: eid → 評価時間帯の通過台数 [台/時]。sim_speed: eid → 平均速度 [km/h]。
    obs_speed: (区間, 方向) → 観測速度 [km/h]。
    """
    groups: dict[tuple[str, str], dict] = defaultdict(lambda: dict(obs=[], sim=[], spd=[], ospd=[]))
    for eid, v in counts.items():
        c = v["counts"][hour]
        if c is None or eid not in sim_flow:
            continue
        g = groups[(v["unit"], v["direction"])]
        g["obs"].append(c)
        g["sim"].append(sim_flow[eid])
        if sim_speed and eid in sim_speed and obs_speed:
            os_ = obs_speed.get((v["section"], v["direction"]))
            if os_ is not None:
                g["spd"].append(sim_speed[eid])
                g["ospd"].append(os_)

    out = {}
    for name, sel in (("calibration", False), ("holdout", True)):
        rows = []
        for (unit, direction), g in groups.items():
            if (unit in holdout) != sel:
                continue
            o, s = sum(g["obs"]) / len(g["obs"]), sum(g["sim"]) / len(g["sim"])
            row = dict(unit=unit, direction=direction, obs=o, sim=s, geh=geh(s, o))
            if g["spd"]:
                row["speed_err"] = sum(g["spd"]) / len(g["spd"]) - sum(g["ospd"]) / len(g["ospd"])
            rows.append(row)
        if not rows:
            out[name] = dict(n=0)
            continue
        to, ts = sum(r["obs"] for r in rows), sum(r["sim"] for r in rows)
        spd = [r["speed_err"] for r in rows if "speed_err" in r]
        out[name] = dict(
            n=len(rows),
            geh_ok_share=round(sum(r["geh"] < GEH_OK for r in rows) / len(rows), 3),
            geh_median=round(sorted(r["geh"] for r in rows)[len(rows) // 2], 2),
            volume_ratio=round(ts / to, 3) if to else None,
            speed_mae_kmh=round(sum(abs(x) for x in spd) / len(spd), 1) if spd else None,
            speed_bias_kmh=round(sum(spd) / len(spd), 1) if spd else None,
            n_speed=len(spd),
        )
    return out


def merge_observations(census: dict[str, dict], detectors: dict[str, dict]) -> list[dict]:
    """センサス(census_counts.json)と感知器(detector_counts.json)の観測を 1 つの一覧にする。

    同じ Edge に両方あるときは両方残す(照合は出典ごとにも出すため)。
    """
    out = []
    for src, d in (("census", census), ("detector", detectors)):
        for eid, v in d.items():
            out.append(
                dict(
                    eid=eid,
                    source=src,
                    unit=v["unit"],
                    direction=v["direction"],
                    counts=v["counts"],
                )
            )
    return out


def observation_groups(obs: list[dict], hour: int) -> list[dict]:
    """(出典, 観測の単位, 方向) ごとに Edge をまとめ、hour の観測値を平均する。"""
    acc: dict[tuple, dict] = {}
    for o in obs:
        c = o["counts"][hour]
        if c is None:
            continue
        g = acc.setdefault(
            (o["source"], o["unit"], o["direction"]),
            dict(source=o["source"], unit=o["unit"], direction=o["direction"], edges=[], vals=[]),
        )
        g["edges"].append(o["eid"])
        g["vals"].append(c)
    out = []
    for g in acc.values():
        vals = g.pop("vals")
        g["obs"] = sum(vals) / len(vals)
        out.append(g)
    return out


def fit_metrics(sim, obs) -> dict:
    """観測とシミュレーションの地点間の一致(docs/sumo-design.md §13.7)。

    r・R²(相関係数の 2 乗)、原点を通る回帰の傾き(sim = b · obs)、RMSE%(RMSE / 観測の平均)、
    GEH < 5 の割合、交通量比(合計どうし)。
    """
    s = [float(x) for x in sim]
    o = [float(x) for x in obs]
    n = len(o)
    if n < 2:
        return dict(n=n)
    ms, mo = sum(s) / n, sum(o) / n
    cov = sum((a - ms) * (b - mo) for a, b in zip(s, o, strict=True))
    vs = sum((a - ms) ** 2 for a in s)
    vo = sum((b - mo) ** 2 for b in o)
    r = cov / math.sqrt(vs * vo) if vs > 0 and vo > 0 else float("nan")
    oo = sum(b * b for b in o)
    slope = sum(a * b for a, b in zip(s, o, strict=True)) / oo if oo > 0 else float("nan")
    rmse = math.sqrt(sum((a - b) ** 2 for a, b in zip(s, o, strict=True)) / n)
    gehs = [geh(a, b) for a, b in zip(s, o, strict=True)]
    return dict(
        n=n,
        r=round(r, 3),
        r2=round(r * r, 3),
        slope=round(slope, 3),
        rmse_pct=round(100 * rmse / mo, 1) if mo > 0 else None,
        geh_ok_share=round(sum(g < GEH_OK for g in gehs) / n, 3),
        volume_ratio=round(sum(s) / sum(o), 3) if sum(o) > 0 else None,
    )
