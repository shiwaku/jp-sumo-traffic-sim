"""現況再現の照合と、需要の補正の部品(docs/sumo-design.md §13.7、D4)。

- 観測: センサス(census_counts.json)と感知器(detector_counts.json)。方向別 Edge ごとに時間交通量
- ホールドアウトは観測の単位(センサスは調査単位区間、感知器は地点)で分ける。1 つの観測を
  複数の Edge が共有するため、Edge で分けると同じ観測が推定側と検証側に入る。所属は単位の ID の
  ハッシュで決める(観測の集合が変わっても、各単位の所属は変わらない)
- 照合は (出典, 単位, 方向) ごとに Edge の値を平均してから比べる(長い区間ほど Edge が多く、
  Edge 単位だと重みが偏るため)
- 指標(§13.7): 相関係数 r・R²・原点を通る回帰の傾き・RMSE%・GEH < 5 の割合・交通量比
"""

from __future__ import annotations

import hashlib
import math
from collections.abc import Callable

GEH_OK = 5.0
HOLDOUT_FRAC = 0.2  # §13.7: 観測の単位の 2 割
HOLDOUT_SALT = "jp-sumo-traffic-sim/holdout"

# 受け入れ条件(§13.7。朝ピーク時間で判定)
ACCEPT_R = 0.9
ACCEPT_SLOPE = (0.9, 1.1)
ACCEPT_GEH_SHARE = 0.6
ACCEPT_HOLDOUT_R = 0.85


def is_holdout(unit: str, frac: float = HOLDOUT_FRAC, salt: str = HOLDOUT_SALT) -> bool:
    """観測の単位がホールドアウト(検証用)か。ID のハッシュで決める。"""
    h = hashlib.sha256(f"{salt}:{unit}".encode()).digest()
    return int.from_bytes(h[:8], "big") / 2**64 < frac


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


def group_sim(groups: list[dict], flows: dict[str, float]) -> list[tuple[dict, float]]:
    """観測のまとまりごとに、シミュレーションの交通量(Edge の平均)。値の無いまとまりは除く。"""
    out = []
    for g in groups:
        vals = [flows[e] for e in g["edges"] if e in flows]
        if vals:
            out.append((g, sum(vals) / len(vals)))
    return out


PARTS: dict[str, Callable[[dict], bool]] = {
    "all": lambda g: True,
    "calibration": lambda g: not is_holdout(g["unit"]),
    "holdout": lambda g: is_holdout(g["unit"]),
}
SOURCES = ("all", "census", "detector")


def evaluate(obs: list[dict], flows: dict[int, dict[str, float]], hours: list[int]) -> dict:
    """時間帯ごとの照合。戻り値: {metrics: 時 → "部分/出典" → 指標, points: 時 → [点]}。

    flows: 時 → eid → 通過台数 [台/時]。点は (観測, シミュレーション, 出典, ホールドアウトか)。
    """
    metrics, points = {}, {}
    for h in hours:
        rows = group_sim(observation_groups(obs, h), flows.get(h, {}))
        m = {}
        for part, sel in PARTS.items():
            for src in SOURCES:
                rr = [(g, s) for g, s in rows if sel(g) and src in ("all", g["source"])]
                m[f"{part}/{src}"] = fit_metrics([s for _, s in rr], [g["obs"] for g, _ in rr])
        metrics[h] = m
        points[h] = [(g["obs"], s, g["source"], is_holdout(g["unit"])) for g, s in rows]
    return dict(metrics=metrics, points=points)


def site_temporal_r(obs: list[dict], flows: dict[int, dict[str, float]], hours: list[int]) -> dict:
    """地点ごとの時刻推移(hours)の相関係数の中央値(出典別)。3 時間以上そろう地点だけ。"""
    out = {}
    for src in ("census", "detector"):
        rs = []
        for o in obs:
            if o["source"] != src:
                continue
            pairs = [
                (o["counts"][h], flows.get(h, {}).get(o["eid"]))
                for h in hours
                if o["counts"][h] is not None and o["eid"] in flows.get(h, {})
            ]
            if len(pairs) < 3:
                continue
            m = fit_metrics([p[1] for p in pairs], [p[0] for p in pairs])
            if m.get("r") == m.get("r"):  # NaN(どちらかが一定)を除く
                rs.append(m["r"])
        rs.sort()
        out[src] = dict(n=len(rs), median_r=rs[len(rs) // 2] if rs else None)
    return out


def acceptance(m: dict) -> dict:
    """1 時間分の指標(evaluate の metrics[時])を §13.7 の受け入れ条件で判定する。"""
    a, ho = m["all/all"], m["holdout/all"]
    lo, hi = ACCEPT_SLOPE
    checks = dict(
        r=(a.get("r"), a.get("r") is not None and a["r"] >= ACCEPT_R),
        slope=(a.get("slope"), a.get("slope") is not None and lo <= a["slope"] <= hi),
        geh_ok_share=(
            a.get("geh_ok_share"),
            a.get("geh_ok_share") is not None and a["geh_ok_share"] >= ACCEPT_GEH_SHARE,
        ),
        holdout_r=(ho.get("r"), ho.get("r") is not None and ho["r"] >= ACCEPT_HOLDOUT_R),
    )
    return dict(
        checks={k: dict(value=v, ok=ok) for k, (v, ok) in checks.items()},
        passed=all(ok for _, ok in checks.values()),
    )
