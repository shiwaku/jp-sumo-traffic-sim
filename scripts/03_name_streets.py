"""通し街路に札幌の街区名(南n条通 / 北n条通 / 西n丁目通 / 東n丁目通)を割り当てる。

KSJ には道路名の属性がない。札幌都心は 130.0m の等間隔格子なので、
config の基準線から算術的に名前が決まる。格子線から外れる通し街路は
街区中央の裏通り(狸小路など)として扱う。

OSM の name タグは**照合にのみ**使う(docs/data-sources.md 参照)。
路線名(真駒内篠路線・国道230号など)や橋名を拾いやすいため、
街路名らしい語を含むものだけを候補にする。

出力: reports/03_street_names.json
"""

import json
import re
import sys
import urllib.parse
import urllib.request
from collections import defaultdict
from pathlib import Path

import geopandas as gpd
import numpy as np
from shapely.geometry import LineString

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from sapporo_sim import config as C

OVERPASS = "https://overpass-api.de/api/interpreter"
CACHE = C.INTERIM / "osm_named_ways.json"
LATTICE_TOL_M = 30.0  # 格子線からこの距離以内なら格子街路とみなす
CROSS_TOL_M = 20.0  # OSM 照合の横断方向許容差
BEARING_TOL_DEG = 20.0

# 街路名として採用する語(路線名・橋名・施設名を弾く)
STREET_LIKE = re.compile(r"(条通|丁目通|大通|小路|条・|通$)")
NAME_DENY = re.compile(r"(橋|線$|号$|地下道|地下歩行空間|CR$|ターミナル|広場|駐車場)")


def fetch_osm(bbox: tuple[float, float, float, float]) -> dict:
    if CACHE.exists():
        return json.loads(CACHE.read_text(encoding="utf-8"))
    s, w, n, e = bbox
    q = f'[out:json][timeout:180];way["highway"]["name"]({s},{w},{n},{e});out tags geom;'
    req = urllib.request.Request(
        OVERPASS,
        data=urllib.parse.urlencode({"data": q}).encode(),
        headers={"User-Agent": "jp-sumo-traffic-sim/0.1"},
    )
    with urllib.request.urlopen(req, timeout=240) as r:  # noqa: S310
        data = json.loads(r.read().decode())
    CACHE.parent.mkdir(parents=True, exist_ok=True)
    CACHE.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
    return data


def lattice(axis: str) -> list[tuple[float, str]]:
    """格子線の (横断座標, 名前) 一覧。コードンの範囲を少し越えて張る。"""
    out = []
    if axis == "EW":
        out += [(C.v_south(n), f"南{n}条通") for n in range(1, 12)]
        out += [(C.v_north(n), f"北{n}条通") for n in range(1, 9)]
        out += [((C.v_south(1) + C.v_north(1)) / 2, "大通")]
    else:
        out += [(C.u_west(n), f"西{n}丁目通") for n in range(1, 16)]
        out += [(C.u_east(n), f"東{n}丁目通") for n in range(1, 5)]
    return sorted(out)


def label(axis: str, cross: float) -> tuple[str, float, str]:
    """格子線に照合して (名前, 残差, 種別) を返す。"""
    lat = lattice(axis)
    d = [(abs(cross - c), nm, c) for c, nm in lat]
    d.sort()
    err, nm, _ = d[0]
    if err <= LATTICE_TOL_M:
        return nm, round(err, 1), "格子街路"
    # 直近2本の間 = 街区中央の裏通り
    below = [(c, nm) for c, nm in lat if c < cross]
    above = [(c, nm) for c, nm in lat if c > cross]
    lo = below[-1][1] if below else "?"
    hi = above[0][1] if above else "?"
    return f"{lo}〜{hi}の裏通り", round(err, 1), "裏通り"


def main() -> None:
    grid = json.loads((C.REPORTS / "02_grid.json").read_text(encoding="utf-8"))
    cordon = C.cordon_polygon()

    pad = 0.004
    w, s_, e, n = gpd.GeoSeries([cordon], crs=C.CRS_PROJ).to_crs("EPSG:4326").total_bounds
    osm = fetch_osm((s_ - pad, w - pad, n + pad, e + pad))

    segs = []
    for el in osm.get("elements", []):
        nm = el.get("tags", {}).get("name")
        geom = el.get("geometry") or []
        if not nm or len(geom) < 2:
            continue
        if NAME_DENY.search(nm) or not STREET_LIKE.search(nm):
            continue
        segs.append((nm, LineString([(p["lon"], p["lat"]) for p in geom])))

    obs: dict = defaultdict(float)
    if segs:
        gs = gpd.GeoSeries([s[1] for s in segs], crs="EPSG:4326").to_crs(C.CRS_PROJ).map(C.to_grid)
        for (nm, _), g in zip(segs, gs, strict=False):
            c = np.asarray(g.coords)[:, :2]
            for a, b in zip(c[:-1], c[1:], strict=False):
                d = b - a
                L = float(np.hypot(*d))
                if L < 1.0:
                    continue
                ang = np.degrees(np.arctan2(d[1], d[0])) % 180.0
                if min(ang % 90.0, 90.0 - ang % 90.0) > BEARING_TOL_DEG:
                    continue
                ax = "EW" if (ang < 45 or ang > 135) else "NS"
                cross = (a[1] + b[1]) / 2 if ax == "EW" else (a[0] + b[0]) / 2
                obs[(nm, ax, round(cross, 1))] += L

    result = {}
    for axis in ("EW", "NS"):
        rows = sorted(
            (t for t in grid["streets"][axis]["table"] if t["through"]),
            key=lambda t: t["cross_m"],
        )
        out = []
        for t in rows:
            nm, err, kind = label(axis, t["cross_m"])
            cand: dict = defaultdict(float)
            for (onm, ax, cross), L in obs.items():
                if ax == axis and abs(cross - t["cross_m"]) <= CROSS_TOL_M:
                    cand[onm] += L
            ranked = sorted(cand.items(), key=lambda kv: -kv[1])
            osm_name = ranked[0][0] if ranked else None
            out.append(
                dict(
                    street_id=t["street_id"],
                    name=nm,
                    kind=kind,
                    lattice_error_m=err,
                    cross_m=t["cross_m"],
                    n_links=t["n_links"],
                    spread_m=t["spread_m"],
                    divided=t["divided"],
                    len_sum=t["len_sum"],
                    osm_name=osm_name,
                    osm_agrees=(osm_name == nm) if osm_name else None,
                )
            )
        result[axis] = out

    agree = [r for a in result.values() for r in a if r["osm_agrees"] is not None]
    result["_summary"] = dict(
        n_through=sum(len(result[a]) for a in ("EW", "NS")),
        n_lattice=sum(1 for a in ("EW", "NS") for r in result[a] if r["kind"] == "格子街路"),
        n_backstreet=sum(1 for a in ("EW", "NS") for r in result[a] if r["kind"] == "裏通り"),
        osm_checked=len(agree),
        osm_agree=sum(1 for r in agree if r["osm_agrees"]),
        max_lattice_error_m=max(
            (
                r["lattice_error_m"]
                for a in ("EW", "NS")
                for r in result[a]
                if r["kind"] == "格子街路"
            ),
            default=None,
        ),
    )

    (C.REPORTS / "03_street_names.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    for axis in ("EW", "NS"):
        print(f"\n=== {axis} 通し街路 ===")
        for r in result[axis]:
            osm = ""
            if r["osm_name"]:
                osm = f"  OSM={r['osm_name']}{'  ✓' if r['osm_agrees'] else '  ✗'}"
            div = " 分離?" if r["divided"] else "     "
            print(
                f"  {r['street_id']} {r['name']:<24s}{div} 誤差{r['lattice_error_m']:5.1f}m"
                f" links={r['n_links']:3d} spread={r['spread_m']:6.1f}{osm}"
            )
    print("\n", json.dumps(result["_summary"], ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
