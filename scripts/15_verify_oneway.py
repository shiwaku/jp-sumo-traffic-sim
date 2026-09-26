"""一方通行の向き(ポリライン頂点順)を OSM の oneway タグで検証する。

設計資料 5.4 の検証。JARTIC の一方通行はレコード上の方向欄がすべて空で、
向きの情報は頂点順しかない(Phase 0 実測)。頂点順の解釈を誤ると
逆走ネットワークになるため、OSM の oneway=yes の way と突合する。

手法: JARTIC 線の各セグメント中点から 15m 以内にある OSM oneway セグメントを探し、
      方向ベクトルの内積で 同方向 / 逆方向 を判定。長さ重みで多数決する。

出力: reports/15_oneway_check.json
"""

import json
import sys
import urllib.parse
import urllib.request
from pathlib import Path

import geopandas as gpd
import numpy as np
from shapely.geometry import LineString
from shapely.strtree import STRtree

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim import config as C

CACHE = C.INTERIM / "osm_oneway.json"
BUFFER_M = 15.0
DOT_TH = 0.85  # |cos| がこれ以上のときだけ判定に使う(直交道路の誤マッチ排除)


def fetch_osm() -> dict:
    if CACHE.exists():
        return json.loads(CACHE.read_text(encoding="utf-8"))
    cordon = C.region_polygon().buffer(C.CLIP_BUFFER_M)
    w, s, e, n = gpd.GeoSeries([cordon], crs=C.CRS_PROJ).to_crs("EPSG:4326").total_bounds
    q = (
        "[out:json][timeout:120];"
        f'way["highway"]["oneway"~"^(yes|-1)$"]({s},{w},{n},{e});'
        "out tags geom;"
    )
    endpoints = [
        "https://overpass-api.de/api/interpreter",
        "https://overpass.kumi.systems/api/interpreter",
        "https://overpass.osm.jp/api/interpreter",
    ]
    d = None
    last = None
    for ep in endpoints:
        try:
            req = urllib.request.Request(
                ep,
                data=urllib.parse.urlencode({"data": q}).encode(),
                headers={"User-Agent": "jp-sumo-traffic-sim/0.1"},
            )
            with urllib.request.urlopen(req, timeout=180) as r:  # noqa: S310
                d = json.loads(r.read().decode())
            break
        except Exception as e:  # 504 やタイムアウトは次のミラーへ
            last = e
    if d is None:
        raise SystemExit(f"Overpass から取得できなかった: {last}")
    CACHE.write_text(json.dumps(d, ensure_ascii=False), encoding="utf-8")
    return d


def segments(geom) -> list[tuple[np.ndarray, np.ndarray]]:
    """(中点, 単位方向ベクトル) の列。"""
    out = []
    lines = geom.geoms if geom.geom_type == "MultiLineString" else [geom]
    for ls in lines:
        c = np.asarray(ls.coords)[:, :2]
        for a, b in zip(c[:-1], c[1:], strict=False):
            d = b - a
            L = float(np.hypot(*d))
            if L < 1.0:
                continue
            out.append(((a + b) / 2, d / L, L))
    return out


def main() -> None:
    reg = gpd.read_file(C.INTERIM / "regulations.gpkg", layer="line")
    ow = reg[reg["code"] == "11"].copy()

    osm = fetch_osm()
    osm_rows = []
    for el in osm.get("elements", []):
        t = el.get("tags", {})
        if t.get("highway") in ("footway", "path", "cycleway", "steps", "pedestrian"):
            continue
        g = el.get("geometry") or []
        if len(g) < 2:
            continue
        ls = LineString([(p["lon"], p["lat"]) for p in g])
        # oneway=-1 は頂点順の逆が通行方向
        osm_rows.append((t.get("name", ""), t.get("oneway"), ls))
    osm_g = gpd.GeoSeries([r[2] for r in osm_rows], crs="EPSG:4326").to_crs(C.CRS_PROJ)

    # OSM セグメントを空間索引に
    osm_segs = []  # (mid, dir, name)
    for (name, oneway, _), geom in zip(osm_rows, osm_g, strict=False):
        sign = -1.0 if oneway == "-1" else 1.0
        for mid, d, _L in segments(geom):
            osm_segs.append((mid, d * sign, name))
    from shapely.geometry import Point

    pts = [Point(m) for m, _, _ in osm_segs]
    tree = STRtree(pts)

    results = []
    for _, r in ow.iterrows():
        votes_same = votes_opp = 0.0
        for mid, d, L in segments(r.geometry):
            idx = tree.query(Point(mid).buffer(BUFFER_M))
            best = None
            for i in idx:
                om, od, _ = osm_segs[i]
                dist = float(np.hypot(*(om - mid)))
                if dist > BUFFER_M:
                    continue
                dot = float(np.dot(d, od))
                if abs(dot) < DOT_TH:
                    continue  # 直交・斜交は判定に使わない
                if best is None or dist < best[0]:
                    best = (dist, dot)
            if best:
                if best[1] > 0:
                    votes_same += L
                else:
                    votes_opp += L
        total = votes_same + votes_opp
        verdict = (
            "未対応(OSMに相手なし)"
            if total == 0
            else "一致"
            if votes_same / total >= 0.8
            else "不一致"
            if votes_opp / total >= 0.8
            else "混在"
        )
        results.append(
            dict(
                uid=r.get("uid"),
                route=r.get("route") or "",
                in_cordon=bool(r.get("in_cordon")),
                len_m=round(float(r.geometry.length), 1),
                matched_m=round(total, 1),
                same_m=round(votes_same, 1),
                opposite_m=round(votes_opp, 1),
                verdict=verdict,
            )
        )

    n = len(results)
    counts = {}
    for v in ("一致", "不一致", "混在", "未対応(OSMに相手なし)"):
        counts[v] = sum(1 for x in results if x["verdict"] == v)
    judged = counts["一致"] + counts["不一致"] + counts["混在"]
    rep = dict(
        n_oneway=n,
        n_in_cordon=sum(1 for x in results if x["in_cordon"]),
        buffer_m=BUFFER_M,
        counts=counts,
        agree_rate=round(counts["一致"] / judged, 3) if judged else None,
        note="頂点順=通行方向と解釈して OSM と比較。「不一致」は解釈が逆であることを示す",
        details=sorted(results, key=lambda x: (x["verdict"] != "不一致", -x["len_m"])),
    )
    (C.REPORTS / "15_oneway_check.json").write_text(
        json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(
        json.dumps({k: v for k, v in rep.items() if k != "details"}, ensure_ascii=False, indent=2)
    )
    print("\n--- 不一致・混在・未対応 ---")
    for x in rep["details"]:
        if x["verdict"] == "一致":
            continue
        print(
            f"  {x['verdict']:<14s} len={x['len_m']:7.1f} matched={x['matched_m']:7.1f} "
            f"same={x['same_m']:7.1f} opp={x['opposite_m']:7.1f} {x['route']}"
        )


if __name__ == "__main__":
    main()
