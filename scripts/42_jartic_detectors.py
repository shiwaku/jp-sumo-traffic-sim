"""JARTIC 車両感知器の位置を特定し、時間帯別の観測値にする(docs/sumo-design.md §13.6、D2)。

1. 断面交通量(typeB、5 分値)を一度だけ読み、感知器ごとに平日の時間帯別平均交通量を集計する
   (data/interim/jartic_detectors.parquet にキャッシュ)
2. 位置の特定(calib/detectors.py)。優先順:
   a. 交差点制御情報(typeC)で感知器のリンクが交差点 Y の流入 → Y に入る道路のうち
      進行方向の合うもの(X の流出かつ Y の流入 = 区間が確定、を含む)
   b. 交差点 X の流出 → X から出る道路のうち進行方向の合うもの
   c. 地点名称の条・丁目の住所 → 街区の中心から ADDR_SNAP_M 以内で進行方向の合う最寄りの道路
   交差点はネットワークの最寄りノード(NODE_SNAP_M 以内)に寄せる。区域外のものは落ちる
3. センサスと同じ形式(方向別 Edge の時間帯別交通量)で書き出す。観測の単位は感知器
4. 検査: a/b と c の両方で位置が決まる感知器で、2 つの位置の距離。センサスと同じ Edge に
   ある感知器で、朝 8 時の交通量の比(2026 年 6 月の感知器 / 2021 年秋のセンサス)

入力:  data/raw/jartic/*/typeB_*.zip・typeC_*.zip、data/interim/tmt_intersections_*.json
       data/processed/edges.gpkg・network_conflated.gpkg・census_counts.json
出力:  data/processed/detector_counts.json、reports/42_jartic_detectors.json
"""

import csv
import io
import json
import sys
import zipfile
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path

import geopandas as gpd
import numpy as np
import pandas as pd
from pyproj import Transformer
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim import config as C
from jp_sumo_traffic_sim.calib import detectors as D
from jp_sumo_traffic_sim.network.jartic import fetch_positions, parse_definition

csv.field_size_limit(2**31 - 1)
CACHE = C.INTERIM / "jartic_detectors.parquet"
SOURCE_CODE = C.CASE["jartic"]["control_source_code"]
NODE_SNAP_M = 60.0  # 交差点の座標 → ネットワークのノード
ADDR_SNAP_M = 120.0  # 住所(街区の中心)→ 道路
HOURS = [f"h{h:02d}" for h in range(24)]


def aggregate_typeB() -> pd.DataFrame:
    """感知器ごとの属性と、平日の時間帯別平均交通量 [台/時]。"""
    if CACHE.exists():
        return pd.read_parquet(CACHE)
    zp = sorted((C.RAW / "jartic").glob("*/typeB_*.zip"))[-1]
    meta: dict[str, dict] = {}
    acc = defaultdict(lambda: [0.0, 0])  # (地点, 日付, 時) → [合計, 標本数]
    with zipfile.ZipFile(zp) as z:
        info = [i for i in z.infolist() if i.filename.lower().endswith(".csv")][0]
        with z.open(info) as fh:
            r = csv.reader(io.TextIOWrapper(fh, encoding="cp932", errors="replace"))
            next(r)
            for row in r:
                if len(row) < 10 or not row[7]:
                    continue
                det = row[2]
                if det not in meta:
                    meta[det] = dict(
                        det=det, name=row[3], mesh=row[4], div=row[5], link=row[6],
                        dist10=row[8],
                    )  # fmt: skip
                try:
                    v = float(row[7])
                except ValueError:
                    continue
                a = acc[(det, row[0][:10], int(row[0][11:13]))]
                a[0] += v
                a[1] += 1
    per_day = defaultdict(lambda: defaultdict(list))  # 地点 → 時 → [時間交通量(日ごと)]
    n_samples = Counter()
    for (det, day, h), (s, n) in acc.items():
        if datetime.strptime(day, "%Y/%m/%d").weekday() >= 5:
            continue  # 平日だけ(2026 年 6 月に祝日は無い)
        n_samples[n] += 1
        per_day[det][h].append(s * 12.0 / n if n < 12 else s)  # 欠けた 5 分値は比例で補う
    rows = []
    for det, m in meta.items():
        row = dict(m)
        for h in range(24):
            vals = per_day[det].get(h, [])
            row[f"h{h:02d}"] = float(np.mean(vals)) if vals else None
        row["n_days"] = max((len(v) for v in per_day[det].values()), default=0)
        rows.append(row)
    df = pd.DataFrame(rows)
    df.attrs["samples_per_hour"] = dict(n_samples.most_common(5))
    df.to_parquet(CACHE)
    return df


def main() -> None:
    det = aggregate_typeB()
    det = det[~det["name"].str.contains("）")]  # 札幌以外(樽）・苫）など)を除く
    case = C.case_module()
    gb = getattr(case, "GRID_BEARING_DEG", 0.0)
    addr_xy = getattr(case, "grid_address_xy", None)

    edges = gpd.read_file(C.PROCESSED / "edges.gpkg", layer="edges")
    nodes = gpd.read_file(C.PROCESSED / "network_conflated.gpkg", layer="nodes")
    node_ids = nodes["nid"].astype(int).to_numpy()
    node_tree = cKDTree(np.c_[nodes.geometry.x, nodes.geometry.y])
    inc, out = defaultdict(list), defaultdict(list)  # ノード → [(eid, 格子方位)]
    for _, e in edges.iterrows():
        c = list(e.geometry.coords)
        inc[int(e["to"])].append((int(e["eid"]), D.grid_bearing(c[-2], c[-1], gb)))
        out[int(e["frm"])].append((int(e["eid"]), D.grid_bearing(c[0], c[1], gb)))

    zc = sorted((C.RAW / "jartic").glob("*/typeC_*.zip"))[-1]
    defs = parse_definition(zc, SOURCE_CODE)
    pos = fetch_positions(C.INTERIM / f"tmt_intersections_{SOURCE_CODE}.json")
    tf = Transformer.from_crs("EPSG:4326", C.CRS_PROJ, always_xy=True)
    ins, outs = defaultdict(list), defaultdict(list)  # DRM リンク → [交差点番号]
    for i, d in defs.items():
        if i not in pos:
            continue
        for lk in d["in_links"]:
            ins[(lk["mesh"], lk["div"], lk["link"])].append(i)
        for lk in d["out_links"]:
            outs[(lk["mesh"], lk["div"], lk["link"])].append(i)

    def node_of(ix: int) -> int | None:
        x, y = tf.transform(*pos[ix])
        dist, k = node_tree.query((x, y))
        return int(node_ids[k]) if dist <= NODE_SNAP_M else None

    mid = edges.geometry.interpolate(0.5, normalized=True)
    edge_tree = cKDTree(np.c_[mid.x, mid.y])
    eids = edges["eid"].astype(int).to_numpy()
    mid_xy = {int(e): (p.x, p.y) for e, p in zip(eids, mid, strict=True)}
    mid_bearing = {
        int(e["eid"]): D.grid_bearing(
            e.geometry.interpolate(0.45, normalized=True).coords[0],
            e.geometry.interpolate(0.55, normalized=True).coords[0],
            gb,
        )
        for _, e in edges.iterrows()
    }

    located, how = {}, Counter()
    addr_check = []
    for _, r in det.iterrows():
        k = (r["mesh"], r["div"], r["link"])
        travel = D.travel_direction(r["name"])
        eid = None
        method = None
        if ins.get(k):
            n = node_of(ins[k][0])
            if n is not None:
                eid = D.pick_edge(inc[n], travel)
                method = "typeC_in_out" if outs.get(k) else "typeC_in"
        if eid is None and outs.get(k):
            n = node_of(outs[k][0])
            if n is not None:
                eid = D.pick_edge(out[n], travel)
                method = "typeC_out"
        ga = D.grid_address(r["name"]) if addr_xy else None
        addr_eid = None
        if ga is not None:
            x, y = addr_xy(*ga)
            cand = [
                (int(eids[i]), mid_bearing[int(eids[i])])
                for i in edge_tree.query_ball_point((x, y), ADDR_SNAP_M)
            ]
            cand.sort(key=lambda c: np.hypot(mid_xy[c[0]][0] - x, mid_xy[c[0]][1] - y))
            addr_eid = D.pick_edge(cand, travel) if travel else None
        if eid is not None and addr_eid is not None:
            (ax, ay), (bx, by) = mid_xy[eid], mid_xy[addr_eid]
            addr_check.append(float(np.hypot(ax - bx, ay - by)))
        if eid is None and addr_eid is not None:
            eid, method = addr_eid, "address"
        if eid is None:
            how["unlocated"] += 1
            continue
        how[method] += 1
        located[r["det"]] = (eid, travel, [r[h] for h in HOURS], method)

    # 同じ Edge に複数の感知器 → 交通量の多い方を残す(車線別の感知器の重複など)
    by_edge: dict[int, tuple] = {}
    for d_id, (eid, travel, counts, method) in located.items():
        vol = counts[8] or 0.0
        if eid not in by_edge or vol > (by_edge[eid][2][8] or 0.0):
            by_edge[eid] = (d_id, travel, counts, method)
    out_edges = {
        str(eid): dict(
            section=f"jartic:{d_id}",
            unit=f"jartic:{d_id}",
            direction=travel or "?",
            method=method,
            counts=[None if (v is None or v != v) else round(float(v), 1) for v in counts],
        )  # fmt: skip
        for eid, (d_id, travel, counts, method) in by_edge.items()
    }
    (C.PROCESSED / "detector_counts.json").write_text(
        json.dumps(dict(hours=list(range(24)), edges=out_edges), ensure_ascii=False),
        encoding="utf-8",
    )

    # センサスと同じ Edge: 朝 8 時の比(感知器 / センサス)
    cen = json.loads((C.PROCESSED / "census_counts.json").read_text(encoding="utf-8"))["edges"]
    ratios, by_method = [], defaultdict(list)
    for e, v in out_edges.items():
        if e in cen and v["counts"][8] and cen[e]["counts"][8]:
            q = v["counts"][8] / cen[e]["counts"][8]
            ratios.append(q)
            by_method[v["method"]].append(q)
    rep = dict(
        n_detectors_sapporo=len(det),
        located_by=dict(how),
        n_located=len(located),
        n_edges_with_detector=len(out_edges),
        n_duplicates_on_same_edge=len(located) - len(out_edges),
        address_vs_typeC_distance_m=(
            dict(
                n=len(addr_check),
                median=round(float(np.median(addr_check)), 1),
                p90=round(float(np.percentile(addr_check, 90)), 1),
            )
            if addr_check
            else None
        ),
        census_same_edge_8h=(
            dict(
                n=len(ratios),
                median_ratio=round(float(np.median(ratios)), 2),
                q25=round(float(np.percentile(ratios, 25)), 2),
                q75=round(float(np.percentile(ratios, 75)), 2),
            )
            if ratios
            else None
        ),
        census_same_edge_8h_by_method={
            k: dict(
                n=len(v),
                median_ratio=round(float(np.median(v)), 2),
                within_30pct=round(float(np.mean([0.7 <= q <= 1.3 for q in v])), 2),
            )
            for k, v in sorted(by_method.items())
        },
        samples_per_hour_top=getattr(det, "attrs", {}).get("samples_per_hour"),
        note="進行方向 = 地点名称の末尾 2 文字の 2 文字目(格子の向き)。区域外の感知器は落ちる",
    )
    (C.REPORTS / "42_jartic_detectors.json").write_text(
        json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(rep, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
