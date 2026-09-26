"""センサス時間帯別交通量を方向別 Edge の観測交通量にする(docs/sumo-design.md §13.5、C3a)。

上り/下りの向きは区間の起点側・終点側の隣接区間から決める(calib/observations.py)。
需要推定(routeSampler)の制約と、照合(断面交通量)の観測値になる。

- 対象: 層[2]の Edge のうち census_id を持つもの(区間が時間帯別の観測を持つ場合)
- 観測の単位(調査単位区間 = 時間帯別交通量のキー)も残す。1 つの観測を複数の区間が
  共有するので、ホールドアウトは区間ではなく単位で分ける
- 検査: 一方通行の区間で、決めた向きが層[2]の一方通行 Edge(JARTIC)と合うか

入力:  data/processed/edges.gpkg、data/raw/census/(変換済み GeoParquet・jikantai)
出力:  data/processed/census_counts.json
       reports/41_census_counts.json
"""

import json
import sys
from collections import Counter
from pathlib import Path

import geopandas as gpd
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim import config as C
from jp_sumo_traffic_sim.calib import observations as O

SRC = C.RAW / "census" / "traffic_census_2021_converted.parquet"
COLS = [
    "交通調査基本区間番号",
    "一方通行フラグ",
    "起点側／交通調査基本区間番号",
    "終点側／交通調査基本区間番号",
    "交通量／都道府県指定市コード",
    "交通量／調査単位区間番号",
    "geometry",
]


def _norm(v) -> str:
    s = str(v).strip()
    return s[:-2] if s.endswith(".0") else s


def main() -> None:
    edges = gpd.read_file(C.PROCESSED / "edges.gpkg", layer="edges")
    edges = edges[edges["census_id"].fillna("") != ""]
    units = json.loads(
        (C.RAW / "census" / "jikantai" / f"{C.CASE['census']['pref_code']}.json").read_text(
            encoding="utf-8"
        )
    )["units"]

    cen = gpd.read_parquet(SRC, columns=COLS).to_crs(C.CRS_PROJ)
    cen["sid"] = cen["交通調査基本区間番号"].astype(str)
    geom_of = cen.groupby("sid").geometry.apply(lambda s: s.union_all())
    first = cen.drop_duplicates("sid").set_index("sid")

    sections: dict[str, dict] = {}
    for sid in sorted(set(edges["census_id"])):
        if sid not in first.index:
            continue
        r = first.loc[sid]
        a, b = _norm(r["交通量／都道府県指定市コード"]), _norm(r["交通量／調査単位区間番号"])
        key = f"{a}_{b}"
        st, en = str(r["起点側／交通調査基本区間番号"]), str(r["終点側／交通調査基本区間番号"])
        dv, how = O.down_vector(
            geom_of[sid], geom_of.get(st) if st in geom_of.index else None,
            geom_of.get(en) if en in geom_of.index else None,
        )  # fmt: skip
        sections[sid] = dict(
            unit=key if key in units else None,
            dv=dv,
            how=how,
            oneway_flag=int(r["一方通行フラグ"] or 0),
        )

    out_edges: dict[str, dict] = {}
    flag_check = Counter()
    for _, e in edges.iterrows():
        sec = sections.get(e["census_id"])
        if not sec or not sec["unit"]:
            continue
        cs = list(e.geometry.coords)
        ev = np.subtract(cs[-1], cs[0])
        counts, direction = O.edge_counts(ev, sec["dv"], units[sec["unit"]])
        out_edges[str(int(e["eid"]))] = dict(
            section=e["census_id"], unit=sec["unit"], direction=direction, counts=counts
        )
        # 一方通行の区間: 1 = 下りのみ、2 = 上りのみ。JARTIC の一方通行 Edge の向きと照合
        if (
            sec["oneway_flag"] in (1, 2)
            and e["oneway_source"] == "jartic"
            and sec["dv"] is not None
        ):
            want = "down" if sec["oneway_flag"] == 1 else "up"
            flag_check["agree" if direction == want else "disagree"] += 1

    C.PROCESSED.mkdir(parents=True, exist_ok=True)
    (C.PROCESSED / "census_counts.json").write_text(
        json.dumps(dict(hours=list(range(24)), edges=out_edges), ensure_ascii=False),
        encoding="utf-8",
    )

    used_units = {v["unit"] for v in out_edges.values()}
    obs_hours = Counter(
        sum(1 for x in O.hourly_direction(units[u], "up") if x is not None) for u in used_units
    )
    rep = dict(
        n_sections_with_edges=len(sections),
        n_sections_with_hourly=sum(1 for s in sections.values() if s["unit"]),
        direction_resolved_by=dict(Counter(s["how"] for s in sections.values())),
        n_edges_with_counts=len(out_edges),
        edges_by_direction=dict(Counter(v["direction"] for v in out_edges.values())),
        n_units=len(used_units),
        units_by_observed_hours={str(k): v for k, v in sorted(obs_hours.items())},
        oneway_flag_check=dict(flag_check),
        note="下り = 起点側 → 終点側(隣接区間から決める)。'avg' は向きが決まらず上下平均",
    )
    (C.REPORTS / "41_census_counts.json").write_text(
        json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(rep, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
