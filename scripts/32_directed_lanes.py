"""単方向エッジ化と車線・速度の割り当て(Phase 1-③)。

入力:
  data/processed/network_conflated.gpkg  (31_conflate)
  data/interim/census.gpkg               センサス区間(車線数・規制速度)
出力:
  data/processed/edges.gpkg   layer=edges(進行方向順ジオメトリ)
  reports/32_directed.json    Edge 数・情報源別内訳・assumed の残数
"""

import json
import sys
from collections import Counter
from pathlib import Path

import geopandas as gpd
from shapely.geometry import LineString

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from sapporo_sim import config as C
from sapporo_sim.network import lanes as L
from sapporo_sim.network.directed import build_edges


def main() -> None:
    links = gpd.read_file(C.PROCESSED / "network_conflated.gpkg", layer="links")
    census = gpd.read_file(C.INTERIM / "census.gpkg", layer="sections")
    census_by_id = {str(r["section_id"]): r for _, r in census.iterrows()}

    recs = []
    for _, row in links.iterrows():
        recs.append(
            dict(
                a=int(row["a"]),
                b=int(row["b"]),
                geometry=list(row.geometry.coords),
                length=float(row["length_m"]),
                oneway=row["oneway"] or "",
                state=row["state"],
                layer=row["layer"],
                category=row["category"],
                width=row["width"],
                speed_kmh=int(row["speed_kmh"] or 0),
                census_id=str(row["census_id"] or ""),
                ksj_ids=row["ksj_ids"],
            )
        )
    edges = build_edges(recs)
    for e in edges:
        cen = census_by_id.get(e.attrs.get("census_id", ""))
        cen_d = dict(cen) if cen is not None else None
        L.assign_lanes(e, cen_d)
        L.assign_speed(e, cen_d)

    # --- 保存 ---
    out = C.PROCESSED / "edges.gpkg"
    cols = dict(
        eid=[e.eid for e in edges],
        frm=[e.frm for e in edges],
        to=[e.to for e in edges],
        length_m=[round(e.length, 2) for e in edges],
        linked_edge=[-1 if e.linked_edge is None else e.linked_edge for e in edges],
        oneway_source=[e.oneway_source for e in edges],
    )
    for key in (
        "state",
        "category",
        "width",
        "census_id",
        "n_lanes",
        "n_sublanes",
        "carriageway_m",
        "lane_source",
        "right_turn_lane",
        "rt_lane_source",
        "speed_kmh",
        "speed_source",
        "ksj_ids",
    ):
        cols[key] = [e.attrs.get(key) for e in edges]
    gpd.GeoDataFrame(
        cols, geometry=[LineString(e.geometry) for e in edges], crs=C.CRS_PROJ
    ).to_file(out, layer="edges", driver="GPKG")

    rep = dict(
        n_links=len(recs),
        n_edges=len(edges),
        n_oneway_edges=sum(1 for e in edges if e.linked_edge is None),
        oneway_source=dict(Counter(e.oneway_source for e in edges)),
        lane_source=dict(Counter(e.attrs["lane_source"] for e in edges)),
        n_lanes_hist={
            str(k): v for k, v in sorted(Counter(e.attrs["n_lanes"] for e in edges).items())
        },
        n_sublanes_hist={
            str(k): v for k, v in sorted(Counter(e.attrs["n_sublanes"] for e in edges).items())
        },
        speed_source=dict(Counter(e.attrs["speed_source"] for e in edges)),
        speed_hist={
            str(k): v for k, v in sorted(Counter(e.attrs["speed_kmh"] for e in edges).items())
        },
        rt_lane_source=dict(Counter(e.attrs["rt_lane_source"] for e in edges)),
        assumed_remaining=dict(
            oneway=sum(1 for e in edges if e.oneway_source.startswith("assumed")),
            lanes=sum(1 for e in edges if e.attrs["lane_source"] == "width_assumed"),
            right_turn_lane=sum(1 for e in edges if e.attrs["rt_lane_source"] == "assumed"),
            speed=sum(1 for e in edges if e.attrs["speed_source"] == "assumed"),
            note="手入力データ(issue #1)が届いたら manual_ortho で置き換える",
        ),
    )
    (C.REPORTS / "32_directed.json").write_text(
        json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(rep, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
