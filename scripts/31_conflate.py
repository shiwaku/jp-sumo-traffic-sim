"""JARTIC 規制・信号・センサスを実ネットワークへ空間結合する(Phase 1-②)。

入力:
  data/processed/network.gpkg      位相構築済みネットワーク(30_build_topology)
  data/interim/regulations.gpkg    JARTIC 交通規制(11_jartic_regulations)
  data/processed/signal_plans.json JARTIC 交差点制御(12_jartic_signals)
  data/interim/census.gpkg         センサス区間(14_census_extract)
出力:
  data/processed/network_conflated.gpkg  layer=nodes / links / stops
  reports/31_join_report.json            段ごとの結合率
"""

import json
import sys
from pathlib import Path

import geopandas as gpd
import numpy as np
from pyproj import Transformer
from shapely.geometry import Point

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim import config as C
from jp_sumo_traffic_sim.network import conflation as CF


def line_coords(geom) -> list[list]:
    parts = geom.geoms if geom.geom_type == "MultiLineString" else [geom]
    return [list(p.coords) for p in parts]


def main() -> None:
    nodes = gpd.read_file(C.PROCESSED / "network.gpkg", layer="nodes")
    links = gpd.read_file(C.PROCESSED / "network.gpkg", layer="links")
    reg_pt = gpd.read_file(C.INTERIM / "regulations.gpkg", layer="point")
    reg_ln = gpd.read_file(C.INTERIM / "regulations.gpkg", layer="line")
    census = gpd.read_file(C.INTERIM / "census.gpkg", layer="sections")
    plans = json.loads((C.PROCESSED / "signal_plans.json").read_text(encoding="utf-8"))

    node_xy = np.array([(g.x, g.y) for g in nodes.geometry])
    pm = CF.PointMatcher(list(nodes["nid"]), node_xy)
    link_coords = [list(g.coords) for g in links.geometry]
    incident: dict[int, list] = {}
    for i, row in links.iterrows():
        for n in (row["a"], row["b"]):
            incident.setdefault(n, []).append((i, link_coords[i]))

    report = {"stages": []}

    # --- 面: ゾーン30(コードン内は0件と確認済み。件数だけ記録) ---
    n_zone = int((reg_ln["zone30"].astype(str) != "").sum())
    report["stages"].append(dict(stage="polygon_zone30", n_source=n_zone, n_matched=0))

    # --- 点: 信号機(コード98) → 最近傍ノード ---
    sig = reg_pt[(reg_pt["code"].astype(str) == "98") & reg_pt["in_cordon"]]
    nodes["has_signal"] = False
    n_hit = 0
    for _, row in sig.iterrows():
        nid = pm.nearest_node(row.geometry.x, row.geometry.y)
        if nid is not None:
            nodes.loc[nodes["nid"] == nid, "has_signal"] = True
            n_hit += 1
    report["stages"].append(
        dict(
            stage="point_signal",
            n_source=len(sig),
            n_matched=n_hit,
            match_rate=round(n_hit / max(1, len(sig)), 3),
            snap_tol_m=CF.PT_SNAP_M,
            n_signal_nodes=int(nodes["has_signal"].sum()),
        )
    )

    # --- 点: 一時停止(コード63) → ノード + 流入リンク ---
    stops = reg_pt[(reg_pt["code"].astype(str) == "63") & reg_pt["in_cordon"]]
    stop_rows = []
    for _, row in stops.iterrows():
        x, y = row.geometry.x, row.geometry.y
        nid = pm.nearest_node(x, y)
        if nid is None:
            continue
        li = CF.PointMatcher.inflow_link(x, y, incident.get(nid, []))
        if li is None:
            continue
        stop_rows.append(dict(uid=row["uid"], node=int(nid), link_index=int(li), x=x, y=y))
    report["stages"].append(
        dict(
            stage="point_stop",
            n_source=len(stops),
            n_matched=len(stop_rows),
            match_rate=round(len(stop_rows) / max(1, len(stops)), 3),
            snap_tol_m=CF.PT_SNAP_M,
        )
    )

    # --- 点: 交差点制御情報(typeC) → 最近傍ノード(40m) ---
    tf = Transformer.from_crs("EPSG:6668", C.CRS_PROJ, always_xy=True)
    nodes["signal_uid"] = ""
    n_hit = 0
    for uid, plan in plans.items():
        x, y = tf.transform(plan["lon"], plan["lat"])
        nid = pm.nearest_node(x, y, tol=40.0)
        if nid is not None:
            nodes.loc[nodes["nid"] == nid, "signal_uid"] = uid
            nodes.loc[nodes["nid"] == nid, "has_signal"] = True
            n_hit += 1
    n_in_cordon_plans = sum(1 for _ in plans)  # signal_plans はコードン内のみ
    report["stages"].append(
        dict(
            stage="point_signal_plan",
            n_source=n_in_cordon_plans,
            n_matched=n_hit,
            match_rate=round(n_hit / max(1, n_in_cordon_plans), 3),
            snap_tol_m=40.0,
        )
    )

    # --- 線: 一方通行(コード11) → リンク + 方向 ---
    ow = reg_ln[(reg_ln["code"].astype(str) == "11") & reg_ln["in_cordon"]].reset_index()
    ow_lines = [line_coords(g) for g in ow.geometry]
    lm = CF.LineMatcher(ow_lines)
    links["oneway"] = ""  # "F"=座標順方向のみ可 / "R"=逆方向のみ可
    matched_feats = set()
    for i, coords in enumerate(link_coords):
        got = lm.best(coords)
        if got is None:
            continue
        fi, sign = got
        matched_feats.add(fi)
        # 頂点順は通行方向の逆(15_verify_oneway) → 符号を反転して通行方向にする
        links.loc[i, "oneway"] = "R" if sign > 0 else "F"
    report["stages"].append(
        dict(
            stage="line_oneway",
            n_source=len(ow),
            n_matched=len(matched_feats),
            match_rate=round(len(matched_feats) / max(1, len(ow)), 3),
            n_links_oneway=int((links["oneway"] != "").sum()),
            buffer_m=CF.LINE_BUF_M,
            dot_th=CF.DOT_TH,
            vertex_order="通行方向の逆(reports/15_oneway_check.json で OSM 照合済み)",
        )
    )

    # --- 線: 最高速度(コード112) → リンク ---
    sp = reg_ln[(reg_ln["code"].astype(str) == "112") & reg_ln["in_cordon"]].reset_index()
    lm = CF.LineMatcher([line_coords(g) for g in sp.geometry])
    links["speed_kmh"] = 0
    matched_feats = set()
    for i, coords in enumerate(link_coords):
        got = lm.best(coords)
        if got is None:
            continue
        fi, _ = got
        matched_feats.add(fi)
        links.loc[i, "speed_kmh"] = int(sp.loc[fi, "speed"] or 0)
    report["stages"].append(
        dict(
            stage="line_speed",
            n_source=len(sp),
            n_matched=len(matched_feats),
            match_rate=round(len(matched_feats) / max(1, len(sp)), 3),
            n_links_with_speed=int((links["speed_kmh"] > 0).sum()),
        )
    )

    # --- 線: センサス区間 → リンク(車線数・右折レーン等は Phase 1-③ で使う) ---
    cs = census[census["in_cordon"]].reset_index()
    lm = CF.LineMatcher([line_coords(g) for g in cs.geometry], buf_m=25.0)
    links["census_id"] = ""
    matched_feats = set()
    for i, coords in enumerate(link_coords):
        got = lm.best(coords)
        if got is None:
            continue
        fi, _ = got
        matched_feats.add(fi)
        links.loc[i, "census_id"] = str(cs.loc[fi, "section_id"])
    # 第2パス: 両側分離の幹線(創成川通・石山通など)はセンサス中心線が
    # 車道リンクから25m超離れる。幅員区分3以上(5.5m〜)のリンクに限定して
    # バッファを45mへ広げる(幅の狭い仲通りへの誤マッチを防ぐ。
    # なお両側分離幹線の車道は KSJ 上 category 3 が多く、category では絞れない)
    lm2 = CF.LineMatcher([line_coords(g) for g in cs.geometry], buf_m=45.0)
    for i, coords in enumerate(link_coords):
        if links.loc[i, "census_id"] != "" or str(links.loc[i, "width"]) not in ("3", "4", "5"):
            continue
        got = lm2.best(coords)
        if got is None:
            continue
        fi, _ = got
        matched_feats.add(fi)
        links.loc[i, "census_id"] = str(cs.loc[fi, "section_id"])
    report["stages"].append(
        dict(
            stage="line_census",
            n_source=len(cs),
            n_matched=len(matched_feats),
            match_rate=round(len(matched_feats) / max(1, len(cs)), 3),
            n_links_with_census=int((links["census_id"] != "").sum()),
            buffer_m=25.0,
            second_pass="幅員区分3以上のみ buffer 45m",
        )
    )

    # --- 保存 ---
    out = C.PROCESSED / "network_conflated.gpkg"
    nodes.to_file(out, layer="nodes", driver="GPKG")
    links.to_file(out, layer="links", driver="GPKG")
    if stop_rows:
        gpd.GeoDataFrame(
            stop_rows, geometry=[Point(r["x"], r["y"]) for r in stop_rows], crs=C.CRS_PROJ
        ).drop(columns=["x", "y"]).to_file(out, layer="stops", driver="GPKG")

    (C.REPORTS / "31_join_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
