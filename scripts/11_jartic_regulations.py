"""JARTIC 交通規制情報(北海道)を対象区域で絞り、種別ごとの件数を数える。

Phase 0 の判断ポイント「一方通行の本数と一時停止の密度が実用に足るか」に答える。
足りなければ手入力に切り替える。

フォーマットの要点(実データで確認):
  - 全項目ダブルクォート囲み、cp932、170列
  - 座標は「規制場所の経度緯度」に `経度 緯度;経度 緯度;...` で入る
    (点と点はセミコロン、経度と緯度はスペース区切り)
  - 座標フィールドは csv の既定上限 131,072 バイトを超えることがある
  - 「点・線・面コード」と実際の点数は一致しないことがある。点数を優先する
  - **一方通行の向きは頂点順しかない**(方向欄はすべて空。実データで確認)

出力: data/interim/regulations.gpkg (layer=point/line/area)
      reports/11_regulations.json
"""

import csv
import io
import json
import sys
import zipfile
from collections import Counter, defaultdict
from pathlib import Path

import geopandas as gpd
from shapely.geometry import LineString, Point, Polygon

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from sapporo_sim import config as C

csv.field_size_limit(2**31 - 1)  # sys.maxsize は Windows の C long を超える

# シミュレーションに効く規制種別(共通規制種別コード)
SIM_RELEVANT = {
    "11": "一方通行",
    "63": "一時停止",
    "112": "最高速度",
    "114": "最高速度(区域)",
    "12": "指定方向外進行禁止",
    "51": "転回禁止",
    "92": "停止線",
    "93": "二段停止線",
    "20": "車両通行帯",
    "98": "信号機",
    "5": "車両通行止め",
    "17": "追越しのためのはみ出し通行禁止",
}
KEEP_COLS = [
    "共通規制種別コード",
    "県別規制種別名称",
    "点・線・面コード",
    "警察署コード",
    "ユニークキー",
    "路線名(代表)",
    "交差点名称(踏切名含む)",
    "速度",
    "ゾーン30・ゾーン30プラス指定コード",
    "車両通行帯数",
    "距離・延長",
    "規制時間1_開始",
    "規制時間1_終了",
    "対象車両コード1_A",
    "指定・禁止方向の別コード",
    "進入方向(文字)",
    "禁止する方向(文字)",
    "指定する方向(文字)",
    "停止線本数",
    "信号の有無コード",
    "データ更新日",
]
COL_ALIAS = {
    "共通規制種別コード": "code",
    "県別規制種別名称": "kind",
    "点・線・面コード": "shape",
    "警察署コード": "police",
    "ユニークキー": "uid",
    "路線名(代表)": "route",
    "交差点名称(踏切名含む)": "crossing",
    "速度": "speed",
    "ゾーン30・ゾーン30プラス指定コード": "zone30",
    "車両通行帯数": "n_lanes",
    "距離・延長": "length",
    "規制時間1_開始": "t_from",
    "規制時間1_終了": "t_to",
    "対象車両コード1_A": "veh",
    "指定・禁止方向の別コード": "dir_kind",
    "進入方向(文字)": "dir_in",
    "禁止する方向(文字)": "dir_deny",
    "指定する方向(文字)": "dir_allow",
    "停止線本数": "n_stoplines",
    "信号の有無コード": "has_signal",
    "データ更新日": "updated",
}


def member_csv(z: zipfile.ZipFile):
    for info in z.infolist():
        if info.is_dir():
            continue
        name = info.filename
        if not (info.flag_bits & 0x800):
            try:
                name = name.encode("cp437").decode("cp932")
            except (UnicodeEncodeError, UnicodeDecodeError):
                pass
        if name.lower().endswith(".csv"):
            yield info, name


def parse_points(raw: str) -> list:
    pts = []
    for chunk in raw.split(";"):
        f = chunk.split()
        if len(f) != 2:
            continue
        try:
            lon, lat = float(f[0]), float(f[1])
        except ValueError:
            continue
        if 122.0 <= lon <= 154.0 and 20.0 <= lat <= 46.0:
            pts.append((lon, lat))
    return pts


def main() -> None:
    zips = sorted((C.RAW / "jartic").glob("*/typeD_hokkaido.zip"))
    if not zips:
        raise SystemExit(
            "data/raw/jartic/*/typeD_hokkaido.zip が無い。"
            "scripts/10_fetch_jartic.py を先に実行する。"
        )
    zp = zips[-1]
    ym = zp.parent.name

    cordon = C.cordon_polygon()
    # 流入路の規制も拾うためバッファを取る
    area_proj = cordon.buffer(C.CLIP_BUFFER_M, join_style=2)
    area_ll = gpd.GeoSeries([area_proj], crs=C.CRS_PROJ).to_crs("EPSG:6668").iloc[0]
    cordon_ll = gpd.GeoSeries([cordon], crs=C.CRS_PROJ).to_crs("EPSG:6668").iloc[0]
    bounds = area_ll.bounds  # 先に矩形で弾く(全県121,897件を走査するため)

    rows = {"point": [], "line": [], "area": []}
    total = kept = 0
    kinds_all = Counter()
    kinds_in = Counter()
    shape_of = defaultdict(Counter)

    with zipfile.ZipFile(zp) as z:
        for info, _ in member_csv(z):
            with z.open(info) as f:
                reader = csv.reader(io.TextIOWrapper(f, encoding="cp932", errors="replace"))
                head = next(reader)
                ci = {c: k for k, c in enumerate(head)}
                gcol = ci["規制場所の経度緯度"]
                for row in reader:
                    total += 1
                    if len(row) <= gcol:
                        continue
                    code = row[ci["共通規制種別コード"]]
                    kinds_all[code] += 1
                    pts = parse_points(row[gcol])
                    if not pts:
                        continue
                    # 矩形で粗く弾く
                    if not any(
                        bounds[0] <= x <= bounds[2] and bounds[1] <= y <= bounds[3] for x, y in pts
                    ):
                        continue
                    shape = row[ci["点・線・面コード"]]
                    if len(pts) == 1:
                        geom, kind = Point(pts[0]), "point"
                    elif shape == "3" and len(pts) >= 3:
                        ring = pts + [pts[0]] if pts[0] != pts[-1] else pts
                        geom, kind = Polygon(ring), "area"
                    else:
                        geom, kind = LineString(pts), "line"
                    if not geom.intersects(area_ll):
                        continue
                    props = {
                        COL_ALIAS[c]: (
                            row[ci[c]].strip() if ci.get(c) is not None and ci[c] < len(row) else ""
                        )
                        for c in KEEP_COLS
                    }
                    props["in_cordon"] = geom.intersects(cordon_ll)
                    props["n_points"] = len(pts)
                    props["geometry"] = geom
                    rows[kind].append(props)
                    kept += 1
                    kinds_in[code] += 1
                    shape_of[code][kind] += 1

    out = C.INTERIM / "regulations.gpkg"
    if out.exists():
        out.unlink()
    layer_counts = {}
    for kind, recs in rows.items():
        if not recs:
            continue
        g = gpd.GeoDataFrame(recs, geometry="geometry", crs="EPSG:6668").to_crs(C.CRS_PROJ)
        g.to_file(out, layer=kind, driver="GPKG")
        layer_counts[kind] = dict(n=len(g), n_in_cordon=int(g["in_cordon"].sum()))

    def name_of(code: str) -> str:
        for k in ("point", "line", "area"):
            for r in rows[k]:
                if r["code"] == code:
                    return r["kind"]
        return SIM_RELEVANT.get(code, "")

    sim_table = []
    for code, label in SIM_RELEVANT.items():
        n_pref = kinds_all.get(code, 0)
        n_area = kinds_in.get(code, 0)
        n_cordon = sum(1 for k in rows for r in rows[k] if r["code"] == code and r["in_cordon"])
        sim_table.append(
            dict(
                code=code,
                label=label,
                n_hokkaido=n_pref,
                n_clip=n_area,
                n_cordon=n_cordon,
                available=n_pref > 0,
                geom=dict(shape_of.get(code, {})),
            )
        )

    rep = dict(
        source=zp.relative_to(C.ROOT).as_posix(),  # OS 非依存の区切りで記録
        target_month=ym,
        n_rows_hokkaido=total,
        n_in_clip=kept,
        layers=layer_counts,
        simulation_relevant=sim_table,
        not_provided_by_hokkaido=[t["label"] for t in sim_table if not t["available"]],
        top_kinds_in_cordon=[
            dict(code=c, n=n)
            for c, n in Counter(
                r["code"] for k in rows for r in rows[k] if r["in_cordon"]
            ).most_common(15)
        ],
    )
    (C.REPORTS / "11_regulations.json").write_text(
        json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(
        json.dumps(
            {k: v for k, v in rep.items() if k != "simulation_relevant"},
            ensure_ascii=False,
            indent=2,
        )
    )
    print("\n=== シミュレーションに効く規制 ===")
    print(f"{'コード':>5s} {'種別':<24s} {'北海道':>8s} {'クリップ':>8s} {'区域内':>7s}")
    for t in sim_table:
        mark = "" if t["available"] else "  ← 北海道は提供なし"
        print(
            f"{t['code']:>5s} {t['label']:<24s} {t['n_hokkaido']:>8,} "
            f"{t['n_clip']:>8,} {t['n_cordon']:>7,}{mark}"
        )


if __name__ == "__main__":
    main()
