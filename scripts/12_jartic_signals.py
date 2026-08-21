"""JARTIC 交差点制御情報(札幌 = 情報源コード 3001)から信号計画を作る。

参考実装 https://github.com/shiwaku/jartic-traffic-signal-cycle-converter (Apache-2.0)
は時間帯別の**平均サイクル長**のみを出力する。本プロジェクトは現示秒数が必要なため
スプリットも保持し、さらに「定義」CSV から流入・流出リンクと
**現示ごとの通行権**を取り出す。

## 実データで分かったこと

- 制御CSV: 時刻, 情報源コード, 交差点番号, サイクル長, スプリット＃1〜6, リンクバージョン
  スプリットは**百分率**(合計100)。現示秒数 = サイクル長 × スプリット ÷ 100
- 定義CSV(150列)には次が入る。設計資料が「含まれない」としていたが実際は含まれる:
    流入リンク数 / 流出リンク数
    流入・流出リンクの (2次メッシュコード, リンク区分, リンク番号) 各8本まで
    **スプリット＃N通行権_流入リンク＃M / 流出リンク＃M** (0/1)
  → 現示と転回の対応(Movement.signal_group)がオープンデータから決まる
- 座標は交差点制御情報に無い。日本交通管理技術協会の交差点位置情報ページから
  `<option value="交差点番号" lon=".." lat="..">` を拾って結合する

出力: data/interim/signals.gpkg (layer=intersections)
      data/processed/signal_plans.json
      reports/12_signals.json
"""

import csv
import io
import json
import re
import sys
import urllib.request
import zipfile
from collections import defaultdict
from pathlib import Path

import geopandas as gpd
import numpy as np
from shapely.geometry import Point

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from sapporo_sim import config as C

csv.field_size_limit(sys.maxsize)

SOURCE_CODE = "3001"  # 北海道(札幌方面)。函館3002 / 旭川3003 / 釧路3004 / 北見3005
POSITION_URL = "https://www.tmt.or.jp/research/index10_1_1.html"
POSITION_CACHE = C.INTERIM / "tmt_intersections_3001.json"
OPT_RE = re.compile(r'<option value="(\d+)" lon="([\d.]+)" lat="([\d.]+)"')
UA = {"User-Agent": "sapporo-micro-traffic-sim/0.1"}

# 黄・全赤の内訳は含まれない。各現示は青+黄+全赤の合計なので標準値を差し引く。
# 現地観測で1-2交差点確認して校正すること(docs/design.md 4.2)。
YELLOW_S = 3.0
ALL_RED_S = 2.0


def member_csv(z: zipfile.ZipFile, keyword: str):
    for info in z.infolist():
        if info.is_dir():
            continue
        name = info.filename
        if not (info.flag_bits & 0x800):
            try:
                name = name.encode("cp437").decode("cp932")
            except (UnicodeEncodeError, UnicodeDecodeError):
                pass
        if keyword in name and name.lower().endswith(".csv"):
            yield info, name


def fetch_positions() -> dict:
    """交差点番号 → (lon, lat)。"""
    if POSITION_CACHE.exists():
        return {
            int(k): v for k, v in json.loads(POSITION_CACHE.read_text(encoding="utf-8")).items()
        }
    req = urllib.request.Request(POSITION_URL, headers=UA)
    with urllib.request.urlopen(req, timeout=90) as r:  # noqa: S310
        html = r.read().decode("utf-8", errors="replace")
    pos = {int(i): [float(lon), float(lat)] for i, lon, lat in OPT_RE.findall(html)}
    if not pos:
        raise SystemExit("交差点位置情報を抽出できなかった。ページ構造が変わった可能性")
    POSITION_CACHE.parent.mkdir(parents=True, exist_ok=True)
    POSITION_CACHE.write_text(
        json.dumps({str(k): v for k, v in pos.items()}, ensure_ascii=False), encoding="utf-8"
    )
    return pos


def parse_control(zp: Path) -> tuple[dict, dict]:
    """(交差点番号, 時) → サイクル長とスプリットの平均。"""
    cyc = defaultdict(lambda: [0.0, 0])
    spl = defaultdict(lambda: [np.zeros(6), 0])
    with zipfile.ZipFile(zp) as z:
        for info, _ in member_csv(z, "制御"):
            with z.open(info) as f:
                reader = csv.reader(io.TextIOWrapper(f, encoding="cp932", errors="replace"))
                next(reader, None)
                for row in reader:
                    if len(row) < 10 or row[1] != SOURCE_CODE or not row[3]:
                        continue
                    try:
                        cycle = int(row[3])
                    except ValueError:
                        continue
                    key = (int(row[2]), row[0][11:13])
                    a = cyc[key]
                    a[0] += cycle
                    a[1] += 1
                    vals = np.zeros(6)
                    for i in range(6):
                        v = row[4 + i]
                        if v:
                            try:
                                vals[i] = float(v)
                            except ValueError:
                                pass
                    b = spl[key]
                    b[0] += vals
                    b[1] += 1
    return cyc, spl


IDX = "１２３４５６７８"


def _links(row: list, ci: dict, kind: str, n: int) -> list:
    """流入/流出リンクの (2次メッシュ, リンク区分, リンク番号) を並べる。"""
    got = []
    for i in range(8):
        j = IDX[i]
        mesh = row[ci[f"{kind}リンク＃{j}定義_２次メッシュコード"]]
        div = row[ci[f"{kind}リンク＃{j}定義_リンク区分"]]
        num = row[ci[f"{kind}リンク＃{j}定義_リンク番号"]]
        if mesh or num:
            got.append({"mesh": mesh, "div": div, "link": num})
    return got[:n] if n else got


def _phases(row: list, ci: dict) -> list:
    """現示(スプリット)ごとに通行権のある流入・流出リンク番号を並べる。"""
    out = []
    for s in range(6):
        sj = IDX[s]
        rin = [row[ci[f"スプリット＃{sj}通行権_流入リンク＃{IDX[k]}"]] for k in range(8)]
        rout = [row[ci[f"スプリット＃{sj}通行権_流出リンク＃{IDX[k]}"]] for k in range(8)]
        if any(v == "1" for v in rin + rout):
            out.append(
                {
                    "split": s + 1,
                    "in": [i + 1 for i, v in enumerate(rin) if v == "1"],
                    "out": [i + 1 for i, v in enumerate(rout) if v == "1"],
                }
            )
    return out


def parse_definition(zp: Path) -> dict:
    """交差点番号 → 流入/流出リンクと現示ごとの通行権。"""
    out = {}
    with zipfile.ZipFile(zp) as z:
        for info, _ in member_csv(z, "定義"):
            with z.open(info) as f:
                reader = csv.reader(io.TextIOWrapper(f, encoding="cp932", errors="replace"))
                head = next(reader)
                ci = {c: k for k, c in enumerate(head)}
                for row in reader:
                    if len(row) < len(head) or row[ci["情報源コード"]] != SOURCE_CODE:
                        continue
                    n_in = int(row[ci["流入リンク数"]] or 0)
                    n_out = int(row[ci["流出リンク数"]] or 0)
                    out[int(row[ci["交差点番号"]])] = {
                        "n_in": n_in,
                        "n_out": n_out,
                        "in_links": _links(row, ci, "流入", n_in),
                        "out_links": _links(row, ci, "流出", n_out),
                        "phases": _phases(row, ci),
                        "link_version": row[ci["リンクバージョン"]],
                    }
    return out


def main() -> None:
    zips = sorted((C.RAW / "jartic").glob("*/typeC_sapporo_*.zip"))
    if not zips:
        raise SystemExit("typeC_sapporo_*.zip が無い。scripts/10_fetch_jartic.py を先に実行する。")
    zp = zips[-1]
    ym = zp.parent.name

    pos = fetch_positions()
    cyc, spl = parse_control(zp)
    defs = parse_definition(zp)

    ids_control = {k[0] for k in cyc}
    ids_pos = set(pos)
    ids_def = set(defs)

    cordon = C.cordon_polygon()
    cordon_ll = gpd.GeoSeries([cordon], crs=C.CRS_PROJ).to_crs("EPSG:6668").iloc[0]

    recs, plans = [], {}
    for iid in sorted(ids_control):
        if iid not in pos:
            continue
        lon, lat = pos[iid]
        pt = Point(lon, lat)
        in_cordon = pt.within(cordon_ll)
        hours = sorted(h for (i, h) in cyc if i == iid)
        by_hour = {}
        for h in hours:
            total, n = cyc[(iid, h)]
            s_sum, s_n = spl[(iid, h)]
            splits = (s_sum / s_n) if s_n else np.zeros(6)
            cycle = total / n
            used = [round(float(v), 2) for v in splits if v > 0]
            greens = [round(cycle * v / 100.0 - YELLOW_S - ALL_RED_S, 2) for v in used]
            by_hour[h] = {
                "cycle_s": round(cycle, 2),
                "samples": n,
                "splits_pct": used,
                "split_sum_pct": round(sum(used), 2),
                "green_s": greens,
                "n_phases": len(used),
            }
        d = defs.get(iid, {})
        recs.append(
            {
                "jartic_id": f"{SOURCE_CODE}-{iid}",
                "intersection": iid,
                "in_cordon": in_cordon,
                "n_hours": len(by_hour),
                "cycle_min_s": min(v["cycle_s"] for v in by_hour.values()) if by_hour else None,
                "cycle_max_s": max(v["cycle_s"] for v in by_hour.values()) if by_hour else None,
                "n_phases": max((v["n_phases"] for v in by_hour.values()), default=0),
                "n_in_links": d.get("n_in"),
                "n_out_links": d.get("n_out"),
                "has_definition": iid in ids_def,
                "geometry": pt,
            }
        )
        if in_cordon:
            plans[f"{SOURCE_CODE}-{iid}"] = {
                "intersection": iid,
                "lon": lon,
                "lat": lat,
                "by_hour": by_hour,
                "definition": d,
                "yellow_s": YELLOW_S,
                "all_red_s": ALL_RED_S,
            }

    g = gpd.GeoDataFrame(recs, geometry="geometry", crs="EPSG:6668").to_crs(C.CRS_PROJ)
    out = C.INTERIM / "signals.gpkg"
    if out.exists():
        out.unlink()
    g.to_file(out, layer="intersections", driver="GPKG")
    C.PROCESSED.mkdir(parents=True, exist_ok=True)
    (C.PROCESSED / "signal_plans.json").write_text(
        json.dumps(plans, ensure_ascii=False, indent=1), encoding="utf-8"
    )

    inner = g[g["in_cordon"]]
    cyc_vals = [r["cycle_max_s"] for r in recs if r["in_cordon"] and r["cycle_max_s"]]
    rep = dict(
        source=str(zp.relative_to(C.ROOT)),
        target_month=ym,
        source_code=SOURCE_CODE,
        n_control=len(ids_control),
        n_position=len(ids_pos),
        n_definition=len(ids_def),
        join_rate=round(len(ids_control & ids_pos) / max(len(ids_control), 1), 4),
        unmatched_control=sorted(ids_control - ids_pos)[:20],
        n_in_cordon=int(len(inner)),
        cordon_cycle_s=dict(
            min=round(float(np.min(cyc_vals)), 1) if cyc_vals else None,
            median=round(float(np.median(cyc_vals)), 1) if cyc_vals else None,
            max=round(float(np.max(cyc_vals)), 1) if cyc_vals else None,
        ),
        cordon_phase_counts=(
            inner["n_phases"].value_counts().sort_index().to_dict() if len(inner) else {}
        ),
        cordon_approach_counts=(
            inner["n_in_links"].value_counts(dropna=False).sort_index().to_dict()
            if len(inner)
            else {}
        ),
        n_with_definition=int(inner["has_definition"].sum()) if len(inner) else 0,
        yellow_s=YELLOW_S,
        all_red_s=ALL_RED_S,
    )
    rep["cordon_phase_counts"] = {str(k): int(v) for k, v in rep["cordon_phase_counts"].items()}
    rep["cordon_approach_counts"] = {
        str(k): int(v) for k, v in rep["cordon_approach_counts"].items()
    }
    (C.REPORTS / "12_signals.json").write_text(
        json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(rep, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
