"""JARTIC 交差点制御情報(typeC)の共通処理。

scripts/12(信号計画)と scripts/42(車両感知器の位置の特定)で共用する。

- 定義 CSV: 交差点ごとの流入・流出リンク(DRM の 2 次メッシュ・リンク区分・リンク番号)と
  現示ごとの通行権
- 交差点の座標は交差点制御情報に無い。日本交通管理技術協会の交差点位置情報ページから
  ``<option value="交差点番号" lon=".." lat="..">`` を拾う
"""

from __future__ import annotations

import csv
import io
import json
import re
import urllib.request
import zipfile
from pathlib import Path

csv.field_size_limit(2**31 - 1)  # sys.maxsize は Windows の C long を超える

POSITION_URL = "https://www.tmt.or.jp/research/index10_1_1.html"
OPT_RE = re.compile(r'<option value="(\d+)" lon="([\d.]+)" lat="([\d.]+)"')
UA = {"User-Agent": "jp-sumo-traffic-sim/0.1"}


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


def fetch_positions(cache: Path) -> dict:
    """交差点番号 → (lon, lat)。cache があればそれを使う。"""
    if cache.exists():
        return {int(k): v for k, v in json.loads(cache.read_text(encoding="utf-8")).items()}
    req = urllib.request.Request(POSITION_URL, headers=UA)
    with urllib.request.urlopen(req, timeout=90) as r:  # noqa: S310
        html = r.read().decode("utf-8", errors="replace")
    pos = {int(i): [float(lon), float(lat)] for i, lon, lat in OPT_RE.findall(html)}
    if not pos:
        raise SystemExit("交差点位置情報を抽出できなかった。ページ構造が変わった可能性")
    cache.parent.mkdir(parents=True, exist_ok=True)
    cache.write_text(
        json.dumps({str(k): v for k, v in pos.items()}, ensure_ascii=False), encoding="utf-8"
    )
    return pos


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


def parse_definition(zp: Path, source_code: str) -> dict:
    """交差点番号 → 流入/流出リンクと現示ごとの通行権。"""
    out = {}
    with zipfile.ZipFile(zp) as z:
        for info, _ in member_csv(z, "定義"):
            with z.open(info) as f:
                reader = csv.reader(io.TextIOWrapper(f, encoding="cp932", errors="replace"))
                head = next(reader)
                ci = {c: k for k, c in enumerate(head)}
                for row in reader:
                    if len(row) < len(head) or row[ci["情報源コード"]] != source_code:
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
