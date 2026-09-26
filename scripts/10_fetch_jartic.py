"""JARTIC オープンデータのうち、札幌都心に必要な分だけを取得する。

**JARTIC は最新1か月分しか配布しない。** 過去月の URL は404になるため、
取得した生zipとカタログのスナップショットを data/raw/jartic/{年月}/ に残す。
配布URLは月次で変わるので、URL はスクリプトに埋め込まず
公式カタログ (opendata.json) から解決する。

取得対象(全国51ファイル・500MBは落とさない):
  typeC_sapporo   交差点制御情報   サイクル長・スプリット
  typeD_hokkaido  交通規制情報     一方通行・最高速度・一時停止・指定方向外進行禁止
  typeB_sapporo   断面交通量情報   検証用

出典: 公益財団法人 日本道路交通情報センター https://www.jartic.or.jp/service/opendata/
参考実装: https://github.com/shiwaku/jartic-traffic-signal-cycle-converter (Apache-2.0)

出力: data/raw/jartic/{年月}/*.zip, catalog.json, manifest.json
      reports/10_jartic_fetch.json
"""

import argparse
import hashlib
import json
import re
import sys
import urllib.request
import zipfile
from datetime import UTC, datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim import config as C

CATALOG_URL = "https://www.jartic.or.jp/d/opendata/opendata.json"
BASE_URL = "https://www.jartic.or.jp/d/opendata"
UA = {"User-Agent": "jp-sumo-traffic-sim/0.1"}

# (カタログのtype, targetList の id) → 用途。取得対象はケースが決める(case.toml)
WANTED = {(c["type"], c["id"]): c["purpose"] for c in C.CASE["jartic"]["catalog"]}


def fetch(url: str) -> bytes:
    req = urllib.request.Request(url, headers=UA)
    with urllib.request.urlopen(req, timeout=300) as r:  # noqa: S310
        return r.read()


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def member_name(info: zipfile.ZipInfo) -> str:
    """zip内のファイル名を復元する(cp932 で格納されている場合がある)。"""
    if info.flag_bits & 0x800:
        return info.filename
    try:
        return info.filename.encode("cp437").decode("cp932")
    except (UnicodeEncodeError, UnicodeDecodeError):
        return info.filename


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--force", action="store_true", help="既存のzipも再取得する")
    args = p.parse_args()

    catalog = json.loads(fetch(CATALOG_URL).decode("utf-8"))
    entries = {e.get("type"): e for e in catalog}

    # 対象年月はカタログから取る(スクリプトに埋め込まない)
    months = {
        e["type"]: re.sub(r"[^0-9]", "", e.get("targetMonth", ""))
        for e in catalog
        if e.get("targetMonth")
    }
    ym = months.get("typeC") or next(iter(months.values()))
    dest = C.RAW / "jartic" / ym
    dest.mkdir(parents=True, exist_ok=True)
    (dest / "catalog.json").write_text(
        json.dumps(catalog, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    manifest = {
        "fetched_at": datetime.now(UTC).isoformat(),
        "target_month": {k: entries[k].get("targetMonth") for k in entries if k in months},
        "release_day": {k: entries[k].get("releaseDay") for k in entries if k in months},
        "files": {},
    }
    report = {"target_month": ym, "files": []}

    for (typ, tid), purpose in WANTED.items():
        entry = entries.get(typ)
        if not entry:
            print(f"カタログに {typ} が無い")
            continue
        target = next((t for t in entry.get("targetList", []) if t.get("id") == tid), None)
        if not target:
            print(f"カタログの {typ} に id={tid} が無い")
            continue
        name = target["link"].split("/")[-1]
        zp = dest / name
        if args.force or not zp.exists():
            url = BASE_URL + target["link"]
            print(f"downloading {url}")
            zp.write_bytes(fetch(url))
        else:
            print(f"cached {name}")

        with zipfile.ZipFile(zp) as z:
            members = [
                {"name": member_name(i), "size": i.file_size}
                for i in z.infolist()
                if not i.is_dir()
            ]
        manifest["files"][name] = {
            "purpose": purpose,
            "type": typ,
            "catalog_id": tid,
            "url": BASE_URL + target["link"],
            "bytes": zp.stat().st_size,
            "sha256": sha256(zp),
            "members": members,
        }
        report["files"].append(
            {
                "name": name,
                "purpose": purpose,
                "mb": round(zp.stat().st_size / 1e6, 1),
                "members": [f"{m['name']} ({m['size'] / 1e6:.1f}MB)" for m in members],
            }
        )

    (dest / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    (C.REPORTS / "10_jartic_fetch.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print()
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
