"""道路交通センサス(令和3年度)の変換済みデータを取得する。

出典: 全国道路・街路交通情勢調査(国土交通省)
      公共データ利用規約 (PDL1.0) — 出典表示 + 加工明示が必要
変換済み配布: https://github.com/shiwaku/mlit-road-traffic-census-converter (MIT)
      箇所別基本表を紐づけ済み(結合率100%)、ジオメトリは MultiLineString 統一

取得するもの(PMTiles 686MB は使わない):
  traffic_census_2021_converted.parquet   94MB  GeoParquet 本体
  traffic_census_2021_jikantai.tar.gz    3.9MB  時間帯別交通量 (北海道 = 01.json)
  SHA256SUMS.txt                                整合性検証用

出力: data/raw/census/
      reports/13_census_fetch.json
"""

import hashlib
import json
import sys
import tarfile
import urllib.request
from datetime import UTC, datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim import config as C

TAG = "data-v1"
BASE = f"https://github.com/shiwaku/mlit-road-traffic-census-converter/releases/download/{TAG}"
ASSETS = [
    "traffic_census_2021_converted.parquet",
    "traffic_census_2021_jikantai.tar.gz",
    "SHA256SUMS.txt",
]
UA = {"User-Agent": "jp-sumo-traffic-sim/0.1"}


def sha256(p: Path) -> str:
    h = hashlib.sha256()
    with p.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main() -> None:
    dest = C.RAW / "census"
    dest.mkdir(parents=True, exist_ok=True)
    manifest = {"release_tag": TAG, "fetched_at": datetime.now(UTC).isoformat(), "files": {}}

    for name in ASSETS:
        p = dest / name
        if not p.exists():
            url = f"{BASE}/{name}"
            print(f"downloading {url}")
            req = urllib.request.Request(url, headers=UA)
            with urllib.request.urlopen(req, timeout=900) as r:  # noqa: S310
                p.write_bytes(r.read())
        else:
            print(f"cached {name}")
        manifest["files"][name] = dict(bytes=p.stat().st_size, sha256=sha256(p))

    # 配布元の SHA256SUMS.txt と突き合わせる
    sums = {}
    sf = dest / "SHA256SUMS.txt"
    if sf.exists():
        for line in sf.read_text(encoding="utf-8").splitlines():
            parts = line.split()
            if len(parts) == 2:
                sums[parts[1].lstrip("*")] = parts[0]
    verified = {}
    for name, info in manifest["files"].items():
        expect = sums.get(name)
        if expect:
            verified[name] = expect == info["sha256"]
    manifest["sha256_verified"] = verified
    if any(v is False for v in verified.values()):
        raise SystemExit(f"SHA256 が一致しない: {verified}")

    # 時間帯別交通量はケースの都道府県分({pref_code}.json)だけ展開する
    pref_json = f"/{C.CASE['census']['pref_code']}.json"
    tgz = dest / "traffic_census_2021_jikantai.tar.gz"
    jikantai = dest / "jikantai"
    if tgz.exists():
        jikantai.mkdir(exist_ok=True)
        with tarfile.open(tgz) as t:
            names = [m for m in t.getnames() if m.endswith((pref_json, "index.json"))]
            for m in names:
                member = t.getmember(m)
                member.name = Path(m).name
                t.extract(member, jikantai, filter="data")
        manifest["jikantai_extracted"] = sorted(p.name for p in jikantai.iterdir())

    (dest / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    (C.REPORTS / "13_census_fetch.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
