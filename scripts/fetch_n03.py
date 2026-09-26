"""国土数値情報 行政区域データ (N03) を取得して展開する(市全域化、docs/sumo-design.md §13.4)。

対象区域(行政界)の切り出しに使う。都道府県単位の配布なので、ケースの
都道府県コード(case.toml の census.pref_code)の分を取る。

出典: 国土交通省 国土数値情報 (https://nlftp.mlit.go.jp/ksj/) / CC BY 4.0
再現性のため、取得日時と SHA256 を data/raw/n03/manifest.json に記録する。
"""

import hashlib
import json
import sys
import urllib.request
import zipfile
from datetime import UTC, datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim import config as C

VINTAGE = "N03-2025"  # 2025年1月1日時点
DATE = "20250101"
BASE = f"https://nlftp.mlit.go.jp/ksj/gml/data/N03/{VINTAGE}"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main() -> None:
    pref = C.CASE["census"]["pref_code"]
    dest = C.RAW / "n03"
    dest.mkdir(parents=True, exist_ok=True)
    name = f"N03-{DATE}_{pref}_GML.zip"
    zp = dest / name
    if not zp.exists():
        url = f"{BASE}/{name}"
        print(f"downloading {url}")
        urllib.request.urlretrieve(url, zp)  # noqa: S310
    else:
        print(f"cached {zp.name}")
    with zipfile.ZipFile(zp) as z:
        z.extractall(dest / name.removesuffix(".zip"))
    manifest = dict(
        vintage=VINTAGE,
        fetched_at=datetime.now(UTC).isoformat(),
        files={name: dict(bytes=zp.stat().st_size, sha256=sha256(zp))},
    )
    (dest / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
