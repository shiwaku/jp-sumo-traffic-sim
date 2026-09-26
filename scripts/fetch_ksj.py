"""国土数値情報 道路データ (N13) を取得して展開する。

出典: 国土交通省 国土数値情報 (https://nlftp.mlit.go.jp/ksj/) / CC BY 4.0
再現性のため、取得日時と SHA256 を data/raw/ksj/manifest.json に記録する。
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

BASE = "https://nlftp.mlit.go.jp/ksj/gml/data/N13/N13-24"
VINTAGE = "N13-24"  # 2024年度版 (2024年9月時点 / 2026年4月更新)


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main() -> None:
    dest = C.RAW / "ksj"
    dest.mkdir(parents=True, exist_ok=True)
    manifest = {"vintage": VINTAGE, "fetched_at": datetime.now(UTC).isoformat(), "files": {}}

    for mesh in C.KSJ_MESHES:
        name = f"{VINTAGE}_{mesh}_SHP.zip"
        zp = dest / name
        if not zp.exists():
            url = f"{BASE}/{name}"
            print(f"downloading {url}")
            urllib.request.urlretrieve(url, zp)  # noqa: S310
        else:
            print(f"cached {zp.name}")
        with zipfile.ZipFile(zp) as z:
            z.extractall(dest / f"{VINTAGE}_{mesh}_SHP")
        manifest["files"][name] = dict(bytes=zp.stat().st_size, sha256=sha256(zp))

    (dest / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
