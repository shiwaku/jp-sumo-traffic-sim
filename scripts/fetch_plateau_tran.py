"""PLATEAU の交通(道路)モデル(CityGML)を取得する(docs/sumo-design.md §13.4、D1b)。

配布は全地物をまとめた ZIP(札幌市 2020 年度 v4 は 2.7 GB)なので、HTTP の範囲指定で
ZIP の目次だけを読み、道路(udx/tran)と、属性の意味を示すコードリスト(codelists)だけを
取り出す。対象区域に掛かる 3 次メッシュ(1 km)のファイルに絞る。

出典: 国土交通省 Project PLATEAU 3D都市モデル(札幌市 2020 年度)/ CC BY 4.0
      https://www.geospatial.jp/ckan/dataset/plateau-01100-sapporo-shi-2020

出力: data/raw/plateau/{zip 名}/udx/tran/*.gml、codelists/*
      data/raw/plateau/manifest.json
"""

import io
import json
import re
import sys
import urllib.request
import zipfile
from datetime import UTC, datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim import config as C
from jp_sumo_traffic_sim.mesh import mesh3_code

URL = (
    "https://assets.cms.plateau.reearth.io/assets/be/3b8cfb-5459-4f9d-b08c-fb4ab72fbdbd/"
    "01100_sapporo-shi_city_2020_citygml_7_op.zip"
)
UA = {"User-Agent": "jp-sumo-traffic-sim/0.1"}
BLOCK = 1 << 20


class HttpRangeFile(io.RawIOBase):
    """HTTP の範囲指定で読むファイル風オブジェクト(zipfile 用)。読んだブロックはキャッシュする。"""

    def __init__(self, url: str):
        self.url = url
        req = urllib.request.Request(url, method="HEAD", headers=UA)
        with urllib.request.urlopen(req, timeout=60) as r:  # noqa: S310
            self.size = int(r.headers["Content-Length"])
        self.pos = 0
        self.cache: dict[int, bytes] = {}
        self.n_requests = 0

    def seekable(self):
        return True

    def readable(self):
        return True

    def tell(self):
        return self.pos

    def seek(self, off, whence=0):
        self.pos = {0: off, 1: self.pos + off, 2: self.size + off}[whence]
        return self.pos

    def _block(self, i: int) -> bytes:
        if i not in self.cache:
            a = i * BLOCK
            b = min(self.size, a + BLOCK) - 1
            req = urllib.request.Request(self.url, headers={**UA, "Range": f"bytes={a}-{b}"})
            with urllib.request.urlopen(req, timeout=120) as r:  # noqa: S310
                self.cache[i] = r.read()
            self.n_requests += 1
        return self.cache[i]

    def read(self, n=-1):
        if n is None or n < 0:
            n = self.size - self.pos
        out = bytearray()
        while n > 0 and self.pos < self.size:
            i, off = divmod(self.pos, BLOCK)
            chunk = self._block(i)[off : off + n]
            out += chunk
            self.pos += len(chunk)
            n -= len(chunk)
        return bytes(out)


def region_meshes() -> set[str]:
    """対象区域に掛かる 3 次メッシュ(8 桁)。

    整数の格子番号で数える(緯度経度を小数で足し進めると取りこぼす)。
    """
    import math

    import geopandas as gpd
    from shapely.geometry import box

    reg = gpd.GeoSeries([C.region_polygon()], crs=C.CRS_PROJ).to_crs("EPSG:6668").iloc[0]
    x0, y0, x1, y1 = reg.bounds
    out = set()
    for i in range(math.floor(y0 * 120), math.floor(y1 * 120) + 1):
        for j in range(math.floor(x0 * 80), math.floor(x1 * 80) + 1):
            if box(j / 80, i / 120, (j + 1) / 80, (i + 1) / 120).intersects(reg):
                out.add(mesh3_code(i, j))
    return out


def main() -> None:
    dest = C.RAW / "plateau" / Path(URL).stem
    dest.mkdir(parents=True, exist_ok=True)
    meshes = region_meshes()
    f = HttpRangeFile(URL)
    z = zipfile.ZipFile(f)
    names = z.namelist()
    tran = [n for n in names if n.startswith("udx/tran/") and n.endswith(".gml")]
    want = [n for n in tran if (m := re.match(r"(\d{8})_", Path(n).name)) and m.group(1) in meshes]
    code = [n for n in names if n.startswith("codelists/") and not n.endswith("/")]
    for n in want + code:
        out = dest / n
        if out.exists():
            continue
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_bytes(z.read(n))
    manifest = dict(
        url=URL,
        fetched_at=datetime.now(UTC).isoformat(),
        zip_bytes=f.size,
        n_tran_files_in_zip=len(tran),
        n_region_meshes=len(meshes),
        n_tran_files_fetched=len(want),
        n_codelists=len(code),
        http_range_requests=f.n_requests,
        license="CC BY 4.0(国土交通省 Project PLATEAU)",
    )
    (C.RAW / "plateau" / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
