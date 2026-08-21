# data/

データ本体は**リポジトリに含めない**。取得スクリプトで再現する。

```
data/
├── raw/        取得したままの生データ(改変しない)
├── interim/    中間生成物(クリップ・投影・位相構築の途中)
└── processed/  シミュレーション入力・出力
```

## 取得

```bash
make fetch    # 国土数値情報 N13 (メッシュ 6441/6440)
```

`data/raw/ksj/manifest.json` に取得日時と SHA256 が記録される。

## 手動で取得するもの

| データ | 取得元 | 備考 |
|---|---|---|
| JARTIC 交差点制御情報 | https://www.jartic.or.jp/service/opendata/ | **最新1か月分しか配布されない。使用した月は必ず自前でアーカイブする** |
| JARTIC 交通規制情報 | 同上 | `[都道府県警察名]_YYYYMM_k_2.1.csv` |
| 道路交通センサス | https://www.mlit.go.jp/road/census/r3/ | GeoParquet 化済みの成果物あり(docs/data-sources.md 参照) |
| 交差点位置情報 | 日本交通管理技術協会 | 交差点番号→座標の結合に必要 |

JARTIC の過去月 URL は 404 になる。再現性のために生データを保全すること(docs/design.md 4.7)。
