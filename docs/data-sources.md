# データ出典と利用条件

このリポジトリのコードは MIT ライセンス。**データは含まれない。**
成果物を公開・発表する際は以下の出典表示を行うこと。

## 国土数値情報 道路データ (N13)

- 提供: 国土交通省 https://nlftp.mlit.go.jp/ksj/
- 版: 2024年度版(データ基準 2024年9月時点 / 2026年4月更新)
- 使用メッシュ: 6441(札幌)、6440(南区西端の補助)
- 座標系: EPSG:6668 (JGD2011 地理座標)
- ライセンス: **CC BY 4.0**

表示例:

> 「国土数値情報(道路データ)」(国土交通省)を加工して作成

**注意:** ノード・リンク構造を持たない。位相は自前で構築する(docs/design.md 5.2)。

## 国土数値情報 行政区域データ (N03)

- 提供: 国土交通省 https://nlftp.mlit.go.jp/ksj/
- 版: 2025年(2025年1月1日時点)、北海道分(`N03-20250101_01_GML.zip`)
- 用途: 対象区域(札幌市 10 区の行政界。市区町村コード 01101〜01110)の切り出し
- 座標系: EPSG:6668 (JGD2011 地理座標)
- ライセンス: **CC BY 4.0**

表示例:

> 「国土数値情報(行政区域データ)」(国土交通省)を加工して作成

## JARTIC オープンデータ

- 提供: 公益財団法人 日本道路交通情報センター https://www.jartic.or.jp/service/opendata/
- 使用: 交差点制御情報(サイクル長・スプリット)、交通規制情報(拡張版標準フォーマット k_2.1)、断面交通量情報
- **最新1か月分しか配布されない。** 過去月の URL は 404。使用した月の生データは自前でアーカイブすること。
- フォーマットが3種混在(標準 / k_2.0 / k_2.1)。2025年1月までの公開分は標準フォーマット。

参考実装: https://github.com/shiwaku/jartic-traffic-signal-cycle-converter (Apache-2.0)

## 全国道路・街路交通情勢調査(道路交通センサス)

- 提供: 国土交通省 https://www.mlit.go.jp/road/census/r3/
- 版: 令和3年度(2021年秋季平日)。一般交通量調査の公表は令和5年6月
- 座標系: EPSG:4612
- ライセンス: **公共データ利用規約 (PDL1.0)** — 出典表示 **+ 加工した旨の明示**が必要

表示例:

> 「全国道路・街路交通情勢調査」(国土交通省)を加工して作成

参考実装: https://github.com/shiwaku/mlit-road-traffic-census-converter (MIT)

## 交通量API (国交省 xROAD)

- https://www.jartic-open-traffic.org/
- 直轄国道の方向別交通量を 5分値・1時間値で提供。検証データとして最も粒度が細かい

## OpenStreetMap

- © OpenStreetMap contributors / **ODbL 1.0**
- 用途: 一方通行 conflation の答え合わせ(docs/design.md 5.4)、街路名の参照
- **シミュレーション入力そのものには使わない**(ODbL の派生データ条項を避けるため、検証用途に限定する)

## 日本交通管理技術協会 交差点位置情報

- 交差点番号 → 座標の結合に必要。全国結合率 97.34%
- 断面交通量のリンク番号 → 座標は**有料版(詳細版B)**。オープンではない

## 参照論文

Saval A, Minh DP, Chapuis K, Tranouez P, Caron C, Daudé É, Taillandier P (2023)
"Dealing with mixed and non-normative traffic. An agent-based simulation with the GAMA platform."
*PLoS ONE* 18(3): e0281658. https://doi.org/10.1371/journal.pone.0281658 — **CC BY 4.0**

補足データ: https://doi.org/10.6084/m9.figshare.21369090.v1(アクセス不安定。GAMA 本体から取得可)
