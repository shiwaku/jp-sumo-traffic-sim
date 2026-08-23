# 札幌都心部 ミクロ交通シミュレーション

札幌市中心部を対象に、**車両1台ごとを再現する**ミクロ交通シミュレーションを構築するプロジェクト。

主な関心事:

- 系統制御(信号オフセット)の効果検証
- 冬季の実質車線数減少が交通流に与える影響
- 街区長が短いことによる**スピルバック**(滞留長が上流交差点を塞ぐ現象)

対象は札幌都心の矩形区域(北5条通・南7条通・創成川通・石山通に囲まれた約 1.5km × 1.2km)。
碁盤目かつ外周が幹線で囲まれているためコードン(境界)の設定が容易で、
街区が短く**マクロ/メソモデルでは表現できないスピルバックがミクロでのみ再現される**ことが区域選定の主要な動機。

モデルは Saval et al. (2023, PLoS ONE) の GAMA driving skill 実装に準拠する
(IDM + MOBIL + サブレーン + 赤信号を「長さ0の停止車両」として扱う)。

- 設計・データ仕様(基礎資料): [docs/design.md](docs/design.md)
- **実装設計書**: [docs/architecture.md](docs/architecture.md)
- データ出典と利用条件: [docs/data-sources.md](docs/data-sources.md)
- Phase 0 予備調査の結果: [docs/phase0-findings.md](docs/phase0-findings.md)

## 構成

```
src/sapporo_sim/     ライブラリ本体
  config.py          区域・座標系・閾値の集中定義
  network/           位相構築・conflation
  sim/               IDM / MOBIL / 信号 / サブレーン
  calib/             キャリブレーション(オフセット推定など)
scripts/             番号順に実行するパイプライン
prototype/           単体で動く簡易版(IDM + 定周期信号 + オフセット + 冬季モード)
viewer/              確認ビューワ(Vite + TS + MapLibre。データは public/data/ に生成)
docs/                設計資料・データ出典・調査結果
reports/             パイプラインが出す診断レポート(JSON、コミット対象)
data/                取得・中間・成果データ(コミット対象外)
tests/
```

`reports/` の JSON は**コミットする**。データ更新のたびに結合率や分布の差分を追えるようにするため。

## セットアップ

[uv](https://docs.astral.sh/uv/) を使う。

```bash
uv sync
```

## パイプライン

```bash
make phase0   # 下記を通しで実行

make fetch         # 国土数値情報 N13 (メッシュ 6441/6440) を取得
make clip          # 対象区域 + バッファでクリップし EPSG:6679 へ投影
make probe         # 予備調査(端点一致・階層順・リンク長・属性充足)
make grid          # グリッド主軸の推定と検算
make names         # 通し街路の命名と OSM 照合
make inventory     # 格子への直接割り当てで街路インベントリを作る
make fetch-jartic  # JARTIC の札幌・北海道分を取得
make regulations   # 交通規制情報を区域で絞り、種別ごとに集計
make signals       # 交差点制御情報から信号計画を作る
make fetch-census  # 道路交通センサスの変換済みデータを取得
make census        # 区域内の区間を抽出し、時間帯別交通量を結合
make viewer        # ブラウザ確認用ビューワを書き出す
```

> **JARTIC は最新1か月分しか配布しない。** 過去月の配布URLは404になる。
> `make fetch-jartic` は生zipとカタログのスナップショットを
> `data/raw/jartic/{年月}/` に SHA256 付きで残す。再現性のために保全すること。

## ビューワ

```bash
make dev-viewer   # データ書き出し + 開発サーバー (http://localhost:8002)
```

Vite + TypeScript + MapLibre GL JS。構成は
[jartic-traffic-signal-cycle-converter/viewer](https://github.com/shiwaku/jartic-traffic-signal-cycle-converter/tree/main/viewer)
をベースにしており、テーマ切替(ライト/ダーク)・背景切替(地理院最適化ベクトルタイル淡色 / 全国最新写真)・
レイヤーごとの説明と不透明度・クリックで属性ポップアップ + ハイライトを持つ。
地図はグリッド方位(-10.906°)で初期化され、碁盤目が画面の縦横に揃う。

レイヤー: コードン / 130m格子線 / KSJ 道路ネットワーク(街路種別で色分け、トンネル・橋の強調)
/ センサス区間 / JARTIC 規制(一方通行・最高速度・一時停止・信号機)
/ JARTIC 交差点制御情報(サイクル長の段階色)。

データは `make viewer` が `viewer/public/data/*.geojson` に書き出す(コミット対象外)。

### 簡易シミュレーションの再生

```bash
make sim         # 理想化グリッドで平常時・冬季の2シナリオを実行
make net-sim     # Phase 1 実ネットワークで同2シナリオを実行(要 make topology conflate edges)
make dev-viewer  # ビューワで「シミュレーション」からシナリオを選ぶ
```

**Phase 1(実ネットワーク)**: `make topology conflate edges` で KSJ から
実トポロジ(471ノード・731リンク → 1,176方向別Edge)を構築し、一方通行39本・
最高速度47区間・信号173交差点・一時停止119箇所・センサス17区間を空間結合済み。
`make net-sim` はこの実ネットワーク上で同じ動力学(IDM + 2現示 + 右折ギャップ +
一時停止)を回す。平常時 15.9 km/h。

理想化した130mグリッド(156交差点)上で IDM + 2現示信号を回す。KSJ 車道の
裏付けが無い格子リンク(大通公園東端・植物園・道庁前など54本)と一方通行の
逆走側(63本)を除いた457リンクを使う
Phase 2 の第一歩。JARTIC の実測サイクル長(18交差点)と東行き green wave の
オフセットを使い、赤信号は論文と同じく「長さ0の停止車両」として扱う。
車両は速度で色分け(赤=渋滞 → 青緑=自由流)、交差点は現示(東西青/南北青)で色が変わる。

| シナリオ | 平均速度 | 備考 |
|---|---|---|
| 平常時 | 14.8 km/h | センサス実測の混雑時旅行速度 14.6km/h(中央値)と同水準 |
| 冬季 | 8.9 km/h | 信号は夏設定のまま。流入ブロックが増える |

**時間の刻み**は3層ある。

| 層 | 値 | 意味 |
|---|---|---|
| 積分ステップ `DT` | 0.5秒 | IDM の加速度計算と位置更新の刻み(論文の GAMA driving skill と同等の粒度) |
| 記録フレーム `FRAME_S` | 1.0秒 | ビューワへ書き出すスナップショット間隔。1.0秒なのはファイルサイズのため(1シナリオ約4MB。0.5秒にすると倍) |
| 再生速度 | 1× / 4× / 10× | ビューワ側の見かけの速度。4× は実時間1秒 = シミュレーション4秒分 |

シミュレーション全体は600秒(ウォームアップ300秒 + 記録300秒)。
シークバーの全長は記録された300秒 = 300フレーム。

一方通行(JARTIC 実測39本のうち格子街路に載る10本・63リンク)と右折の対向
ギャップ受容は適用済み。残る簡略化(単車線・MOBILなし・転回率固定など)は
`src/sapporo_sim/sim/simple.py` の docstring に列挙してある。

個別に実行する場合:

```bash
uv run python scripts/00_clip_ksj.py
```

## 座標系

| 用途 | CRS |
|---|---|
| 入力(KSJ N13) | EPSG:6668 (JGD2011 地理座標) |
| シミュレーション内部 | **EPSG:6679** (JGD2011 平面直角座標系 XI系) |

札幌は11系。距離・速度計算を素直にするため内部処理はすべて投影座標で行う。

## 実装フェーズ

| Phase | 内容 | 状態 |
|---|---|---|
| 0 | 予備調査(データ素性の確認) | **完了**(KSJ / JARTIC / センサス) |
| 1 | ネットワーク構築(位相 + conflation + 手入力) | — |
| 2 | シミュレーション本体(IDM / MOBIL / 信号 / サブレーン) | 簡易版が動作(IDM + 2現示信号 + 基本図テスト通過)。MOBIL・サブレーンは未実装 |
| 3 | キャリブレーション(オフセット推定・ホールドアウト検証) | — |
| 4 | シナリオ(冬季・系統再設計・路上駐車) | — |
| 5 | 可視化(PMTiles + setFeatureState) | — |

詳細は [docs/design.md](docs/design.md) 第7章。

## データの出典

このリポジトリのコードは MIT ライセンス([LICENSE](LICENSE))。
**データは含まれない**(取得スクリプトで再現する)。各データの出典表示義務については
[docs/data-sources.md](docs/data-sources.md) を必ず参照すること。

- 国土数値情報 道路データ (N13) — 国土交通省 / CC BY 4.0
- 交差点位置情報 — 日本交通管理技術協会(交差点番号→座標の結合)
- JARTIC オープンデータ(交差点制御情報・交通規制情報・断面交通量情報)— 日本道路交通情報センター
- 全国道路・街路交通情勢調査(道路交通センサス)— 国土交通省 / 公共データ利用規約 (PDL1.0)
- OpenStreetMap — © OpenStreetMap contributors / ODbL 1.0(一方通行の答え合わせに使用)

## 関連リポジトリ

| リポジトリ | 役割 |
|---|---|
| [jartic-traffic-signal-cycle-converter](https://github.com/shiwaku/jartic-traffic-signal-cycle-converter) | 交差点制御情報 → 平均サイクル長の全国 PMTiles |
| [jartic-traffic-regulation-converter](https://github.com/shiwaku/jartic-traffic-regulation-converter) | 交通規制情報 → 規制種別レイヤーの全国 PMTiles |
| [mlit-road-traffic-census-converter](https://github.com/shiwaku/mlit-road-traffic-census-converter) | 道路交通センサス → GeoParquet / PMTiles |
| [ksj-route-search-api](https://github.com/shiwaku/ksj-route-search-api) | KSJ からのネットワーク構築と PMTiles 可視化 |

本プロジェクトは全国データを扱う部分をこれらに委ね、区域分の処理に集中する。

## 参照

Saval A, Minh DP, Chapuis K, Tranouez P, Caron C, Daudé É, Taillandier P (2023)
"Dealing with mixed and non-normative traffic. An agent-based simulation with the GAMA platform."
*PLoS ONE* 18(3): e0281658. https://doi.org/10.1371/journal.pone.0281658 (CC BY 4.0)
