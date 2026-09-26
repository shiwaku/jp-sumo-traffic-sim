# jp-sumo-traffic-sim

日本のオープンデータ(国土数値情報 N13・JARTIC 交通規制/交差点制御情報・道路交通センサス)から
[Eclipse SUMO](https://eclipse.dev/sumo/) の交通シミュレーションを構築するプロジェクト。

入力データはいずれも全国で配布されているため、**汎用のパイプライン + 地域ごとのケース**という
構成を目指す。最初のケースは**札幌**。

> シミュレーション本体は SUMO(自前実装の IDM + MOBIL + サブレーンから移行済み、issue #30)。
> 移行と汎用化の設計は [docs/sumo-design.md](docs/sumo-design.md)。
> 地域パラメータは `cases/<name>/case.toml` に分離済み(ケースは環境変数 `JPSUMO_CASE` で選ぶ。既定 `sapporo`)。
> データ置き場(`data/`・`reports/`)はまだケース別に分けていない。

## しくみ

```
[1] データ取得       KSJ N13 / JARTIC(規制・交差点制御)/ 道路交通センサス
        ↓
[2] ネットワーク構築  位相構築(立体交差の判定)→ 規制・信号・センサスの空間結合
                    → 単方向エッジ化・車線割当
        ↓
[3] シミュレーション  SUMO(ネットワーク変換・需要生成・実行・計測)
        ↓
[4] キャリブレーション 断面交通量・旅行速度との照合、信号オフセット推定、ホールドアウト検証
        ↓
[5] 可視化          ブラウザのビューワで車両の動きと指標を再生
```

- **共通キーの無いデータを空間結合でつなぐ**のが層[2]の中心。KSJ には一方通行も信号も無いので、
  JARTIC の規制(一方通行・最高速度・一時停止)と交差点制御(サイクル長・スプリット)、
  センサス(車線数・交通量・旅行速度)を方位角条件付きの空間結合で載せる
- **立体交差を誤接続しない。** KSJ の階層順だけでは足りない(トンネルが階層順0のまま入っている)ため、
  道路状態(橋・トンネル)で判定する
- 需要は OD ではなく**コードン流入 + 転回率**(都心区域の場合)。市全域への拡大では
  OD + 経路選択に切り替える(設計 §13)

## ケース: 札幌

対象区域は**札幌市全域**(10 区、1,121 km²)。市全域は幹線と主要市道をメソで、
都心区域(北5条通・南7条通・創成川通・石山通に囲まれた約 1.5 km × 1.7 km)は全道路をミクロで回す
(設計 §13。市全域の需要推定とミクロへの接続は作業中、issue #37)。

主な関心事:

- 系統制御(信号オフセット)の効果検証
- 冬季の実質車線数減少が交通流に与える影響(雪堤による幅員減少)
- 街区長が短いことによる**スピルバック**(滞留長が上流交差点を塞ぐ現象)

都心は碁盤目かつ外周が幹線で囲まれているためコードン(境界)の設定が容易で、
街区が短く**マクロ/メソモデルでは表現できないスピルバックがミクロでのみ再現される**ことが区域選定の動機。

| 項目 | 値 |
|---|---|
| 実ネットワーク(市全域) | 12,108 ノード・13,139 リンク → 方向別 25,152 Edge |
| 空間結合済み | 一方通行 1,126 Edge・最高速度・信号 2,582 交差点・一時停止・センサス 426 区間 |
| 座標系 | EPSG:6679(平面直角座標系 XI 系) |
| KSJ メッシュ | 6441 / 6440 |
| 行政界 | 国土数値情報 N03(2025)、市区町村コード 01101〜01110 |
| JARTIC 情報源コード | 3001(北海道警 札幌方面) |

### 進捗

| Phase | 内容 | 状態 |
|---|---|---|
| 0 | 予備調査(データ素性の確認) | **完了** |
| 1 | ネットワーク構築(位相 + 空間結合 + 単方向化・車線) | **完了**(手入力データの整備は issue #1) |
| 2 | シミュレーション本体 | 自前実装で完了後、**SUMO へ移行済み**(ネットワーク変換・需要・ビューワ。issue #30) |
| 3 | キャリブレーション | ① 実測需要と断面照合まで完了(交通量比 0.17、速度 MAE 5.6 km/h)。②以降は SUMO 上で再開(issue #28) |
| 4 | シナリオ(冬季・系統再設計・路上駐車) | — |
| 5 | 可視化 | ビューワで再生可能 |

## ドキュメント

- **SUMO 移行・汎用化の設計**: [docs/sumo-design.md](docs/sumo-design.md)
- 設計・データ仕様(基礎資料、札幌ケース): [docs/design.md](docs/design.md)
- 実装設計書(層構成・データモデル): [docs/architecture.md](docs/architecture.md)
- データ出典と利用条件: [docs/data-sources.md](docs/data-sources.md)
- Phase 0 予備調査の結果: [docs/phase0-findings.md](docs/phase0-findings.md)

## 構成

```
src/jp_sumo_traffic_sim/  ライブラリ本体(汎用部)
  config.py          パス・汎用閾値の定義とケースの読み込み
  cases/             ケース固有のコード(札幌: グリッド座標・コードン)
  network/           位相構築・空間結合・単方向化・車線
  sumo/              SUMO への変換(ネットワーク・信号・需要・車両)と実行
  sim/               センサス時間帯別交通量の読み取り
  calib/             キャリブレーション
scripts/             番号順に実行するパイプライン(汎用)
cases/sapporo/       札幌ケース: case.toml(地域パラメータ)と札幌専用スクリプト(scripts/)
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

## パイプライン(札幌ケース)

```bash
# 層[1] データ取得
make fetch         # 国土数値情報 N13 (メッシュ 6441/6440) を取得
make fetch-n03     # 国土数値情報 N03(行政区域)を取得 = 対象区域
make fetch-jartic  # JARTIC の札幌・北海道分を取得
make fetch-census  # 道路交通センサスの変換済みデータを取得

# Phase 0 予備調査(make phase0 で通し実行)
make clip          # 対象区域 + バッファでクリップし EPSG:6679 へ投影
make probe         # 端点一致・階層順・リンク長・属性充足
make grid          # グリッド主軸の推定と検算(札幌固有)
make names         # 通し街路の命名と OSM 照合(札幌固有)
make inventory     # 格子への直接割り当てで街路インベントリを作る(札幌固有)
make regulations   # 交通規制情報を区域で絞り、種別ごとに集計
make signals       # 交差点制御情報から信号計画を作る
make census        # 区域内の区間を抽出し、時間帯別交通量を結合

# 層[2] ネットワーク構築
make topology      # KSJ から実ネットワークの位相を構築する
make conflate      # 規制・信号・センサスを実ネットワークへ結合する
make edges         # 単方向エッジ化と車線・速度の割り当て

# 層[3][4] シミュレーション(SUMO)と照合
make sumo-net      # 方向別 Edge から SUMO ネットワーク(平常時・冬季)を作る
make sumo-viewer   # 平常時・冬季を回し、ビューワの再生データを書き出す
make demand-check  # 実測需要で回し、センサス断面交通量・旅行速度と照合する
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

レイヤー: コードン / 格子線 / KSJ 道路ネットワーク(街路種別で色分け、トンネル・橋の強調)
/ センサス区間 / JARTIC 規制(一方通行・最高速度・一時停止・信号機)
/ JARTIC 交差点制御情報(サイクル長の段階色)。

「シミュレーション」からシナリオ(平常時・冬季)を選ぶと、SUMO の車両の動きを再生できる
(`make sumo-viewer` が朝8時の需要で 300 秒分を 1 秒刻みで書き出す)。
車両は速度で色分け(赤=渋滞 → 青緑=自由流)、交差点は現示で色が変わる。

データは `make viewer` が `viewer/public/data/` に書き出す。

## データの出典

このリポジトリのコードは MIT ライセンス([LICENSE](LICENSE))。
**データは含まれない**(取得スクリプトで再現する)。各データの出典表示義務については
[docs/data-sources.md](docs/data-sources.md) を必ず参照すること。

- 国土数値情報 道路データ (N13)・行政区域データ (N03) — 国土交通省 / CC BY 4.0
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

本プロジェクトは全国データの変換をこれらに委ね、対象地域のシミュレーション構築に集中する。

## 参照

- Eclipse SUMO — https://eclipse.dev/sumo/ (EPL 2.0)
- Saval A, Minh DP, Chapuis K, Tranouez P, Caron C, Daudé É, Taillandier P (2023)
  "Dealing with mixed and non-normative traffic. An agent-based simulation with the GAMA platform."
  *PLoS ONE* 18(3): e0281658. https://doi.org/10.1371/journal.pone.0281658 (CC BY 4.0)
  — 自前実装(Phase 2)の動力学はこの論文の GAMA driving skill 実装に準拠した
