.PHONY: help sync fetch clip probe grid names inventory viewer phase0 lint test clean open-viewer \
        fetch-jartic regulations signals fetch-census census jartic

help:
	@grep -E '^[a-z0-9-]+:.*?##' $(MAKEFILE_LIST) | sed 's/:.*##/\t/'

sync:  ## 依存を同期
	uv sync

fetch:  ## 国土数値情報 N13 を取得(メッシュ 6441/6440)
	uv run python scripts/fetch_ksj.py

fetch-n03:  ## 国土数値情報 N03(行政区域)を取得(対象区域 = 市の行政界)
	uv run python scripts/fetch_n03.py

fetch-plateau:  ## PLATEAU 札幌市の道路モデル(CityGML、区域分だけ範囲指定で)を取得
	uv run python scripts/fetch_plateau_tran.py

osm-lanes:  ## OSM の車線数・turn:lanes・転回制約の充足を調べる(Overpass に 1 回問い合わせ)
	uv run python scripts/16_osm_lane_tags.py

clip:  ## 対象区域 + バッファでクリップし EPSG:6679 へ投影
	uv run python scripts/00_clip_ksj.py

probe:  ## Phase 0 予備調査
	uv run python scripts/01_probe_network.py

grid:  ## [札幌] グリッド主軸の推定と検算
	uv run python cases/sapporo/scripts/02_grid_frame.py

names:  ## [札幌] 通し街路の命名と OSM 照合
	uv run python cases/sapporo/scripts/03_name_streets.py

inventory:  ## [札幌] 格子への直接割り当てで街路インベントリを作る
	uv run python cases/sapporo/scripts/04_street_inventory.py

viewer:  ## ビューワ用データ(GeoJSON + meta.json)を書き出す
	uv run python scripts/05_export_viewer.py

topology:  ## Phase 1: KSJ から実ネットワークの位相を構築する
	uv run python scripts/30_build_topology.py

conflate:  ## Phase 1: 規制・信号・センサスを実ネットワークへ結合する
	uv run python scripts/31_conflate.py

edges:  ## Phase 1: 単方向エッジ化と車線・速度の割り当て
	uv run python scripts/32_directed_lanes.py

census-counts:  ## センサス時間帯別交通量を方向別 Edge の観測値にする(需要推定・照合用)
	uv run python scripts/41_census_counts.py

detectors:  ## JARTIC 車両感知器の位置を特定し、時間帯別の観測値にする
	uv run python scripts/42_jartic_detectors.py

demand-check:  ## SUMO を実測需要で回し、センサス断面交通量・旅行速度と照合する(要 make sumo-net)
	uv run python scripts/40_demand_check.py

sumo-net:  ## 方向別 Edge から SUMO ネットワーク(平常時・冬季)を作る(data/sumo/)
	uv run python scripts/50_build_sumo_net.py

sumo-viewer:  ## SUMO を平常時・冬季で回し、ビューワの再生データを書き出す(要 make sumo-net)
	uv run python scripts/53_export_sumo_viewer.py

dev-viewer: viewer  ## ビューワを開発サーバーで起動 (http://localhost:8002)
	cd viewer && npm install --silent && npm run dev

fetch-jartic:  ## JARTIC の札幌・北海道分を取得(最新1か月分のみ配布)
	uv run python scripts/10_fetch_jartic.py

regulations:  ## 交通規制情報を区域で絞り、種別ごとに集計
	uv run python scripts/11_jartic_regulations.py

signals:  ## 交差点制御情報から信号計画(サイクル長・スプリット・現示)を作る
	uv run python scripts/12_jartic_signals.py

fetch-census:  ## 道路交通センサス(令和3年度)の変換済みデータを取得
	uv run python scripts/13_fetch_census.py

census:  ## センサスから区域内の区間を抽出し、時間帯別交通量を結合
	uv run python scripts/14_census_extract.py

jartic: fetch-jartic regulations signals  ## JARTIC 一式

phase0: fetch clip probe grid names inventory fetch-jartic regulations signals \
        fetch-census census viewer  ## Phase 0 を通しで実行

lint:
	uv run ruff check src scripts
	uv run ruff format --check src scripts

test:
	uv run pytest -q

clean:  ## 中間データを削除(生データは残す)
	rm -rf data/interim/* data/processed/*
