.PHONY: help sync fetch clip probe grid names inventory viewer phase0 lint test clean open-viewer \
        fetch-jartic regulations signals fetch-census census jartic

help:
	@grep -E '^[a-z0-9-]+:.*?##' $(MAKEFILE_LIST) | sed 's/:.*##/\t/'

sync:  ## 依存を同期
	uv sync

fetch:  ## 国土数値情報 N13 を取得(メッシュ 6441/6440)
	uv run python scripts/fetch_ksj.py

clip:  ## 対象区域 + バッファでクリップし EPSG:6679 へ投影
	uv run python scripts/00_clip_ksj.py

probe:  ## Phase 0 予備調査
	uv run python scripts/01_probe_network.py

grid:  ## グリッド主軸の推定と検算
	uv run python scripts/02_grid_frame.py

names:  ## 通し街路の命名と OSM 照合
	uv run python scripts/03_name_streets.py

inventory:  ## 格子への直接割り当てで街路インベントリを作る
	uv run python scripts/04_street_inventory.py

viewer:  ## ビューワ用データ(GeoJSON + meta.json)を書き出す
	uv run python scripts/05_export_viewer.py

topology:  ## Phase 1: KSJ から実ネットワークの位相を構築する
	uv run python scripts/30_build_topology.py

conflate:  ## Phase 1: 規制・信号・センサスを実ネットワークへ結合する
	uv run python scripts/31_conflate.py

sim:  ## 簡易ミクロシミュレーションを実行し再生データを書き出す
	uv run python scripts/20_run_simple_sim.py

dev-viewer: viewer sim  ## ビューワを開発サーバーで起動 (http://localhost:8002)
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
