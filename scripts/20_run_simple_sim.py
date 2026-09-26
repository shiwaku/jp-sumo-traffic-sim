"""簡易ミクロシミュレーションを実行し、ビューワ用の再生データを書き出す。

シナリオ: normal(平常時)/ winter(冬季)。同一シード・同一信号設定で、
IDMパラメータと自由速度だけを差し替える(信号は冬も夏の設定のまま =
系統ズレの論点をそのまま見せる。docs/design.md 6.5)。

出力:
  viewer/public/data/sim_{scenario}.json   1秒刻みの車両位置・信号状態
  reports/20_sim.json                      シナリオ別の集計
"""

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from pyproj import Transformer

from jp_sumo_traffic_sim import config as C
from jp_sumo_traffic_sim.cases import sapporo as S
from jp_sumo_traffic_sim.sim.simple import DT, GridSim

WARMUP_S = 300.0
RECORD_S = 300.0
FRAME_S = 1.0
OUT = C.ROOT / "viewer" / "public" / "data"


def street_classes() -> dict:
    """街路名 → arterial / minor。インベントリの道路分類(国道1/道道2)から。"""
    inv = json.loads((C.REPORTS / "04_inventory.json").read_text(encoding="utf-8"))
    out = {}
    for s in inv["streets"]:
        if s["kind"] != "格子街路":
            continue
        out[s["street"]] = "arterial" if any(c in ("1", "2") for c in s["cats"]) else "minor"
    return out


def signal_plans() -> dict:
    f = C.PROCESSED / "signal_plans.json"
    return json.loads(f.read_text(encoding="utf-8")) if f.exists() else {}


def oneway_lines() -> list:
    """一方通行(コード11)の線形。通行方向順の頂点列(EPSG:6679)で返す。

    JARTIC の頂点順は通行方向の逆と確定している
    (reports/15_oneway_check.json)ので、ここで反転する。
    """
    f = C.INTERIM / "regulations.gpkg"
    if not f.exists():
        return []
    import geopandas as gpd

    g = gpd.read_file(f, layer="line")
    g = g[(g["code"].astype(str) == "11") & g["in_cordon"]]
    return [list(geom.coords)[::-1] for geom in g.geometry]


def ksj_lines() -> list:
    """コードン内の KSJ 車道の線形(EPSG:6679)。格子リンクの実在チェックに使う。"""
    f = C.INTERIM / "ksj_clip.gpkg"
    if not f.exists():
        return []
    import geopandas as gpd

    g = gpd.read_file(f, layer="roads")
    g = g[g["in_cordon"]]
    out = []
    for geom in g.geometry:
        parts = geom.geoms if geom.geom_type == "MultiLineString" else [geom]
        out += [list(p.coords) for p in parts]
    return out


def run(scenario: str, classes: dict, plans: dict, oneways: list, ksj: list) -> tuple[dict, dict]:
    sim = GridSim(
        scenario=scenario,
        seed=42,
        signal_plans=plans,
        street_class=classes,
        oneways=oneways,
        ksj_lines=ksj,
    )

    # 原点(コードン中心)の投影座標と経緯度
    g = S.CORDON_GRID
    cx, cy = sim.grid_to_proj((g["west"] + g["east"]) / 2, (g["south"] + g["north"]) / 2)
    tf = Transformer.from_crs(C.CRS_PROJ, "EPSG:6668", always_xy=True)
    lon0, lat0 = tf.transform(cx, cy)

    nodes_dm = []
    for n in sim.nodes:
        x, y = sim.grid_to_proj(n.u, n.v)
        nodes_dm.append([round((x - cx) * 10), round((y - cy) * 10)])

    t0 = time.time()
    steps_per_frame = round(FRAME_S / DT)
    frames = []
    series = []
    spill_max = 0
    total_steps = int((WARMUP_S + RECORD_S) / DT)
    for k in range(total_steps):
        sim.step()
        if sim.t <= WARMUP_S or k % steps_per_frame:
            continue
        snap = sim.snapshot()
        flat = []
        for x, y, v in snap["pts"]:
            flat += [round((x - cx) * 10), round((y - cy) * 10), round(v * 10)]
        frames.append({"t": snap["t"], "p": flat, "sig": snap["sig"]})
        st = sim.stats()
        series.append(st)
        spill_max = max(spill_max, st["n_links_backed_up"])

    n_series = [s["n_vehicles"] for s in series]
    v_series = [s["mean_speed_ms"] for s in series]
    stats = dict(
        scenario=scenario,
        sim_seconds=WARMUP_S + RECORD_S,
        warmup_s=WARMUP_S,
        wall_clock_s=round(time.time() - t0, 1),
        n_nodes=len(sim.nodes),
        n_links=len(sim.links),
        n_entries=len(sim.entries),
        n_jartic_cycles=sim.n_jartic_matched,
        n_oneway_lines=len(oneways),
        n_oneway_lines_matched=sim.n_oneway_lines_matched,
        n_oneway_links_blocked=sim.n_oneway_blocked,
        n_links_masked_no_ksj=sim.n_links_masked,
        spawned=sim.n_spawned,
        exited=sim.n_exited,
        blocked_spawn=sim.n_blocked_spawn,
        vehicles=dict(
            min=min(n_series),
            mean=round(sum(n_series) / len(n_series), 1),
            max=max(n_series),
        ),
        mean_speed_kmh=round(sum(v_series) / len(v_series) * 3.6, 1),
        max_links_backed_up=spill_max,
    )
    data = dict(
        scenario=scenario,
        origin=[round(lon0, 6), round(lat0, 6)],
        frame_s=FRAME_S,
        unit="0.1m / 0.1m/s",
        nodes=nodes_dm,
        frames=frames,
    )
    return data, stats


def main() -> None:
    classes = street_classes()
    plans = signal_plans()
    oneways = oneway_lines()
    ksj = ksj_lines()
    OUT.mkdir(parents=True, exist_ok=True)
    report = {}
    for scenario in ("normal", "winter"):
        data, stats = run(scenario, classes, plans, oneways, ksj)
        f = OUT / f"sim_{scenario}.json"
        f.write_text(json.dumps(data, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
        stats["file_mb"] = round(f.stat().st_size / 1e6, 2)
        report[scenario] = stats
        print(
            f"{scenario}: {stats['vehicles']['mean']}台(平均) "
            f"{stats['mean_speed_kmh']}km/h  {stats['file_mb']}MB "
            f"({stats['wall_clock_s']}s)"
        )
    (C.REPORTS / "20_sim.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
