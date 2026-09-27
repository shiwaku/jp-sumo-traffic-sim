"""現況再現の照合と補正の部品(calib/estimation.py)の単体テスト。"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim.calib import estimation as E
from jp_sumo_traffic_sim.sumo.sim import read_hourly_flows


def _counts(**by_hour):
    c = [None] * 24
    for k, v in by_hour.items():
        c[int(k[1:])] = v
    return c


CENSUS = {
    "1": dict(unit="U1", direction="down", counts=_counts(h7=900.0, h8=1000.0, h9=800.0)),
    "2": dict(unit="U1", direction="down", counts=_counts(h7=900.0, h8=1000.0, h9=800.0)),
    "3": dict(unit="U2", direction="up", counts=_counts(h8=400.0)),
}
DETS = {"3": dict(unit="jartic:9", direction="北", counts=_counts(h7=300.0, h8=380.0, h9=350.0))}


def test_holdout_is_stable_and_about_the_fraction():
    units = [f"U{i}" for i in range(2000)]
    h = [u for u in units if E.is_holdout(u)]
    assert 0.17 < len(h) / len(units) < 0.23
    assert [u for u in units[:500] if E.is_holdout(u)] == [u for u in h if int(u[1:]) < 500]


def test_geh():
    assert E.geh(1000, 1000) == 0
    assert round(E.geh(1100, 1000), 2) == 3.09
    assert E.geh(0, 0) == 0


def test_obs_edgedata_excludes_units_and_missing_hours():
    xml, n = E.obs_edgedata_xml(CENSUS, [7, 8], 7, exclude_units={"U2"})
    assert n == 4  # U1 の 2 Edge × 2 時間。U2 は除外
    assert 'begin="3600" end="7200"' in xml and '<edge id="1" entered="1000"/>' in xml
    assert 'id="3"' not in xml


def test_groups_average_edges_per_source():
    obs = E.merge_observations(CENSUS, DETS)
    gs = {(g["source"], g["unit"]): g for g in E.observation_groups(obs, 8)}
    assert gs[("census", "U1")]["edges"] == ["1", "2"] and gs[("census", "U1")]["obs"] == 1000.0
    assert gs[("detector", "jartic:9")]["obs"] == 380.0  # 同じ Edge でも出典ごとに別


def test_fit_metrics():
    m = E.fit_metrics([110, 190, 410], [100, 200, 400])
    assert m["n"] == 3 and m["r"] > 0.99 and abs(m["slope"] - 1.0) < 0.05
    assert m["geh_ok_share"] == 1.0 and m["volume_ratio"] == 1.014  # 710 / 700
    assert E.fit_metrics([1], [1]) == dict(n=1)


def test_evaluate_and_acceptance():
    obs = E.merge_observations(CENSUS, DETS)
    flows = {8: {"1": 950.0, "2": 1050.0, "3": 390.0}}
    ev = E.evaluate(obs, flows, [8])
    m = ev["metrics"][8]
    assert m["all/all"]["n"] == 3 and m["all/census"]["n"] == 2 and m["all/detector"]["n"] == 1
    a = E.acceptance(m)
    assert a["checks"]["r"]["ok"] and a["checks"]["slope"]["ok"]
    assert len(ev["points"][8]) == 3


def test_site_temporal_r():
    obs = E.merge_observations(CENSUS, DETS)
    flows = {h: {"1": v, "2": v, "3": v / 3} for h, v in ((7, 950.0), (8, 1100.0), (9, 850.0))}
    t = E.site_temporal_r(obs, flows, [7, 8, 9])
    assert t["census"]["n"] == 2 and t["census"]["median_r"] > 0.9
    assert t["detector"]["n"] == 1


def test_read_hourly_flows_counts_upstream_part(tmp_path):
    p = tmp_path / "ed.xml"
    p.write_text(
        '<meandata><interval begin="0" end="3600">'
        '<edge id="5" entered="100"/><edge id="5.-60" entered="90"/></interval>'
        '<interval begin="3600" end="7200"><edge id="5" entered="120"/></interval>'
        '<interval begin="7200" end="7500"><edge id="5" entered="9"/></interval></meandata>'
    )
    f = read_hourly_flows(p, start_hour=7)
    assert f == {7: {"5": 100.0}, 8: {"5": 120.0}}  # 分割の下流側と 1 時間でない区間は除く
