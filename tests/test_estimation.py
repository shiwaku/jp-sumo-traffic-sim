"""需要推定・照合の部品(calib/estimation.py)の単体テスト。"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim.calib import estimation as E


def _counts(v8):
    c = [None] * 24
    c[8] = v8
    return c


COUNTS = {
    "1": dict(section="S1", unit="U1", direction="down", counts=_counts(1000)),
    "2": dict(section="S1", unit="U1", direction="down", counts=_counts(1000)),
    "3": dict(section="S2", unit="U2", direction="up", counts=_counts(400)),
    "4": dict(section="S3", unit="U3", direction="avg", counts=_counts(None)),
}


def test_split_holdout_is_by_unit_and_reproducible():
    units = [f"U{i}" for i in range(10)] * 3
    h = E.split_holdout(units, 0.2, seed=1)
    assert len(h) == 2 and h == E.split_holdout(units, 0.2, seed=1)


def test_geh():
    assert E.geh(1000, 1000) == 0
    assert round(E.geh(1100, 1000), 2) == 3.09
    assert E.geh(0, 0) == 0


def test_obs_edgedata_excludes_holdout_and_missing_hours():
    xml, n = E.obs_edgedata_xml(COUNTS, [8], 7, exclude_units={"U2"})
    assert n == 2  # U2 は検証用、Edge 4 は観測なし
    assert 'begin="3600" end="7200"' in xml and '<edge id="1" entered="1000"/>' in xml
    assert 'id="3"' not in xml


def test_compare_groups_by_unit_and_direction():
    sim = {"1": 900.0, "2": 1100.0, "3": 200.0}
    r = E.compare(COUNTS, sim, 8, holdout={"U2"})
    cal, hold = r["calibration"], r["holdout"]
    assert cal["n"] == 1 and cal["volume_ratio"] == 1.0 and cal["geh_ok_share"] == 1.0
    assert hold["n"] == 1 and hold["volume_ratio"] == 0.5
