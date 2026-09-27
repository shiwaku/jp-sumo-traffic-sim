"""ゾーン(od/zones.py)と地域メッシュ(mesh.py)の単体テスト。"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim.mesh import mesh3_code, mesh6_bounds, mesh6_code, mesh6_index
from jp_sumo_traffic_sim.od.zones import kanji_to_int, match_towns, normalize_town


def test_kanji_to_int():
    assert kanji_to_int("一") == 1
    assert kanji_to_int("十") == 10
    assert kanji_to_int("十五") == 15
    assert kanji_to_int("二十三") == 23


def test_normalize_town_econ_and_boundary_agree():
    # 経済センサスは全角の算用数字、国勢調査の境界は漢数字
    assert normalize_town("旭ケ丘１丁目") == normalize_town("旭ケ丘一丁目") == "旭ケ丘1丁目"
    assert normalize_town("大通西４丁目") == normalize_town("大通西四丁目")
    assert normalize_town("北二十一条西１２丁目") == "北21条西12丁目"
    assert normalize_town("円山西町（番地）") == "円山西町"
    # 番地の単位が続かない漢数字は地名の一部として残す
    assert normalize_town("八軒一条西１丁目") == "八軒1条西1丁目"


def test_match_towns_exact_then_prefix():
    econ = {"01101": ["北17条西15丁目", "旭ケ丘1丁目", "その他"]}
    bound = {"01101": ["北17条", "旭ケ丘1丁目", "北1条西1丁目"]}
    m = match_towns(econ, bound)
    assert m[("01101", "旭ケ丘1丁目")] == ("01101", "旭ケ丘1丁目")
    assert m[("01101", "北17条西15丁目")] == ("01101", "北17条")
    assert ("01101", "その他") not in m


def test_mesh_codes():
    assert mesh3_code(64 * 80, 141 * 80) == "64410000"
    # e-Stat の 125m メッシュ 64410005221 の南西端(141.071875, 42.6666…)
    i, j = mesh6_index(141.071875 + 1e-6, 42.666666666666664 + 1e-6)
    assert mesh6_code(i, j) == "64410005221"
    x0, y0, x1, y1 = mesh6_bounds(i, j)
    assert abs(x0 - 141.071875) < 1e-9 and abs(y1 - 42.66770833333333) < 1e-9
    # この点は 3 次メッシュの中で南から 0、西から 6 番目。北東の隅(7, 7)の番号は 444
    assert (i % 8, j % 8) == (0, 6)
    assert mesh6_code(i + 7, j + 1)[-3:] == "444"
