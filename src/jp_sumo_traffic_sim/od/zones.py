"""125m メッシュのゾーンと、その人口・従業者数(docs/sumo-design.md §13.5、D3)。

- 内部ゾーン: 対象区域に中心が入る 6 次メッシュ(125m)のうち、人口か従業者のあるもの
- 人口: 国勢調査 2020 の 125m メッシュ(人口総数は秘匿されておらず、そのまま足せる)
- 従業者: 経済センサス 2021 の小地域(町丁・字)を国勢調査 2020 の小地域の境界に町丁名で
  対応付け、面積の比でメッシュに配る。経済センサスの小地域コードは国勢調査の境界の
  KEY_CODE と体系が違う(17 桁と 11 桁)ので、コードではなく名称で対応付ける
- 外部ゾーン: ネットワークの端(区域の外の行き止まり = 抽出の切れ目)ごとに 1 つ
"""

from __future__ import annotations

import re
import unicodedata

KANJI_DIGITS = {ch: k for k, ch in enumerate("〇一二三四五六七八九")}
# 漢数字を算用数字に直すのは、後ろに番地の単位が続くときだけ(「一条」「八軒」の地名は残す)
KANJI_NUM = re.compile(r"[〇一二三四五六七八九十]+(?=(丁目|条|線|号|番|地割))")


def kanji_to_int(s: str) -> int:
    """「二十三」→ 23(十の位まで。札幌の条・丁目はこれで足りる)。"""
    total, cur = 0, 0
    for ch in s:
        if ch == "十":
            total += (cur or 1) * 10
            cur = 0
        else:
            cur = KANJI_DIGITS[ch]
    return total + cur


def normalize_town(name: str) -> str:
    """町丁名を比べられる形にする: NFKC(全角数字 → 半角)、条・丁目の漢数字 → 算用数字、
    「ヶ」→「ケ」、空白と括弧書き(「(番地)」など)を除く。"""
    s = unicodedata.normalize("NFKC", str(name))
    s = KANJI_NUM.sub(lambda m: str(kanji_to_int(m.group())), s)
    s = re.sub(r"\(.*?\)", "", s)
    return re.sub(r"\s+", "", s.replace("ヶ", "ケ").replace("ヵ", "カ"))


def match_towns(econ: dict[str, list[str]], bound: dict[str, list[str]]) -> dict[tuple, tuple]:
    """(市区町村コード, 正規化した町丁名) の対応。完全一致、無ければ境界側の名称のうち最長の
    前方一致(経済センサスの「北17条西15丁目」→ 境界の「北17条」のように境界が粗い場合)。

    econ・bound: 市区町村コード → 正規化した町丁名の一覧。戻り値: 経済センサス側 → 境界側。
    """
    out = {}
    for city, names in econ.items():
        cands = set(bound.get(city, []))
        by_len = sorted(cands, key=len, reverse=True)
        for n in names:
            if n in cands:
                out[(city, n)] = (city, n)
                continue
            hit = next((b for b in by_len if len(b) >= 2 and n.startswith(b)), None)
            if hit is not None:
                out[(city, n)] = (city, hit)
    return out
