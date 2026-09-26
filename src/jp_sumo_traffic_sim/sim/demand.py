"""コードン流入需要(Phase 3-①、architecture.md §5.5)。

センサス時間帯別交通量(data/processed/census_hourly.json、区域内17区間・
観測は7〜18時)から、流入 Edge の時間帯別流入レート [台/時] を作る。

- OD 行列は使わない。コードン流入 + 転回率(design.md の方針)
- センサスの上り/下りは路線の起終点基準で、Edge の進行方向へ機械的に
  確定できない(路線ごとに起点の向きが違う)。ここでは**方向平均**
  ((up + down) / 2)を使う。実測では方向差が最大4割あるため、
  方向別の合わせ込みは転回率調整(Phase 3-②)以降の課題
- 台数は小型(s)+ 大型(l)の合計。車種は乗用車のみなので大型も
  1台として数える(大型の占有幅・性能差はサブレーンの将来課題)
- センサスの裏付けが無い流入(細街路など)は既定値のまま
"""

from __future__ import annotations

DEFAULT_RATE = {"arterial": 500.0, "minor": 150.0}  # 裏付けなしの既定値 [台/時]


def hourly_rate(section: dict | None, hour: int) -> float | None:
    """センサス1区間の指定時刻の方向平均交通量 [台/時]。観測が無ければ None。

    section は census_hourly.json の1エントリ({"up": {"s": [...], "l": [...]},
    "down": {...}})。s/l の欠測(null)は0として足し、上下とも全欠測の
    時刻は None を返す(観測時間帯は7〜18時)。
    """
    if not section:
        return None
    total, n_dir = 0.0, 0
    for d in ("up", "down"):
        arrs = section.get(d) or {}
        seen = False
        for k in ("s", "l"):
            arr = arrs.get(k)
            if arr and len(arr) > hour and arr[hour] is not None:
                total += float(arr[hour])
                seen = True
        if seen:
            n_dir += 1
    if n_dir == 0:
        return None
    return total / n_dir if n_dir == 2 else total  # 片方向しか無ければそのまま使う
