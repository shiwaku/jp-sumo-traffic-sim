"""ゾーンと OD(docs/sumo-design.md §13.5)。"""

# D4 の時間帯: シミュレーション時刻 0 = START_HOUR 時。最初の時間は助走
START_HOUR = 6
HOURS = [6, 7, 8, 9]
REF_HOUR = 8  # 初期 OD の水準を当てはめる基準時
