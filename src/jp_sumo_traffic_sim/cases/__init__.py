"""地域ごとのケース固有処理(docs/sumo-design.md §14)。

ケースの地域パラメータは cases/<name>/case.toml、区域ポリゴンなど
コードで書く必要のある処理はこのパッケージのモジュールに置く。
各モジュールは少なくとも region_polygon() を提供する。
"""
