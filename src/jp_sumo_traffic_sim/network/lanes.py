"""Edge への車線数・サブレーン数の割り当て(Phase 1-③、architecture.md §4.3)。

決定順:
  1. センサス箇所別基本表の車線数(幹線のみ、最優先)
  2. PLATEAU の道路ポリゴンで実測した幅員からの推定(docs/sumo-design.md §13.4)
  3. KSJ 幅員区分からの推定(最後の手段)
  (航空写真からの手入力 issue #1 は未着)

サブレーンは道路幅員を「最小車両1台分の幅」(1.75m)で分割する。
冬季の実質車線減少は n_sublanes の時間変化として Phase 2 で扱う。
"""

from __future__ import annotations

from jp_sumo_traffic_sim.ksj_codes import WIDTH_REPRESENTATIVE_M

SUBLANE_W_M = 1.75  # 二輪1台分
LANE_W_M = 3.0  # 車線1本の標準幅(推定用)
DEFAULT_CARRIAGEWAY_M = 4.0  # 幅員不明時の保守値(幅員区分2の代表値)
CARRIAGEWAY_SHARE = 0.7  # KSJ 幅員区分は道路幅なので歩道分を差し引く(推定用)
# PLATEAU の道路ポリゴンの幅はセンサスの道路部幅員(歩道込み)と一致する(中央値の比 1.00)。
# 車道幅員はその 0.68 倍(センサス区間 355 本の中央値、D1b)
PLATEAU_CARRIAGEWAY_SHARE = 0.68
# PLATEAU の車道幅から車線数を出すときの 1 車線あたりの実効の幅(路肩・停車帯を含む)。
# OSM の lanes と比べて決めた(3,168 組で一致率 89%。3.0m で割ると 56% で平均 0.8 車線過大、D1c)。
# 札幌の道路は路肩・停車帯(除雪の堆雪帯)の余裕が大きい
PLATEAU_LANE_W_M = 4.25
MAX_LANES_TWOWAY = 3  # 片方向あたりの上限(推定・品質ガード)
MAX_LANES_ONEWAY = 4
CENSUS_LANE_GUARD = 5  # センサス由来の片方向車線数がこれ以上なら側道誤結合とみなす
# センサス「代表信号交差点/右折専用車線の有無等」のコード(R3 箇所別基本表の説明資料
# kasyorep.pdf (43)②): 1=右折専用車線あり 2=なし 3=右折禁止 4=調査路線が右折
RT_LANE_PRESENT = 1


def assign_lanes(edge, census: dict | None, road_width_m: float | None = None) -> None:
    """edge.attrs に n_lanes / n_sublanes / carriageway_m / lane_source を書き込む。

    census: 当該エッジの census_id に対応する行(無ければ None)。
    road_width_m: PLATEAU の道路ポリゴンで実測した道路幅(歩道込み、両方向)。無ければ None。
      n_lanes は両方向合計(断面)、w_carriageway も断面幅。
    片側あたりに直すため、双方向道路(linked_edge あり)は 1/2 にする。
    一方通行は全幅が単方向に使える。
    """
    both_dir = edge.linked_edge is not None
    share = 0.5 if both_dir else 1.0
    cap = MAX_LANES_TWOWAY if both_dir else MAX_LANES_ONEWAY

    n = w = None
    source = ""
    if census is not None and census.get("n_lanes"):
        n = max(1, round(float(census["n_lanes"]) * share))
        w = float(census.get("w_carriageway") or 0.0) * share
        if w <= 0:
            w = n * LANE_W_M
        source = "census"
        if n >= CENSUS_LANE_GUARD:
            # 断面車線数を一方通行の側道等が引き当てた誤結合。幅員推定へ落とす
            n = None
            source = ""
    if n is None and road_width_m:
        w = road_width_m * PLATEAU_CARRIAGEWAY_SHARE * share
        n = max(1, min(cap, round(w / PLATEAU_LANE_W_M)))
        source = "plateau_width"
    if n is None:
        rep = WIDTH_REPRESENTATIVE_M.get(str(edge.attrs.get("width", "")), None)
        w = (rep if rep is not None else DEFAULT_CARRIAGEWAY_M) * share * CARRIAGEWAY_SHARE
        # 手入力(manual_ortho)が届くまでは幅員推定 = assumed 扱い(issue #1)。
        # キャップは推定にのみ適用する(センサス実測の片側4車線は潰さない)
        # 四捨五入: 幅員区分4(13〜19.5m)の双方向道路は片側 5.6m → 2 車線。切り捨てだと
        # 1 車線になり、市全域の需要推定で幅の広い市道が容量不足になった(C3b)
        n = max(1, min(cap, round(w / LANE_W_M)))
        source = "width_assumed"

    edge.attrs["n_lanes"] = int(n)
    edge.attrs["n_sublanes"] = max(1, int(w // SUBLANE_W_M))
    edge.attrs["carriageway_m"] = round(w, 2)
    edge.attrs["lane_source"] = source
    # 右折専用車線(センサスは区間代表値。交差点別は手入力待ち = assumed)。
    # コード 1 だけが「あり」(2=なし・3=右折禁止・4=調査路線が右折)
    code = census.get("right_turn_lane") if census is not None else None
    if code is not None and code == code:  # NaN を除く
        edge.attrs["right_turn_lane"] = int(round(float(code)) == RT_LANE_PRESENT)
        edge.attrs["rt_lane_source"] = "census_section"
    else:
        edge.attrs["right_turn_lane"] = 0
        edge.attrs["rt_lane_source"] = "assumed"


# 市区町村道の既定の最高速度 [km/h](KSJ の幅員区分 → 速度)。JARTIC の線規制のある市区町村道の
# 中央値: 幅員 3〜13m(区分 2・3)は 30、13m 以上(区分 4・5)は 50(D4)。区分 1(3m 未満)も 30。
# 一律 40 だと、幅の広い市道と並行する細街路の所要時間が同じになり、最短経路が細街路に散る
MUNICIPAL_SPEED_BY_WIDTH = {"1": 30, "2": 30, "3": 30, "4": 50, "5": 50}


def assign_speed(edge, census: dict | None, default_kmh: dict | None = None) -> None:
    """最高速度の決定順: JARTIC 線規制 → センサス規制速度 → 分類による既定値。

    市区町村道の既定値は幅員区分で分ける(MUNICIPAL_SPEED_BY_WIDTH)。
    """
    defaults = default_kmh or {"1": 50, "2": 50, "3": 40}
    cur = int(edge.attrs.get("speed_kmh") or 0)
    if cur > 0:
        edge.attrs["speed_source"] = "jartic"
        return
    if census is not None and census.get("speed_limit"):
        edge.attrs["speed_kmh"] = int(float(census["speed_limit"]))
        edge.attrs["speed_source"] = "census"
        return
    cat = str(edge.attrs.get("category", ""))
    width = str(edge.attrs.get("width", ""))
    if default_kmh is None and cat == "3" and width in MUNICIPAL_SPEED_BY_WIDTH:
        edge.attrs["speed_kmh"] = MUNICIPAL_SPEED_BY_WIDTH[width]
    else:
        edge.attrs["speed_kmh"] = defaults.get(cat, 40)
    edge.attrs["speed_source"] = "assumed"
