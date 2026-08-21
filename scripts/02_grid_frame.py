"""札幌グリッドの主軸を推定し、回転座標系で街路を同定する。

碁盤目が平面直角座標系に対して約13度傾いているため、
主軸方向に回転した座標 (u, v) を導入すると
「同一の通り」が一定の横断座標を共有するようになる。

出力: reports/02_grid.json, data/interim/grid_check.gpkg

注意: 街路の同定は 04_street_inventory.py(格子への直接割り当て)を正とする。
本スクリプトはグリッド方位の推定と検算のために残している。
"""

import json
import sys
from pathlib import Path

import geopandas as gpd
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from sapporo_sim import config as C
from sapporo_sim.ksj_codes import ROAD_WIDTH as WIDTH_LABEL

# 区域スパンのこの割合以上の延長を持つものを「通し街路」とみなす
THROUGH_FRAC = 0.6
# 通し街路の横断方向の広がりがこれを超えたら上下線分離の疑い [m]
DIVIDED_SPREAD_M = 15.0


def seg_bearings(gdf: gpd.GeoDataFrame):
    """各リンクを構成する線分の方位角[deg, 0-180)と長さを返す。"""
    ang, wt = [], []
    for geom in gdf.geometry:
        lines = geom.geoms if geom.geom_type == "MultiLineString" else [geom]
        for ls in lines:
            c = np.asarray(ls.coords)[:, :2]
            d = np.diff(c, axis=0)
            L = np.hypot(d[:, 0], d[:, 1])
            a = np.degrees(np.arctan2(d[:, 1], d[:, 0])) % 180.0
            ang.append(a)
            wt.append(L)
    return np.concatenate(ang), np.concatenate(wt)


def _circmean90(ang: np.ndarray, wt: np.ndarray) -> float:
    th = np.radians(ang % 90.0) * 4.0  # mod90 -> 円周へ
    s = np.average(np.sin(th), weights=wt)
    c = np.average(np.cos(th), weights=wt)
    return float((np.degrees(np.arctan2(s, c)) / 4.0) % 90.0)


def estimate_bearing(ang: np.ndarray, wt: np.ndarray, reject_deg: float = 15.0) -> float:
    """長さ重み付きで直交2方向の主軸角を推定 (mod 90 の円周平均)。

    都心には斜行路(月寒通・北1条雁来通など)が混ざるため、
    一度推定した軸から reject_deg 以上ずれる線分を落として再推定する。
    これを入れないと推定値が 0.4 度ほど引っ張られ、
    1.6km 先で 12m の横断誤差になって街路のクラスタリングが壊れる。
    """
    t = _circmean90(ang, wt)
    for _ in range(3):
        off = np.minimum((ang - t) % 90.0, 90.0 - ((ang - t) % 90.0))
        m = off <= reject_deg
        if m.sum() < 10:
            break
        t = _circmean90(ang[m], wt[m])
    return t


def main() -> None:
    src = C.INTERIM / "ksj_clip.gpkg"
    roads = gpd.read_file(src, layer="roads")
    cordon = gpd.read_file(src, layer="cordon").query("kind == 'cordon'").geometry.iloc[0]
    inner = roads[roads["in_cordon"]].copy()

    # 主軸は実測で推定するが、フレーム自体は config の固定値を使う
    # (毎回の推定値のゆらぎで街路の横断座標が動かないようにするため)
    ang, wt = seg_bearings(inner)
    keep = wt >= 10.0  # 10m 未満の線分は方位が不安定
    theta_est = estimate_bearing(ang[keep], wt[keep])
    theta = C.GRID_BEARING_DEG

    rot = C.to_grid
    inner["geom_rot"] = inner.geometry.map(rot)

    # 回転後の各リンクの主方向 (u=EW軸, v=NS軸)
    def classify(geom):
        c = np.asarray((geom if geom.geom_type == "LineString" else list(geom.geoms)[0]).coords)[
            :, :2
        ]
        dx, dy = c[-1] - c[0]
        return "EW" if abs(dx) >= abs(dy) else "NS"

    inner["axis"] = inner["geom_rot"].map(classify)
    inner["u"] = inner["geom_rot"].map(lambda g: g.centroid.x)
    inner["v"] = inner["geom_rot"].map(lambda g: g.centroid.y)

    # 傾きの残差(グリッドから外れたリンクの検出)
    def resid(geom):
        c = np.asarray((geom if geom.geom_type == "LineString" else list(geom.geoms)[0]).coords)[
            :, :2
        ]
        d = c[-1] - c[0]
        a = np.degrees(np.arctan2(d[1], d[0])) % 90.0
        return round(float(min(a, 90 - a)), 1)

    inner["off_grid_deg"] = inner["geom_rot"].map(resid)

    rep: dict = dict(
        grid_bearing_deg_from_x_axis=round(theta, 3),
        grid_bearing_deg_estimated=round(theta_est, 3),
        grid_bearing_residual_deg=round(theta_est - theta, 3),
        note=(
            "EPSG:6679 の X 軸(東)からの反時計回り角。"
            "この角度だけ逆回転するとグリッドが軸に整列する。"
        ),
        n_links=len(inner),
        axis_counts=inner["axis"].value_counts().to_dict(),
        off_grid=dict(
            n_over_10deg=int((inner["off_grid_deg"] > 10).sum()),
            n_over_20deg=int((inner["off_grid_deg"] > 20).sum()),
            total_len_over_20deg_m=round(
                float(inner.loc[inner["off_grid_deg"] > 20, "length_m"].sum()), 1
            ),
        ),
    )

    # --- 街路の同定: 横断座標でクラスタリング ---
    # スパンは回転後のコードンで測る
    minx, miny, maxx, maxy = rot(cordon).bounds
    streets = {}
    for axis, key in (("EW", "v"), ("NS", "u")):
        sub = inner[(inner["axis"] == axis) & (inner["off_grid_deg"] <= 20)].copy()
        vals = sub[key].to_numpy()
        order = np.argsort(vals)
        sub = sub.iloc[order]
        vals = vals[order]
        # 20m 以内は同一の通り(上下線分離を含む)
        gid, groups = 0, []
        for i, v in enumerate(vals):
            if i and v - vals[i - 1] > 20.0:
                gid += 1
            groups.append(gid)
        sub["street_id"] = [f"{axis}{g:02d}" for g in groups]
        agg = sub.groupby("street_id").agg(
            n_links=("link_id", "size"),
            cross_m=(key, "mean"),
            spread_m=(key, lambda s: round(float(s.max() - s.min()), 1)),
            len_sum=("length_m", "sum"),
            cats=("N13_003", lambda s: sorted(set(s.astype(str)))),
            widths=("N13_006", lambda s: sorted(set(s.astype(str)))),
        )
        agg["cross_m"] = agg["cross_m"].round(1)
        agg["len_sum"] = agg["len_sum"].round(1)
        # 通し街路の判定: 区域を横断する延長を持つか
        span = (maxx - minx) if axis == "EW" else (maxy - miny)
        agg["through"] = agg["len_sum"] >= THROUGH_FRAC * span
        # 上下線分離の判定: 横断方向の広がりが車道1本を超え、かつ通し街路
        agg["divided"] = agg["through"] & (agg["spread_m"] >= DIVIDED_SPREAD_M)

        def spacing(sub):
            c = np.sort(sub["cross_m"].to_numpy())
            g = np.round(np.diff(c), 1)
            return dict(
                n=len(c),
                median=round(float(np.median(g)), 1) if len(g) else None,
                values=g.tolist(),
            )

        streets[axis] = dict(
            span_m=round(float(span), 1),
            n_streets=len(agg),
            n_through=int(agg["through"].sum()),
            n_divided=int(agg["divided"].sum()),
            spacing_all_m=spacing(agg),
            spacing_through_m=spacing(agg[agg["through"]]),
            table=[
                dict(
                    street_id=i,
                    **{k: (v.item() if isinstance(v, np.generic) else v) for k, v in r.items()},
                )
                for i, r in agg.iterrows()
            ],
        )
        inner.loc[sub.index, "street_id"] = sub["street_id"]
    rep["streets"] = streets

    out = inner.drop(columns=["geom_rot"])
    out.to_file(C.INTERIM / "grid_check.gpkg", layer="roads", driver="GPKG")
    (C.REPORTS / "02_grid.json").write_text(
        json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    print(f"grid bearing = {theta:.3f} deg (config) / {theta_est:.3f} deg (実測)")
    print(
        json.dumps({k: v for k, v in rep.items() if k != "streets"}, ensure_ascii=False, indent=2)
    )
    for axis in ("EW", "NS"):
        s = streets[axis]
        print(
            f"\n=== {axis} 街路 {s['n_streets']}本 "
            f"(通し {s['n_through']} / 上下線分離の疑い {s['n_divided']}) ==="
        )
        print(
            f"  全街路の間隔 中央値 {s['spacing_all_m']['median']}m  {s['spacing_all_m']['values']}"
        )
        print(
            f"  通し街路の間隔 中央値 {s['spacing_through_m']['median']}m  "
            f"{s['spacing_through_m']['values']}"
        )
        for r in s["table"]:
            mark = "通" if r["through"] else "  "
            div = "分離?" if r["divided"] else "    "
            widths = [WIDTH_LABEL.get(w, w) for w in r["widths"]]
            print(
                f"  {r['street_id']} {mark}{div} cross={r['cross_m']:9.1f}"
                f" links={r['n_links']:3d} spread={r['spread_m']:6.1f}"
                f" len={r['len_sum']:7.1f} 分類={r['cats']} 幅員={widths}"
            )


if __name__ == "__main__":
    main()
