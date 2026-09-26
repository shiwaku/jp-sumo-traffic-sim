"""格子線への直接割り当てで街路インベントリを作る。

02 の連鎖クラスタリング(横断距離20m以内を同一街路とみなす)は、
上下線分離や食い違いのある広い街路で隣の通りまで数珠つなぎに連結してしまう。
実測で 130.0m 等間隔の格子が確認できた(reports/03_street_names.json)ので、
リンクを格子線・街区中央線に直接割り当てるほうが正しい。

割り当て先:
  格子線     南n条通 / 北n条通 / 大通 / 西n丁目通 / 東n丁目通
  街区中央線 「A〜Bの裏通り」(格子線の中点)

出力: data/interim/streets.gpkg (layer=roads, street 列付き)
      reports/04_inventory.json
"""

import json
import sys
from pathlib import Path

import geopandas as gpd
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src"))
from jp_sumo_traffic_sim import config as C
from jp_sumo_traffic_sim.cases import sapporo as S

ASSIGN_TOL_M = 35.0  # 割り当て許容差(格子間隔130mの1/4未満)
BEARING_TOL_DEG = 25.0  # グリッド軸からのずれ許容
THROUGH_FRAC = 0.6  # 区域スパンのこの割合以上を通し街路とみなす


def lattice_lines(axis: str) -> list[tuple[float, str, str]]:
    """(横断座標, 名前, 種別) の一覧。格子線とその中点を張る。"""
    if axis == "EW":
        base = [(S.v_south(n), f"南{n}条通") for n in range(1, 12)]
        base += [((S.v_south(1) + S.v_north(1)) / 2, "大通")]
        base += [(S.v_north(n), f"北{n}条通") for n in range(1, 9)]
    else:
        base = [(S.u_west(n), f"西{n}丁目通") for n in range(1, 16)]
        base += [(S.u_east(n), f"東{n}丁目通") for n in range(1, 5)]
    base.sort()
    out = [(c, nm, "格子街路") for c, nm in base]
    for (c0, n0), (c1, n1) in zip(base[:-1], base[1:], strict=False):
        out.append(((c0 + c1) / 2, f"{n0}〜{n1}の裏通り", "裏通り"))
    return sorted(out)


def link_axis_cross(geom):
    """グリッド座標での主軸と横断座標(長さ重み付き)を返す。"""
    g = S.to_grid(geom)
    lines = g.geoms if g.geom_type == "MultiLineString" else [g]
    acc = {"EW": [0.0, 0.0], "NS": [0.0, 0.0]}
    off_sum = wsum = 0.0
    for ls in lines:
        c = np.asarray(ls.coords)[:, :2]
        d = np.diff(c, axis=0)
        L = np.hypot(d[:, 0], d[:, 1])
        if not len(L):
            continue
        ang = np.degrees(np.arctan2(d[:, 1], d[:, 0])) % 180.0
        off = np.minimum(ang % 90.0, 90.0 - ang % 90.0)
        mid = (c[:-1] + c[1:]) / 2
        is_ew = (ang < 45) | (ang > 135)
        for k, m in (("EW", is_ew), ("NS", ~is_ew)):
            if m.any():
                acc[k][0] += float((mid[m, 1 if k == "EW" else 0] * L[m]).sum())
                acc[k][1] += float(L[m].sum())
        off_sum += float((off * L).sum())
        wsum += float(L.sum())
    axis = "EW" if acc["EW"][1] >= acc["NS"][1] else "NS"
    if acc[axis][1] == 0:
        return None, None, None
    return axis, acc[axis][0] / acc[axis][1], (off_sum / wsum if wsum else None)


def main() -> None:
    src = C.INTERIM / "ksj_clip.gpkg"
    roads = gpd.read_file(src, layer="roads")
    inner = roads[roads["in_cordon"]].copy()

    axes, crosses, offs = [], [], []
    for geom in inner.geometry:
        a, c, o = link_axis_cross(geom)
        axes.append(a)
        crosses.append(c)
        offs.append(o)
    inner["axis"] = axes
    inner["cross_m"] = crosses
    inner["off_grid_deg"] = np.round(offs, 1)

    names, kinds, errs = [], [], []
    triples = zip(inner["axis"], inner["cross_m"], inner["off_grid_deg"], strict=False)
    for axis, cross, off in triples:
        if axis is None or off is None or off > BEARING_TOL_DEG:
            names.append(None)
            kinds.append("斜行・非格子")
            errs.append(None)
            continue
        cands = lattice_lines(axis)
        err, nm, kd = min((abs(cross - c), nm, kd) for c, nm, kd in cands)
        if err > ASSIGN_TOL_M:
            names.append(None)
            kinds.append("格子外")
            errs.append(round(err, 1))
        else:
            names.append(nm)
            kinds.append(kd)
            errs.append(round(err, 1))
    inner["street"] = names
    inner["street_kind"] = kinds
    inner["assign_err_m"] = errs

    g = S.CORDON_GRID
    span = {"EW": g["east"] - g["west"], "NS": g["north"] - g["south"]}

    table = []
    for (axis, street), sub in inner.dropna(subset=["street"]).groupby(["axis", "street"]):
        length = float(sub["length_m"].sum())
        table.append(
            dict(
                axis=axis,
                street=street,
                kind=sub["street_kind"].iloc[0],
                n_links=int(len(sub)),
                len_sum_m=round(length, 1),
                coverage=round(length / span[axis], 2),
                through=bool(length >= THROUGH_FRAC * span[axis]),
                cross_mean_m=round(float(sub["cross_m"].mean()), 1),
                cross_spread_m=round(float(sub["cross_m"].max() - sub["cross_m"].min()), 1),
                assign_err_max_m=round(float(sub["assign_err_m"].max()), 1),
                widths=sorted(set(sub["N13_006"].astype(str))),
                cats=sorted(set(sub["N13_003"].astype(str))),
            )
        )
    table.sort(key=lambda r: (r["axis"], r["cross_mean_m"]))

    unassigned = inner[inner["street"].isna()]
    rep = dict(
        assign_tol_m=ASSIGN_TOL_M,
        n_links=int(len(inner)),
        n_assigned=int(inner["street"].notna().sum()),
        n_unassigned=int(len(unassigned)),
        unassigned_by_kind=unassigned["street_kind"].value_counts().to_dict(),
        unassigned_len_km=round(float(unassigned["length_m"].sum()) / 1000, 2),
        n_streets=len(table),
        n_through=sum(1 for r in table if r["through"]),
        n_through_lattice=sum(1 for r in table if r["through"] and r["kind"] == "格子街路"),
        n_through_backstreet=sum(1 for r in table if r["through"] and r["kind"] == "裏通り"),
        max_assign_err_m=round(float(inner["assign_err_m"].max()), 1),
        streets=table,
    )
    inner.to_file(C.INTERIM / "streets.gpkg", layer="roads", driver="GPKG")
    (C.REPORTS / "04_inventory.json").write_text(
        json.dumps(rep, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    print(
        json.dumps({k: v for k, v in rep.items() if k != "streets"}, ensure_ascii=False, indent=2)
    )
    for axis in ("EW", "NS"):
        print(f"\n=== {axis} ===")
        for r in table:
            if r["axis"] != axis:
                continue
            mark = "通" if r["through"] else "  "
            print(
                f"  {mark} {r['street']:<26s} links={r['n_links']:3d}"
                f" 延長={r['len_sum_m']:7.1f}m 被覆={r['coverage']:4.2f}"
                f" 広がり={r['cross_spread_m']:5.1f}m 誤差最大={r['assign_err_max_m']:4.1f}m"
            )


if __name__ == "__main__":
    main()
