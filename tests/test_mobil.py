"""MOBIL サブレーン車線変更(sim/mobil.py)の単体テスト。

architecture.md §5.3: a_bias の符号は論文(右側通行)と逆。
左側通行の「左寄せ・右追越」をここで固定する(§8 test_mobil_left_hand)。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from jp_sumo_traffic_sim.sim.mobil import (
    apply_mobil,
    decide_shift,
    leader_follower,
    overlaps,
)
from jp_sumo_traffic_sim.sim.simple import CAR_LEN, DT, Vehicle, idm_acc

V0 = 50 / 3.6


def car(pos, speed, sublane, v0=V0, a=1.5, b=1.8, s0=2.0, T=1.1, width=2):
    return Vehicle(
        pos=pos, speed=speed, v0=v0, a=a, b=b, s0=s0, T=T, turn=0, sublane=sublane, width=width
    )


def run(vehicles, n_sublanes, seconds, observe=None):
    """単路(長さ無制限)で MOBIL + IDM を回す。observe(t) を毎ステップ呼ぶ。"""
    for k in range(int(seconds / DT)):
        apply_mobil(vehicles, n_sublanes, DT)
        accs = []
        for v in vehicles:
            lead = leader_follower(vehicles, v, v.sublane)[0]
            if lead is None:
                accs.append(idm_acc(v, 1e9, 0.0))
            else:
                accs.append(idm_acc(v, lead.pos - CAR_LEN - v.pos, v.speed - lead.speed))
        for v, acc in zip(vehicles, accs, strict=True):
            v.speed = max(0.0, v.speed + acc * DT)
            v.pos += v.speed * DT
        if observe is not None:
            observe(k * DT)


def test_overlaps_and_leader_follower():
    assert overlaps(0, 2, 1, 2)  # 1サブレーンずれは重なる
    assert not overlaps(0, 2, 2, 2)  # 隣の車線は重ならない
    vs = [car(100, 10, 0), car(50, 10, 0), car(75, 10, 2)]
    lead, foll = leader_follower(vs, vs[1], 0)
    assert lead is vs[0] and foll is None  # 別車線(2〜3)の車は見えない
    lead, foll = leader_follower(vs, vs[1], 1)  # 1歩右(1〜2)では見える
    assert lead is vs[2]


def test_mobil_left_hand():
    """左側通行の符号固定: 遅い車は右(番号が増える向き)から追い越す。

    両車とも左寄せで走るため絶対のサブレーン番号は決め打ちせず、
    併走中に「必ず相手より右側(重ならない)」であることを確認する。
    右側通行の実装(符号逆)ならここが左側になって落ちる。
    """
    slow = car(80.0, 5.0, 2, v0=5.0)
    fast = car(0.0, 13.0, 2)
    passing_side = []

    def observe(t):
        if abs(fast.pos - slow.pos) < 10.0:  # 併走区間のみ記録
            passing_side.append(fast.sublane - slow.sublane)

    run([slow, fast], 6, 90.0, observe)
    assert fast.pos > slow.pos + 50.0, "追い越せていない"
    assert passing_side, "併走区間が観測されていない"
    assert min(passing_side) >= slow.width, (
        f"右から抜いていない(左側通行違反): min diff={min(passing_side)}"
    )
    assert fast.sublane == 0, f"追越後に左端へ戻らない: {fast.sublane}"


def test_keep_left_bias():
    """空き道路では右にいる車が左端まで寄る(A_BIAS > A_TH の帰結)。"""
    v = car(0.0, 10.0, 4)
    run([v], 6, 30.0)
    assert v.sublane == 0


def test_safety_blocks_shift():
    """変更先の新後続に -B_SAFE を超える減速を強いる変更はしない。"""
    me = car(50.0, 10.0, 0)
    slow_lead = car(70.0, 5.0, 0, v0=5.0)
    fast_foll = car(30.0, 13.9, 2)  # 右車線を接近中
    assert decide_shift([me, slow_lead, fast_foll], me, 4) == 0

    # 後続が十分遠ければ(安全基準を満たせば)右追越に出る
    far_foll = car(-100.0, 13.9, 2)
    assert decide_shift([me, slow_lead, far_foll], me, 4) == 1


def test_no_overlap_multilane():
    """3車線相当(6サブレーン)の混在交通で車体が重ならない。"""
    vs = [car(15.0 * k, 8.0, (k % 3) * 2, v0=V0 * (0.8 + 0.05 * (k % 5))) for k in range(20)]

    def observe(t):
        for a in vs:
            for b in vs:
                if a is b or not overlaps(a.sublane, a.width, b.sublane, b.width):
                    continue
                assert abs(a.pos - b.pos) >= CAR_LEN - 1e-6, (
                    f"t={t}: 重なり pos={a.pos:.2f}/{b.pos:.2f} lane={a.sublane}/{b.sublane}"
                )

    run(vs, 6, 120.0, observe)


def test_shift_cooldown():
    """変更直後はクールダウンで連続変更しない(ふらつき防止)。"""
    v = car(0.0, 10.0, 4)
    apply_mobil([v], 6, DT)
    assert v.sublane == 3 and v.t_shift > 0
    apply_mobil([v], 6, DT)
    assert v.sublane == 3, "クールダウン中に連続変更した"


def test_narrow_road_no_shift():
    """幅が自車以下(1車線相当)なら変更は起きない。"""
    v = car(0.0, 10.0, 0)
    assert decide_shift([v], v, 2) == 0
