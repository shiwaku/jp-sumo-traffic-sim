"""MOBIL によるサブレーン車線変更(Phase 2-①、architecture.md §5.3)。

道路幅を 1.75m のサブレーンに分割し(network/lanes.py の n_sublanes)、
車両は連続する複数サブレーンを占有する(二輪1 / 乗用車2 / 大型3)。
変更は「1サブレーンずつ横にずれる」動作で、判断は MOBIL の
安全基準とインセンティブ基準による。すり抜けの表現がここから出る。

## 規約

- サブレーン番号は**進行方向に向かって 0 = 左端**(路肩側)。
- 日本は左側通行なので、バイアスは左寄せに働き追越は右(番号が増える向き)
  に出る。**論文(右側通行)と符号が逆**(architecture.md §5.3)。
  符号は tests/test_mobil.py の test_mobil_left_hand で固定する。
- 変更の決定と加速度計算はステップ内で分ける(architecture.md §5.1 の
  更新順序 4→5)。呼び出し側は apply_mobil() → IDM の順で使う。
  同時に決めると変更先の先行車を二重評価して不安定になる。

## 判断(進行方向 d = -1: 左 / +1: 右)

安全基準: 変更先の新後続・自分の双方が -b_safe より強い減速を強いられない。

インセンティブ基準:
    (a_c_new - a_c_old) + p * (a_n_new - a_n_old) > a_th ± a_bias
右へ出るには a_bias 分だけ余分な利得が要り、左へ戻るのは a_bias 分だけ
軽くなる。A_BIAS > A_TH なので空いていれば無条件に左へ戻る(左寄せ)。

a_c_new は1歩ずれた位置ではなく**隣接車線相当(自車幅ぶん)先の整列**まで
見て評価する。乗用車(幅2)は1歩ずれても元のレーンの先行車と重なったまま
なので、1歩先だけを見ると追越の動機が永遠に発生しない。
"""

from __future__ import annotations

from sapporo_sim.sim.simple import CAR_LEN, Vehicle, idm_acc

# MOBIL パラメータ(Treiber & Kesting の市街地レンジ)。
# a_th / a_bias は Phase 3 でキャリブレーションする(architecture.md §7)
B_SAFE = 4.0  # 安全基準: 変更で強いてよい減速の上限 [m/s²]
POLITENESS = 0.3  # 新後続の損失を自分の利得に算入する割合 p
A_TH = 0.2  # 変更しきい値 [m/s²](ふらつき防止のヒステリシス)
A_BIAS = 0.3  # 左寄せバイアス [m/s²]。A_TH より大きく、空いていれば左へ戻る
SHIFT_COOLDOWN_S = 2.0  # 連続変更の最小間隔 [s]
MIN_GAP = 0.5  # 変更直後に許す最小車間 [m]


def overlaps(s1: int, w1: int, s2: int, w2: int) -> bool:
    """サブレーン区間 [s1, s1+w1) と [s2, s2+w2) が重なるか。"""
    return s1 < s2 + w2 and s2 < s1 + w1


def leader_follower(
    vehicles: list, me: Vehicle, start: int
) -> tuple[Vehicle | None, Vehicle | None]:
    """me が整列 start にいるとした場合の直近先行車と直近後続車。

    me 自身は除く。同一 pos の他車は先行側に数える(安全側)。
    """
    lead = foll = None
    for v in vehicles:
        if v is me or not overlaps(start, me.width, v.sublane, v.width):
            continue
        if v.pos >= me.pos:
            if lead is None or v.pos < lead.pos:
                lead = v
        elif foll is None or v.pos > foll.pos:
            foll = v
    return lead, foll


def acc_to(v: Vehicle, lead: Vehicle | None) -> float:
    """先行車 lead(無ければ自由走行)に対する v の IDM 加速度。"""
    if lead is None:
        return idm_acc(v, 1e9, 0.0)
    return idm_acc(v, lead.pos - CAR_LEN - v.pos, v.speed - lead.speed)


def decide_shift(vehicles: list, me: Vehicle, n_sublanes: int) -> int:
    """me の横移動を決める。-1 = 左へ1サブレーン / 0 = 維持 / +1 = 右。

    両方向の純利得(gain - しきい値)を比べ、大きい方を選ぶ。同点は左
    (番号が減る向き)を優先し、左寄せを既定の状態にする。片方向ずつ
    順に判定すると、追越の途中で「コスト0の左戻り」が常に成立して
    右へ出られないままふらつく。
    """
    if me.t_shift > 0 or me.width >= n_sublanes:
        return 0
    a_old = acc_to(me, leader_follower(vehicles, me, me.sublane)[0])

    best_d, best_net = 0, 0.0
    for d in (-1, 1):
        s1 = me.sublane + d
        if s1 < 0 or s1 + me.width > n_sublanes:
            continue

        # --- 安全基準(1歩ずれた直後の状態で判定) ---
        new_lead, new_foll = leader_follower(vehicles, me, s1)
        if new_lead is not None:
            gap = new_lead.pos - CAR_LEN - me.pos
            if gap < MIN_GAP or idm_acc(me, gap, me.speed - new_lead.speed) < -B_SAFE:
                continue
        if new_foll is not None:
            gap = me.pos - CAR_LEN - new_foll.pos
            if gap < MIN_GAP or idm_acc(new_foll, gap, new_foll.speed - me.speed) < -B_SAFE:
                continue

        # --- インセンティブ: 隣接車線相当(自車幅ぶん)先の整列まで見た最良値 ---
        a_new = -1e9
        s = s1
        for _ in range(me.width):
            if s < 0 or s + me.width > n_sublanes:
                break
            a_new = max(a_new, acc_to(me, leader_follower(vehicles, me, s)[0]))
            s += d

        # --- 新後続の損失(politeness)。自分が新しい先行車になる場合のみ ---
        d_foll = 0.0
        if new_foll is not None:
            cur_lead = leader_follower(vehicles, new_foll, new_foll.sublane)[0]
            if cur_lead is None or cur_lead.pos > me.pos:
                d_foll = acc_to(new_foll, me) - acc_to(new_foll, cur_lead)

        gain = (a_new - a_old) + POLITENESS * d_foll
        threshold = A_TH + (A_BIAS if d > 0 else -A_BIAS)
        net = gain - threshold
        if net > best_net:
            best_d, best_net = d, net
    return best_d


def apply_mobil(vehicles: list, n_sublanes: int, dt: float) -> int:
    """1ステップぶんの車線変更を決めて適用する。戻り値は変更台数。

    vehicles は同一エッジ上の全車両(順不同でよい)。先頭(pos 降順)から
    決定・適用し、後続は確定済みの新しい配置を見て判断する。
    """
    n_shifted = 0
    for me in sorted(vehicles, key=lambda v: -v.pos):
        me.t_shift = max(0.0, me.t_shift - dt)
        d = decide_shift(vehicles, me, n_sublanes)
        if d != 0:
            me.sublane += d
            me.t_shift = SHIFT_COOLDOWN_S
            n_shifted += 1
    return n_shifted
