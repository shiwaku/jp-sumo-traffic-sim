"""車両パラメータ → SUMO vType(docs/sumo-design.md §5)。

平常時の IDM は Phase 2-① で飽和交通流率 1,845 台/時/車線に合わせた値
(自前実装 sim/simple.py の PARAMS と同じ)。SUMO の IDM でも飽和交通流率が
1,800〜2,000 台/時/車線に入ることを tests/test_sumo_behaviour.py で確認する。
"""

from __future__ import annotations

# architecture.md §5.2 の記号 → 値。v0_factor は自由走行速度(規制速度 × coeff_v)の倍率
PARAMS = {
    "normal": dict(a=1.5, b=1.8, s0=2.0, T=1.1, v0_factor=1.0),
    "winter": dict(a=0.7, b=0.55, s0=5.0, T=1.6, v0_factor=0.7),
}
CAR_LENGTH_M = 4.5  # 自前実装の CAR_LEN と同じ
CAR_WIDTH_M = 1.8  # サブレーン 1.75m で約2本を占める
# 運転者の遵法傾向 coeff_v(自前実装: max(0.7, N(0.95, 0.08)))
COEFF_V = dict(mean=0.95, dev=0.08, lo=0.7, hi=1.3)


def speed_factor(v0_factor: float = 1.0) -> str:
    c = COEFF_V
    f = v0_factor
    return f"normc({c['mean'] * f:.3f},{c['dev'] * f:.3f},{c['lo'] * f:.3f},{c['hi'] * f:.3f})"


def vtype_xml(scenario: str = "normal", vtype_id: str = "car", speed_dev: bool = True) -> str:
    """乗用車の vType 要素。speed_dev=False なら速度のばらつきを付けない(検証用)。"""
    p = PARAMS[scenario]
    sf = speed_factor(p["v0_factor"]) if speed_dev else f"{p['v0_factor']}"
    return (
        f'<vType id="{vtype_id}" vClass="passenger" carFollowModel="IDM" '
        f'accel="{p["a"]}" decel="{p["b"]}" minGap="{p["s0"]}" tau="{p["T"]}" delta="4" '
        f'length="{CAR_LENGTH_M}" width="{CAR_WIDTH_M}" speedFactor="{sf}"/>'
    )
