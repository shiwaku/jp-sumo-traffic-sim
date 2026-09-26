"""SUMO の挙動テスト(docs/sumo-design.md §10)。

自前実装のテスト(左側通行の追越・飽和交通流率・右折の譲り)と同じシナリオを
SUMO で再現する。ネットワークは本番と同じ変換(sumo.build)で作る。
"""

import traci
from sumo_util import build, close, edge, node, start, two_way, write_routes


def test_lefthand_overtake_on_right(tmp_path):
    """遅い先行車を**右側から**追い越し、抜いたら左(路肩側)に戻る。"""
    nodes = {1: node(1, 0, 0), 2: node(2, 2000, 0)}
    stats = build(tmp_path, list(nodes.values()), [edge(10, 1, 2, nodes, n_lanes=2)])
    routes = write_routes(
        tmp_path,
        '  <vType id="slow" vClass="passenger" maxSpeed="5" length="4.5"/>\n'
        '  <route id="r" edges="10"/>\n'
        '  <vehicle id="slow" type="slow" depart="0" departLane="0" departPos="150" route="r"/>\n'
        '  <vehicle id="fast" type="car" depart="0" departLane="0" departPos="0" '
        'departSpeed="max" route="r"/>',
    )
    start(stats["net"], routes)
    try:
        lanes_while_passing, passed, back_left = set(), False, False
        for _ in range(600):
            traci.simulationStep()
            ids = traci.vehicle.getIDList()
            if "fast" not in ids or "slow" not in ids:
                break
            xf = traci.vehicle.getPosition("fast")[0]
            xs = traci.vehicle.getPosition("slow")[0]
            lane = traci.vehicle.getLaneIndex("fast")
            if abs(xf - xs) < 10:
                lanes_while_passing.add(lane)
            if xf > xs + 10:
                passed = True
            if passed and xf > xs + 80 and lane == 0:
                back_left = True
                break
        assert passed, "追い越せていない"
        # 左側通行: lane 0 = 路肩側(左)。追越は中央側(右)の lane 1 で行う
        assert lanes_while_passing == {1}, f"並走時の車線 {lanes_while_passing}"
        assert back_left, "追越後に左へ戻っていない"
    finally:
        close()


def test_saturation_flow(tmp_path):
    """赤で詰めた待ち行列を青で発進させ、7〜41 台目の車頭時間から飽和交通流率を測る。

    日本の基準値の帯 1,800〜2,000 台/時/車線(自前実装の test_saturation_flow と同じ条件:
    規制速度 50 km/h、待ち行列 45 台)。
    """
    nodes = {1: node(1, -800, 0), 2: node(2, 0, 0, signal=True), 3: node(3, 400, 0)}
    edges = [edge(10, 1, 2, nodes), edge(11, 2, 3, nodes)]
    stats = build(tmp_path, list(nodes.values()), edges)
    n_queue = 45
    body = '  <route id="r" edges="10 11"/>\n' + "\n".join(
        f'  <vehicle id="v{k:02d}" type="car" depart="{k}" departSpeed="max" route="r"/>'
        for k in range(n_queue)
    )
    routes = write_routes(tmp_path, body)
    # 停止線直後(交差点の出口)で通過時刻を取る。手前に置くと赤の間に先頭車が通過してしまう
    add = tmp_path / "det.add.xml"
    add.write_text(
        '<additional><inductionLoop id="stop" lane="11_0" pos="1.0" '
        'period="1000" file="NUL"/></additional>',
        encoding="utf-8",
    )
    start(stats["net"], routes, ["-a", str(add)])
    try:
        traci.trafficlight.setRedYellowGreenState("2", "r")
        entry: dict[str, float] = {}
        for _ in range(int(120 / 0.5)):  # 赤 120 秒で全車を停止線に詰める
            traci.simulationStep()
        traci.trafficlight.setRedYellowGreenState("2", "G")
        for _ in range(int(240 / 0.5)):
            traci.simulationStep()
            for vid, _l, t_in, _t_out, _typ in traci.inductionloop.getVehicleData("stop"):
                entry.setdefault(vid, t_in)
    finally:
        close()
    times = [entry[f"v{k:02d}"] for k in range(n_queue) if f"v{k:02d}" in entry]
    assert len(times) == n_queue, "待ち行列が捌け切っていない"
    heads = [b - a for a, b in zip(times[6:41], times[7:42], strict=False)]
    flow = 3600.0 / (sum(heads) / len(heads))
    print(f"saturation_flow={flow:.0f}")
    assert 1800 <= flow <= 2000, f"飽和交通流率 {flow:.0f} 台/時"


def _cross(tmp_path):
    """東西(国道)× 南北(市道)の信号交差点。東西が A 軸(先に青)。"""
    nodes = {
        0: node(0, 0, 0, signal=True),
        1: node(1, -400, 0),
        2: node(2, 400, 0),
        3: node(3, 0, 400),
        4: node(4, 0, -400),
    }
    edges = (
        two_way(10, 1, 0, nodes, category="1")
        + two_way(20, 0, 2, nodes, category="1")
        + two_way(30, 3, 0, nodes)
        + two_way(40, 0, 4, nodes)
    )
    return nodes, build(tmp_path, list(nodes.values()), edges)


def test_right_turn_yields_to_oncoming(tmp_path):
    """青中の右折(従属青 g)は対向直進に譲る。対向があると右折の捌け量が大きく落ちる。"""

    def right_turns_done(oncoming_vph: int, sub) -> tuple[int, int]:
        d = tmp_path / sub
        d.mkdir()
        _, stats = _cross(d)
        # 西→東の流入(10)から右折 = 南へ(40)。対向 = 東→西の直進(21→11)
        body = (
            '  <route id="rt" edges="10 40"/>\n  <route id="onc" edges="21 11"/>\n'
            '  <flow id="rt" type="car" route="rt" begin="0" end="600" vehsPerHour="600" '
            'departSpeed="max"/>\n'
        )
        if oncoming_vph:
            body += (
                f'  <flow id="onc" type="car" route="onc" begin="0" end="600" '
                f'vehsPerHour="{oncoming_vph}" departSpeed="max"/>'
            )
        routes = write_routes(d, body)
        start(str(stats["net"]), routes, label=sub)
        try:
            traci.trafficlight.setPhase("0", 0)  # A(東西)青で固定
            traci.trafficlight.setPhaseDuration("0", 10000)
            state = traci.trafficlight.getRedYellowGreenState("0")
            done, collisions = 0, 0
            for _ in range(int(600 / 0.5)):
                traci.simulationStep()
                done += sum(1 for v in traci.simulation.getArrivedIDList() if v.startswith("rt"))
                collisions += traci.simulation.getCollidingVehiclesNumber()
        finally:
            traci.close()
        assert "g" in state, f"右折が従属青になっていない: {state}"
        return done, collisions

    free, c0 = right_turns_done(0, "free")
    busy, c1 = right_turns_done(1500, "busy")
    assert c0 == 0 and c1 == 0, "衝突が発生した"
    print(f"right_turns free={free} busy={busy}")
    assert busy < 0.5 * free, f"対向ありの右折 {busy} 台 / 対向なし {free} 台"


def test_signal_offset_is_start_of_a_green(tmp_path):
    """offset = A 青の開始時刻(自前実装の lt = (t - offset) % cycle と同じ向き)。"""
    nodes, _ = {}, None
    d = tmp_path / "off"
    d.mkdir()
    nodes = {
        0: node(0, 0, 0, signal=True),
        1: node(1, -400, 0),
        2: node(2, 400, 0),
        3: node(3, 0, 400),
        4: node(4, 0, -400),
    }
    edges = (
        two_way(10, 1, 0, nodes, category="1")
        + two_way(20, 0, 2, nodes, category="1")
        + two_way(30, 3, 0, nodes)
        + two_way(40, 0, 4, nodes)
    )
    stats = build(d, list(nodes.values()), edges, offsets={0: 30.0})
    routes = write_routes(d, "")
    start(stats["net"], routes, label="off")
    try:
        phases = {}
        for _ in range(int(40 / 0.5)):
            traci.simulationStep()
            phases[round(traci.simulation.getTime(), 1)] = traci.trafficlight.getPhase("0")
    finally:
        close()
    assert phases[29.0] == 5, f"t=29 の現示 {phases[29.0]}(直前サイクルの全赤のはず)"
    assert phases[31.0] == 0, f"t=31 の現示 {phases[31.0]}(A 青のはず)"
