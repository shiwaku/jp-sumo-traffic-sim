"""実ネットワーク上のミクロ交通シミュレーション(Phase 1-④)。

Phase 1 で構築した方向別 Edge(位相・一方通行・車線・速度・信号・一時停止が
結合済み)の上で、simple.py で検証した動力学をそのまま回す:

- IDM 追従、赤信号 = 停止線位置の「長さ0の停止車両」
- 満杯 Edge へは停止線で待ち、停止線通過後は下流最後尾に追従(めり込まない)
- 右折は対向直進・左折にギャップ受容(5.5s)で譲る
- 転回は固定比率(0.70/0.15/0.15)を存在する転回先で再正規化
- コードン流入はポアソン到着

実ネットワークで新しくなる点:

- 転回は Edge の方位角差で分類(直進 ±45° / 左折 / 右折。U ターンは除外)
- 信号は has_signal ノードのみ2現示(軸はグリッド方位への回転で分類)。
  無信号ノードは常時青、一時停止(JARTIC 実測)の流入 Edge は停止線で
  一旦停止してから進入する
- 車両は Edge の実ジオメトリ(折れ線)に沿って動く

簡略化(グリッド版から引き継ぎ): 単車線・MOBILなし・転回率固定・
交差点内の交錯待ち車両が交差方向を塞ぐ相互作用なし。
車線数(n_lanes)はまだ動力学に使わない(Phase 2 のサブレーン/MOBIL で使う)。
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from sapporo_sim import config as C
from sapporo_sim.sim.simple import (
    ALL_RED_S,
    CAR_LEN,
    DT,
    PARAMS,
    PROGRESSION_MS,
    RT_CRITICAL_GAP_S,
    SPAWN_SPEED,
    STOPLINE_M,
    TURN_P,
    Vehicle,
    idm_acc,
)

STRAIGHT_DEG = 45.0  # 方位角差がこの範囲なら直進
UTURN_DEG = 135.0  # これを超える転回は U ターンとして除外
STOP_CLEAR_SPEED = 0.3  # 一時停止とみなす速度 [m/s]
STOP_ZONE_M = 3.0  # 停止線からこの距離以内で停止したら発進してよい


@dataclass
class NNode:
    nid: int
    x: float
    y: float
    has_signal: bool = False
    cycle: float = 120.0
    offset: float = 0.0
    g_half: float = 57.0  # 各現示の青時間
    out: list = field(default_factory=list)  # NEdge

    def green(self, axis: str, t: float) -> bool:
        if not self.has_signal:
            return True
        lt = (t - self.offset) % self.cycle
        if axis == "EW":
            return lt < self.g_half
        return self.g_half + ALL_RED_S <= lt < self.cycle - ALL_RED_S

    def phase_char(self, t: float) -> str:
        """1=東西青 / 0=南北青 / 2=全赤"""
        if self.green("EW", t):
            return "1"
        return "0" if self.green("NS", t) else "2"


@dataclass
class NEdge:
    eid: int
    frm: NNode
    to: NNode
    length: float
    geometry: list  # 進行方向順
    v_limit: float  # [m/s]
    klass: str  # arterial / minor(流入需要用)
    axis: str = "EW"  # 下流ノードでの信号軸
    heading_in: float = 0.0  # 終端方位角 [rad]
    heading_out: float = 0.0  # 始端方位角 [rad]
    stop_sign: bool = False  # 一時停止(下流ノード側)
    linked: int | None = None  # 対向 Edge の eid
    nexts: dict = field(default_factory=dict)  # turn(0/1/2) -> NEdge
    oncoming: object = None  # NEdge | None
    vehicles: list = field(default_factory=list)
    _cum: list = field(default_factory=list)

    def tail_space(self) -> float:
        if not self.vehicles:
            return self.length
        return self.vehicles[-1].pos - CAR_LEN

    def green(self, t: float) -> bool:
        return self.to.green(self.axis, t)

    def xy_at(self, pos: float) -> tuple[float, float]:
        """折れ線に沿った位置 pos [m] の座標。"""
        if not self._cum:
            acc = [0.0]
            for p, q in zip(self.geometry, self.geometry[1:], strict=False):
                acc.append(acc[-1] + math.dist(p, q))
            self._cum = acc
        pos = min(max(pos, 0.0), self._cum[-1])
        for k in range(1, len(self._cum)):
            if pos <= self._cum[k] or k == len(self._cum) - 1:
                seg = self._cum[k] - self._cum[k - 1]
                f = 0.0 if seg <= 0 else (pos - self._cum[k - 1]) / seg
                (x1, y1), (x2, y2) = self.geometry[k - 1], self.geometry[k]
                return (x1 + (x2 - x1) * f, y1 + (y2 - y1) * f)
        return self.geometry[-1]


def _bearing(p, q) -> float:
    return math.atan2(q[1] - p[1], q[0] - p[0])


def _angdiff(a: float, b: float) -> float:
    """a-b を (-180, 180] 度で返す。"""
    d = math.degrees(a - b)
    while d <= -180:
        d += 360
    while d > 180:
        d -= 360
    return d


class NetSim:
    """実ネットワーク上の IDM + 信号 + 一時停止のシミュレータ。"""

    def __init__(
        self,
        nodes: list[dict],
        edges: list[dict],
        scenario: str = "normal",
        seed: int = 42,
        demand_scale: float = 1.0,
        signal_plans: dict | None = None,
        stop_edges: set | None = None,
        entry_nodes: set | None = None,
    ):
        self.scenario = scenario
        self.p = PARAMS[scenario]
        self.rng = random.Random(seed)
        self.t = 0.0
        self.n_spawned = self.n_exited = self.n_blocked_spawn = 0

        th = math.radians(C.GRID_BEARING_DEG)

        def to_grid(x: float, y: float) -> tuple[float, float]:
            ox, oy = C.GRID_ORIGIN
            dx, dy = x - ox, y - oy
            return (
                ox + dx * math.cos(-th) - dy * math.sin(-th),
                oy + dx * math.sin(-th) + dy * math.cos(-th),
            )

        # --- ノード ---
        self.nodes: dict[int, NNode] = {}
        for n in nodes:
            self.nodes[n["nid"]] = NNode(
                n["nid"], n["x"], n["y"], has_signal=bool(n.get("has_signal"))
            )

        # --- 信号: 2現示 + 東行き green wave オフセット ---
        plans = signal_plans or {}
        u_min = min(to_grid(n.x, n.y)[0] for n in self.nodes.values())
        self.n_signalized = 0
        for n in nodes:
            node = self.nodes[n["nid"]]
            if not node.has_signal:
                continue
            self.n_signalized += 1
            uid = n.get("signal_uid") or ""
            if uid and uid in plans:
                cycles = sorted(h["cycle_s"] for h in plans[uid]["by_hour"].values())
                node.cycle = cycles[len(cycles) // 2]
            node.g_half = 0.5 * node.cycle - ALL_RED_S
            node.offset = (to_grid(node.x, node.y)[0] - u_min) / PROGRESSION_MS

        # --- Edge ---
        stop_edges = stop_edges or set()
        self.edges: dict[int, NEdge] = {}
        for e in edges:
            geom = list(e["geometry"])
            h_in = _bearing(geom[-2], geom[-1])
            gx1, gy1 = to_grid(*geom[-2])
            gx2, gy2 = to_grid(*geom[-1])
            axis = "EW" if abs(gx2 - gx1) >= abs(gy2 - gy1) else "NS"
            ne = NEdge(
                eid=e["eid"],
                frm=self.nodes[e["frm"]],
                to=self.nodes[e["to"]],
                length=float(e["length"]),
                geometry=geom,
                v_limit=float(e.get("speed_kmh") or 40) / 3.6,
                klass="arterial" if str(e.get("category", "")) in ("1", "2") else "minor",
                axis=axis,
                heading_in=h_in,
                heading_out=_bearing(geom[0], geom[1]),
                stop_sign=e["eid"] in stop_edges,
                linked=None if e.get("linked_edge", -1) in (-1, None) else e["linked_edge"],
            )
            self.edges[ne.eid] = ne
        for ne in self.edges.values():
            ne.frm.out.append(ne)

        # --- 転回先と対向 Edge を前計算 ---
        for ne in self.edges.values():
            best: dict[int, tuple[float, NEdge]] = {}
            for cand in ne.to.out:
                if cand.eid == ne.linked:
                    continue  # U ターン(対向 Edge)は除外
                d = _angdiff(cand.heading_out, ne.heading_in)
                if abs(d) <= STRAIGHT_DEG:
                    turn = 0
                elif d > 0 and d < UTURN_DEG:
                    turn = 1  # 左折(反時計回り)
                elif d < 0 and d > -UTURN_DEG:
                    turn = 2  # 右折
                else:
                    continue
                score = abs(d) if turn == 0 else abs(abs(d) - 90.0)
                if turn not in best or score < best[turn][0]:
                    best[turn] = (score, cand)
            ne.nexts = {k: v[1] for k, v in best.items()}
            # 対向: 同じノードに逆向きで入る Edge(自分の対向 Edge の同一リンクは除外)
            onc, onc_score = None, 45.0
            for cand in self.edges.values():
                if cand.to is not ne.to or cand is ne:
                    continue
                d = abs(_angdiff(cand.heading_in, ne.heading_in + math.pi))
                if d < onc_score:
                    onc, onc_score = cand, d
            ne.oncoming = onc

        # --- 流入点: 境界(次数1)ノードから出る Edge ---
        entry_nodes = entry_nodes if entry_nodes is not None else self._degree1_nodes()
        self.entries = [e for e in self.edges.values() if e.frm.nid in entry_nodes]
        self.entry_rate = {
            e.eid: (500 if e.klass == "arterial" else 150) / 3600.0 * demand_scale
            for e in self.entries
        }

    def _degree1_nodes(self) -> set:
        """無向次数1のノード(ネットワーク末端 = コードン境界の流出入口)。"""
        deg: dict[int, int] = {}
        seen: set[int] = set()
        for e in self.edges.values():
            key = e.eid if e.linked is None else min(e.eid, e.linked)
            if key in seen:
                continue
            seen.add(key)
            for nid in (e.frm.nid, e.to.nid):
                deg[nid] = deg.get(nid, 0) + 1
        return {nid for nid, d in deg.items() if d == 1}

    # --- 動力学(simple.py の検証済みロジックの移植) ---

    def _sample_turn(self, edge: NEdge) -> int:
        opts = [i for i in range(3) if i in edge.nexts]
        if not opts:
            return 0  # 転回先なし = 境界で流出
        r = self.rng.random() * sum(TURN_P[i] for i in opts)
        acc = 0.0
        for i in opts:
            acc += TURN_P[i]
            if r < acc:
                return i
        return opts[-1]

    def _make_vehicle(self, edge: NEdge) -> Vehicle:
        coeff = max(0.7, self.rng.gauss(0.95, 0.08))
        v = Vehicle(
            pos=0.0,
            speed=min(SPAWN_SPEED, edge.v_limit),
            v0=edge.v_limit * coeff * self.p["v0_factor"],
            a=self.p["a"],
            b=self.p["b"],
            s0=self.p["s0"],
            T=self.p["T"],
            turn=self._sample_turn(edge),
        )
        v.stop_cleared = False
        return v

    def _next_edge(self, edge: NEdge, veh: Vehicle) -> NEdge | None:
        return edge.nexts.get(veh.turn)

    def _oncoming_clear(self, edge: NEdge) -> bool:
        onc = edge.oncoming
        if onc is None:
            return True
        stop_o = onc.length - STOPLINE_M
        for v in onc.vehicles:
            if v.turn == 2:
                continue
            d = stop_o - v.pos
            if d < 0:
                return False
            if d / max(v.speed, 0.1) < RT_CRITICAL_GAP_S:
                return False
        return True

    def step(self) -> None:
        t = self.t
        for e in self.edges.values():
            vs = e.vehicles
            stop = e.length - STOPLINE_M
            for k, veh in enumerate(vs):
                if k > 0:
                    lead = vs[k - 1]
                    gap = lead.pos - CAR_LEN - veh.pos
                    dv = veh.speed - lead.speed
                else:
                    gap, dv = 1e9, 0.0
                    nxt = self._next_edge(e, veh)
                    committed = veh.pos > stop
                    if not committed:
                        blocked = nxt is not None and nxt.tail_space() < CAR_LEN + veh.s0
                        yield_rt = veh.turn == 2 and not self._oncoming_clear(e)
                        # 一時停止: 停止線近傍で一旦停止するまで進入しない
                        need_stop = e.stop_sign and not veh.stop_cleared
                        if (
                            need_stop
                            and veh.speed < STOP_CLEAR_SPEED
                            and veh.pos > stop - STOP_ZONE_M
                        ):
                            veh.stop_cleared = True
                            need_stop = False
                        if not e.green(t) or blocked or yield_rt or need_stop:
                            gap, dv = stop - veh.pos, veh.speed
                        elif nxt is not None and nxt.vehicles:
                            tailv = nxt.vehicles[-1]
                            gap = (e.length - veh.pos) + tailv.pos - CAR_LEN
                            dv = veh.speed - tailv.speed
                    elif nxt is not None and nxt.vehicles:
                        tailv = nxt.vehicles[-1]
                        gap = (e.length - veh.pos) + tailv.pos - CAR_LEN
                        dv = veh.speed - tailv.speed
                acc = idm_acc(veh, gap, dv)
                veh.speed = max(0.0, veh.speed + acc * DT)
                veh.pos += veh.speed * DT
            for k in range(1, len(vs)):
                cap = vs[k - 1].pos - CAR_LEN - 0.1
                if vs[k].pos > cap:
                    vs[k].pos = cap
                    vs[k].speed = min(vs[k].speed, vs[k - 1].speed)

        # 転移
        for e in self.edges.values():
            while e.vehicles and e.vehicles[0].pos >= e.length:
                veh = e.vehicles[0]
                nxt = self._next_edge(e, veh)
                if nxt is None:
                    e.vehicles.pop(0)
                    self.n_exited += 1
                    continue
                if nxt.tail_space() < CAR_LEN + 0.2:
                    veh.pos = e.length
                    veh.speed = 0.0
                    break
                e.vehicles.pop(0)
                veh.pos -= e.length
                veh.turn = self._sample_turn(nxt)
                veh.stop_cleared = False
                nxt.vehicles.append(veh)

        # 流入(ポアソン)
        for e in self.entries:
            lam = self.entry_rate[e.eid] * DT
            limit = math.exp(-lam)
            k, pp = 0, 1.0
            while True:
                pp *= self.rng.random()
                if pp <= limit:
                    break
                k += 1
            for _ in range(k):
                if e.tail_space() > CAR_LEN + 3.0:
                    e.vehicles.append(self._make_vehicle(e))
                    self.n_spawned += 1
                else:
                    self.n_blocked_spawn += 1

        self.t += DT

    # --- 記録 ---

    def snapshot(self) -> dict:
        pts = []
        for e in self.edges.values():
            for veh in e.vehicles:
                x, y = e.xy_at(veh.pos)
                pts.append((x, y, veh.speed))
        sig = "".join(n.phase_char(self.t) for n in self.nodes.values() if n.has_signal)
        return {"t": round(self.t, 1), "pts": pts, "sig": sig}

    def stats(self) -> dict:
        n = sum(len(e.vehicles) for e in self.edges.values())
        sp = [v.speed for e in self.edges.values() for v in e.vehicles]
        mean = sum(sp) / len(sp) if sp else 0.0
        spill = sum(
            1
            for e in self.edges.values()
            if e.vehicles and e.vehicles[-1].pos < 10.0 and e.vehicles[-1].speed < 0.5
        )
        return dict(n_vehicles=n, mean_speed_ms=round(mean, 2), n_links_backed_up=spill)
