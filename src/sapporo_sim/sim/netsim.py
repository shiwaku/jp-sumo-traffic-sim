"""実ネットワーク上のミクロ交通シミュレーション(Phase 1-④ / Phase 2-②)。

Phase 1 で構築した方向別 Edge(位相・一方通行・車線・速度・信号・一時停止が
結合済み)の上で、simple.py で検証した動力学を回す:

- IDM 追従、赤信号 = 停止線位置の「長さ0の停止車両」
- 満杯 Edge へは停止線で待ち、停止線通過後は下流最後尾に追従(めり込まない)
- 右折は対向直進・左折にギャップ受容(5.5s)で譲る
- 転回は固定比率(0.70/0.15/0.15)を存在する転回先で再正規化
- コードン流入はポアソン到着

Phase 2-② で加わったサブレーン動力学(sim/mobil.py):

- 車両は n_sublanes(実効値)の帯の中で連続サブレーンを占有し(乗用車2)、
  MOBIL で1サブレーンずつ横にずれる(左寄せ・右追越)。IDM の先行車は
  「スパンが重なる直近先行車」
- 右折専用車線: right_turn_lane の Edge は停止線手前 RT_ZONE_M で右端
  RT_SUB サブレーンを右折車専用にする。右折待ちは自スパンだけを塞ぎ、
  直進・左折は左側を通過できる。専用車線が無い(または幅が足りない)
  Edge では右折待ちが後続の直進も止める(実態どおり)
- 冬季は PARAMS の sublane_loss で実効サブレーンを減らす(雪堤は路肩側
  なので、使える帯は中央寄り)。「2車線が実質1.5車線」の表現

残る簡略化: 転回率固定・交差点内の交錯待ち車両が交差方向を塞ぐ
相互作用なし・車種は乗用車のみ(幅2サブレーン)。
"""

from __future__ import annotations

import math
import random
from collections import deque
from dataclasses import dataclass, field

from sapporo_sim import config as C
from sapporo_sim.network.lanes import SUBLANE_W_M
from sapporo_sim.sim.demand import DEFAULT_RATE, hourly_rate
from sapporo_sim.sim.mobil import (
    SHIFT_COOLDOWN_S,
    decide_shift,
    overlaps,
    shift_safe,
)
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

RT_SUB = 2  # 右折専用レーンの幅 [サブレーン](乗用車1台分)
RT_ZONE_M = 60.0  # 停止線からこの距離で右折レーンの出入りを始める
RT_MIN_SUB = 4  # 専用レーンが成立する最小実効サブレーン数(直進1車線分を残す)

# 転回の連鎖で区域内を巡回し続ける車両への上限(design.md §5.5)。
# 超えたら最近傍の流出点へ向ける。コードンの対角は交差点 20〜25 個分
MAX_HOPS = 30


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
    census_id: str = ""  # センサス区間(断面照合・実測需要用。無ければ空)
    n_sub: int = 2  # サブレーン数(データ由来)
    n_sub_eff: int = 2  # 実効サブレーン数(冬季は雪堤で減る)
    rt_lane: bool = False  # 右折専用車線あり(センサス区間代表値)
    # turn(0/1/2) -> [(重み, NEdge), ...] 重み降順。上下分離・並走車道では
    # 同じ転回クラスに複数候補が入る(最良1本だけだと残りの Edge が飢餓する)
    nexts: dict = field(default_factory=dict)
    oncoming: object = None  # NEdge | None
    vehicles: list = field(default_factory=list)
    _cum: list = field(default_factory=list)

    def best(self, turn: int):
        """転回クラス turn の最良候補(NEdge)。無ければ None。"""
        cands = self.nexts.get(turn)
        return cands[0][1] if cands else None

    def span_tail(self, s: int, w: int) -> Vehicle | None:
        """サブレーン区間 [s, s+w) と重なる最後尾(最小 pos)の車両。"""
        rear = None
        for v in self.vehicles:
            if overlaps(s, w, v.sublane, v.width) and (rear is None or v.pos < rear.pos):
                rear = v
        return rear

    def span_tail_space(self, s: int, w: int) -> float:
        rear = self.span_tail(s, w)
        return self.length if rear is None else rear.pos - CAR_LEN

    def green(self, t: float) -> bool:
        return self.to.green(self.axis, t)

    def xy_at(self, pos: float, lat: float = 0.0) -> tuple[float, float]:
        """折れ線に沿った位置 pos [m] の座標。lat は進行方向左向きのオフセット [m]。"""
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
                x, y = x1 + (x2 - x1) * f, y1 + (y2 - y1) * f
                if lat and seg > 0:
                    ux, uy = (x2 - x1) / seg, (y2 - y1) / seg
                    x, y = x - uy * lat, y + ux * lat  # 左法線 = (-uy, ux)
                return (x, y)
        return self.geometry[-1]

    def lateral_of(self, veh: Vehicle) -> float:
        """車両中心の横位置 [m]。進行方向左向きが正。

        左側通行では自方向の車道は中心線の左側にある。双方向道路の
        geometry は道路中心線なので、左オフセットは (実効帯の右端 =
        中心線) から車両中心までの距離。一方通行は全幅が自方向なので
        中心線をまたいで両側に広がる。冬季は実効帯が縮み、雪堤(路肩側)
        から離れて中央寄りを走る。
        """
        c = veh.sublane + veh.width / 2.0
        if self.linked is not None:
            return (self.n_sub_eff - c) * SUBLANE_W_M
        return (self.n_sub_eff / 2.0 - c) * SUBLANE_W_M


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
    """実ネットワーク上の IDM + MOBIL + 信号 + 一時停止のシミュレータ。"""

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
        entry_hourly: dict | None = None,
        entry_hour: int = 8,
    ):
        self.scenario = scenario
        self.p = PARAMS[scenario]
        self.rng = random.Random(seed)
        self.t = 0.0
        self.n_spawned = self.n_exited = self.n_blocked_spawn = 0
        self.n_lane_changes = 0
        self.edge_flow: dict[int, int] = {}  # Edge 下流端の通過台数(断面交通量)

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
        loss_ratio = float(self.p.get("sublane_loss", 0.0))
        self.edges: dict[int, NEdge] = {}
        for e in edges:
            geom = list(e["geometry"])
            h_in = _bearing(geom[-2], geom[-1])
            gx1, gy1 = to_grid(*geom[-2])
            gx2, gy2 = to_grid(*geom[-1])
            axis = "EW" if abs(gx2 - gx1) >= abs(gy2 - gy1) else "NS"
            n_sub = int(e.get("n_sublanes") or 2)
            loss = max(1, int(n_sub * loss_ratio)) if loss_ratio > 0 else 0
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
                n_sub=n_sub,
                n_sub_eff=max(1, n_sub - loss),
                rt_lane=bool(e.get("right_turn_lane")),
                census_id=str(e.get("census_id") or ""),
            )
            self.edges[ne.eid] = ne
        for ne in self.edges.values():
            ne.frm.out.append(ne)

        # --- 転回先と対向 Edge を前計算 ---
        # 各転回クラスに全候補を残し、理想角からのずれで重み付けする
        # (w = 1/(1+ずれ)²)。最良1本だけにすると上下分離・並走車道の
        # 「もう1本」がどの流入からも選ばれず、ネットワークの半分が
        # 飢餓する(Phase 3-① で発覚。到達可能 594/1176 → 1051/1176)
        for ne in self.edges.values():
            cand: dict[int, list] = {0: [], 1: [], 2: []}
            for c in ne.to.out:
                if c.eid == ne.linked:
                    continue  # U ターン(対向 Edge)は除外
                d = _angdiff(c.heading_out, ne.heading_in)
                if abs(d) <= STRAIGHT_DEG:
                    turn, score = 0, abs(d)
                elif d > 0 and d < UTURN_DEG:
                    turn, score = 1, abs(d - 90.0)  # 左折(反時計回り)
                elif d < 0 and d > -UTURN_DEG:
                    turn, score = 2, abs(-d - 90.0)  # 右折
                else:
                    continue
                cand[turn].append((1.0 / (1.0 + score) ** 2, c))
            # 転回先が対向・鋭角折返ししか無いノードは行き止まり = 流出。
            # コードン境界のほか、創成トンネル坑口のような「ネットワークの
            # 外へ続く道」も内部の吸い込み口としてここで流出する。折返しを
            # 許すと上下線を往復する閉回路ができ、デッドロックの温床になる
            ne.nexts = {t: sorted(v, key=lambda x: -x[0]) for t, v in cand.items() if v}
            # 対向: 同じノードに逆向きで入る Edge(自分の対向 Edge の同一リンクは除外)
            onc, onc_score = None, 45.0
            for cand in self.edges.values():
                if cand.to is not ne.to or cand is ne:
                    continue
                d = abs(_angdiff(cand.heading_in, ne.heading_in + math.pi))
                if d < onc_score:
                    onc, onc_score = cand, d
            ne.oncoming = onc

        # --- 流出点への最短ホップ経路(design.md §5.5 の巡回上限用) ---
        # 出口 = nexts が空の Edge(境界の行き止まり)。逆 BFS で各 Edge の
        # 「出口へ向かう次の Edge」を前計算する
        self.exit_next: dict[int, int] = {}
        dist: dict[int, int] = {}
        preds: dict[int, list[int]] = {}
        dq = deque()
        for e in self.edges.values():
            if not e.nexts:
                dist[e.eid] = 0
                dq.append(e.eid)
            for cands in e.nexts.values():
                for _, c in cands:
                    preds.setdefault(c.eid, []).append(e.eid)
        while dq:
            eid = dq.popleft()
            for p in preds.get(eid, []):
                if p not in dist:
                    dist[p] = dist[eid] + 1
                    self.exit_next[p] = eid
                    dq.append(p)

        # --- 流入点: 境界(次数1)ノードから出る Edge ---
        # レートはセンサス時間帯別交通量(entry_hourly、方向平均)を優先し、
        # 裏付けの無い流入(細街路など)は分類の既定値(sim/demand.py)
        entry_nodes = entry_nodes if entry_nodes is not None else self._degree1_nodes()
        self.entries = [e for e in self.edges.values() if e.frm.nid in entry_nodes]
        self.entry_rate = {}
        self.n_census_entries = 0
        for e in self.entries:
            rate = None
            if entry_hourly and e.census_id:
                rate = hourly_rate(entry_hourly.get(e.census_id), entry_hour)
            if rate is None:
                rate = DEFAULT_RATE[e.klass]
            else:
                self.n_census_entries += 1
            self.entry_rate[e.eid] = rate / 3600.0 * demand_scale

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

    # --- 動力学(simple.py の検証済みロジック + サブレーン化) ---

    def _sample_next(self, edge: NEdge) -> tuple[int, NEdge | None]:
        """転回クラスを固定比率で、クラス内の候補を角度適合の重みで引く。"""
        opts = [i for i in range(3) if i in edge.nexts]
        if not opts:
            return 0, None  # 転回先なし = 境界で流出
        r = self.rng.random() * sum(TURN_P[i] for i in opts)
        acc, turn = 0.0, opts[-1]
        for i in opts:
            acc += TURN_P[i]
            if r < acc:
                turn = i
                break
        cands = edge.nexts[turn]
        r2 = self.rng.random() * sum(w for w, _ in cands)
        acc = 0.0
        for w, c in cands:
            acc += w
            if r2 < acc:
                return turn, c
        return turn, cands[-1][1]

    def _make_vehicle(self, edge: NEdge) -> Vehicle:
        coeff = max(0.7, self.rng.gauss(0.95, 0.08))
        turn, nxt = self._sample_next(edge)
        v = Vehicle(
            pos=0.0,
            speed=min(SPAWN_SPEED, edge.v_limit),
            v0=edge.v_limit * coeff * self.p["v0_factor"],
            a=self.p["a"],
            b=self.p["b"],
            s0=self.p["s0"],
            T=self.p["T"],
            turn=turn,
        )
        v.stop_cleared = False
        v.next_edge = nxt
        v.hops = 0
        return v

    def _assign_next(self, veh: Vehicle, edge: NEdge) -> None:
        """edge に入った veh の転回先を決める。巡回上限を超えたら出口へ向ける。"""
        veh.turn, veh.next_edge = self._sample_next(edge)
        if veh.hops < MAX_HOPS:
            return
        ex = self.exit_next.get(edge.eid)
        if ex is None:
            return
        nxt = self.edges[ex]
        for t, cands in edge.nexts.items():
            if any(c is nxt for _, c in cands):
                veh.turn, veh.next_edge = t, nxt
                return

    def _next_edge(self, edge: NEdge, veh: Vehicle) -> NEdge | None:
        """veh の具体的な転回先。サンプリング済みの候補を使う。

        テスト等で veh.turn を後から書き換えた場合(保存済み候補とクラスが
        食い違う場合)は、そのクラスの最良候補に落とす。
        """
        cands = edge.nexts.get(veh.turn)
        if not cands:
            return None
        nxt = getattr(veh, "next_edge", None)
        if nxt is not None and any(c is nxt for _, c in cands):
            return nxt
        return cands[0][1]

    def _entry_sublane(self, nxt: NEdge, veh: Vehicle) -> int:
        """次 Edge へ移るときの整列(現在の整列を実効帯へクランプ)。"""
        return max(0, min(veh.sublane, nxt.n_sub_eff - veh.width))

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

    def _lane_pass(self, e: NEdge) -> None:
        """MOBIL と右折レーンの出入りを決める(加速度計算とは分離)。

        右折専用車線が有効な Edge では、停止線手前 RT_ZONE_M で
        右折車を右端 RT_SUB サブレーンへ寄せ、非右折車を締め出す。
        どちらも安全基準(shift_safe)を満たすまで毎ステップ再試行する。
        """
        n_eff = e.n_sub_eff
        rt_active = e.rt_lane and n_eff >= RT_MIN_SUB
        for veh in e.vehicles:
            veh.t_shift = max(0.0, veh.t_shift - DT)
            if veh.t_shift > 0 or veh.width >= n_eff:
                continue
            in_zone = rt_active and (e.length - veh.pos) <= RT_ZONE_M
            if in_zone and veh.turn == 2:
                # 右折: 専用レーン(右端)へ寄る。インセンティブは問わない
                if veh.sublane + veh.width < n_eff and shift_safe(e.vehicles, veh, veh.sublane + 1):
                    veh.sublane += 1
                    veh.t_shift = SHIFT_COOLDOWN_S
                    self.n_lane_changes += 1
                continue
            if in_zone and veh.sublane + veh.width > n_eff - RT_SUB:
                # 非右折: 専用レーンから左へ抜ける
                if veh.sublane > 0 and shift_safe(e.vehicles, veh, veh.sublane - 1):
                    veh.sublane -= 1
                    veh.t_shift = SHIFT_COOLDOWN_S
                    self.n_lane_changes += 1
                continue
            # 通常の MOBIL。非右折車はゾーン内では専用レーンに入れない
            hi = n_eff - RT_SUB if in_zone else n_eff
            d = decide_shift(e.vehicles, veh, hi)
            if d != 0:
                veh.sublane += d
                veh.t_shift = SHIFT_COOLDOWN_S
                self.n_lane_changes += 1

    def step(self) -> None:
        t = self.t
        for e in self.edges.values():
            e.vehicles.sort(key=lambda v: -v.pos)
            self._lane_pass(e)
            vs = e.vehicles
            stop = e.length - STOPLINE_M
            for i, veh in enumerate(vs):
                lead = None
                for j in range(i - 1, -1, -1):  # 直近の先行車から(スパン重なりのみ)
                    if overlaps(veh.sublane, veh.width, vs[j].sublane, vs[j].width):
                        lead = vs[j]
                        break
                if lead is not None:
                    gap = lead.pos - CAR_LEN - veh.pos
                    dv = veh.speed - lead.speed
                else:
                    # 自スパンの先頭: 信号・満杯・右折待ち・一時停止の判定
                    gap, dv = 1e9, 0.0
                    nxt = self._next_edge(e, veh)
                    tailv = None
                    if nxt is not None:
                        tailv = nxt.span_tail(self._entry_sublane(nxt, veh), veh.width)
                    committed = veh.pos > stop
                    if not committed:
                        space = (
                            (nxt.length if tailv is None else tailv.pos - CAR_LEN)
                            if nxt is not None
                            else 1e9
                        )
                        blocked = nxt is not None and space < CAR_LEN + veh.s0
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
                        elif tailv is not None:
                            gap = (e.length - veh.pos) + tailv.pos - CAR_LEN
                            dv = veh.speed - tailv.speed
                    elif tailv is not None:
                        gap = (e.length - veh.pos) + tailv.pos - CAR_LEN
                        dv = veh.speed - tailv.speed
                acc = idm_acc(veh, gap, dv)
                veh.speed = max(0.0, veh.speed + acc * DT)
                veh.pos += veh.speed * DT
            # 重なり補正(同一スパンの直近先行車に対して)
            for i in range(1, len(vs)):
                for j in range(i - 1, -1, -1):
                    if overlaps(vs[i].sublane, vs[i].width, vs[j].sublane, vs[j].width):
                        cap = vs[j].pos - CAR_LEN - 0.1
                        if vs[i].pos > cap:
                            vs[i].pos = cap
                            vs[i].speed = min(vs[i].speed, vs[j].speed)
                        break

        # 転移(スパン別 FIFO: 前の車が塞いでいれば重なり補正で末端に届かない)
        for e in self.edges.values():
            for veh in [v for v in e.vehicles if v.pos >= e.length]:
                nxt = self._next_edge(e, veh)
                if nxt is None:
                    e.vehicles.remove(veh)
                    self.n_exited += 1
                    self.edge_flow[e.eid] = self.edge_flow.get(e.eid, 0) + 1
                    continue
                entry_s = self._entry_sublane(nxt, veh)
                if nxt.span_tail_space(entry_s, veh.width) < CAR_LEN + 0.2:
                    veh.pos = e.length
                    veh.speed = 0.0
                    continue
                e.vehicles.remove(veh)
                self.edge_flow[e.eid] = self.edge_flow.get(e.eid, 0) + 1
                veh.pos -= e.length
                veh.sublane = entry_s
                veh.hops = getattr(veh, "hops", 0) + 1
                self._assign_next(veh, nxt)
                veh.stop_cleared = False
                veh.t_shift = 0.0
                nxt.vehicles.append(veh)

        # 流入(ポアソン)。左端から順に空いている整列へ入れる
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
                for s in range(max(1, e.n_sub_eff - 1)):
                    if e.span_tail_space(s, 2) > CAR_LEN + 3.0:
                        veh = self._make_vehicle(e)
                        veh.sublane = s
                        e.vehicles.append(veh)
                        self.n_spawned += 1
                        break
                else:
                    self.n_blocked_spawn += 1

        self.t += DT

    # --- 記録 ---

    def snapshot(self) -> dict:
        pts = []
        for e in self.edges.values():
            for veh in e.vehicles:
                x, y = e.xy_at(veh.pos, e.lateral_of(veh))
                pts.append((x, y, veh.speed))
        sig = "".join(n.phase_char(self.t) for n in self.nodes.values() if n.has_signal)
        return {"t": round(self.t, 1), "pts": pts, "sig": sig}

    def stats(self) -> dict:
        n = sum(len(e.vehicles) for e in self.edges.values())
        sp = [v.speed for e in self.edges.values() for v in e.vehicles]
        mean = sum(sp) / len(sp) if sp else 0.0
        spill = 0
        for e in self.edges.values():
            if not e.vehicles:
                continue
            rear = min(e.vehicles, key=lambda v: v.pos)
            if rear.pos < 10.0 and rear.speed < 0.5:
                spill += 1
        return dict(
            n_vehicles=n,
            mean_speed_ms=round(mean, 2),
            n_links_backed_up=spill,
            n_lane_changes=self.n_lane_changes,
        )
