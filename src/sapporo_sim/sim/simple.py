"""簡易ミクロ交通シミュレーション。

Phase 0 の実測から理想化したグリッド網を組み、IDM + 2現示信号で車両を流す。
手入力データ(停止線・転回制約)が届く前に、ビューワで挙動を確認するためのもの。

## 使う実測値

- 通し街路のインベントリ(reports/04_inventory.json) → 街路の位置と分類
- 格子座標系(config.GRID_BEARING_DEG / u_west / v_south ...) → 幾何
- JARTIC 交差点制御情報(data/processed/signal_plans.json) → サイクル長

## 簡略化(本実装との差)

- 車線は方向別に1本。MOBIL・サブレーンなし
- 信号は全交差点2現示(東西青/南北青)、スプリット0.5。
  JARTIC の20交差点はサイクル長のみ実測値で上書き
- 一方通行を無視する(区域内39本の実測はあるが未適用)
- 転回は固定比率(直進0.70 / 左折0.15 / 右折0.15)
- 満杯リンクへは停止線で待つ。停止線通過後は下流最後尾に追従して交差点内で
  待つが、交差点内の待ち車両が交差方向の流れを塞ぐ相互作用はまだ無い
- 赤信号 = 停止線位置の「長さ0の停止車両」(論文と同じ扱い)
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from sapporo_sim import config as C

DT = 0.5  # 積分ステップ [s]
CAR_LEN = 4.5  # 車長 [m]
STOPLINE_M = 14.0  # 交差点中心から停止線までの距離 [m]
ALL_RED_S = 3.0  # 現示間の全赤(黄を含めた損失) [s]
TURN_P = (0.70, 0.15, 0.15)  # 直進 / 左折 / 右折
SPAWN_SPEED = 8.0  # 流入時の初速 [m/s]

# IDM パラメータ (docs/design.md 2.2)
PARAMS = {
    "normal": dict(a=1.2, b=1.8, s0=2.5, T=1.3, v0_factor=1.0),
    "winter": dict(a=0.7, b=0.55, s0=5.0, T=1.6, v0_factor=0.7),
}
SPEED_LIMIT = {"arterial": 50 / 3.6, "minor": 40 / 3.6}  # [m/s]

# 東行きの系統進行速度(オフセット設計値)。冬も同じ設定のまま使う
# (信号パラメータが季節依存という論点の確認用。docs/design.md 6.5)
PROGRESSION_MS = 40 / 3.6

# 左側通行の転回: 進行方位 → (直進, 左折, 右折) の方位
TURN_MAP = {
    "E": ("E", "N", "S"),
    "W": ("W", "S", "N"),
    "N": ("N", "W", "E"),
    "S": ("S", "E", "W"),
}


@dataclass
class Node:
    nid: int
    u: float
    v: float
    cycle: float = 120.0
    offset: float = 0.0
    g_ew: float = 0.0  # 東西青の長さ [s]
    g_ns: float = 0.0
    out: dict = field(default_factory=dict)  # 方位 -> Link

    def ew_green(self, t: float) -> bool:
        lt = (t - self.offset) % self.cycle
        return lt < self.g_ew

    def ns_green(self, t: float) -> bool:
        lt = (t - self.offset) % self.cycle
        return self.g_ew + ALL_RED_S <= lt < self.cycle - ALL_RED_S


@dataclass
class Vehicle:
    pos: float
    speed: float
    v0: float
    a: float
    b: float
    s0: float
    T: float
    turn: int  # 0=直進 1=左折 2=右折


@dataclass
class Link:
    lid: int
    frm: Node
    to: Node
    heading: str  # E/W/N/S
    length: float
    klass: str  # arterial / minor
    vehicles: list = field(default_factory=list)  # 先頭(下流)が index 0

    @property
    def axis(self) -> str:
        return "EW" if self.heading in ("E", "W") else "NS"

    def tail_space(self) -> float:
        """リンク上流端の空き [m]。"""
        if not self.vehicles:
            return self.length
        last = self.vehicles[-1]
        return last.pos - CAR_LEN

    def green(self, t: float) -> bool:
        return self.to.ew_green(t) if self.axis == "EW" else self.to.ns_green(t)


def idm_acc(v: Vehicle, gap: float, dv: float) -> float:
    gap = max(gap, 0.1)
    s_star = v.s0 + max(0.0, v.speed * v.T + v.speed * dv / (2 * math.sqrt(v.a * v.b)))
    acc = v.a * (1 - (v.speed / v.v0) ** 4 - (s_star / gap) ** 2)
    return max(acc, -9.0)


class GridSim:
    """理想化グリッド上の IDM + 信号のシミュレータ。"""

    def __init__(
        self,
        scenario: str = "normal",
        seed: int = 42,
        demand_scale: float = 1.0,
        signal_plans: dict | None = None,
        street_class: dict | None = None,
    ):
        self.scenario = scenario
        self.p = PARAMS[scenario]
        self.rng = random.Random(seed)
        self.t = 0.0
        self.n_spawned = 0
        self.n_exited = 0
        self.n_blocked_spawn = 0

        # --- 幾何: 通し格子街路の座標 ---
        self.us = [C.u_west(n) for n in range(11, 0, -1)] + [C.u_east(1)]
        self.vs = (
            [C.v_south(n) for n in range(7, 0, -1)]
            + [(C.v_south(1) + C.v_north(1)) / 2]
            + [C.v_north(n) for n in range(1, 6)]
        )
        self.ns_names = [f"西{n}丁目通" for n in range(11, 0, -1)] + ["東1丁目通"]
        self.ew_names = (
            [f"南{n}条通" for n in range(7, 0, -1)] + ["大通"] + [f"北{n}条通" for n in range(1, 6)]
        )
        street_class = street_class or {}

        # --- ノード ---
        self.nodes: list[Node] = []
        self.grid: dict[tuple[int, int], Node] = {}
        for j, v in enumerate(self.vs):
            for i, u in enumerate(self.us):
                n = Node(len(self.nodes), u, v)
                self.nodes.append(n)
                self.grid[(i, j)] = n

        # --- 信号: 2現示、東行きの green wave オフセット ---
        u0 = self.us[0]
        for n in self.nodes:
            n.offset = (n.u - u0) / PROGRESSION_MS
        # JARTIC のサイクル長で上書き(最近傍40m以内)
        n_matched = 0
        for plan in (signal_plans or {}).values():
            pu, pv = self._to_grid_uv(plan["lon"], plan["lat"])
            best = min(self.nodes, key=lambda n: (n.u - pu) ** 2 + (n.v - pv) ** 2)
            if (best.u - pu) ** 2 + (best.v - pv) ** 2 <= 40.0**2:
                cycles = [h["cycle_s"] for h in plan["by_hour"].values()]
                cycles.sort()
                best.cycle = cycles[len(cycles) // 2]
                n_matched += 1
        self.n_jartic_matched = n_matched
        for n in self.nodes:
            n.g_ew = 0.5 * n.cycle - ALL_RED_S
            n.g_ns = 0.5 * n.cycle - ALL_RED_S

        # --- リンク(両方向) ---
        self.links: list[Link] = []

        def add_link(a: Node, b: Node, heading: str, klass: str) -> None:
            length = abs((b.u - a.u) + (b.v - a.v))
            ln = Link(len(self.links), a, b, heading, length, klass)
            self.links.append(ln)
            a.out[heading] = ln

        for j, name in enumerate(self.ew_names):
            klass = street_class.get(name, "minor")
            for i in range(len(self.us) - 1):
                add_link(self.grid[(i, j)], self.grid[(i + 1, j)], "E", klass)
                add_link(self.grid[(i + 1, j)], self.grid[(i, j)], "W", klass)
        for i, name in enumerate(self.ns_names):
            klass = street_class.get(name, "minor")
            for j in range(len(self.vs) - 1):
                add_link(self.grid[(i, j)], self.grid[(i, j + 1)], "N", klass)
                add_link(self.grid[(i, j + 1)], self.grid[(i, j)], "S", klass)

        # --- 流入点: 境界ノードから内向きのリンク ---
        imax, jmax = len(self.us) - 1, len(self.vs) - 1
        self.entries: list[Link] = []
        for j in range(jmax + 1):
            self.entries.append(self.grid[(0, j)].out["E"])
            self.entries.append(self.grid[(imax, j)].out["W"])
        for i in range(imax + 1):
            self.entries.append(self.grid[(i, 0)].out["N"])
            self.entries.append(self.grid[(i, jmax)].out["S"])
        self.entry_rate = {
            ln.lid: (500 if ln.klass == "arterial" else 150) / 3600.0 * demand_scale
            for ln in self.entries
        }

    # --- 座標変換 ---

    @staticmethod
    def _rot(u: float, v: float, sign: float) -> tuple[float, float]:
        th = math.radians(C.GRID_BEARING_DEG) * sign
        ox, oy = C.GRID_ORIGIN
        du, dv = u - ox, v - oy
        return (
            ox + du * math.cos(th) - dv * math.sin(th),
            oy + du * math.sin(th) + dv * math.cos(th),
        )

    def grid_to_proj(self, u: float, v: float) -> tuple[float, float]:
        return self._rot(u, v, +1.0)

    def _to_grid_uv(self, lon: float, lat: float) -> tuple[float, float]:
        from pyproj import Transformer

        if not hasattr(self, "_tf"):
            self._tf = Transformer.from_crs("EPSG:6668", C.CRS_PROJ, always_xy=True)
        x, y = self._tf.transform(lon, lat)
        return self._rot(x, y, -1.0)

    def veh_xy(self, ln: Link, pos: float) -> tuple[float, float]:
        f = pos / ln.length
        u = ln.frm.u + (ln.to.u - ln.frm.u) * f
        v = ln.frm.v + (ln.to.v - ln.frm.v) * f
        return self.grid_to_proj(u, v)

    # --- 動力学 ---

    def _v0_for(self, ln: Link) -> float:
        coeff = max(0.7, self.rng.gauss(0.95, 0.08))
        return SPEED_LIMIT[ln.klass] * coeff * self.p["v0_factor"]

    def _make_vehicle(self, ln: Link) -> Vehicle:
        r = self.rng.random()
        turn = 0 if r < TURN_P[0] else (1 if r < TURN_P[0] + TURN_P[1] else 2)
        return Vehicle(
            pos=0.0,
            speed=min(SPAWN_SPEED, SPEED_LIMIT[ln.klass]),
            v0=self._v0_for(ln),
            a=self.p["a"],
            b=self.p["b"],
            s0=self.p["s0"],
            T=self.p["T"],
            turn=turn,
        )

    def _next_link(self, ln: Link, veh: Vehicle) -> Link | None:
        """転回先のリンク。None = コードン外へ流出。

        境界ノードには外向きのリンクが無いが、実際の街路はコードンの外へも
        続いている。無い方向への転回はそのまま区域外への流出として扱う
        (直進への差し替えは外周の転回率を直進1.0に歪めるためしない)。
        """
        heading = TURN_MAP[ln.heading][veh.turn]
        return ln.to.out.get(heading)

    def step(self) -> None:
        t = self.t
        # 1. 加速度と位置の更新
        for ln in self.links:
            vs = ln.vehicles
            stop = ln.length - STOPLINE_M
            for k, veh in enumerate(vs):
                if k > 0:
                    lead = vs[k - 1]
                    gap = lead.pos - CAR_LEN - veh.pos
                    dv = veh.speed - lead.speed
                else:
                    gap, dv = 1e9, 0.0
                    nxt = self._next_link(ln, veh)
                    committed = veh.pos > stop
                    if not committed:
                        blocked = nxt is not None and nxt.tail_space() < CAR_LEN + veh.s0
                        if not ln.green(t) or blocked:
                            # 赤信号(または満杯) = 停止線位置の長さ0の停止車両
                            gap, dv = stop - veh.pos, veh.speed
                        elif nxt is not None and nxt.vehicles:
                            tailv = nxt.vehicles[-1]
                            gap = (ln.length - veh.pos) + tailv.pos - CAR_LEN
                            dv = veh.speed - tailv.speed
                    elif nxt is not None and nxt.vehicles:
                        # 停止線通過後も下流最後尾へ追従する。
                        # 下流が満杯なら手前(交差点内)で止まり、めり込まない
                        tailv = nxt.vehicles[-1]
                        gap = (ln.length - veh.pos) + tailv.pos - CAR_LEN
                        dv = veh.speed - tailv.speed
                acc = idm_acc(veh, gap, dv)
                veh.speed = max(0.0, veh.speed + acc * DT)
                veh.pos += veh.speed * DT
            # 数値誤差での追い越しを禁止(最小車間 CAR_LEN + 0.1m を常に保証)。
            # 転移時に CAR_LEN + 0.2m の空きを要求するので cap は負にならない
            for k in range(1, len(vs)):
                cap = vs[k - 1].pos - CAR_LEN - 0.1
                if vs[k].pos > cap:
                    vs[k].pos = cap
                    vs[k].speed = min(vs[k].speed, vs[k - 1].speed)

        # 2. リンク間の転移
        for ln in self.links:
            while ln.vehicles and ln.vehicles[0].pos >= ln.length:
                veh = ln.vehicles[0]
                nxt = self._next_link(ln, veh)
                if nxt is None:
                    ln.vehicles.pop(0)
                    self.n_exited += 1
                    continue
                if nxt.tail_space() < CAR_LEN + 0.2:
                    # 下流に物理的な空きが無い間はリンク下流端で待つ
                    # (追従で普通は止まる。1ステップ内の行き過ぎの保険)
                    veh.pos = ln.length
                    veh.speed = 0.0
                    break
                ln.vehicles.pop(0)
                veh.pos -= ln.length
                r = self.rng.random()
                veh.turn = 0 if r < TURN_P[0] else (1 if r < TURN_P[0] + TURN_P[1] else 2)
                nxt.vehicles.append(veh)

        # 3. コードン流入(ポアソン到着)
        for ln in self.entries:
            lam = self.entry_rate[ln.lid] * DT
            if self.rng.random() < lam:
                if ln.tail_space() > CAR_LEN + 3.0:
                    ln.vehicles.append(self._make_vehicle(ln))
                    self.n_spawned += 1
                else:
                    self.n_blocked_spawn += 1

        self.t += DT

    # --- 記録 ---

    def snapshot(self) -> dict:
        pts = []
        for ln in self.links:
            for veh in ln.vehicles:
                x, y = self.veh_xy(ln, min(veh.pos, ln.length))
                pts.append((x, y, veh.speed))
        sig = "".join("1" if n.ew_green(self.t) else "0" for n in self.nodes)
        return {"t": round(self.t, 1), "pts": pts, "sig": sig}

    def stats(self) -> dict:
        n = sum(len(ln.vehicles) for ln in self.links)
        sp = [v.speed for ln in self.links for v in ln.vehicles]
        mean = sum(sp) / len(sp) if sp else 0.0
        # スピルバック候補: 待ち行列がリンク上流端に達しているリンク
        spill = sum(
            1
            for ln in self.links
            if ln.vehicles and ln.vehicles[-1].pos < 10.0 and ln.vehicles[-1].speed < 0.5
        )
        return dict(n_vehicles=n, mean_speed_ms=round(mean, 2), n_links_backed_up=spill)
