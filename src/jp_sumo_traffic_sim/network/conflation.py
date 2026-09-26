"""実ネットワークへの空間結合(Phase 1-②、architecture.md §4.2)。

共通キーが存在しないため、すべて空間結合。着手順序は 面 → 点 → 線。

- 点(信号機・一時停止): 最近傍ノードへスナップ + 流入リンクの特定
- 線(一方通行・最高速度・センサス区間): バッファ内判定 かつ 方位角差が閾値以内。
  バッファのみだと交差点付近で直交道路を誤マッチするため方位角条件は必須

一方通行の頂点順は「通行方向の逆」と確定済み(reports/15_oneway_check.json)。
呼び出し側が反転せずに渡し、direction 判定でここが反転を担う。
"""

from __future__ import annotations

import math

import numpy as np
from scipy.spatial import cKDTree

PT_SNAP_M = 30.0  # 点規制→ノードの最近傍許容(格子間隔65mの半分未満)
LINE_BUF_M = 15.0  # 線規制→リンクの横断許容(15_verify_oneway と同値)
DOT_TH = 0.85  # 方位角条件: |cos| がこれ以上(≒32°以内)
COVER_MIN = 0.5  # リンク延長のこの割合以上を覆えばマッチ


def _segments(coords: list) -> list[tuple[np.ndarray, np.ndarray, float]]:
    """座標列 → (中点, 単位ベクトル, 長さ) のリスト。長さ0は捨てる。"""
    out = []
    for a, b in zip(coords, coords[1:], strict=False):
        ax, ay = a
        bx, by = b
        ln = math.dist(a, b)
        if ln < 1e-9:
            continue
        out.append(
            (
                np.array([(ax + bx) / 2, (ay + by) / 2]),
                np.array([(bx - ax) / ln, (by - ay) / ln]),
                ln,
            )
        )
    return out


class LineMatcher:
    """規制線・センサス区間の線形を、ネットワークリンクへ照合する。"""

    def __init__(self, reg_lines: list[list], buf_m: float = LINE_BUF_M):
        """reg_lines: feature ごとのパート列。各パートは座標列。

        MultiLineString の feature はパートを複数持つ(先頭パートだけ見ると
        センサス区間の後半を取り落とす)。
        """
        self.buf = buf_m
        self.segs = []  # (feature_idx, mid, vec, len)
        for i, parts in enumerate(reg_lines):
            for part in parts:
                for mid, vec, ln in _segments(part):
                    self.segs.append((i, mid, vec, ln))
        self._tree = cKDTree(np.array([s[1] for s in self.segs])) if self.segs else None
        # 検索半径には規制側セグメントの半長も足す(センサス等は1セグメントが
        # 数百mあり、中点だけでは近傍から漏れる)
        self._max_half = max((s[3] for s in self.segs), default=0.0) / 2

    def match(self, link_coords: list) -> dict[int, tuple[float, float]]:
        """リンク座標列に沿う規制セグメントを探す。

        戻り値: {feature_idx: (被覆長 [m], 方向内積の長さ重み和)}。
        方向内積は「規制線の頂点順」基準。一方通行は頂点順が通行方向の逆
        なので、通行方向との一致は符号を反転して解釈する。
        """
        if self._tree is None:
            return {}
        hits: dict[int, tuple[float, float]] = {}
        for mid, vec, ln in _segments(link_coords):
            # リンクセグメント上を複数点サンプリングして近傍の規制セグメントを集める
            n_sub = max(2, int(ln // (2 * self.buf)))
            samples = [mid + vec * ln * (t - 0.5) for t in np.linspace(0.1, 0.9, n_sub)]
            seen = set()
            for p in samples:
                for si in self._tree.query_ball_point(p, self.buf + ln / 2 + self._max_half):
                    seen.add(si)
            for si in seen:
                fi, rmid, rvec, rlen = self.segs[si]
                dot = float(np.dot(vec, rvec))
                if abs(dot) < DOT_TH:
                    continue
                # 横断距離: 規制セグメント中点からリンクセグメント直線への距離
                d = rmid - mid
                cross = abs(d[0] * vec[1] - d[1] * vec[0])
                if cross > self.buf:
                    continue
                # 縦断の重なり: リンクセグメント方向への射影で概算
                along = float(np.dot(d, vec))
                lo = max(-ln / 2, along - rlen / 2)
                hi = min(ln / 2, along + rlen / 2)
                ov = max(0.0, hi - lo)
                if ov <= 0:
                    continue
                cov, dots = hits.get(fi, (0.0, 0.0))
                hits[fi] = (cov + ov, dots + dot * ov)
        return hits

    def best(self, link_coords: list, cover_min: float = COVER_MIN) -> tuple[int, float] | None:
        """最も長く覆う規制 feature を返す。(feature_idx, 方向符号) or None。

        方向符号: +1 = 規制線の頂点順とリンク座標順が同方向、-1 = 逆方向
        """
        length = sum(s[2] for s in _segments(link_coords))
        hits = self.match(link_coords)
        if not hits:
            return None
        fi, (cov, dots) = max(hits.items(), key=lambda kv: kv[1][0])
        if cov < cover_min * length:
            return None
        return fi, (1.0 if dots >= 0 else -1.0)


class PointMatcher:
    """点規制をノードへスナップし、流入リンクを特定する。"""

    def __init__(self, node_ids: list, node_xy: np.ndarray):
        self.node_ids = list(node_ids)
        self._tree = cKDTree(node_xy) if len(node_xy) else None

    def nearest_node(self, x: float, y: float, tol: float = PT_SNAP_M):
        if self._tree is None:
            return None
        d, i = self._tree.query((x, y))
        return self.node_ids[int(i)] if d <= tol else None

    @staticmethod
    def inflow_link(x: float, y: float, incident: list[tuple[object, list]]):
        """標識座標に最も近い接続リンクを流入リンクとして返す。

        incident: [(リンク識別子, 座標列), ...]
        """
        best_key, best_d = None, float("inf")
        p = np.array([x, y])
        for key, coords in incident:
            for mid, vec, ln in _segments(coords):
                d = p - mid
                along = abs(float(np.dot(d, vec)))
                cross = abs(d[0] * vec[1] - d[1] * vec[0])
                dist = cross if along <= ln / 2 else math.hypot(cross, along - ln / 2)
                if dist < best_d:
                    best_key, best_d = key, dist
        return best_key
