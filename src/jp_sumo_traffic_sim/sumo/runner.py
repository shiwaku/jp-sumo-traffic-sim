"""SUMO 本体(eclipse-sumo パッケージ)の実行ファイルと共通オプション。"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import sumo

SUMO_HOME = Path(sumo.SUMO_HOME)

# 動力学の共通オプション(docs/sumo-design.md §5.2・§5.3)
STEP_LENGTH_S = 0.5  # 自前実装と同じ。0.2s に細分しても飽和交通流率は 1% も変わらない
LATERAL_RESOLUTION_M = 1.75  # サブレーン幅。network.lanes.SUBLANE_W_M と同じ
SIM_OPTIONS = [
    "--step-length",
    str(STEP_LENGTH_S),
    "--lateral-resolution",
    str(LATERAL_RESOLUTION_M),
    "--no-step-log",
    "true",
]


def tool(name: str) -> str:
    """SUMO_HOME/bin の実行ファイルのパス。"""
    exe = name + (".exe" if sys.platform == "win32" else "")
    return str(SUMO_HOME / "bin" / exe)


def env() -> dict:
    """SUMO_HOME を設定した環境変数(XML 検証・型定義の参照に要る)。"""
    e = os.environ.copy()
    e["SUMO_HOME"] = str(SUMO_HOME)
    return e


def run_tool(name: str, args: list[str]) -> subprocess.CompletedProcess:
    """SUMO のツールを実行する。失敗したら標準エラーを添えて例外にする。"""
    proc = subprocess.run(
        [tool(name), *args], capture_output=True, text=True, env=env(), check=False
    )
    if proc.returncode != 0:
        raise RuntimeError(f"{name} failed ({proc.returncode}):\n{proc.stderr[-4000:]}")
    return proc


def warnings_of(proc: subprocess.CompletedProcess) -> list[str]:
    """ツール出力の Warning 行。"""
    return [ln for ln in (proc.stderr + proc.stdout).splitlines() if ln.startswith("Warning")]
