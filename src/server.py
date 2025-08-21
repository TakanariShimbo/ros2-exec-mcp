#!/usr/bin/env python3
"""
0. ROS2 Exec MCP Server

This server exposes a single tool to execute ROS 2 (ros2) CLI commands via stdio transport.

Environment Variables:
- ROS2_EXEC_TIMEOUT: Default timeout seconds for command execution (default: "30")
- ALLOW_NON_ROS2   : If set to "true", allows executing non-ros2 commands (default: "false")
- DEFAULT_CWD      : Optional default working directory for command execution

Example:
uvx takanarishimbo-ros2-exec-mcp
ROS2_EXEC_TIMEOUT=60 uvx takanarishimbo-ros2-exec-mcp

0. ROS2 実行 MCP サーバー

このサーバーは、1つのツールで ROS 2 (ros2) CLI コマンドを stdio 経由で実行します。

環境変数:
- ROS2_EXEC_TIMEOUT: コマンド実行のデフォルトタイムアウト秒（デフォルト: "30"）
- ALLOW_NON_ROS2   : "true" の場合、ros2 以外のコマンドも許可（デフォルト: "false"）
- DEFAULT_CWD      : コマンド実行時のデフォルト作業ディレクトリ（任意）

例:
uvx takanarishimbo-ros2-exec-mcp
ROS2_EXEC_TIMEOUT=60 uvx takanarishimbo-ros2-exec-mcp
"""

from __future__ import annotations

import os
import shlex
import subprocess
from typing import Annotated, Optional

from pydantic import Field
from mcp.server.fastmcp import FastMCP

"""
1. Environment Configuration / 環境設定
"""
ROS2_EXEC_TIMEOUT = int(os.environ.get("ROS2_EXEC_TIMEOUT", "30"))
ALLOW_NON_ROS2 = os.environ.get("ALLOW_NON_ROS2", "false").lower() == "true"
DEFAULT_CWD = os.environ.get("DEFAULT_CWD")

"""
2. Server Initialization / サーバー初期化
"""
mcp = FastMCP("ros2-exec-mcp")


"""
3. Tool Definition / ツール定義

Tool: ros2_exec
- Executes a ROS 2 CLI command. Only commands beginning with "ros2" are allowed by default.
- Returns combined stdout/stderr and exit code.

入力:
- args (必須): 例 ["topic", "list"] → 実行コマンド: ros2 topic list
- timeout（任意）: タイムアウト秒（既定: ROS2_EXEC_TIMEOUT）
- cwd（任意）: 作業ディレクトリ（既定: DEFAULT_CWD）

備考:
- セキュリティのため、既定では "ros2" で始まらないコマンドは拒否します（ALLOW_NON_ROS2=true で無効化可能）。
"""


@mcp.tool(
    name="ros2_exec",
    description="Execute a ROS 2 CLI command (ros2 ...) and return its output.",
)
def ros2_exec(
    args: Annotated[
        list[str],
        Field(description=("Arguments for the ROS 2 CLI (e.g., ['topic','list'] runs 'ros2 topic list'). " "The 'ros2' program is implied and automatically prefixed.")),
    ],
    timeout: Annotated[
        Optional[int],
        Field(description=f"Timeout in seconds (optional). Default: {ROS2_EXEC_TIMEOUT}", ge=1, le=300),
    ] = None,
    cwd: Annotated[
        Optional[str],
        Field(description=("Working directory to run the command in (optional). " f"Default: {DEFAULT_CWD if DEFAULT_CWD else 'current directory'}")),
    ] = None,
) -> str:
    if not isinstance(args, list) or not all(isinstance(a, str) for a in args):
        raise ValueError("'args' must be a list of strings")

    cmd = ["ros2"] + args

    # If non-ros2 allowed and user accidentally included ros2, dedupe
    if ALLOW_NON_ROS2 and len(args) > 0 and args[0] == "ros2":
        cmd = args

    if not ALLOW_NON_ROS2 and (len(cmd) == 0 or cmd[0] != "ros2"):
        raise ValueError("Command must start with 'ros2' (set ALLOW_NON_ROS2=true to override)")

    t = timeout or ROS2_EXEC_TIMEOUT
    workdir = cwd or DEFAULT_CWD or os.getcwd()

    try:
        proc = subprocess.run(
            cmd,
            cwd=workdir,
            capture_output=True,
            text=True,
            timeout=t,
            check=False,
        )
    except subprocess.TimeoutExpired as e:
        raise TimeoutError(f"ros2 command timed out after {t}s: {shlex.join(cmd)}") from e
    except FileNotFoundError as e:
        raise FileNotFoundError("ros2 executable not found in PATH. Ensure ROS 2 is installed and your environment is sourced.") from e

    stdout = proc.stdout or ""
    stderr = proc.stderr or ""
    code = proc.returncode

    # Return a readable combined result
    result_lines = [
        f"Command: {shlex.join(cmd)}",
        f"Exit code: {code}",
    ]
    if stdout:
        result_lines.append("--- STDOUT ---\n" + stdout.rstrip())
    if stderr:
        result_lines.append("--- STDERR ---\n" + stderr.rstrip())

    return "\n".join(result_lines)


"""
4. Server Startup Function / サーバー起動関数
"""


def main() -> None:
    print("ROS2 Exec MCP Server running on stdio")
    print(f"Default timeout: {ROS2_EXEC_TIMEOUT}s")
    print(f"Allow non-ros2: {ALLOW_NON_ROS2}")
    if DEFAULT_CWD:
        print(f"Default cwd: {DEFAULT_CWD}")
    mcp.run(transport="stdio")
