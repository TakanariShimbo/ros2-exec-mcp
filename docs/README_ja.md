[English](README.md) | [日本語](README_ja.md) | **README**

# ROS2 Exec MCP サーバー

stdio を通じて ROS 2（`ros2`）CLI コマンドを実行する Model Context Protocol (MCP) サーバーです。参照の uvx datetime MCP サーバーと同じ構成・コーディングルールに従い、`ros2` コマンド実行のツールを 1 つだけ提供します。

## 機能

- ROS 2 CLI コマンドの実行（例：`ros2 topic list`、`ros2 node list`）
- 環境変数でデフォルトのタイムアウトを設定
- 作業ディレクトリの指定（任意）
- 既定で安全：`ros2` で始まるコマンドのみ許可（無効化可）

## 使用方法

MCP クライアントで `uvx` を用いてこのサーバーを起動します：

```json
{
  "mcpServers": {
    "ros2": {
      "command": "uvx",
      "args": ["takanarishimbo-ros2-exec-mcp"],
      "env": {
        "ROS2_EXEC_TIMEOUT": "30"
      }
    }
  }
}
```

既定の作業ディレクトリや非 `ros2` コマンドの許可を設定することもできます：

```json
{
  "mcpServers": {
    "ros2": {
      "command": "uvx",
      "args": ["takanarishimbo-ros2-exec-mcp"],
      "env": {
        "ROS2_EXEC_TIMEOUT": "60",
        "DEFAULT_CWD": "/your/ros2/ws",
        "ALLOW_NON_ROS2": "false"
      }
    }
  }
}
```

## 環境変数

- `ROS2_EXEC_TIMEOUT`: コマンド実行のデフォルトタイムアウト秒（デフォルト：`30`）
- `DEFAULT_CWD`: コマンド実行時のデフォルト作業ディレクトリ（任意）
- `ALLOW_NON_ROS2`: `true` で `ros2` 以外のコマンドも許可（デフォルト：`false`）

## 利用可能なツール

### `ros2_exec`

ROS 2 CLI コマンドを実行します。

引数:

- `command`（必須）：コマンド全文の文字列。例：`"ros2 topic list"`
- `timeout`（任意）：タイムアウト秒（`ROS2_EXEC_TIMEOUT` を上書き）
- `cwd`（任意）：作業ディレクトリ（`DEFAULT_CWD` を上書き）

戻り値は stdout/stderr と終了コードをまとめた文字列です。

## 開発

1.  依存関係を `uv` でインストール：

    ```bash
    uv sync
    ```

2.  サーバーを実行：

    ```bash
    uv run takanarishimbo-ros2-exec-mcp
    ```

3.  MCP Inspector でテスト（任意）：

    ```bash
    npx @modelcontextprotocol/inspector uv run takanarishimbo-ros2-exec-mcp
    ```

## PyPI への公開

このプロジェクトは GitHub Actions による PyPI Trusted Publishers に対応しています。参照リポジトリの手順に従い、名称を `ros2-exec-mcp` / `takanarishimbo-ros2-exec-mcp` に置き換えてください。

## コード品質

`ruff` による lint/format：

```bash
uv run ruff check
uv run ruff check --fix
uv run ruff format
```

## プロジェクト構造

```
ros2-exec-mcp/
├── src/
│   ├── __init__.py
│   ├── __main__.py
│   └── server.py
├── pyproject.toml
├── uv.lock
├── .github/
│   └── workflows/
│       └── pypi-publish.yml
├── scripts/
│   └── release.sh
├── docs/
│   ├── README.md
│   └── README_ja.md
└── .gitignore
```

## ライセンス

MIT
