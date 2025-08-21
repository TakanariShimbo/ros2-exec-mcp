[English](README.md) | [日本語](README_ja.md) | **README**

# ROS2 Exec MCP サーバー

ROS 2（`ros2`）CLI コマンドを実行する Model Context Protocol (MCP) サーバーです。

## 機能

- ROS 2 CLI コマンドの実行（例：`ros2 topic list`、`ros2 node list`）
- 環境変数でデフォルトのタイムアウトを設定
- 作業ディレクトリの指定（任意）
- 既定で安全：`ros2` で始まるコマンドのみ許可（無効化可）

## 使用方法

以下は stdio と streamable-http の両方の例です。

### Stdio（デフォルト）

MCP クライアント設定例：

```json
{
  "mcpServers": {
    "ros2": {
      "command": "uvx",
      "args": ["takanarishimbo-ros2-exec-mcp"]
    }
  }
}
```

タイムアウトや既定の作業ディレクトリ、非 `ros2` コマンドの許可を設定することもできます：

```json
{
  "mcpServers": {
    "ros2": {
      "command": "uvx",
      "args": ["takanarishimbo-ros2-exec-mcp"],
      "env": {
        "ROS2_EXEC_TIMEOUT": "60",
        "DEFAULT_CWD": "/your/ros2/ws",
        "ALLOW_NON_ROS2": "true",
        "MCP_TRANSPORT": "stdio"
      }
    }
  }
}
```

### streamable-http

事前にロボット側で MCP サーバーを起動：

```bash
# 必要に応じてホスト/ポートを指定可能
MCP_TRANSPORT=streamable-http MCP_HOST=0.0.0.0 MCP_PORT=8000 \
  uvx takanarishimbo-ros2-exec-mcp
```

MCP クライアント設定例：

```json
{
  "mcpServers": {
    "ros2": {
      "url": "http://xxx.xxx.xxx.xxx:8000/mcp",
      "env": {
        "MCP_TRANSPORT": "streamable-http"
      }
    }
  }
}
```

## 環境変数

- `ROS2_EXEC_TIMEOUT`: コマンド実行のデフォルトタイムアウト秒（デフォルト：`30`）
- `DEFAULT_CWD`: コマンド実行時のデフォルト作業ディレクトリ（任意）
- `ALLOW_NON_ROS2`: `true` で `ros2` 以外のコマンドも許可（デフォルト：`false`）
- `MCP_TRANSPORT`: トランスポート種別。`stdio`（デフォルト）または `streamable-http`
- `MCP_HOST`: `streamable-http` 用の HTTP ホスト/インターフェース（デフォルト：`0.0.0.0`）
- `MCP_PORT`: `streamable-http` 用の HTTP ポート（デフォルト：`8080`）

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

このプロジェクトは PyPI の Trusted Publishers 機能を使用して、GitHub Actions からトークンなしで安全に公開します。

### 1. PyPI Trusted Publisher の設定

1. **PyPI にログイン**（必要に応じてアカウントを作成）

   - https://pypi.org/ にアクセス

2. **公開設定に移動**

   - アカウント設定に移動
   - 「Publishing」をクリックまたは https://pypi.org/manage/account/publishing/ にアクセス

3. **GitHub Publisher を追加**
   - 「Add a new publisher」をクリック
   - 「GitHub」を選択
   - 以下を入力：
     - **Owner**: `TakanariShimbo`（あなたの GitHub ユーザー名/組織）
     - **Repository**: `ros2-exec-mcp`
     - **Workflow name**: `pypi-publish.yml`
     - **Environment**: `pypi`（オプションですが推奨）
   - 「Add」をクリック

### 2. GitHub 環境の設定（推奨）

1. **リポジトリ設定に移動**

   - GitHub リポジトリに移動
   - 「Settings」→「Environments」をクリック

2. **PyPI 環境を作成**
   - 「New environment」をクリック
   - Name: `pypi`
   - 保護ルールの設定（オプション）：
     - 必要なレビュアーを追加
     - 特定のブランチ/タグに制限

### 3. GitHub パーソナルアクセストークンの設定（リリーススクリプト用）

リリーススクリプトは GitHub にプッシュする必要があるため、GitHub トークンが必要です：

1. **GitHub パーソナルアクセストークンの作成**

   - https://github.com/settings/tokens にアクセス
   - 「Generate new token」→「Generate new token (classic)」をクリック
   - 有効期限を設定（推奨：90 日またはカスタム）
   - スコープを選択：
     - ✅ `repo`（プライベートリポジトリのフルコントロール）
   - 「Generate token」をクリック
   - 生成されたトークンをコピー（`ghp_`で始まる）

2. **Git にトークンを設定**

   ```bash
   # オプション1：GitHub CLIを使用（推奨）
   gh auth login

   # オプション2：gitを設定してトークンを使用
   git config --global credential.helper store
   # パスワードを求められたら、代わりにトークンを使用
   ```

### 4. 新しいバージョンのリリース

リリーススクリプトを使用して、自動的にバージョン管理、タグ付け、公開をトリガー：

```bash
# 初回セットアップ
chmod +x scripts/release.sh

# パッチバージョンを増分（0.1.0 → 0.1.1）
./scripts/release.sh patch

# マイナーバージョンを増分（0.1.0 → 0.2.0）
./scripts/release.sh minor

# メジャーバージョンを増分（0.1.0 → 1.0.0）
./scripts/release.sh major

# 特定のバージョンを設定
./scripts/release.sh 1.2.3
```

### 5. 公開の確認

1. **GitHub Actions を確認**

   - リポジトリの「Actions」タブに移動
   - 「Publish to PyPI」ワークフローが正常に完了したことを確認

2. **PyPI パッケージを確認**
   - 訪問：https://pypi.org/project/takanarishimbo-ros2-exec-mcp/
   - または実行：`pip show takanarishimbo-ros2-exec-mcp`

### リリースプロセスフロー

1. `release.sh`スクリプトがすべてのファイルのバージョンを更新
2. git コミットとタグを作成
3. GitHub にプッシュ
4. 新しいタグで GitHub Actions ワークフローがトリガー
5. ワークフローが OIDC を使用して PyPI に認証（トークン不要！）
6. ワークフローがプロジェクトをビルドして PyPI に公開
7. パッケージが`pip install`や`uvx`でグローバルに利用可能になる

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
