[English](README.md) | [日本語](README_ja.md) | **README**

# ROS2 Exec MCP Server

A Model Context Protocol (MCP) server that executes ROS 2 (`ros2`) CLI commands via stdio. This follows the same structure and coding rules as the reference uvx datetime MCP server, but provides a single tool to run `ros2` commands.

## Features

- Execute ROS 2 CLI commands (e.g., `ros2 topic list`, `ros2 node list`)
- Configurable default timeout via environment variable
- Optional working directory control
- Secure by default: only allows commands starting with `ros2` (overridable)

## Usage

Configure your MCP client to launch this server with `uvx`:

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

You can also set a default working directory or allow non-ros2 commands:

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

## Environment Variables

- `ROS2_EXEC_TIMEOUT`: Default timeout seconds for command execution (default: `30`)
- `DEFAULT_CWD`: Default working directory for command execution (optional)
- `ALLOW_NON_ROS2`: If set to `true`, allows executing non-`ros2` commands (default: `false`)

## Available Tools

### `ros2_exec`

Execute a ROS 2 CLI command.

Parameters:

- `command` (required): Full command string, e.g., `"ros2 topic list"`
- `timeout` (optional): Timeout seconds (overrides `ROS2_EXEC_TIMEOUT`)
- `cwd` (optional): Working directory (overrides `DEFAULT_CWD`)

Returns combined stdout/stderr and exit code.

## Development

1.  Clone and install dependencies with `uv`:

    ```bash
    uv sync
    ```

2.  Run the server:

    ```bash
    uv run takanarishimbo-ros2-exec-mcp
    ```

3.  Test with MCP Inspector (optional):

    ```bash
    npx @modelcontextprotocol/inspector uv run takanarishimbo-ros2-exec-mcp
    ```

## Publishing to PyPI

This project is set up for PyPI Trusted Publishers via GitHub Actions. See the reference repository’s guide; replace names with `ros2-exec-mcp` and `takanarishimbo-ros2-exec-mcp`.

## Code Quality

Uses `ruff` for linting and formatting:

```bash
uv run ruff check
uv run ruff check --fix
uv run ruff format
```

## Project Structure

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

## License

MIT
