# rosmon2

`rosmon2` is a rosmon-style launcher and terminal process monitor for ROS 2.
It runs launch files through the native ROS 2 `launch` engine, so existing
Python, XML, and YAML launch files keep their normal arguments, substitutions,
and includes.

`rosmon2` is inspired by [xqms/rosmon](https://github.com/xqms/rosmon), but is
an independent ROS 2 implementation and does not require ROS 1.


## Screenshot

![rosmon2 terminal process monitor](docs/rosmon2-terminal.png)


## Installation and quick start

Add this repository to a ROS 2 workspace, install its dependencies, and build
it with `colcon`:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/GibsonHu/rosmon2.git

cd ~/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select rosmon2
source install/setup.bash
```

Then launch the included talker/listener demo:

```bash
mon2 launch rosmon2 demo.launch.py
```

Launch arguments use the standard ROS 2 `name:=value` syntax:

```bash
mon2 launch rosmon2 demo.launch.py namespace:=demo
```

You can launch a file by package and filename, as above, or by path:

```bash
mon2 launch path/to/system.launch.py use_sim_time:=true
```

The `rosmon2` executable is an alias for `mon2`, so this is equivalent:

```bash
rosmon2 launch rosmon2 demo.launch.py
```

## Terminal controls

While a launch is running, the status bar shows each process and its state.
Select a process with its displayed key (`a-z`, `A-Z`, or `0-9`), then press:

| Key | Action |
| --- | --- |
| `s` | Start the selected process |
| `k` | Stop the selected process |
| `m` | Mute the selected process |
| `u` | Unmute the selected process |
| `d` | Start the selected process under `gdb` |

Global controls are available without selecting a process:

| Key | Action |
| --- | --- |
| `F5` | Toggle namespace mode |
| `F6` | Start all processes |
| `F7` | Stop all processes |
| `F8` | Toggle WARN-and-higher output |
| `F9` | Mute all process output |
| `F10` | Unmute all process output |
| `/` | Search nodes by full name |
| `Ctrl-C` | Gracefully stop the complete launch |

Node search matches substrings against full names, including namespaces. Type
`/` to start searching, use `Tab` or the arrow keys to select a match, and
press `Enter` to open its node actions. Press `Escape` to cancel the search.

Namespace mode groups processes by their top-level ROS namespace, including
nodes in child namespaces. Each namespace displays `[alive:dead]` process
counts. Its background is green when all processes are alive, yellow when only
some are alive, and red when all are dead.

Select a namespace with its displayed key, then press:

| Key | Namespace action |
| --- | --- |
| `s` | Start every process in the namespace |
| `k` | Stop every process in the namespace |
| `m` | Mute output from the namespace |
| `u` | Unmute output from the namespace |
| `i` | Inspect and control the individual processes |
| `Backspace` | Return from inspection to the namespace list |

<details>
<summary><strong>Advanced usage</strong></summary>

### Command-line options

Run without the interactive terminal UI:

```bash
mon2 launch --disable-ui rosmon2 demo.launch.py
```

List the arguments declared by a launch file:

```bash
mon2 launch --list-args rosmon2 demo.launch.py
```

Load a launch description and exit, which is useful for benchmarking launch
file parsing:

```bash
mon2 launch --benchmark rosmon2 demo.launch.py
```

Discover processes without leaving them running:

```bash
mon2 launch --no-start rosmon2 demo.launch.py
```

Write combined stdout and stderr to a chosen file:

```bash
mon2 launch --log ./system.log --flush-log my_package system.launch.py
```

By default, process output is also written to a timestamped file under
`/tmp/rosmon2_*.log`. Use `mon2 launch --help` to see every option.

### Agent control and JSON output

Every launch creates a private local Unix-socket session. Name the session so
another terminal, a script, or a coding agent can inspect and control the same
supervisor:

```bash
# Terminal 1
mon2 launch --session hardware rs_launch hardware.launch.py

# Terminal 2
mon2 status --session hardware --json
mon2 logs --session hardware --namespace /ur10e \
  --severity ERROR --since 120 --json
mon2 restart --session hardware \
  --node /ur10e/ur_ros_rtde/command_server --json
mon2 wait --session hardware --namespace /ur10e \
  --state running --timeout 60 --json
```

The available control commands are:

| Command | Purpose |
| --- | --- |
| `status` | Return node, namespace, PID, state, exit-code, and mute information |
| `logs` | Query the in-memory structured process log |
| `events` | Stream live JSON events from the supervisor |
| `start`, `stop`, `restart` | Control a node, namespace, or every process |
| `mute`, `unmute` | Control process output without stopping it |
| `wait` | Wait deterministically for matching processes to reach a state |

Mutating commands require exactly one explicit target:
`--node FULL_NAME`, `--namespace NAMESPACE`, or `--all`. Session names map to
sockets under `$XDG_RUNTIME_DIR/rosmon2`, or `/tmp/rosmon2-$UID` when no XDG
runtime directory is available. The directory and socket are accessible only
to the current user.

The socket API uses protocol version 1: one UTF-8 JSON request and response per
line. For example, sending
`{"command":"restart","node":"/ur10e/command_server"}` performs the same action
as the `mon2 restart` command. An `events` request first returns a subscription
acknowledgement and then streams event objects until disconnected. The CLI is
the supported client and avoids requiring callers to handle socket paths or
protocol framing themselves.

For a continuous, machine-readable launch stream, use:

```bash
mon2 launch --session hardware --json-events \
  rs_launch hardware.launch.py
```

`--json-events` disables the interactive TUI and writes one JSON object per
line. Events include session startup/shutdown, process starts/exits, control
actions, and structured log records. `mon2 events --session hardware` can
subscribe to the same stream from another process. Use `--no-control` only
when no external session socket is wanted.

The TUI, JSON CLI, and MCP server all use the same supervisor:

```text
ROS 2 launch processes
          |
    rosmon2 supervisor
      /       |       \
    TUI   JSON CLI   MCP server
```

</details>

<details>
<summary><strong>MCP usage</strong></summary>

### MCP integration

`rosmon2-mcp` is a dependency-free MCP stdio server implementing protocol
revision `2025-06-18`. It exposes these tools:

- `rosmon2_status`, `rosmon2_logs`, and `rosmon2_wait`
- `rosmon2_start`, `rosmon2_stop`, and `rosmon2_restart`
- `rosmon2_mute` and `rosmon2_unmute`

After building the workspace, register it with Codex from the workspace root.
The explicit setup and runtime paths let Codex use the server when it is
started from another repository:

```bash
rosmon2_setup="$(realpath install/setup.bash)"
if [[ -n "${XDG_RUNTIME_DIR:-}" ]]; then
  rosmon2_runtime_dir="$XDG_RUNTIME_DIR/rosmon2"
else
  rosmon2_runtime_dir="/tmp/rosmon2-$UID"
fi

codex mcp add rosmon2 \
  --env ROSMON2_SETUP="$rosmon2_setup" \
  --env ROSMON2_RUNTIME_DIR="$rosmon2_runtime_dir" \
  -- bash -lc 'source "$ROSMON2_SETUP" && exec rosmon2-mcp'
codex mcp list
```

If `rosmon2` was registered previously with only
`codex mcp add rosmon2 -- rosmon2-mcp`, remove that entry with
`codex mcp remove rosmon2` before registering it again.

### Test with Codex CLI

Start a named rosmon2 session in one terminal:

```bash
source install/setup.bash
mon2 launch --session demo rosmon2 demo.launch.py
```

In a second terminal, from any workspace, ask Codex to inspect the session
through MCP:

```bash
codex exec \
  'Use the rosmon2 MCP server to inspect session "demo". Call rosmon2_status and summarize which processes are running.'
```

Codex should call the read-only `rosmon2_status` tool with
`{"session": "demo"}` and report the demo processes. To test interactively,
run `codex`, enter `/mcp` to confirm that `rosmon2` is active, and then enter
the same request.

The MCP process does not launch or own ROS nodes. It translates typed MCP tool
calls into requests to the named rosmon2 session, so closing the MCP client
does not stop the robot launch. Start rosmon2 with `--session hardware`, then
pass `"session": "hardware"` to the tools. Read-only MCP tools are annotated
accordingly, while start/stop/restart/mute operations require an explicit
node, namespace, or `all: true` target.

</details>

## Building from source

`rosmon2` is an `ament_python` package. If the repository is your workspace
root, build it from that directory. If it is inside a workspace's `src/`
directory, run `colcon build` from the workspace root:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --packages-select rosmon2
source install/setup.bash
```

To run the tests:

```bash
colcon test --packages-select rosmon2
colcon test-result --verbose
```

If packages installed in `~/.local` override your ROS 2 or workspace build
tools, repeat the build with `PYTHONNOUSERSITE=1` in the environment.

## License

`rosmon2` is licensed under the [BSD 3-Clause License](LICENSE).
