# rosmon2

`rosmon2` is a rosmon-style terminal launcher and process monitor for ROS 2. It
uses the native ROS 2 `launch` engine, so Python, XML, and YAML launch files and
their normal substitutions/includes remain supported.

```bash
conda activate ros_env
# Prevent packages in ~/.local from overriding ros_env's build tools.
export PYTHONNOUSERSITE=1

colcon build --packages-select rosmon2
source install/setup.bash

mon2 launch path/to/system.launch.py use_sim_time:=true
mon2 launch my_package system.launch.py use_sim_time:=true
```

If this repository is itself the workspace root, run those commands from this
directory. If it lives under a workspace `src/` directory, run `colcon build`
from the workspace root as usual.

The interactive UI follows rosmon's controls:

- `a-z`, `A-Z`, `0-9`: select a process; then `s` starts, `k` stops, `m` mutes,
  `u` unmutes, and `d` starts it under gdb.
- `F6` / `F7`: start/stop all processes.
- `F8`: show stderr/WARN+ output only.
- `F9` / `F10`: mute/unmute all process output.
- `Ctrl-C`: gracefully stop the complete launch.

Useful non-interactive forms include:

```bash
mon2 launch --disable-ui my_package system.launch.py
mon2 launch --list-args my_package system.launch.py
mon2 launch --benchmark my_package system.launch.py
mon2 launch --no-start my_package system.launch.py
```

The terminal design and CLI conventions are modeled after
[xqms/rosmon](https://github.com/xqms/rosmon), which targets ROS 1. This package
is an independent ROS 2 implementation and does not depend on ROS 1.
