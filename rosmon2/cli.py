"""Command line entry point compatible with ``mon launch`` conventions."""

import argparse
import asyncio
from datetime import datetime
from pathlib import Path
import signal
import sys

from ament_index_python.packages import PackageNotFoundError
from launch.launch_description_sources import get_launch_description_from_any_launch_file
from ros2launch.api import get_share_file_path_from_package
from ros2launch.api import print_arguments_of_launch_file

from .supervisor import Supervisor


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='mon2',
        description='rosmon-style ROS 2 launch process monitor',
        usage='mon2 launch [options] PACKAGE FILE [name:=value ...]\n'
              '       mon2 launch [options] path/to/file.launch.py [name:=value ...]',
    )
    subparsers = parser.add_subparsers(dest='command')
    launch_parser = subparsers.add_parser('launch', help='launch and monitor a ROS 2 launch file')
    launch_parser.add_argument('--disable-ui', action='store_true',
                               help='disable the interactive terminal UI')
    launch_parser.add_argument('--benchmark', action='store_true',
                               help='exit after loading the launch file')
    launch_parser.add_argument('--list-args', action='store_true',
                               help='list launch file arguments')
    launch_parser.add_argument('--log', metavar='FILE', help='write combined process log to FILE')
    launch_parser.add_argument('--flush-log', action='store_true')
    launch_parser.add_argument('--flush-stdout', action='store_true')
    launch_parser.add_argument('--name', help='accepted for rosmon CLI compatibility')
    launch_parser.add_argument('--no-start', action='store_true',
                               help="discover processes but don't leave them running")
    launch_parser.add_argument('--stop-timeout', type=float, default=5.0, metavar='SECONDS')
    launch_parser.add_argument(
        '--disable-diagnostics', action='store_true', help=argparse.SUPPRESS)
    launch_parser.add_argument('--diagnostics-prefix', help=argparse.SUPPRESS)
    launch_parser.add_argument('--cpu-limit', help=argparse.SUPPRESS)
    launch_parser.add_argument('--memory-limit', help=argparse.SUPPRESS)
    launch_parser.add_argument('--output-attr', choices=['obey', 'ignore'], help=argparse.SUPPRESS)
    launch_parser.add_argument('--auto-increment-spawn-delay', type=float, help=argparse.SUPPRESS)
    launch_parser.add_argument('launch_spec', nargs='+', metavar='LAUNCH')
    return parser


def resolve_launch_spec(parts):
    """Resolve PATH or PACKAGE FILE followed by ROS launch arguments."""
    first_argument = next((i for i, part in enumerate(parts) if ':=' in part), len(parts))
    spec, launch_arguments = parts[:first_argument], parts[first_argument:]
    if any(':=' not in item for item in launch_arguments):
        raise ValueError('a non-argument was specified after a name:=value argument')
    if len(spec) == 1:
        path = Path(spec[0]).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"launch file '{spec[0]}' does not exist")
        return str(path), launch_arguments
    if len(spec) == 2:
        package, filename = spec
        try:
            return get_share_file_path_from_package(
                package_name=package, file_name=filename), launch_arguments
        except PackageNotFoundError as exc:
            raise FileNotFoundError(f"ROS 2 package '{package}' was not found") from exc
    raise ValueError('expected either a launch file path or PACKAGE FILE')


async def _run_supervisor(supervisor: Supervisor) -> int:
    loop = asyncio.get_running_loop()
    stopping = False

    def request_shutdown():
        nonlocal stopping
        if not stopping:
            stopping = True
            loop.create_task(supervisor.shutdown())

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, request_shutdown)
        except NotImplementedError:
            pass
    return await supervisor.run()


def main(argv=None) -> int:
    parser = make_parser()
    args = parser.parse_args(argv)
    if args.command != 'launch':
        parser.print_help()
        return 1
    if args.stop_timeout < 0:
        parser.error('--stop-timeout cannot be negative')
    try:
        launch_file, launch_arguments = resolve_launch_spec(args.launch_spec)
        if args.list_args:
            print_arguments_of_launch_file(launch_file_path=launch_file)
            return 0
        if args.benchmark:
            get_launch_description_from_any_launch_file(launch_file)
            return 0
    except (FileNotFoundError, ValueError, RuntimeError) as exc:
        parser.error(str(exc))

    log_file = args.log
    if log_file == 'syslog':
        parser.error('--log=syslog is not available; use a file path with ROS 2')
    if not log_file:
        stamp = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        log_file = f'/tmp/rosmon2_{stamp}.log'
        print(f'Tip: process output is also written to {log_file}')

    supervisor = Supervisor(
        launch_file,
        launch_arguments,
        ui=not args.disable_ui,
        no_start=args.no_start,
        stop_timeout=args.stop_timeout,
        log_file=log_file,
        flush_log=args.flush_log,
        flush_stdout=args.flush_stdout,
    )
    try:
        return asyncio.run(_run_supervisor(supervisor))
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    sys.exit(main())
