"""Command line entry point compatible with ``mon launch`` conventions."""

import argparse
import asyncio
import json
import os
import signal
import sys
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError
from launch.launch_description_sources import get_launch_description_from_any_launch_file
from ros2launch.api import get_share_file_path_from_package
from ros2launch.api import print_arguments_of_launch_file

from .control import ControlClient, ControlError, validate_session_name
from .supervisor import Supervisor


ROSMON_CONSOLE_OUTPUT_FORMAT = '[{severity}] [{function_name}]: {message}'


def configure_ros_console_output() -> None:
    """Use rosmon's compact function/message layout unless the user overrides it."""
    os.environ.setdefault(
        'RCUTILS_CONSOLE_OUTPUT_FORMAT',
        ROSMON_CONSOLE_OUTPUT_FORMAT,
    )


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='mon2',
        description='rosmon-style ROS 2 launch process monitor',
        usage='mon2 launch [options] PACKAGE FILE [name:=value ...]\n'
              '       mon2 launch [options] path/to/file.launch.py [name:=value ...]\n'
              '       mon2 COMMAND [options]',
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
    launch_parser.add_argument(
        '--session', default='default', metavar='NAME',
        help='control session name (default: default)')
    launch_parser.add_argument(
        '--json-events', action='store_true',
        help='emit structured JSONL events and disable the interactive UI')
    launch_parser.add_argument(
        '--no-control', action='store_true',
        help='do not create a local control socket')
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

    def add_client_options(command_parser):
        command_parser.add_argument(
            '--session', default='default', metavar='NAME',
            help='running rosmon2 session (default: default)')
        command_parser.add_argument(
            '--json', action='store_true',
            help='emit compact JSON instead of indented JSON')

    def add_target_options(command_parser, *, allow_all=False):
        targets = command_parser.add_mutually_exclusive_group(required=allow_all)
        targets.add_argument('--node', metavar='FULL_NAME')
        targets.add_argument('--namespace', metavar='NAMESPACE')
        if allow_all:
            targets.add_argument('--all', action='store_true', dest='all_nodes')

    status_parser = subparsers.add_parser(
        'status', help='show process state for a running session')
    add_client_options(status_parser)
    add_target_options(status_parser)

    logs_parser = subparsers.add_parser(
        'logs', help='query recent structured process logs')
    add_client_options(logs_parser)
    add_target_options(logs_parser)
    logs_parser.add_argument('--severity', choices=['DEBUG', 'INFO', 'WARN', 'WARNING',
                                                   'ERROR', 'FATAL'])
    logs_parser.add_argument('--since', type=float, default=0, metavar='SECONDS')
    logs_parser.add_argument('--limit', type=int, default=200)

    events_parser = subparsers.add_parser(
        'events', help='stream structured events from a running session')
    add_client_options(events_parser)

    for action in ('start', 'stop', 'restart', 'mute', 'unmute'):
        action_parser = subparsers.add_parser(
            action, help=f'{action} a node, namespace, or all processes')
        add_client_options(action_parser)
        add_target_options(action_parser, allow_all=True)

    wait_parser = subparsers.add_parser(
        'wait', help='wait until matching processes reach a state')
    add_client_options(wait_parser)
    add_target_options(wait_parser)
    wait_parser.add_argument(
        '--state', choices=[state for state in ('running', 'idle', 'crashed', 'waiting')],
        default='running')
    wait_parser.add_argument('--timeout', type=float, default=30.0)
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
    if args.command is None:
        parser.print_help()
        return 1
    if args.command != 'launch':
        return _run_client_command(parser, args)
    if args.stop_timeout < 0:
        parser.error('--stop-timeout cannot be negative')
    try:
        validate_session_name(args.session)
    except ValueError as exc:
        parser.error(str(exc))
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
        if not args.json_events:
            print(f'Tip: process output is also written to {log_file}')

    configure_ros_console_output()
    supervisor = Supervisor(
        launch_file,
        launch_arguments,
        ui=not args.disable_ui and not args.json_events,
        no_start=args.no_start,
        stop_timeout=args.stop_timeout,
        log_file=log_file,
        flush_log=args.flush_log,
        flush_stdout=args.flush_stdout,
        session=args.session,
        json_events=args.json_events,
        control=not args.no_control,
    )
    try:
        return asyncio.run(_run_supervisor(supervisor))
    except ControlError as exc:
        if args.json_events:
            print(json.dumps({'ok': False, 'error': str(exc)}, separators=(',', ':')))
        else:
            print(f'mon2: error: {exc}', file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


def _target_request(args):
    request = {}
    if getattr(args, 'node', None):
        request['node'] = args.node
    elif getattr(args, 'namespace', None):
        request['namespace'] = args.namespace
    elif getattr(args, 'all_nodes', False):
        request['all'] = True
    return request


def _print_response(response, compact: bool) -> None:
    if compact:
        print(json.dumps(response, separators=(',', ':'), sort_keys=True))
    else:
        print(json.dumps(response, indent=2, sort_keys=True))


def _run_client_command(parser, args) -> int:
    try:
        validate_session_name(args.session)
    except ValueError as exc:
        parser.error(str(exc))
    request = {'command': args.command, **_target_request(args)}
    if args.command == 'logs':
        request.update({
            'severity': args.severity,
            'since_seconds': args.since,
            'limit': args.limit,
        })
    elif args.command == 'wait':
        request.update({'state': args.state, 'timeout': args.timeout})

    timeout = args.timeout + 2.0 if args.command == 'wait' else 10.0
    client = ControlClient(args.session, timeout=timeout)
    try:
        if args.command == 'events':
            for event in client.events():
                _print_response(event, compact=True)
            return 0
        response = client.request(request)
        _print_response(response, compact=args.json)
        return 0
    except (ControlError, EOFError) as exc:
        error = {'ok': False, 'error': str(exc), 'session': args.session}
        if args.json:
            _print_response(error, compact=True)
        else:
            print(f'mon2: error: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
