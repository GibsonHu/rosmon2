"""ROS 2 launch integration and process supervision."""

import asyncio
import json
import logging
import sys
import time
from collections import deque
from datetime import datetime, timezone
from typing import Callable, Dict, Optional

import launch
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessIO, OnProcessStart
from launch.events.process import ShutdownProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch.logging
from launch_ros.actions import Node
from ros2launch.api.api import parse_launch_arguments

from .control import ControlError, ControlServer
from .model import ProcessRecord, selection_key, State
from .terminal import TerminalUI


class _UILogStream:
    """File-like adapter routing launch framework logs through TerminalUI."""

    encoding = getattr(sys.stdout, 'encoding', 'utf-8')

    def __init__(self, ui: TerminalUI):
        self._ui = ui

    def write(self, message: str) -> int:
        if message and message.strip():
            self._ui.log('launch', message)
        return len(message)

    def flush(self) -> None:
        self._ui.flush()


class Supervisor:
    """Run one launch description and expose rosmon-like process controls."""

    def __init__(self, launch_file: str, launch_arguments, *, ui: bool = True,
                 no_start: bool = False, stop_timeout: float = 5.0,
                 log_file: Optional[str] = None, flush_log: bool = False,
                 flush_stdout: bool = False, session: str = 'default',
                 json_events: bool = False, control: bool = True):
        self.launch_file = launch_file
        self.launch_arguments = list(launch_arguments)
        self.no_start = no_start
        self.stop_timeout = stop_timeout
        self.flush_stdout = flush_stdout
        self.session = session
        self.json_events = json_events
        self.records = []
        self._by_action: Dict[object, ProcessRecord] = {}
        self._next_key = 0
        self._launch_service: Optional[LaunchService] = None
        self._context = None
        self._shutting_down = False
        self._no_start_applied = set()
        self._pending_restarts = set()
        self._event_sequence = 0
        self._event_listeners = []
        self._logs = deque(maxlen=5000)
        self._control_server = ControlServer(self, session) if control else None
        self._log_handle = (
            open(log_file, 'a', buffering=1 if flush_log else -1)
            if log_file else None
        )
        self.ui = TerminalUI(ui, self.handle_key, output_enabled=not json_events)
        if json_events:
            self.add_event_listener(self._print_json_event)

    async def run(self) -> int:
        """Run until the launch service is idle or the user interrupts it."""
        handlers = [
            RegisterEventHandler(OnProcessStart(on_start=self._on_start)),
            RegisterEventHandler(OnProcessIO(
                on_stdout=lambda event: self._on_output(event, False),
                on_stderr=lambda event: self._on_output(event, True),
            )),
            RegisterEventHandler(OnProcessExit(on_exit=self._on_exit)),
        ]
        include = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(self.launch_file),
            launch_arguments=parse_launch_arguments(self.launch_arguments),
        )
        description = LaunchDescription(handlers + [include])
        self._launch_service = LaunchService(argv=self.launch_arguments, noninteractive=True)
        self._launch_service.include_launch_description(description)
        loop = asyncio.get_running_loop()
        screen_handler = None
        original_stream = None
        if self.ui.enabled or self.json_events:
            # launch writes directly to stdout by default, which can overwrite
            # our persistent status bar.  Preserve its messages but route them
            # through the same erase/log/redraw path as process output.
            screen_handler = launch.logging.launch_config.get_screen_handler()
            original_stream = screen_handler.setStream(_UILogStream(self.ui))
        control_started = False
        session_started = False
        try:
            if self._control_server is not None:
                await self._control_server.start()
                control_started = True
            self.ui.start(loop)
            self.ui.set_records(self.records)
            self._emit_event(
                'session_started',
                launch_file=self.launch_file,
                launch_arguments=self.launch_arguments,
                socket=str(self._control_server.path) if self._control_server else None,
            )
            session_started = True
            # A monitor must remain available after every process is stopped;
            # otherwise F6 / per-node start could never bring them back.
            return await self._launch_service.run_async(shutdown_when_idle=False)
        finally:
            if session_started:
                self._emit_event('session_stopping')
            if screen_handler is not None and original_stream is not None:
                screen_handler.setStream(original_stream)
            self.ui.close(loop)
            if self._control_server is not None and control_started:
                await self._control_server.close()
            if self._log_handle:
                self._log_handle.close()

    async def shutdown(self) -> None:
        if self._shutting_down:
            return
        self._shutting_down = True
        if self._context is not None:
            # LaunchService.shutdown() is thread-safe by blocking on the launch
            # loop.  We are already in that loop, so emit directly to avoid a
            # self-deadlock.
            from launch.events import Shutdown
            self._context.emit_event_sync(Shutdown(reason='rosmon2 shutdown requested'))

    def _on_start(self, event, context):
        self._context = context
        record = self._by_action.get(event.action)
        if record is None:
            record = self._record_for_new_action(event)
            self.records.append(record)
        elif record.pid is not None:
            record.restart_count += 1
        record.action = event.action
        record.cmd = list(event.cmd)
        record.cwd = event.cwd
        record.env = dict(event.env) if event.env else None
        record.pid = event.pid
        record.return_code = None
        record.state = State.RUNNING
        self._by_action[event.action] = record
        self._silence_native_process_screen_logger(event.process_name)
        self._write_log(record.display_name, f'process started with pid {event.pid}', False)
        self._emit_event('node_started', node=self._record_dict(record))
        if self.no_start and record.key not in self._no_start_applied:
            self._no_start_applied.add(record.key)
            record.manually_stopped = True
            context.asyncio_loop.call_soon(self.stop, record)
        self.ui.set_records(self.records)

    def _record_for_new_action(self, event) -> ProcessRecord:
        linked = getattr(event.action, '_rosmon2_record', None)
        if linked is not None:
            self._by_action[event.action] = linked
            return linked
        display = self._display_name(event.action, event.process_name)
        record = ProcessRecord(key=self._next_key, display_name=display)
        self._next_key += 1
        return record

    @staticmethod
    def _display_name(action, fallback: str) -> str:
        if isinstance(action, Node):
            try:
                # node_name is already the fully-qualified name after Node.execute().
                name = action.node_name
                if Node.UNSPECIFIED_NODE_NAME in name:
                    # With no explicit ``name=`` ROS uses the name chosen by
                    # the executable at runtime.  Launch cannot expose that
                    # value here, but its process label defaults to the node
                    # executable and is the best available representation.
                    name = name.replace(
                        Node.UNSPECIFIED_NODE_NAME,
                        Supervisor._process_name_without_counter(fallback),
                    )
                return Supervisor._normalize_display_name(name)
            except (RuntimeError, AttributeError):
                pass
        return Supervisor._process_name_without_counter(fallback)

    @staticmethod
    def _process_name_without_counter(name: str) -> str:
        """Remove launch's numeric ``-N`` suffix from a process label."""
        base, separator, counter = name.rpartition('-')
        return base if separator and counter.isdigit() else name

    @staticmethod
    def _normalize_display_name(name: str) -> str:
        """Format a ROS name like rosmon, without its leading root slash."""
        name = name.replace('<node_namespace_unspecified>', '')
        return name.lstrip('/')

    @staticmethod
    def _silence_native_process_screen_logger(process_name: str) -> None:
        screen_handler = launch.logging.launch_config.get_screen_handler()
        for suffix in ('-stdout', '-stderr'):
            logger = logging.getLogger(process_name + suffix)
            if screen_handler in logger.handlers:
                logger.removeHandler(screen_handler)

    def _on_output(self, event, is_stderr: bool):
        record = self._by_action.get(event.action)
        source = record.display_name if record else event.process_name
        text = event.text.decode(errors='replace')
        self._write_log(source, text.rstrip('\n'), is_stderr)
        self._record_output(source, text, is_stderr)
        if record is None or not record.muted:
            self.ui.log(source, text, is_stderr=is_stderr)
        if self.flush_stdout:
            self.ui.flush()

    def _write_log(self, source: str, text: str, is_stderr: bool) -> None:
        if not self._log_handle:
            return
        channel = 'stderr' if is_stderr else 'stdout'
        for line in text.splitlines() or ['']:
            self._log_handle.write(f'[{channel}] {source}: {line}\n')

    def _on_exit(self, event, context):
        record = self._by_action.get(event.action)
        if record is None:
            return
        record.pid = None
        record.return_code = event.returncode
        if record.manually_stopped or event.returncode == 0:
            record.state = State.IDLE
        else:
            record.state = State.CRASHED
        self._write_log(record.display_name,
                        f'process exited with code {event.returncode}', event.returncode != 0)
        self._emit_event('node_exited', node=self._record_dict(record))
        self.ui.set_records(self.records)
        if record.key in self._pending_restarts:
            self._pending_restarts.discard(record.key)
            context.asyncio_loop.call_soon(self.start, record)

    def handle_key(self, key: str) -> None:
        """Apply rosmon's two-key node action interface."""
        if self.ui.search_active:
            self._handle_search_key(key)
            return

        if key == 'F5':
            self.ui.namespace_mode = not self.ui.namespace_mode
            self.ui.namespace_inspect = None
            self.ui.selected = None
            self.ui.redraw()
            return

        if self.ui.selected is None:
            if key == '/':
                self.ui.search_active = True
                self.ui.search_query = ''
                self.ui.search_selected = 0
                self.ui.redraw()
                return
            if key == 'F6':
                for record in self.records:
                    self.start(record)
                return
            if key == 'F7':
                for record in self.records:
                    self.stop(record)
                return
            if key == 'F8':
                self.ui.warn_only = not self.ui.warn_only
                self.ui.redraw()
                return
            if key == 'F9':
                for record in self.records:
                    record.muted = True
                self.ui.redraw()
                return
            if key == 'F10':
                for record in self.records:
                    record.muted = False
                self.ui.redraw()
                return
            if (self.ui.namespace_mode and self.ui.namespace_inspect is not None
                    and key in ('\b', '\x7f')):
                self.ui.namespace_inspect = None
                self.ui.redraw()
                return
            selectable_count = (
                len(self.ui.namespaces())
                if self.ui.namespace_mode and self.ui.namespace_inspect is None
                else len(self.ui.visible_records())
            )
            for index in range(selectable_count):
                if key == selection_key(index):
                    self.ui.selected = index
                    self.ui.redraw()
                    return
            return

        index = self.ui.selected
        self.ui.selected = None
        if self.ui.namespace_mode and self.ui.namespace_inspect is None:
            namespaces = self.ui.namespaces()
            if index >= len(namespaces):
                return
            namespace = namespaces[index]
            records = self.ui.records_in_namespace(namespace)
            if key == 's':
                for record in records:
                    self.start(record)
            elif key == 'k':
                for record in records:
                    self.stop(record)
            elif key == 'i':
                self.ui.namespace_inspect = namespace
            elif key == 'm':
                for record in records:
                    record.muted = True
            elif key == 'u':
                for record in records:
                    record.muted = False
            self.ui.redraw()
            return

        records = self.ui.visible_records()
        if index >= len(records):
            return
        record = records[index]
        if key == 's':
            self.start(record)
        elif key == 'k':
            self.stop(record)
        elif key == 'm':
            record.muted = True
        elif key == 'u':
            record.muted = False
        elif key == 'd':
            self.debug(record)
        self.ui.redraw()

    def _handle_search_key(self, key: str) -> None:
        """Edit or navigate the interactive full-name node search."""
        matches = self.ui.search_matches()
        if key in ('\n', '\r'):
            selected = (
                matches[self.ui.search_selected]
                if self.ui.search_selected < len(matches) else None
            )
            self.ui.search_active = False
            self.ui.search_query = ''
            self.ui.search_selected = 0
            self.ui.selected = None
            if selected is not None:
                # Search always selects an individual node, even when it was
                # opened from the namespace overview.
                self.ui.namespace_mode = False
                self.ui.namespace_inspect = None
                self.ui.selected = self.records.index(selected)
            self.ui.redraw()
            return

        if key == 'ESC':
            self.ui.search_active = False
            self.ui.search_query = ''
            self.ui.search_selected = 0
            self.ui.selected = None
            self.ui.redraw()
            return

        if key in ('\b', '\x7f'):
            self.ui.search_query = self.ui.search_query[:-1]
            self.ui.search_selected = 0
        elif key in ('\t', 'RIGHT', 'DOWN'):
            if matches:
                self.ui.search_selected = (self.ui.search_selected + 1) % len(matches)
        elif key in ('LEFT', 'UP'):
            if matches:
                self.ui.search_selected = (self.ui.search_selected - 1) % len(matches)
        elif len(key) == 1 and key.isprintable() and not key.isspace():
            self.ui.search_query += key
            self.ui.search_selected = 0

        matches = self.ui.search_matches()
        if matches and self.ui.search_selected >= len(matches):
            self.ui.search_selected = 0
        self.ui.redraw()

    def stop(self, record: ProcessRecord) -> None:
        """Gracefully stop one running launch process."""
        record.manually_stopped = True
        if record.pid is None or self._context is None:
            record.state = State.IDLE
            self.ui.redraw()
            return
        target = record.action
        # The keyboard reader runs in the launch event loop.  Calling the
        # thread-safe LaunchService.emit_event() here would wait on this same
        # loop and deadlock it.
        self._context.emit_event_sync(
            ShutdownProcess(process_matcher=lambda action: action is target)
        )

    def start(self, record: ProcessRecord) -> None:
        """Start a stopped process again from its fully expanded command."""
        if record.pid is not None or not record.cmd or self._context is None:
            return
        record.manually_stopped = False
        record.state = State.WAITING
        record.restart_count += 1
        action = ExecuteProcess(
            cmd=record.cmd,
            cwd=record.cwd,
            env=record.env,
            name=f'rosmon2_{record.key}_{record.restart_count}',
            output='log',
            sigterm_timeout=str(self.stop_timeout),
            sigkill_timeout=str(max(1.0, self.stop_timeout)),
        )
        action._rosmon2_record = record
        self._by_action[action] = record
        action.execute(self._context)
        self.ui.redraw()

    def restart(self, record: ProcessRecord) -> None:
        """Restart a process, waiting for a running instance to exit first."""
        if record.pid is None:
            self.start(record)
            return
        self._pending_restarts.add(record.key)
        self.stop(record)

    def debug(self, record: ProcessRecord) -> None:
        """Restart a stopped process under gdb when it is installed."""
        import shutil
        if shutil.which('gdb') is None:
            self.ui.notice('gdb is not installed; cannot debug this process', error=True)
            return
        if record.pid is not None:
            self.stop(record)
            self.ui.notice("stop completed; press the node key then 'd' again to start gdb")
            return
        original = record.cmd
        record.cmd = ['gdb', '--args'] + original
        self.start(record)
        record.cmd = original

    def add_event_listener(self, listener: Callable[[Dict], None]) -> None:
        """Register a callback for structured supervisor events."""
        if listener not in self._event_listeners:
            self._event_listeners.append(listener)

    def remove_event_listener(self, listener: Callable[[Dict], None]) -> None:
        """Remove a structured event callback."""
        try:
            self._event_listeners.remove(listener)
        except ValueError:
            pass

    def _emit_event(self, event_type: str, **fields) -> Dict:
        self._event_sequence += 1
        event = {
            'event': event_type,
            'sequence': self._event_sequence,
            'session': self.session,
            'timestamp': datetime.now(timezone.utc).isoformat(),
        }
        event.update(fields)
        for listener in tuple(self._event_listeners):
            listener(event)
        return event

    @staticmethod
    def _print_json_event(event: Dict) -> None:
        print(json.dumps(event, separators=(',', ':'), sort_keys=True), flush=True)

    def _record_output(self, source: str, text: str, is_stderr: bool) -> None:
        for line in text.replace('\r\n', '\n').replace('\r', '\n').splitlines():
            severity = self.ui._severity(line, None, is_stderr)
            entry = {
                'timestamp': datetime.now(timezone.utc).isoformat(),
                'timestamp_epoch': time.time(),
                'node': source,
                'stream': 'stderr' if is_stderr else 'stdout',
                'severity': severity,
                'message': line,
            }
            self._logs.append(entry)
            public_entry = dict(entry)
            public_entry.pop('timestamp_epoch')
            self._emit_event('log', log=public_entry)

    @staticmethod
    def _namespace_for(record: ProcessRecord) -> str:
        parts = [part for part in record.display_name.strip('/').split('/') if part]
        return parts[0] if len(parts) > 1 else '/'

    def _record_dict(self, record: ProcessRecord) -> Dict:
        return {
            'key': record.key,
            'name': '/' + record.display_name.lstrip('/'),
            'namespace': self._namespace_for(record),
            'state': record.state.value,
            'pid': record.pid,
            'muted': record.muted,
            'restart_count': record.restart_count,
            'return_code': record.return_code,
            'command': list(record.cmd),
        }

    def _selected_records(self, request: Dict, *, strict: bool = True):
        node = request.get('node')
        namespace = request.get('namespace')
        all_nodes = bool(request.get('all'))
        selected = list(self.records)
        if node:
            normalized = str(node).lstrip('/')
            selected = [
                record for record in selected
                if record.display_name.lstrip('/') == normalized
            ]
        elif namespace:
            normalized = str(namespace).strip('/')
            if not normalized:
                selected = [
                    record for record in selected
                    if self._namespace_for(record) == '/'
                ]
            else:
                prefix = normalized + '/'
                selected = [
                    record for record in selected
                    if record.display_name.lstrip('/').startswith(prefix)
                ]
        elif not all_nodes:
            if strict:
                raise ControlError('specify node, namespace, or all=true')
            return selected
        if strict and not selected:
            target = node if node else namespace
            raise ControlError(f'no processes match target {target!r}')
        return selected

    def _status(self, request: Dict) -> Dict:
        if any(request.get(field) for field in ('node', 'namespace')):
            records = self._selected_records(request)
        else:
            records = list(self.records)
        states = {state.value: 0 for state in State}
        for record in records:
            states[record.state.value] += 1
        namespaces = []
        for namespace in sorted(
                {self._namespace_for(record) for record in records},
                key=lambda value: (value != '/', value)):
            members = [
                record for record in records
                if self._namespace_for(record) == namespace
            ]
            alive = sum(record.state is State.RUNNING for record in members)
            namespaces.append({
                'name': namespace,
                'alive': alive,
                'dead': len(members) - alive,
                'muted': bool(members) and all(record.muted for record in members),
            })
        return {
            'ok': True,
            'session': self.session,
            'launch_file': self.launch_file,
            'shutting_down': self._shutting_down,
            'summary': {'total': len(records), **states},
            'namespaces': namespaces,
            'nodes': [self._record_dict(record) for record in records],
        }

    def _log_response(self, request: Dict) -> Dict:
        selected_names = None
        if any(request.get(field) for field in ('node', 'namespace')):
            selected_names = {
                record.display_name.lstrip('/')
                for record in self._selected_records(request)
            }
        severity = request.get('severity')
        if severity:
            severity = str(severity).upper()
            if severity == 'WARN':
                severity = 'WARNING'
        since_seconds = float(request.get('since_seconds', 0))
        cutoff = time.time() - since_seconds if since_seconds > 0 else 0
        limit = int(request.get('limit', 200))
        if limit < 1 or limit > 5000:
            raise ControlError('log limit must be between 1 and 5000')
        matches = []
        for entry in self._logs:
            if selected_names is not None and entry['node'].lstrip('/') not in selected_names:
                continue
            if severity and entry['severity'] != severity:
                continue
            if entry['timestamp_epoch'] < cutoff:
                continue
            public_entry = dict(entry)
            public_entry.pop('timestamp_epoch')
            matches.append(public_entry)
        return {
            'ok': True,
            'session': self.session,
            'logs': matches[-limit:],
        }

    async def control_request(self, request: Dict) -> Dict:
        """Execute one machine-facing request in the launch event loop."""
        command = request.get('command')
        if command == 'status':
            return self._status(request)
        if command == 'logs':
            return self._log_response(request)
        if command == 'wait':
            return await self._wait_for_state(request)
        if command not in ('start', 'stop', 'restart', 'mute', 'unmute'):
            raise ControlError(f'unknown control command: {command!r}')

        records = self._selected_records(request)
        for record in records:
            if command == 'start':
                self.start(record)
            elif command == 'stop':
                self.stop(record)
            elif command == 'restart':
                self.restart(record)
            elif command == 'mute':
                record.muted = True
            elif command == 'unmute':
                record.muted = False
        self.ui.redraw()
        self._emit_event(
            'control_action',
            action=command,
            nodes=[self._record_dict(record) for record in records],
        )
        return {
            'ok': True,
            'session': self.session,
            'action': command,
            'matched': len(records),
            'nodes': [self._record_dict(record) for record in records],
        }

    async def _wait_for_state(self, request: Dict) -> Dict:
        desired = str(request.get('state', State.RUNNING.value)).lower()
        if desired not in {state.value for state in State}:
            raise ControlError(
                'state must be one of: ' +
                ', '.join(state.value for state in State)
            )
        timeout = float(request.get('timeout', 30.0))
        if timeout < 0:
            raise ControlError('timeout cannot be negative')
        deadline = asyncio.get_running_loop().time() + timeout
        while True:
            records = self._selected_records(request, strict=False)
            if records and all(record.state.value == desired for record in records):
                return {
                    'ok': True,
                    'session': self.session,
                    'state': desired,
                    'matched': len(records),
                    'nodes': [self._record_dict(record) for record in records],
                }
            if asyncio.get_running_loop().time() >= deadline:
                current = [self._record_dict(record) for record in records]
                raise ControlError(
                    f'timed out after {timeout:g}s waiting for state {desired}; '
                    f'current nodes: {current}'
                )
            await asyncio.sleep(0.1)
