"""ROS 2 launch integration and process supervision."""

import asyncio
import json
import logging
import os
import re
import shlex
import shutil
import sys
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
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


CODEX_EDIT_REQUEST_RE = re.compile(
    r'^\s*(?:(?:please|kindly)\s+)?'
    r'(?:(?:(?:can|could|would)\s+you|i\s+(?:want|need)\s+you\s+to|'
    r'go\s+ahead\s+and)\s+(?:please\s+)?)?'
    r'(?:apply|change|create|edit|fix|generate|implement|modify|patch|'
    r'repair|resolve|write)\b',
    re.IGNORECASE,
)
CODEX_GREETING_RE = re.compile(
    r'^\s*(?:hi|hello|hey|help|good\s+(?:morning|afternoon|evening))'
    r'(?:\s+there)?[.!?]*\s*$',
    re.IGNORECASE,
)
CODEX_NODE_KEY_RE = re.compile(
    r"(?:"
    r"\b(?:node|key|letter)\s*['\"`]?"
    r"|"
    r"\b(?:what(?:'s|s| is)?\s+wrong\s+with|diagnose|check|inspect|"
    r"start|stop|restart|mute|unmute|debug|kill|run)\s+"
    r"(?:node\s+)?['\"`]?"
    r")"
    r"(?P<key>[a-zA-Z0-9])\b['\"`]?",
    re.IGNORECASE,
)
CODEX_NODE_ACTION_REQUEST_RE = re.compile(
    r'^\s*(?:(?:please|kindly)\s+)?'
    r'(?:(?:(?:can|could|would|will)\s+you|'
    r'i\s+(?:want|need)\s+you\s+to|go\s+ahead\s+and)\s+(?:please\s+)?)?'
    r'(?:start|stop|restart|mute|unmute|debug|kill|run)\b',
    re.IGNORECASE,
)
CODEX_ROS_OPERATION_REQUEST_RE = re.compile(
    r'^\s*(?:(?:please|kindly)\s+)?'
    r'(?:(?:(?:can|could|would|will)\s+you|'
    r'i\s+(?:want|need)\s+you\s+to|go\s+ahead\s+and)\s+(?:please\s+)?)?'
    r'(?:'
    r'(?:call|invoke|trigger|run|use)\b.*\bservice\b'
    r'|(?:send|execute|run)\b.*\b(?:action|goal)\b'
    r'|(?:move|rotate|home|grip|release|drive|navigate)\b'
    r'|(?:open|close)\b.*\b(?:gripper|hand|tool)\b'
    r'|set\b.*\b(?:joint|pose|robot|arm|gripper|controller)\b'
    r')',
    re.IGNORECASE,
)
CODEX_ROS_OPERATION_FOLLOWUP_RE = re.compile(
    r'^\s*(?:(?:please|use|in|the|it(?:\'s)?|frame|relative\s+to)\s+)*'
    r'(?:'
    r'(?:base|world|tool|tcp|local|global)(?:\s+(?:frame|coordinates?))?'
    r'|(?:positive|negative|\+|-)?\s*(?:x|y|z)'
    r'|(?:up|down|left|right|forward|back(?:ward)?|clockwise|'
    r'counterclockwise)'
    r'|(?:yes|yeah|yep|confirm|confirmed|proceed|go\s+ahead)'
    r'|(?:\+|-)?\d+(?:\.\d+)?\s*'
    r'(?:mm|cm|m|deg(?:ree)?s?|rad(?:ian)?s?|'
    r'm/s|m/s\^?2|mm/s|mm/s\^?2|%)'
    r')'
    r'(?:\s+(?:please|frame|coordinates?))?\s*[.!?]?\s*$',
    re.IGNORECASE,
)
CODEX_ARM_MOTION_REQUEST_RE = re.compile(
    r'\b(?:move|rotate|position|jog)\b.*'
    r'\b(?:robot\s+arm|arm|joint|end[- ]?effector|tool|tcp|pose)\b'
    r'|\b(?:robot\s+arm|arm|joint|end[- ]?effector|tool|tcp)\b.*'
    r'\b(?:move|rotate|position|jog)\b'
    r'|\b(?:move|rotate|jog)\b.*'
    r'\b\d+(?:\.\d+)?\s*(?:mm|cm|m|deg(?:ree)?s?|rad(?:ian)?s?)\b',
    re.IGNORECASE,
)
CODEX_ARM_TARGET_RE = re.compile(
    r'(?:'
    r'\b(?:home|named\s+(?:pose|target))\b'
    r'|'
    r'(?:\b(?:x|y|z|up|down|left|right|forward|back(?:ward)?|'
    r'clockwise|counterclockwise|joint\s*\d+)\b.*'
    r'\b\d+(?:\.\d+)?\s*(?:mm|cm|m|deg(?:ree)?s?|rad(?:ian)?s?)\b)'
    r'|'
    r'(?:\b\d+(?:\.\d+)?\s*(?:mm|cm|m|deg(?:ree)?s?|rad(?:ian)?s?)\b.*'
    r'\b(?:x|y|z|up|down|left|right|forward|back(?:ward)?|'
    r'clockwise|counterclockwise|joint\s*\d+)\b)'
    r')',
    re.IGNORECASE,
)
CODEX_UNSAFE_ROS_REQUEST_RE = re.compile(
    r'\b(?:bypass|disable|ignore|override)\b.*\b(?:safety|limit|interlock)\b'
    r'|\braw\s+(?:joint\s+)?(?:effort|torque|velocity)\b',
    re.IGNORECASE,
)
CODEX_PYTHON_NODE_REQUEST_RE = re.compile(
    r'^\s*(?:(?:please|kindly)\s+)?'
    r'(?:(?:(?:can|could|would|will)\s+you|'
    r'i\s+(?:want|need)\s+you\s+to|go\s+ahead\s+and)\s+(?:please\s+)?)?'
    r'(?!.*\b(?:(?:do\s+not|don[’\']t)\s+'
    r'(?:start|launch|run(?:\s*it)?)'
    r'|without\s+(?:starting|running|launching))\b)'
    r'(?=.*(?:\bpython\b|\bscript\b|\bnode\b|\.py\b))'
    r'(?:(?:create|generate|write)\b.*'
    r'(?:\b(?:start|launch)\b|\brun(?:\s*it)?\b)'
    r'|(?:\b(?:start|launch)\b|\brun(?:\s*it)?\b)).*$',
    re.IGNORECASE,
)
CODEX_PYTHON_NODE_WRITE_REQUEST_RE = re.compile(
    r'^\s*(?:(?:please|kindly)\s+)?'
    r'(?:(?:(?:can|could|would|will)\s+you|'
    r'i\s+(?:want|need)\s+you\s+to|go\s+ahead\s+and)\s+(?:please\s+)?)?'
    r'(?=.*\b(?:create|generate|write|implement|add|make|build|scaffold)\b)'
    r'(?=.*(?:\bnodes?\b|(?:^|[^A-Za-z0-9])nodes?(?:[^A-Za-z0-9]|$)))'
    r'.*$',
    re.IGNORECASE,
)
ROS_NAME_RE = re.compile(r'^/?[A-Za-z0-9_][A-Za-z0-9_~/]*$')
ROS_MANAGED_NODE_NAME_RE = re.compile(
    r'^/?[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*$'
)
ROS_PARAMETER_RE = re.compile(
    r'^[A-Za-z_][A-Za-z0-9_]*(?:\.[A-Za-z_][A-Za-z0-9_]*)*$'
)
CODEX_APP_SERVER_STREAM_LIMIT = 16 * 1024 * 1024
ROS_INTERFACE_RE = re.compile(
    r'^[A-Za-z][A-Za-z0-9_]*/(?:msg|srv|action)/'
    r'[A-Za-z][A-Za-z0-9_]*$'
)
DIAGNOSIS_STALL_RE = re.compile(
    r'\b(?:stalled?|deadlock(?:ed)?|unresponsive|timed?\s*out|timeout|'
    r'waiting\s+for\s+(?:the\s+)?service|could\s+not\s+contact|'
    r'failed\s+to\s+acquire\s+lock|no\s+progress)\b',
    re.IGNORECASE,
)
DIAGNOSIS_WINDOW_SECONDS = 30.0
DIAGNOSIS_ERROR_THRESHOLD = 5
DIAGNOSIS_STALL_THRESHOLD = 3


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
                 json_events: bool = False, control: bool = True,
                 codex_command: str = 'codex', codex_workspace: Optional[str] = None):
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
        self.codex_command = codex_command
        self.codex_workspace = Path(codex_workspace or os.getcwd()).expanduser().resolve()
        self.agent_node_workspace = (Path.home() / 'rosmon2').resolve()
        self._codex_task: Optional[asyncio.Task] = None
        self._codex_process = None
        self._ros_tool_process = None
        self._codex_mode: Optional[str] = None
        self._codex_cancel_requested = False
        self._codex_history = deque(maxlen=6)
        self._diagnosis_chat_history = deque(maxlen=6)
        self._codex_pending_ros_operation_question: Optional[str] = None
        self._codex_yes_no_pending = False
        self._codex_yes_no_mode: Optional[str] = None
        self._codex_pending_fix_question = ''
        self._codex_usage_task: Optional[asyncio.Task] = None
        self._codex_usage_process = None
        self._codex_auth_task: Optional[asyncio.Task] = None
        self._codex_auth_process = None
        self._codex_logged_in: Optional[bool] = None
        self._diagnosis_task: Optional[asyncio.Task] = None
        self._diagnosis_process = None
        self._diagnosis_cancel_requested = False
        self._diagnosis_pending_reasons = set()
        self._diagnosis_health = {}
        self._diagnosis_error_times = {}
        self._diagnosis_stall_times = {}
        self._diagnosis_poll_timer = None
        self._log_handle = (
            open(log_file, 'a', buffering=1 if flush_log else -1)
            if log_file else None
        )
        agent_settings_path = (
            Path.home() / '.config' / 'rosmon2' / 'agent-settings.json'
            if ui else None
        )
        self.ui = TerminalUI(
            ui,
            self.handle_key,
            output_enabled=not json_events,
            agent_settings_path=agent_settings_path,
        )
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
        self._context = self._launch_service.context
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
            if self.ui.enabled:
                await self._refresh_codex_login_status()
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
            await self._stop_diagnosis()
            await self._stop_codex_auth()
            await self._stop_codex_usage()
            await self._stop_codex()
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
        self._diagnosis_error_times.pop(record.display_name, None)
        self._diagnosis_stall_times.pop(record.display_name, None)
        self._by_action[event.action] = record
        self._silence_native_process_screen_logger(event.process_name)
        self._write_log(record.display_name, f'process started with pid {event.pid}', False)
        self._emit_event('node_started', node=self._record_dict(record))
        if self.no_start and record.key not in self._no_start_applied:
            self._no_start_applied.add(record.key)
            record.manually_stopped = True
            context.asyncio_loop.call_soon(self.stop, record)
        self.ui.set_records(self.records)
        self._diagnosis_record_changed(record, 'node started')

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
        if record is not None:
            self._diagnosis_record_changed(record, 'log health changed')
        if self.flush_stdout:
            self.ui.flush()

    def _write_log(self, source: str, text: str, is_stderr: bool) -> None:
        """Append process output to the ``--log`` file, one line per line.

        Timestamped, because the file outlives the session and the in-memory
        ring does not: ``rosmon2_logs`` carries a timestamp per entry but holds
        only the last ``_logs.maxlen`` across *all* processes, so on a launch of
        any size a post-mortem has nothing but this file. Without a clock on it,
        "node A went quiet while node B was still retrying" is unanswerable —
        which is the question a multi-process log exists to answer.

        Local time to the millisecond, matching what most ROS nodes print in
        their own lines, so the two read as one stream. The timestamp is a
        prefix rather than an insertion, leaving ``[channel] source: line``
        intact as a suffix — existing greps keep working.
        """
        if not self._log_handle:
            return
        channel = 'stderr' if is_stderr else 'stdout'
        stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        for line in text.splitlines() or ['']:
            self._log_handle.write(f'[{stamp}] [{channel}] {source}: {line}\n')

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
        self._diagnosis_record_changed(
            record, f'node exited with code {event.returncode}')
        if record.key in self._pending_restarts:
            self._pending_restarts.discard(record.key)
            context.asyncio_loop.call_soon(self.start, record)

    def handle_key(self, key: str) -> None:
        """Apply rosmon's two-key node action interface."""
        if (
                (self.ui.codex_active or self.ui.diagnosis_active)
                and self._handle_global_function_key(key)):
            return

        if self.ui.diagnosis_active:
            if key == 'F4':
                if self._codex_task is not None:
                    self._cancel_codex()
                self._cancel_diagnosis()
                self._cancel_diagnosis_poll()
                self.ui.close_diagnosis()
                self._open_codex()
                return
            self._handle_diagnosis_key(key)
            return

        if self.ui.codex_active:
            if key == 'F3':
                if self._codex_task is not None:
                    self._cancel_codex()
                self.ui.close_codex()
                self._open_diagnosis()
                return
            self._handle_codex_key(key)
            return

        if key == 'F3':
            self._open_diagnosis()
            return

        if key == 'F4':
            self._open_codex()
            return

        if self.ui.search_active:
            self._handle_search_key(key)
            return

        if key == 'F5':
            self._handle_global_function_key(key)
            return

        if self.ui.selected is None:
            if key == '/':
                self.ui.search_active = True
                self.ui.search_query = ''
                self.ui.search_selected = 0
                self.ui.redraw()
                return
            if self._handle_global_function_key(key):
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

    def _handle_global_function_key(self, key: str) -> bool:
        """Run a global Rosmon key even while an embedded panel has focus."""
        if key == 'F5':
            self.ui.namespace_mode = not self.ui.namespace_mode
            self.ui.namespace_inspect = None
            self.ui.selected = None
            self.ui.redraw()
            return True
        if key == 'F6':
            for record in self.records:
                self.start(record)
            return True
        if key == 'F7':
            for record in self.records:
                self.stop(record)
            return True
        if key == 'F8':
            self.ui.warn_only = not self.ui.warn_only
            self.ui.redraw()
            return True
        if key == 'F9':
            for record in self.records:
                record.muted = True
            self.ui.redraw()
            return True
        if key == 'F10':
            for record in self.records:
                record.muted = False
            self.ui.redraw()
            return True
        return False

    def _open_diagnosis(self) -> None:
        """Open diagnosis mode and request one initial agent assessment."""
        self._diagnosis_cancel_requested = False
        rows = self._diagnosis_rows()
        self._diagnosis_health = {
            record.key: self._diagnosis_signature(record)
            for record in self.records
        }
        self.ui.set_diagnosis_rows(rows)
        self.ui.open_diagnosis()
        self._request_codex_usage()
        self._queue_diagnosis_agent('initial diagnosis check')
        self._schedule_diagnosis_poll()

    def _open_codex(self) -> None:
        """Open Agent and expose one-key browser login when signed out."""
        self.ui.open_codex()
        if self._codex_logged_in is False:
            self.ui.open_codex_login_option()
        self._request_codex_usage()

    def _handle_diagnosis_key(self, key: str) -> None:
        """Chat about diagnosis or control the selected unhealthy node."""
        if key == 'F3':
            if self._codex_task is not None:
                self._cancel_codex()
            self._cancel_diagnosis()
            self._cancel_diagnosis_poll()
            self.ui.close_diagnosis()
            return
        if self.ui.codex_model_picker_active:
            if key == 'F2':
                self.ui.close_codex_model_picker()
            elif key == 'ESC':
                if not self.ui.back_codex_model_picker():
                    self.ui.close_codex_model_picker()
            elif key == 'UP':
                self.ui.move_codex_model_selection(-1)
            elif key == 'DOWN':
                self.ui.move_codex_model_selection(1)
            elif key in ('\n', '\r'):
                action = self.ui.apply_codex_model_selection()
                if action is not None:
                    self._start_codex_auth(action, mode='diagnosis')
            return
        if key == 'ESC':
            if self._codex_task is not None:
                self._cancel_codex()
            self._cancel_diagnosis()
            self._cancel_diagnosis_poll()
            self.ui.close_diagnosis()
            return
        if key == 'F2':
            if self._codex_task is None and self._codex_auth_task is None:
                self.ui.open_codex_model_picker()
            return
        if (self._codex_yes_no_pending
                and self._codex_yes_no_mode == 'diagnosis'
                and key.lower() in ('y', 'n')):
            self._handle_codex_yes_no(key.lower(), mode='diagnosis')
            return
        if key in ('\n', '\r'):
            question = self.ui.diagnosis_prompt.strip()
            if (
                    question == '/model'
                    and self._codex_task is None
                    and self._codex_auth_task is None):
                self.ui.diagnosis_prompt = ''
                self.ui.open_codex_model_picker()
                return
            if (
                    question
                    and self._codex_task is None
                    and self._codex_auth_task is None):
                if not self._codex_login_available('diagnosis'):
                    return
                self._codex_yes_no_pending = False
                self._codex_yes_no_mode = None
                self._codex_pending_fix_question = ''
                self.ui.diagnosis_prompt = ''
                self.ui.add_diagnosis_message('You', question)
                self._codex_cancel_requested = False
                self._codex_mode = 'diagnosis'
                self._codex_task = asyncio.create_task(
                    self._run_codex(question, mode='diagnosis'))
                self.ui.set_diagnosis_chat_running(True)
            return
        if key == '\t':
            self.ui.diagnosis_chat_focused = (
                not self.ui.diagnosis_chat_focused)
            self.ui.redraw()
            return
        if key == 'PAGE_UP':
            self.ui.scroll_diagnosis_chat(
                self.ui.DIAGNOSIS_CHAT_VISIBLE_LINES)
            return
        if key == 'PAGE_DOWN':
            self.ui.scroll_diagnosis_chat(
                -self.ui.DIAGNOSIS_CHAT_VISIBLE_LINES)
            return
        if key in ('\b', '\x7f'):
            self.ui.diagnosis_prompt = self.ui.diagnosis_prompt[:-1]
            self.ui.redraw()
            return
        if key == '\x15':
            self.ui.diagnosis_prompt = ''
            self.ui.redraw()
            return
        if self.ui.diagnosis_chat_focused and key == 'UP':
            self.ui.scroll_diagnosis_chat(1)
            return
        if self.ui.diagnosis_chat_focused and key == 'DOWN':
            self.ui.scroll_diagnosis_chat(-1)
            return
        rows = self.ui.diagnosis_rows
        if rows and key == 'DOWN':
            self.ui.diagnosis_selected = (
                self.ui.diagnosis_selected + 1
            ) % len(rows)
            self.ui.redraw()
            return
        if rows and key == 'UP':
            self.ui.diagnosis_selected = (
                self.ui.diagnosis_selected - 1
            ) % len(rows)
            self.ui.redraw()
            return
        if (self._codex_task is None and len(key) == 1 and key.isprintable()
                and not (
                    not self.ui.diagnosis_prompt
                    and key in ('K', 'R', 'N', 'X')
                )):
            self.ui.diagnosis_prompt += key
            self.ui.redraw()
            return
        if not rows:
            return
        index = min(self.ui.diagnosis_selected, len(rows) - 1)
        if index < 0 or index >= len(rows):
            return
        record = next(
            (
                candidate for candidate in self.records
                if candidate.key == rows[index]['record_key']
            ),
            None,
        )
        if record is None:
            return
        if key == 'R':
            self.ui.set_diagnosis_summary(
                f'- Restart requested for /{record.display_name.lstrip("/")}.')
            self.restart(record)
        elif key == 'K':
            self._pending_restarts.discard(record.key)
            if record.pid is not None:
                self.stop(record)
                message = (
                    f'- Stop requested for '
                    f'/{record.display_name.lstrip("/")}.'
                )
            else:
                message = (
                    f'- /{record.display_name.lstrip("/")} is not running; '
                    'nothing was stopped.'
                )
            self.ui.set_diagnosis_summary(message)
        elif key == 'N':
            namespace = self._namespace_for(record)
            members = [
                candidate for candidate in self.records
                if self._namespace_for(candidate) == namespace
            ]
            self.ui.set_diagnosis_summary(
                f'- Restart requested for {len(members)} node(s) in '
                f'namespace {namespace}.')
            for member in members:
                self.restart(member)
        elif key == 'X':
            namespace = self._namespace_for(record)
            members = [
                candidate for candidate in self.records
                if self._namespace_for(candidate) == namespace
            ]
            running = [member for member in members if member.pid is not None]
            for member in members:
                self._pending_restarts.discard(member.key)
            for member in running:
                self.stop(member)
            self.ui.set_diagnosis_summary(
                f'- Stop requested for {len(running)} running node(s) in '
                f'namespace {namespace}.')

    def _diagnosis_counts(self, record: ProcessRecord):
        """Return recent error and stall-symptom counts for one process."""
        now = time.monotonic()
        cutoff = now - DIAGNOSIS_WINDOW_SECONDS

        def recent(bucket):
            values = bucket.setdefault(record.display_name, deque())
            while values and values[0] < cutoff:
                values.popleft()
            return len(values)

        return (
            recent(self._diagnosis_error_times),
            recent(self._diagnosis_stall_times),
        )

    def _diagnosis_log_hint(self, record: ProcessRecord) -> Optional[str]:
        """Return the newest useful warning/error as a likely-cause hint."""
        warning = None
        for entry in reversed(self._logs):
            if entry['node'] != record.display_name:
                continue
            if entry['severity'] not in ('WARNING', 'ERROR', 'FATAL'):
                continue
            message = self.ui._message_body(str(entry['message']))
            message = re.sub(r'\x1b\[[0-?]*[ -/]*[@-~]', '', message)
            message = ' '.join(message.split()).strip()
            if not message:
                continue
            if entry['severity'] in ('ERROR', 'FATAL'):
                return message[:180]
            if warning is None:
                warning = message[:180]
        return warning

    def _diagnosis_row(
            self, record: ProcessRecord, display_index: int = 0) -> Dict:
        errors, stall_signals = self._diagnosis_counts(record)
        hint = self._diagnosis_log_hint(record)
        if record.state is State.CRASHED:
            health = 'Down'
            detail = f'Exit {record.return_code}'
            detail += f'; {hint}' if hint else '; inspect recent error logs'
        elif record.state is State.IDLE:
            health = 'Stopped'
            detail = (
                'Stopped by user' if record.manually_stopped
                else (
                    'Exited normally' if record.return_code == 0
                    else f'Exited with code {record.return_code}'
                )
            )
        elif record.state is State.WAITING:
            health = 'Starting'
            detail = hint or 'Process start is still pending'
        elif stall_signals >= DIAGNOSIS_STALL_THRESHOLD:
            health = 'Stalled'
            detail = (
                f'{stall_signals} wait/timeout signals; '
                + (hint or 'a dependency may be unavailable')
            )
        elif errors >= DIAGNOSIS_ERROR_THRESHOLD:
            health = 'Erroring'
            detail = (
                f'{errors} errors in 30s; '
                + (hint or 'inspect recent error logs')
            )
        else:
            health = 'Healthy'
            detail = (
                f'{errors} recent error(s), below alert'
                if errors else 'Running normally'
            )
        return {
            'record_key': record.key,
            'selection_key': selection_key(display_index) or ' ',
            'name': record.display_name,
            'namespace': self._namespace_for(record),
            'state': record.state.value,
            'health': health,
            'errors': errors,
            'detail': detail,
        }

    def _diagnosis_rows(self, *, include_healthy: bool = False):
        rows = [
            self._diagnosis_row(record, index)
            for index, record in enumerate(self.records)
        ]
        if include_healthy:
            return rows
        return [row for row in rows if row['health'] != 'Healthy']

    def _diagnosis_signature(self, record: ProcessRecord):
        try:
            display_index = self.records.index(record)
        except ValueError:
            display_index = 0
        row = self._diagnosis_row(record, display_index)
        return row['state'], row['health']

    def _diagnosis_record_changed(
            self, record: ProcessRecord, reason: str) -> None:
        """Refresh the table and notify the agent only on a health transition."""
        if not self.ui.diagnosis_active:
            return
        signature = self._diagnosis_signature(record)
        previous = self._diagnosis_health.get(record.key)
        self._diagnosis_health[record.key] = signature
        self.ui.set_diagnosis_rows(self._diagnosis_rows())
        if previous != signature:
            before = 'new' if previous is None else '/'.join(previous)
            after = '/'.join(signature)
            self._queue_diagnosis_agent(
                f'{reason}: /{record.display_name.lstrip("/")} '
                f'changed from {before} to {after}'
            )

    def _schedule_diagnosis_poll(self) -> None:
        """Refresh time-window health without invoking the agent per log line."""
        if not self.ui.diagnosis_active or self._diagnosis_poll_timer is not None:
            return
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            return
        self._diagnosis_poll_timer = loop.call_later(
            1.0, self._poll_diagnosis)

    def _poll_diagnosis(self) -> None:
        self._diagnosis_poll_timer = None
        if not self.ui.diagnosis_active:
            return
        transitions = []
        for record in self.records:
            signature = self._diagnosis_signature(record)
            previous = self._diagnosis_health.get(record.key)
            self._diagnosis_health[record.key] = signature
            if previous is not None and previous != signature:
                transitions.append(
                    f'/{record.display_name.lstrip("/")} changed from '
                    f'{"/".join(previous)} to {"/".join(signature)}'
                )
        self.ui.set_diagnosis_rows(self._diagnosis_rows())
        if transitions:
            self._queue_diagnosis_agent('; '.join(transitions))
        self._schedule_diagnosis_poll()

    def _cancel_diagnosis_poll(self) -> None:
        if self._diagnosis_poll_timer is not None:
            self._diagnosis_poll_timer.cancel()
            self._diagnosis_poll_timer = None

    def _queue_diagnosis_agent(self, reason: str) -> None:
        """Coalesce lifecycle changes while one diagnosis turn is running."""
        if not self.ui.diagnosis_active:
            return
        if not self._codex_login_available('diagnosis'):
            return
        if self._diagnosis_task is not None:
            self._diagnosis_pending_reasons.add(reason)
            return
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            self.ui.set_diagnosis_summary(
                '- Agent check is waiting for the launch event loop.')
            return
        self._diagnosis_cancel_requested = False
        self._diagnosis_task = loop.create_task(
            self._run_diagnosis_agent(reason))

    def _diagnosis_context(self, reason: str) -> str:
        rows = self._diagnosis_rows(include_healthy=True)
        snapshot = [
            f"- /{row['name'].lstrip('/')}: state={row['state']}, "
            f"health={row['health']}, errors_30s={row['errors']}, "
            f"detail={row['detail']}"
            for row in rows
        ] or ['- No processes have been discovered yet.']
        recent_logs = []
        for entry in reversed(self._logs):
            if entry['severity'] not in ('WARNING', 'ERROR', 'FATAL'):
                continue
            recent_logs.append(
                f"- [{entry['severity']}] /{entry['node'].lstrip('/')}: "
                f"{str(entry['message'])[:400]}"
            )
            if len(recent_logs) == 20:
                break
        recent_logs.reverse()
        if not recent_logs:
            recent_logs.append('- No recent warning or error logs.')
        return '\n'.join([
            'You are the read-only Rosmon diagnosis agent for a live ROS 2 launch.',
            f'Workspace: {self.codex_workspace}',
            f'Lifecycle event: {reason}',
            'Current node health table:',
            *snapshot,
            'Recent warning and error evidence:',
            *recent_logs,
            '',
            'Return at most four concise Markdown dot points, each beginning with "- ".',
            'Identify which nodes need attention, whether the evidence suggests software, '
            'hardware, network/configuration, or uncertainty, and the safest next check.',
            'Do not edit files, restart processes, command hardware, or claim that log '
            'evidence alone proves a physical hardware fault.',
        ])

    async def _run_diagnosis_agent(self, reason: str) -> None:
        """Run one read-only agent turn for a diagnosis lifecycle change."""
        self.ui.set_diagnosis_running(True)
        command = [
            self.codex_command,
            '--ask-for-approval', 'never',
            'exec',
            '--config',
            f'model_reasoning_effort="{self.ui.codex_reasoning_effort()}"',
            *(
                ['--model', self.ui.codex_selected_model]
                if self.ui.codex_selected_model is not None else
                []
            ),
            '--ephemeral',
            '--sandbox', 'read-only',
            '--color', 'never',
            self._diagnosis_context(reason),
        ]
        try:
            if self._diagnosis_cancel_requested:
                return
            if not self.codex_workspace.is_dir():
                raise FileNotFoundError(
                    f"Rosmon workspace '{self.codex_workspace}' is not a directory")
            if shutil.which(self.codex_command) is None:
                raise FileNotFoundError(
                    f"Codex CLI command '{self.codex_command}' was not found on PATH")
            self._diagnosis_process = await asyncio.create_subprocess_exec(
                *command,
                cwd=str(self.codex_workspace),
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
            if self._diagnosis_cancel_requested:
                try:
                    self._diagnosis_process.terminate()
                except ProcessLookupError:
                    pass
            stdout, stderr = await self._diagnosis_process.communicate()
            if self._diagnosis_cancel_requested:
                return
            if self._diagnosis_process.returncode == 0:
                self.ui.set_diagnosis_summary(self._codex_output(stdout))
            else:
                detail = self._codex_output(stderr)
                self.ui.set_diagnosis_summary(
                    f'- Diagnosis agent exited with status '
                    f'{self._diagnosis_process.returncode}: {detail}')
        except (FileNotFoundError, OSError) as exc:
            if not self._diagnosis_cancel_requested:
                self.ui.set_diagnosis_summary(f'- {exc}')
        finally:
            self._diagnosis_process = None
            self._diagnosis_task = None
            self.ui.set_diagnosis_running(False)
            if self.ui.diagnosis_active and self._diagnosis_pending_reasons:
                pending = '; '.join(sorted(self._diagnosis_pending_reasons))
                self._diagnosis_pending_reasons.clear()
                self._queue_diagnosis_agent(pending)

    def _cancel_diagnosis(self) -> None:
        """Cancel a diagnosis agent turn without touching the ROS launch."""
        self._diagnosis_cancel_requested = True
        self._diagnosis_pending_reasons.clear()
        process = self._diagnosis_process
        if process is not None and process.returncode is None:
            try:
                process.terminate()
            except ProcessLookupError:
                pass

    async def _stop_diagnosis(self) -> None:
        """Wait briefly for the diagnosis child process during shutdown."""
        self._cancel_diagnosis_poll()
        task = self._diagnosis_task
        if task is None:
            return
        self._cancel_diagnosis()
        try:
            await asyncio.wait_for(asyncio.shield(task), timeout=3.0)
        except asyncio.TimeoutError:
            process = self._diagnosis_process
            if process is not None and process.returncode is None:
                try:
                    process.kill()
                except ProcessLookupError:
                    pass
            await task
        except asyncio.CancelledError:
            pass

    def _start_codex_auth(self, action: str, *, mode: str) -> None:
        """Start one explicit F2 Codex login or logout operation."""
        if action not in ('login', 'logout') or self._codex_auth_task is not None:
            return
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            self._chat_add_message(
                mode,
                'Codex',
                '- Codex account changes require the running terminal loop.',
            )
            return
        self._codex_auth_task = loop.create_task(
            self._run_codex_auth(action, mode=mode))

    def _codex_login_available(self, mode: str) -> bool:
        """Explain the login requirement before starting an Agent request."""
        if self._codex_logged_in is not False:
            return True
        message = (
            '- Please log in before talking to the Agent. '
            'Press F2 and select Log in.'
        )
        messages = (
            self.ui.diagnosis_messages
            if mode == 'diagnosis' else
            self.ui.codex_messages
        )
        if not messages or messages[-1] != ('Codex', message):
            self._chat_add_message(mode, 'Codex', message)
        self._chat_set_running(mode, False)
        return False

    async def _refresh_codex_login_status(self) -> Optional[bool]:
        """Refresh cached authentication state using the Codex CLI."""
        process = None
        try:
            if shutil.which(self.codex_command) is None:
                self._codex_logged_in = None
                self.ui.set_codex_login_state(None)
                return None
            process = await asyncio.create_subprocess_exec(
                self.codex_command,
                'login',
                'status',
                cwd=str(self.codex_workspace),
                stdin=asyncio.subprocess.DEVNULL,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
            await asyncio.wait_for(process.communicate(), timeout=5.0)
            self._codex_logged_in = process.returncode == 0
            self.ui.set_codex_login_state(self._codex_logged_in)
        except (FileNotFoundError, OSError, asyncio.TimeoutError):
            self._codex_logged_in = None
            self.ui.set_codex_login_state(None)
            if process is not None and process.returncode is None:
                try:
                    process.terminate()
                except ProcessLookupError:
                    pass
                await process.wait()
        return self._codex_logged_in

    async def _run_codex_auth(self, action: str, *, mode: str) -> None:
        """Run browser login or logout and show its output in the panel."""
        process = None
        label = (
            'Logging in to Codex'
            if action == 'login' else
            'Logging out of Codex'
        )
        command = [self.codex_command, action]
        try:
            if shutil.which(self.codex_command) is None:
                raise FileNotFoundError(
                    f"Codex CLI command '{self.codex_command}' was not found on PATH")
            self._chat_add_message(
                mode,
                'Codex',
                (
                    '- Opening the browser for Codex login. '
                    'Complete sign-in in the browser.'
                    if action == 'login' else
                    '- Removing the stored Codex login.'
                ),
            )
            self._chat_set_running(mode, True)
            self._chat_set_execution(mode, label)
            process = await asyncio.create_subprocess_exec(
                *command,
                cwd=str(self.codex_workspace),
                stdin=asyncio.subprocess.DEVNULL,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
            )
            self._codex_auth_process = process
            while True:
                line = await process.stdout.readline()
                if not line:
                    break
                text = self._codex_output(line)
                if text:
                    self._chat_add_message(mode, 'Codex', f'- {text}')
            return_code = await process.wait()
            if return_code != 0:
                self._chat_add_message(
                    mode,
                    'Codex',
                    f'- Codex {action} failed with exit code {return_code}.',
                )
            elif action == 'login':
                self._codex_logged_in = True
                self.ui.set_codex_login_state(True)
                self._chat_add_message(
                    mode, 'Codex', '- Codex login completed.')
            else:
                self._codex_logged_in = False
                self.ui.set_codex_login_state(False)
                self.ui.set_codex_usage(None)
                self._chat_add_message(
                    mode, 'Codex', '- Codex logout completed.')
        except (FileNotFoundError, OSError) as exc:
            self._chat_add_message(
                mode, 'Codex', f'- Could not {action} Codex: {exc}')
        finally:
            self._codex_auth_process = None
            self._codex_auth_task = None
            self._chat_set_execution(mode, None)
            self._chat_set_running(mode, False)
            self._request_codex_usage()

    async def _stop_codex_auth(self) -> None:
        """Stop an in-flight Codex account operation during shutdown."""
        task = self._codex_auth_task
        if task is None:
            return
        process = self._codex_auth_process
        if process is not None and process.returncode is None:
            try:
                process.terminate()
            except ProcessLookupError:
                pass
        try:
            await asyncio.wait_for(asyncio.shield(task), timeout=2.0)
        except asyncio.TimeoutError:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        except asyncio.CancelledError:
            pass

    def _request_codex_usage(self) -> None:
        """Refresh authenticated usage and the installed CLI model catalogue."""
        if self._codex_usage_task is not None:
            return
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            self.ui.set_codex_usage(None)
            self.ui.set_codex_models_loading(False)
            return
        self.ui.set_codex_usage(
            self.ui.codex_usage_remaining, loading=True)
        self.ui.set_codex_models_loading(True)
        self._codex_usage_task = loop.create_task(
            self._fetch_codex_usage())

    @staticmethod
    async def _read_app_server_response(stream, request_id: int) -> Dict:
        """Read JSONL notifications until the requested response arrives."""
        while True:
            line = await stream.readline()
            if not line:
                raise OSError('Codex app server closed before returning a response')
            try:
                message = json.loads(line)
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue
            if message.get('id') != request_id:
                continue
            if 'error' in message:
                raise OSError(
                    f"Codex app server request failed: {message['error']}")
            result = message.get('result')
            if not isinstance(result, dict):
                raise OSError('Codex app server response did not contain a result')
            return result

    @staticmethod
    def _codex_remaining_percent(result: Dict) -> Optional[int]:
        """Return the most constrained remaining percentage in the snapshot."""
        snapshots = result.get('rateLimitsByLimitId')
        snapshot = (
            snapshots.get('codex')
            if isinstance(snapshots, dict) else None
        )
        if not isinstance(snapshot, dict):
            snapshot = result.get('rateLimits')
        if not isinstance(snapshot, dict):
            return None

        remaining = []
        for name in ('primary', 'secondary'):
            window = snapshot.get(name)
            if not isinstance(window, dict):
                continue
            used = window.get('usedPercent')
            if isinstance(used, (int, float)):
                remaining.append(max(0, min(100, 100 - int(used))))
        individual = snapshot.get('individualLimit')
        if isinstance(individual, dict):
            value = individual.get('remainingPercent')
            if isinstance(value, (int, float)):
                remaining.append(max(0, min(100, int(value))))
        return min(remaining) if remaining else None

    async def _fetch_codex_usage(self) -> None:
        """Query Codex metadata without consuming an Agent turn."""
        process = None
        try:
            if shutil.which(self.codex_command) is None:
                raise FileNotFoundError(
                    f"Codex CLI command '{self.codex_command}' was not found on PATH")
            process = await asyncio.create_subprocess_exec(
                self.codex_command,
                'app-server',
                '--stdio',
                cwd=str(self.codex_workspace),
                stdin=asyncio.subprocess.PIPE,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
            self._codex_usage_process = process
            initialize = {
                'id': 1,
                'method': 'initialize',
                'params': {
                    'clientInfo': {
                        'name': 'rosmon2',
                        'version': '0.1.0',
                    },
                },
            }
            process.stdin.write(
                (json.dumps(initialize) + '\n').encode())
            await process.stdin.drain()
            await asyncio.wait_for(
                self._read_app_server_response(process.stdout, 1),
                timeout=5.0,
            )
            requests = [
                {'method': 'initialized'},
                {
                    'id': 2,
                    'method': 'account/rateLimits/read',
                    'params': None,
                },
            ]
            process.stdin.write(
                ''.join(json.dumps(item) + '\n' for item in requests).encode())
            await process.stdin.drain()
            try:
                result = await asyncio.wait_for(
                    self._read_app_server_response(process.stdout, 2),
                    timeout=5.0,
                )
                self.ui.set_codex_usage(
                    self._codex_remaining_percent(result))
            except (OSError, asyncio.TimeoutError):
                # Model selection remains useful on CLI builds or accounts
                # that do not expose rate-limit metadata.
                self.ui.set_codex_usage(None)
            process.stdin.write((json.dumps({
                'id': 3,
                'method': 'model/list',
                'params': {
                    'includeHidden': False,
                    'limit': 100,
                },
            }) + '\n').encode())
            await process.stdin.drain()
            try:
                result = await asyncio.wait_for(
                    self._read_app_server_response(process.stdout, 3),
                    timeout=5.0,
                )
                models = []
                for item in result.get('data', []):
                    if not isinstance(item, dict) or item.get('hidden'):
                        continue
                    models.append({
                        'model': item.get('model') or item.get('id'),
                        'display_name': (
                            item.get('displayName')
                            or item.get('model')
                            or item.get('id')
                        ),
                        'is_default': bool(item.get('isDefault')),
                        'supported_reasoning_efforts': [
                            (
                                effort.get('reasoningEffort')
                                if isinstance(effort, dict) else
                                effort
                            )
                            for effort in item.get(
                                'supportedReasoningEfforts', [])
                        ],
                        'default_reasoning_effort': item.get(
                            'defaultReasoningEffort'),
                    })
                self.ui.set_codex_models(models)
            except (OSError, asyncio.TimeoutError):
                self.ui.set_codex_models_loading(False)
        except (FileNotFoundError, OSError, asyncio.TimeoutError):
            self.ui.set_codex_usage(None)
            self.ui.set_codex_models_loading(False)
        finally:
            self._codex_usage_process = None
            self._codex_usage_task = None
            if process is not None:
                if process.stdin is not None:
                    process.stdin.close()
                try:
                    await asyncio.wait_for(process.wait(), timeout=1.0)
                except asyncio.TimeoutError:
                    if process.returncode is None:
                        process.terminate()
                    await process.wait()

    async def _stop_codex_usage(self) -> None:
        """Stop an in-flight usage query during supervisor shutdown."""
        task = self._codex_usage_task
        if task is None:
            return
        process = self._codex_usage_process
        if process is not None and process.returncode is None:
            try:
                process.terminate()
            except ProcessLookupError:
                pass
        try:
            await asyncio.wait_for(asyncio.shield(task), timeout=2.0)
        except asyncio.TimeoutError:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

    def _handle_codex_yes_no(self, answer: str, *, mode: str) -> None:
        """Apply the software-repair choice in its originating chat mode."""
        self._codex_yes_no_pending = False
        self._codex_yes_no_mode = None
        self._chat_add_message(mode, 'You', answer)
        if answer == 'n':
            self._codex_pending_fix_question = ''
            self._chat_add_message(
                mode, 'Rosmon', 'Okay. I will not make any changes.')
            return
        original_question = self._codex_pending_fix_question
        self._codex_pending_fix_question = ''
        fix_question = (
            'Fix the software issue you just diagnosed. Inspect the workspace, '
            'make the smallest safe change, run the relevant tests, and report '
            f'exactly what changed. Original question: {original_question}'
        )
        self._codex_cancel_requested = False
        self._codex_mode = mode
        self._codex_task = asyncio.create_task(
            self._run_codex(fix_question, mode=mode))
        self._chat_set_running(mode, True)

    def _handle_codex_key(self, key: str) -> None:
        """Edit and submit one Codex request without blocking the launch loop."""
        if key == 'F4':
            if self._codex_task is not None:
                self._cancel_codex()
            self.ui.close_codex()
            return
        if self.ui.codex_model_picker_active:
            if key == 'F2':
                self.ui.close_codex_model_picker()
            elif key == 'ESC':
                if not self.ui.back_codex_model_picker():
                    self.ui.close_codex_model_picker()
            elif key == 'UP':
                self.ui.move_codex_model_selection(-1)
            elif key == 'DOWN':
                self.ui.move_codex_model_selection(1)
            elif key in ('\n', '\r'):
                action = self.ui.apply_codex_model_selection()
                if action is not None:
                    self._start_codex_auth(action, mode='agent')
            return
        if key == 'ESC':
            if self._codex_task is not None:
                self._cancel_codex()
            self.ui.close_codex()
            return
        if key == 'F2':
            if self._codex_task is None and self._codex_auth_task is None:
                self.ui.open_codex_model_picker()
            return
        if (self._codex_yes_no_pending
                and self._codex_yes_no_mode == 'agent'
                and key.lower() in ('y', 'n')):
            self._handle_codex_yes_no(key.lower(), mode='agent')
            return
        if key in ('\n', '\r'):
            question = self.ui.codex_prompt.strip()
            if (
                    question == '/model'
                    and self._codex_task is None
                    and self._codex_auth_task is None):
                self.ui.codex_prompt = ''
                self.ui.open_codex_model_picker()
                return
            if (
                    question
                    and self._codex_task is None
                    and self._codex_auth_task is None):
                if not self._codex_login_available('agent'):
                    return
                self._codex_yes_no_pending = False
                self._codex_yes_no_mode = None
                self._codex_pending_fix_question = ''
                self.ui.codex_prompt = ''
                self.ui.add_codex_message('You', question)
                if CODEX_GREETING_RE.fullmatch(question):
                    self.ui.add_codex_message(
                        'Codex',
                        'Hi!\n'
                        'What would you like me to help with?',
                    )
                    self.ui.set_codex_running(
                        False, 'Ready — ask about a node')
                    return
                self._codex_cancel_requested = False
                self._codex_mode = 'agent'
                self._codex_task = asyncio.create_task(
                    self._run_codex(question, mode='agent'))
                self.ui.set_codex_running(True, 'Starting Codex…')
            return
        if key == 'UP':
            self.ui.scroll_codex(1)
            return
        if key == 'DOWN':
            self.ui.scroll_codex(-1)
            return
        if key == 'PAGE_UP':
            self.ui.scroll_codex(self.ui.CODEX_VISIBLE_LINES)
            return
        if key == 'PAGE_DOWN':
            self.ui.scroll_codex(-self.ui.CODEX_VISIBLE_LINES)
            return
        if key in ('\b', '\x7f'):
            self.ui.codex_prompt = self.ui.codex_prompt[:-1]
        elif key == '\x15':  # Ctrl-U, conventional terminal line erase.
            self.ui.codex_prompt = ''
        elif (self._codex_task is None and len(key) == 1 and key.isprintable()):
            self.ui.codex_prompt += key
        self.ui.redraw()

    def _codex_referenced_record(
            self, question: str) -> Optional[ProcessRecord]:
        """Resolve a displayed selection key mentioned in a user question."""
        stripped = question.strip().strip("'\"`")
        referenced_key = stripped if len(stripped) == 1 else None
        if referenced_key is None:
            match = CODEX_NODE_KEY_RE.search(question)
            referenced_key = match.group('key') if match else None
        if referenced_key is None:
            return None
        # In namespace overview mode the displayed letters represent namespace
        # groups, not individual nodes, so do not silently choose one member.
        if self.ui.namespace_mode and self.ui.namespace_inspect is None:
            return None
        for index, record in enumerate(self.ui.visible_records()):
            if selection_key(index) == referenced_key:
                return record
        return None

    def _codex_focus_record(
            self, question: str = '') -> Optional[ProcessRecord]:
        """Use a referenced key, highlighted node, or sole crash as focus."""
        referenced = self._codex_referenced_record(question)
        if referenced is not None:
            return referenced
        if self.ui.diagnosis_active and self.ui.diagnosis_rows:
            index = min(
                self.ui.diagnosis_selected,
                len(self.ui.diagnosis_rows) - 1,
            )
            record_key = self.ui.diagnosis_rows[index]['record_key']
            diagnosed = next(
                (
                    record for record in self.records
                    if record.key == record_key
                ),
                None,
            )
            if diagnosed is not None:
                return diagnosed
        showing_namespace_list = (
            self.ui.namespace_mode and self.ui.namespace_inspect is None
        )
        if self.ui.selected is not None and not showing_namespace_list:
            records = self.ui.visible_records()
            if self.ui.selected < len(records):
                return records[self.ui.selected]
        crashed = [record for record in self.records if record.state is State.CRASHED]
        return crashed[0] if len(crashed) == 1 else None

    def _codex_context(
            self, question: str, *, mode: str = 'agent',
            ros_operation_allowed: Optional[bool] = None) -> str:
        """Construct bounded context for a general or diagnostic Agent turn."""
        focus = self._codex_focus_record(question)
        nodes = []
        for record in self.records:
            return_code = '-' if record.return_code is None else str(record.return_code)
            pid = '-' if record.pid is None else str(record.pid)
            origin = ', origin=agent-created' if record.agent_created else ''
            nodes.append(
                f"- /{record.display_name.lstrip('/')}: state={record.state.value}, "
                f"pid={pid}, exit_code={return_code}, restarts={record.restart_count}"
                f"{origin}"
            )
        if not nodes:
            nodes.append('- no launch processes have been discovered yet')

        relevant_logs = []
        focus_name = focus.display_name if focus is not None else None
        # Newest first in memory, chronological in the prompt. Keep enough
        # evidence for a crash without flooding the model's context window.
        log_limit = 40 if focus_name else 20
        for entry in reversed(self._logs):
            if focus_name and entry['node'] != focus_name:
                continue
            relevant_logs.append(entry)
            if len(relevant_logs) == log_limit:
                break
        relevant_logs.reverse()
        log_lines = []
        for entry in relevant_logs:
            message = str(entry['message']).replace('\x1b', '').replace('\n', ' ')
            log_lines.append(
                f"[{entry['severity']}] /{entry['node'].lstrip('/')}: {message[:600]}"
            )
        if not log_lines:
            log_lines.append('- no captured process output is available yet')

        focus_text = (
            f"/{focus.display_name.lstrip('/')} "
            f"(state={focus.state.value}, exit_code={focus.return_code})"
            if focus is not None else
            'none'
        )
        changes_allowed = bool(CODEX_EDIT_REQUEST_RE.search(question))
        control_allowed = bool(CODEX_NODE_ACTION_REQUEST_RE.search(question))
        if ros_operation_allowed is None:
            ros_operation_allowed = (
                mode == 'agent'
                and bool(CODEX_ROS_OPERATION_REQUEST_RE.search(question))
            )
        python_node_allowed = (
            mode == 'agent'
            and bool(CODEX_PYTHON_NODE_REQUEST_RE.search(question))
        )
        python_node_write_allowed = (
            mode == 'agent'
            and bool(CODEX_PYTHON_NODE_WRITE_REQUEST_RE.search(question))
        )
        if changes_allowed:
            permissions = (
                'The user explicitly requested a fix/change. You may edit only files '
                'inside the workspace and run relevant tests.'
            )
        elif control_allowed or ros_operation_allowed:
            permissions = (
                'The Human requested an operational action, either directly in this '
                'message or in the immediately pending request that this message '
                'clarifies. Do not edit files. '
                'Use rosmon_control for supervised-process actions and ros2_interface '
                'for live ROS service/action operations. Execute only the requested '
                'action and report the actual tool result.'
            )
        else:
            permissions = (
                'The Human did not directly request a mutation. Do not edit files or '
                'change ROS or process state. You may inspect the ROS graph and explain '
                'actionable options without changing state.'
            )
        if python_node_write_allowed:
            permissions += (
                ' The Human directly requested writing an Agent-created ROS node. If no '
                'language was specified, implement it as a Python script. Your active '
                f'working directory for this request is {self.agent_node_workspace}. '
                'Write every file for this node inside that directory; do not write any '
                f'part of it under the launch workspace {self.codex_workspace}, another '
                'ROS package, or another workspace.'
            )
        if python_node_allowed:
            permissions += (
                ' The Human directly requested starting an Agent-created ROS node. '
                'After the script exists and has been checked, call rosmon_python_node '
                'to register and start it as a supervised process so it appears in the '
                'node GUI. Do not try rosmon_control before registration, and do not '
                'start it with a shell command.'
            )
        history = (
            self._diagnosis_chat_history
            if mode == 'diagnosis' else
            self._codex_history
        )
        conversation = [
            f"{speaker}: {text[:2000]}" for speaker, text in history
        ]
        if not conversation:
            conversation.append('- no earlier conversation in this mode')

        shared_instructions = [
            'When, and only when, the Human directly asks you to start, stop, restart, '
            'mute, unmute, or debug supervised nodes, use the rosmon_control tool. Use '
            'the exact node or namespace shown in the live snapshot and report the tool '
            'result honestly. Never use shell commands to control supervised nodes.',
            'For greetings, thanks, and casual conversation, use natural sentences '
            'without bullet markers. For informational, technical, or actionable '
            'responses, use Markdown dot points beginning with “- ”. Preserve fenced '
            'code blocks when showing commands or code.',
            permissions,
            'If you make a change, state exactly which files changed and which tests or '
            'commands you ran, including failures.',
        ]
        if mode == 'diagnosis':
            role = (
                'You are the interactive Diagnosis assistant for a running ROS 2 launch.'
            )
            mode_instructions = [
                'Focus on explaining unhealthy, stalled, noisy, waiting, or crashed nodes '
                'using the live state and logs. Use simple language.',
                'Diagnosis is read-only for ROS hardware: never call ROS services, send '
                'action goals, command actuators, or connect directly to hardware.',
                'For a fault response, use the exact Markdown heading '
                '“## What might be wrong”, followed by the exact subheadings '
                '“### Hardware” and “### Software”. Never output a '
                '“What to try next” heading. Under Hardware, list the plausible physical '
                'device, power, cable, port, or connectivity causes. Under Software, list '
                'the plausible code, driver, launch, dependency, or configuration causes. '
                'Every reason must begin with “- ”. If the evidence does not support one '
                'category, include one dot point saying that no cause in that category is '
                'currently indicated.',
                'Explain naturally which category is better supported, whether both remain '
                'possible, or whether the evidence is uncertain. Never label fields Scope, '
                'Classification, Confidence, or Evidence. Logs alone cannot prove a '
                'physical hardware fault, so include the measurement, connection check, '
                'or device diagnostic needed to confirm a hardware reason inside the '
                'Hardware section.',
                'If a non-running node has a repairable software or workspace-configuration '
                'issue, end with exactly “- Would you like me to try to fix this software '
                'issue? [y/n]”. Do not offer this for hardware-only, mixed, or uncertain '
                'problems.',
            ]
        else:
            role = (
                'You are the general-purpose Rosmon Agent for a live ROS 2 workspace.'
            )
            mode_instructions = [
                'Handle open-ended ROS, software, workspace, launch, code, testing, and '
                'operational questions. Give practical answers and concrete next actions.',
                'Do not force responses into fault-diagnosis headings and do not assume '
                'something is broken merely because node and log context is available. '
                'Follow the Human’s actual intent.',
                'You may inspect the workspace and run non-disruptive checks. Edit files '
                'and run relevant tests only when the Human directly requests a change. '
                'Keep the monitored launch running while doing independent workspace work.',
                'Shell commands are available in Agent mode, including network and ROS '
                'loopback access. You may write temporary intermediate files under /tmp. '
                'Keep durable source files inside the workspace, and do not edit workspace '
                'files unless the Human directly requested a change.',
                'Use ros2_interface to inspect the live ROS graph, discover exact service '
                'and action names/types, and inspect their interfaces. Only call a service '
                'or send an action goal when the Human requested that live ROS operation '
                'in the current message or the current message is an immediately pending '
                'clarification such as a coordinate frame. Never substitute a shell '
                'command for ros2_interface.',
                'For any direct operational command, use the selected interface’s '
                'declared or configured defaults for omitted optional, non-safety '
                'parameters. Explicit Human values override those defaults. Never '
                'invent values for required targets or safety-critical parameters.',
                'For physical robot motion, first inspect the relevant action/service '
                'interface and current graph. When a compatible live action or service '
                'exists, use ros2_interface directly. Do not create a Python motion '
                'script or try to start one with rosmon_control or an external rosmon2 '
                'tool as an alternate transport. Only write and register a Python node '
                'when the Human explicitly asks for a node or script. Never guess a '
                'missing target, joint, '
                'direction, distance, angle, coordinate frame, or safety-critical limit; '
                'ask the Human for it. Speed and acceleration are optional tuning '
                'parameters: when omitted, do not ask for them. Use the selected '
                'controller’s existing configured defaults, discovered through live ROS '
                'parameters, interface documentation, or workspace configuration. Omit '
                'optional request fields when that is how the interface selects its '
                'defaults; include the discovered configured values when the interface '
                'requires them. Explicit Human speed or acceleration values override '
                'those defaults but must remain within controller safety limits. Never '
                'invent a default. Never bypass safety limits or interlocks, and never '
                'publish raw effort, torque, or velocity commands.',
                'When the Human directly asks you to write and start a Python ROS node, '
                'write the script inside the workspace, check it, then call '
                'rosmon_python_node with its exact GUI node name. This is the only '
                'allowed way to start an Agent-created script because Rosmon must capture '
                'its lifecycle and output. Never use it for a hypothetical request.',
            ]
        return '\n'.join([
            role,
            f'Workspace: {self.codex_workspace}',
            f'Agent-created node workspace: {self.agent_node_workspace}',
            'Active working directory: '
            + str(
                self.agent_node_workspace
                if python_node_write_allowed else self.codex_workspace
            ),
            f'Launch file: {self.launch_file or "unknown"}',
            f'Selected node: {focus_text}',
            'Live node snapshot:',
            *nodes,
            'Recent relevant launch output:',
            *log_lines,
            'Recent embedded conversation:',
            *conversation,
            '',
            f'User request: {question}',
            '',
            *mode_instructions,
            *shared_instructions,
        ])

    @staticmethod
    def _clean_codex_text(text: str) -> str:
        """Turn Codex text into a terminal-safe message."""
        value = text.replace('\x1b', '')
        value = re.sub(r'[\x00-\x08\x0b-\x1f\x7f]', '', value).strip()
        return value or 'Rosmon completed without a final message.'

    @classmethod
    def _codex_output(cls, text: bytes) -> str:
        """Turn CLI byte output into a terminal-safe message."""
        return cls._clean_codex_text(text.decode(errors='replace'))

    def _chat_add_message(self, mode: str, speaker: str, text: str) -> None:
        if mode == 'diagnosis':
            self.ui.add_diagnosis_message(speaker, text)
        else:
            self.ui.add_codex_message(speaker, text)

    def _chat_set_running(self, mode: str, running: bool) -> None:
        if mode == 'diagnosis':
            self.ui.set_diagnosis_chat_running(running)
        else:
            status = (
                'Inspecting live context…'
                if running else 'Ready — ask a follow-up'
            )
            self.ui.set_codex_running(running, status)

    def _chat_begin_stream(self, mode: str) -> None:
        if mode == 'diagnosis':
            self.ui.begin_diagnosis_stream()
        else:
            self.ui.begin_codex_stream()

    def _chat_append_stream(self, mode: str, text: str) -> None:
        if mode == 'diagnosis':
            self.ui.append_diagnosis_stream(text)
        else:
            self.ui.append_codex_stream(text)

    def _chat_finish_stream(
            self, mode: str, speaker: str, text: str) -> None:
        if mode == 'diagnosis':
            self.ui.finish_diagnosis_stream(speaker, text)
        else:
            self.ui.finish_codex_stream(speaker, text)

    def _chat_clear_stream(self, mode: str) -> None:
        if mode == 'diagnosis':
            self.ui.clear_diagnosis_stream()
        else:
            self.ui.clear_codex_stream()

    def _chat_set_execution(
            self, mode: str, label: Optional[str]) -> None:
        self.ui.set_agent_execution(mode, label)

    @staticmethod
    def _execution_text(value, *, limit: int = 140) -> Optional[str]:
        if not isinstance(value, str):
            return None
        clean = ' '.join(
            value.replace('\x1b', '').replace('\x00', '').split())
        if not clean:
            return None
        if len(clean) > limit:
            clean = clean[:limit - 1].rstrip() + '…'
        return clean

    @classmethod
    def _execution_label_from_item(cls, item) -> Optional[str]:
        """Describe one executable app-server item in compact user language."""
        if not isinstance(item, dict):
            return None
        item_type = item.get('type')
        if item_type == 'commandExecution':
            return cls._command_activity_label(item)
        if item_type == 'fileChange':
            changes = item.get('changes')
            paths = []
            if isinstance(changes, list):
                for change in changes:
                    if not isinstance(change, dict):
                        continue
                    path = cls._execution_text(change.get('path'), limit=100)
                    if path and path not in paths:
                        paths.append(path)
            if not paths:
                return 'Editing workspace files'
            suffix = (
                f' (+{len(paths) - 1} more)'
                if len(paths) > 1 else ''
            )
            return f'Editing {paths[0]}{suffix}'
        if item_type == 'mcpToolCall':
            app_context = item.get('appContext')
            if isinstance(app_context, dict):
                app = cls._execution_text(
                    app_context.get('appName'), limit=50)
                action = cls._execution_text(
                    app_context.get('actionName'), limit=70)
                if app or action:
                    detail = ': '.join(
                        part for part in (app, action) if part)
                    return f'Using {detail}'
            server = cls._execution_text(item.get('server'), limit=50)
            tool = cls._execution_text(item.get('tool'), limit=70)
            detail = '/'.join(part for part in (server, tool) if part)
            return f'Using {detail or "tool"}'
        if item_type == 'dynamicToolCall':
            return cls._tool_execution_label(item.get('tool'))
        if item_type == 'collabAgentToolCall':
            tool = cls._execution_text(item.get('tool'), limit=80)
            return f'Running agent task {tool}' if tool else 'Running agent task'
        if item_type == 'webSearch':
            query = cls._execution_text(item.get('query'), limit=110)
            return f'Searching the web for {query}' if query else 'Searching the web'
        if item_type == 'imageView':
            path = cls._execution_text(item.get('path'), limit=110)
            return f'Reading image {path}' if path else 'Reading image'
        if item_type == 'sleep':
            duration = item.get('durationMs')
            if isinstance(duration, int) and duration >= 0:
                return f'Waiting for {duration / 1000:g}s'
            return 'Waiting'
        if item_type == 'imageGeneration':
            return 'Generating image'
        if item_type == 'reasoning':
            return 'Thinking'
        if item_type == 'plan':
            return 'Planning next steps'
        return None

    @classmethod
    def _reasoning_activity_label(cls, text: str) -> Optional[str]:
        """Turn a user-visible reasoning summary into one compact status line."""
        clean = cls._execution_text(text, limit=180)
        if clean is None:
            return None
        clean = re.sub(r'^[#>*_`\-\s]+', '', clean)
        clean = re.sub(r'[*_`]+', '', clean).strip()
        if not clean:
            return None
        return f'Thinking: {clean.rstrip(".…")}'

    @classmethod
    def _command_activity_label(cls, item: Dict) -> str:
        """Prefer Codex's parsed command action over shell-text guessing."""
        actions = item.get('commandActions')
        if isinstance(actions, list):
            for action in actions:
                if not isinstance(action, dict):
                    continue
                action_type = action.get('type')
                path = cls._execution_text(action.get('path'), limit=100)
                if action_type == 'read':
                    name = cls._execution_text(action.get('name'), limit=100)
                    return f'Reading {name or path or "file"}'
                if action_type == 'listFiles':
                    return f'Listing files{f" in {path}" if path else ""}'
                if action_type == 'search':
                    query = cls._execution_text(
                        action.get('query'), limit=80)
                    detail = f' for {query}' if query else ''
                    location = f' in {path}' if path else ''
                    return f'Searching{detail}{location}'

        command = cls._execution_text(item.get('command'), limit=130)
        if command is None:
            return 'Running command'
        executable_match = re.search(
            r'(?:^|[;&|]\s*)(?:\S+=\S+\s+)*(?:sudo\s+)?'
            r'(?:\S*/)?(?P<name>[A-Za-z0-9_.+-]+)\b',
            command,
        )
        executable = (
            executable_match.group('name').lower()
            if executable_match is not None else ''
        )
        if executable in {'cat', 'head', 'tail', 'less', 'more', 'sed'}:
            return f'Reading with {command}'
        if executable in {'rg', 'grep', 'find', 'fd'}:
            return f'Searching with {command}'
        if executable in {'ls', 'tree'}:
            return f'Listing files with {command}'
        return f'Running {command}'

    @staticmethod
    def _tool_execution_label(tool) -> str:
        labels = {
            'rosmon_control': 'Executing node control',
            'ros2_interface': 'Executing ROS operation',
            'rosmon_python_node': 'Starting Python node',
        }
        if isinstance(tool, str) and tool:
            return labels.get(tool, f'Executing {tool.replace("_", " ")}')
        return 'Executing tool'

    @staticmethod
    async def _write_app_server_message(process, message: Dict) -> None:
        """Write one JSONL request to a running Codex app server."""
        if process.stdin is None:
            raise OSError('Codex app server stdin is unavailable')
        process.stdin.write((json.dumps(message) + '\n').encode())
        await process.stdin.drain()

    @staticmethod
    def _agent_message_from_item(item) -> Optional[str]:
        """Return user-visible final text from an app-server thread item."""
        if not isinstance(item, dict) or item.get('type') != 'agentMessage':
            return None
        if item.get('phase') == 'commentary':
            return None
        text = item.get('text')
        return text if isinstance(text, str) and text.strip() else None

    @staticmethod
    def _codex_control_tools(question: str):
        """Expose rosmon mutation tools only for a direct Human action request."""
        if not CODEX_NODE_ACTION_REQUEST_RE.search(question):
            return []
        return [{
            'type': 'function',
            'name': 'rosmon_control',
            'description': (
                'Perform an explicitly requested rosmon action on supervised ROS nodes. '
                'Use only when the Human directly asks for the action. Never call this '
                'for a diagnosis, hypothetical question, suggestion, or implied recovery. '
                'Debug is valid only for one node.'
            ),
            'inputSchema': {
                'type': 'object',
                'additionalProperties': False,
                'required': ['action', 'scope'],
                'properties': {
                    'action': {
                        'type': 'string',
                        'enum': [
                            'start', 'stop', 'restart',
                            'mute', 'unmute', 'debug',
                        ],
                    },
                    'scope': {
                        'type': 'string',
                        'enum': ['node', 'namespace', 'all'],
                    },
                    'target': {
                        'type': 'string',
                        'description': (
                            'Exact node or namespace from the live snapshot. Omit only '
                            'for all, or to use the focused node.'
                        ),
                    },
                },
            },
        }]

    @staticmethod
    def _codex_ros_tools(
            question: str, *, mutation_allowed: Optional[bool] = None):
        """Expose bounded ROS graph operations and authorized requested actions."""
        inspect_operations = [
            'list_nodes', 'node_info',
            'list_parameters', 'get_parameter',
            'list_topics', 'topic_info',
            'list_services', 'service_type',
            'list_actions', 'action_info',
            'interface_show',
        ]
        operations = list(inspect_operations)
        if mutation_allowed is None:
            mutation_allowed = bool(
                CODEX_ROS_OPERATION_REQUEST_RE.search(question))
        if mutation_allowed:
            operations.extend(['call_service', 'send_action_goal'])
        return [{
            'type': 'function',
            'name': 'ros2_interface',
            'description': (
                'Inspect the live ROS 2 graph and, only for a direct Human '
                'request or its immediately pending clarification, call a '
                'service or send an action goal. Discover the '
                'exact interface first. Never guess motion direction, distance, '
                'frame, target, or safety parameters. For omitted motion speed '
                'and acceleration, inspect and use the controller configuration '
                'defaults rather than asking the Human or inventing values.'
            ),
            'inputSchema': {
                'type': 'object',
                'additionalProperties': False,
                'required': ['operation'],
                'properties': {
                    'operation': {
                        'type': 'string',
                        'enum': operations,
                    },
                    'name': {
                        'type': 'string',
                        'description': (
                            'Exact ROS node, topic, service, or action name.'
                        ),
                    },
                    'interface_type': {
                        'type': 'string',
                        'description': (
                            'Exact pkg/msg/Type, pkg/srv/Type, or '
                            'pkg/action/Type discovered from the live graph.'
                        ),
                    },
                    'parameter': {
                        'type': 'string',
                        'description': (
                            'Exact ROS parameter name for get_parameter.'
                        ),
                    },
                    'values': {
                        'type': 'string',
                        'description': (
                            'ROS request or goal values in YAML syntax.'
                        ),
                    },
                    'timeout_seconds': {
                        'type': 'number',
                        'minimum': 1,
                        'maximum': 120,
                    },
                    'feedback': {
                        'type': 'boolean',
                    },
                },
            },
        }]

    @staticmethod
    def _codex_python_node_tools(question: str):
        """Expose managed Python-node startup only for a direct Human request."""
        if not CODEX_PYTHON_NODE_REQUEST_RE.search(question):
            return []
        return [{
            'type': 'function',
            'name': 'rosmon_python_node',
            'description': (
                'Start a Python ROS node script as a Rosmon-supervised process after '
                'the Human directly requested it and the script has been written and '
                'checked inside the workspace. The node then appears in the GUI, uses '
                'an orange background, streams logs, and supports normal node controls. '
                'Never start the script separately with a shell command.'
            ),
            'inputSchema': {
                'type': 'object',
                'additionalProperties': False,
                'required': ['path', 'name'],
                'properties': {
                    'path': {
                        'type': 'string',
                        'description': (
                            'Path to the existing Python script. Relative paths resolve '
                            'inside ~/rosmon2, and absolute paths must also be inside '
                            '~/rosmon2.'
                        ),
                    },
                    'name': {
                        'type': 'string',
                        'description': (
                            'Exact ROS-style GUI name, optionally including namespace, '
                            'for example tools/health_probe.'
                        ),
                    },
                    'script_arguments': {
                        'type': 'array',
                        'maxItems': 32,
                        'items': {'type': 'string', 'maxLength': 1024},
                        'description': (
                            'Optional arguments passed directly to the Python script.'
                        ),
                    },
                },
            },
        }]

    @staticmethod
    def _ros_tool_arguments(params: Dict) -> Optional[Dict]:
        arguments = params.get('arguments')
        if isinstance(arguments, str):
            try:
                arguments = json.loads(arguments)
            except json.JSONDecodeError:
                return None
        return arguments if isinstance(arguments, dict) else None

    @staticmethod
    def _valid_ros_name(value) -> bool:
        return (
            isinstance(value, str)
            and 0 < len(value) <= 512
            and ROS_NAME_RE.fullmatch(value) is not None
        )

    @staticmethod
    def _valid_ros_interface(value, kind: Optional[str] = None) -> bool:
        if (
                not isinstance(value, str)
                or len(value) > 256
                or ROS_INTERFACE_RE.fullmatch(value) is None):
            return False
        return kind is None or f'/{kind}/' in value

    async def _run_codex_ros_tool(
            self, params: Dict, question: str, *,
            mutation_allowed: Optional[bool] = None,
            authorization_question: Optional[str] = None) -> tuple[bool, str]:
        """Validate and run one bounded ROS CLI graph/service/action operation."""
        if params.get('tool') != 'ros2_interface':
            return False, f"Unknown Agent tool {params.get('tool')!r}."
        arguments = self._ros_tool_arguments(params)
        if arguments is None:
            return False, 'ROS tool arguments must be a JSON object.'

        operation = arguments.get('operation')
        inspect_operations = {
            'list_nodes', 'node_info',
            'list_parameters', 'get_parameter',
            'list_topics', 'topic_info',
            'list_services', 'service_type',
            'list_actions', 'action_info',
            'interface_show',
        }
        mutation_operations = {'call_service', 'send_action_goal'}
        if mutation_allowed is None:
            mutation_allowed = bool(
                CODEX_ROS_OPERATION_REQUEST_RE.search(question))
        effective_question = authorization_question or question
        if operation not in inspect_operations | mutation_operations:
            return False, f'Unsupported ROS operation {operation!r}.'
        if (
                operation in mutation_operations
                and not mutation_allowed):
            return False, (
                'The Human did not directly request a ROS service call or '
                'action goal. Inspecting the graph is still allowed.'
            )
        if (
                operation in mutation_operations
                and CODEX_UNSAFE_ROS_REQUEST_RE.search(effective_question)):
            return False, (
                'Rosmon will not bypass robot safety limits or interlocks.'
            )
        if (
                operation == 'send_action_goal'
                and CODEX_ARM_MOTION_REQUEST_RE.search(effective_question)
                and not CODEX_ARM_TARGET_RE.search(effective_question)):
            return False, (
                'Robot motion is missing an explicit target or a direction '
                'and numeric distance/angle. Ask the Human for those details; '
                'do not guess them.'
            )

        name = arguments.get('name')
        interface_type = arguments.get('interface_type')
        values = arguments.get('values', '{}')
        if not isinstance(values, str) or len(values) > 32768:
            return False, 'ROS values must be a YAML string no larger than 32 KiB.'
        timeout = arguments.get('timeout_seconds', 60 if operation in mutation_operations else 10)
        if (
                isinstance(timeout, bool)
                or not isinstance(timeout, (int, float))
                or not 1 <= timeout <= 120):
            return False, 'ROS timeout_seconds must be between 1 and 120.'
        timeout_text = str(int(timeout) if float(timeout).is_integer() else timeout)

        if operation == 'list_nodes':
            command = ['ros2', 'node', 'list']
        elif operation == 'node_info':
            if not self._valid_ros_name(name):
                return False, 'node_info requires a valid exact ROS node name.'
            command = ['ros2', 'node', 'info', name]
        elif operation == 'list_parameters':
            if not self._valid_ros_name(name):
                return False, (
                    'list_parameters requires a valid exact ROS node name.'
                )
            command = ['ros2', 'param', 'list', name]
        elif operation == 'get_parameter':
            parameter = arguments.get('parameter')
            if not self._valid_ros_name(name):
                return False, (
                    'get_parameter requires a valid exact ROS node name.'
                )
            if (
                    not isinstance(parameter, str)
                    or len(parameter) > 256
                    or ROS_PARAMETER_RE.fullmatch(parameter) is None):
                return False, (
                    'get_parameter requires a valid exact ROS parameter name.'
                )
            command = ['ros2', 'param', 'get', name, parameter]
        elif operation == 'list_topics':
            command = ['ros2', 'topic', 'list', '-t']
        elif operation == 'topic_info':
            if not self._valid_ros_name(name):
                return False, 'topic_info requires a valid exact ROS topic name.'
            command = ['ros2', 'topic', 'info', name, '--verbose']
        elif operation == 'list_services':
            command = ['ros2', 'service', 'list', '-t']
        elif operation == 'service_type':
            if not self._valid_ros_name(name):
                return False, 'service_type requires a valid exact service name.'
            command = ['ros2', 'service', 'type', name]
        elif operation == 'list_actions':
            command = ['ros2', 'action', 'list', '-t']
        elif operation == 'action_info':
            if not self._valid_ros_name(name):
                return False, 'action_info requires a valid exact action name.'
            command = ['ros2', 'action', 'info', name]
        elif operation == 'interface_show':
            if not self._valid_ros_interface(interface_type):
                return False, 'interface_show requires a valid exact ROS interface type.'
            command = ['ros2', 'interface', 'show', interface_type]
        elif operation == 'call_service':
            if not self._valid_ros_name(name):
                return False, 'call_service requires a valid exact service name.'
            if not self._valid_ros_interface(interface_type, 'srv'):
                return False, 'call_service requires an exact pkg/srv/Type.'
            command = ['ros2', 'service', 'call', name, interface_type, values]
        else:
            if not self._valid_ros_name(name):
                return False, 'send_action_goal requires a valid exact action name.'
            if not self._valid_ros_interface(interface_type, 'action'):
                return False, 'send_action_goal requires an exact pkg/action/Type.'
            command = ['ros2', 'action', 'send_goal']
            if arguments.get('feedback') is True:
                command.append('--feedback')
            command.extend([
                '--timeout', timeout_text,
                name, interface_type, values,
            ])

        if shutil.which('ros2') is None:
            return False, (
                "The 'ros2' command is not on rosmon2's PATH. Source the ROS 2 "
                'and workspace setup files before starting mon2.'
            )

        process = None
        try:
            process = await asyncio.create_subprocess_exec(
                *command,
                cwd=str(self.codex_workspace),
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
            self._ros_tool_process = process
            stdout, stderr = await asyncio.wait_for(
                process.communicate(), timeout=float(timeout) + 2.0)
        except asyncio.TimeoutError:
            if process is not None and process.returncode is None:
                process.terminate()
                await process.wait()
            return False, (
                f'ROS operation timed out after {timeout_text} seconds.'
            )
        except (FileNotFoundError, OSError) as exc:
            return False, f'Could not run the ROS operation: {exc}'
        finally:
            if self._ros_tool_process is process:
                self._ros_tool_process = None

        output = (stdout + stderr).decode(errors='replace').strip()
        if len(output) > 12000:
            output = output[-12000:]
            output = '[earlier output truncated]\n' + output
        command_text = ' '.join(shlex.quote(part) for part in command)
        if process.returncode != 0:
            return False, (
                f'ROS command failed with exit code {process.returncode}:\n'
                f'$ {command_text}\n{output or "(no output)"}'
            )
        if operation in mutation_operations:
            self._codex_pending_ros_operation_question = None
            self._emit_event(
                'ros_action',
                operation=operation,
                name=name,
                interface_type=interface_type,
            )
        return True, (
            f'$ {command_text}\n'
            f'{output or "ROS operation completed without output."}'
        )

    async def _run_codex_python_node_tool(
            self, params: Dict, question: str) -> tuple[bool, str]:
        """Validate, register, and launch one Agent-created Python ROS node."""
        if params.get('tool') != 'rosmon_python_node':
            return False, f"Unknown Agent tool {params.get('tool')!r}."
        if not CODEX_PYTHON_NODE_REQUEST_RE.search(question):
            return False, (
                'The Human did not directly request starting a Python ROS node.'
            )
        arguments = self._ros_tool_arguments(params)
        if arguments is None:
            return False, 'Python-node tool arguments must be a JSON object.'

        raw_path = arguments.get('path')
        if not isinstance(raw_path, str) or not raw_path.strip():
            return False, (
                'path must name an existing Python script inside ~/rosmon2.')
        candidate = Path(raw_path).expanduser()
        if not candidate.is_absolute():
            candidate = self.agent_node_workspace / candidate
        try:
            script_path = candidate.resolve(strict=True)
            script_path.relative_to(self.agent_node_workspace)
        except (FileNotFoundError, OSError, ValueError):
            return False, (
                'The Python script must be an existing file inside ~/rosmon2.'
            )
        if not script_path.is_file() or script_path.suffix.lower() != '.py':
            return False, 'The managed node path must be a .py file.'
        try:
            if script_path.stat().st_size > 1024 * 1024:
                return False, 'The managed Python node script must be 1 MiB or smaller.'
            source = script_path.read_text(encoding='utf-8')
            compile(source, str(script_path), 'exec')
        except (OSError, UnicodeError, SyntaxError) as exc:
            return False, f'The Python node script could not be checked: {exc}'

        raw_name = arguments.get('name')
        if (
                not isinstance(raw_name, str)
                or len(raw_name) > 256
                or ROS_MANAGED_NODE_NAME_RE.fullmatch(raw_name) is None):
            return False, (
                'name must be a valid ROS-style node name, optionally including '
                'a namespace.'
            )
        display_name = self._normalize_display_name(raw_name)
        if any(
                record.display_name.lstrip('/') == display_name
                for record in self.records):
            return False, (
                f"A supervised node named '/{display_name}' already exists."
            )

        script_arguments = arguments.get('script_arguments', [])
        if (
                not isinstance(script_arguments, list)
                or len(script_arguments) > 32
                or any(
                    not isinstance(item, str)
                    or len(item) > 1024
                    or '\x00' in item
                    for item in script_arguments)):
            return False, (
                'script_arguments must contain at most 32 strings of at most '
                '1024 characters each.'
            )
        if self._context is None or self._shutting_down:
            return False, (
                'The live launch context is not available, so Rosmon cannot '
                'supervise a new node right now.'
            )

        record = ProcessRecord(
            key=self._next_key,
            display_name=display_name,
            cmd=[sys.executable, str(script_path), *script_arguments],
            cwd=str(self.agent_node_workspace),
            agent_created=True,
        )
        self._next_key += 1
        self.records.append(record)
        self.ui.set_records(self.records)
        try:
            self.start(record, count_restart=False)
        except Exception as exc:
            self.records.remove(record)
            self.ui.set_records(self.records)
            return False, f'Rosmon could not start the Python node: {exc}'

        self._emit_event(
            'agent_node_registered',
            node=self._record_dict(record),
            script=str(script_path),
        )
        return True, (
            f"Started '/{display_name}' as an Agent-created supervised node. "
            'It is now in the node GUI with an orange background; Rosmon will '
            'capture its output and lifecycle and can stop, start, restart, '
            'mute, or debug it normally.'
        )

    async def _run_codex_control_tool(
            self, params: Dict, question: str) -> tuple[bool, str]:
        """Validate and execute one dynamic rosmon control tool call."""
        if params.get('tool') != 'rosmon_control':
            return False, f"Unknown Agent tool {params.get('tool')!r}."
        if not CODEX_NODE_ACTION_REQUEST_RE.search(question):
            return False, 'No direct node action was requested by the Human.'

        arguments = params.get('arguments')
        if isinstance(arguments, str):
            try:
                arguments = json.loads(arguments)
            except json.JSONDecodeError:
                arguments = None
        if not isinstance(arguments, dict):
            return False, 'Tool arguments must be a JSON object.'

        action = arguments.get('action')
        scope = arguments.get('scope')
        target = arguments.get('target')
        valid_actions = {
            'start', 'stop', 'restart', 'mute', 'unmute', 'debug',
        }
        if action not in valid_actions:
            return False, f'Unsupported rosmon action {action!r}.'
        if scope not in ('node', 'namespace', 'all'):
            return False, f'Unsupported rosmon scope {scope!r}.'

        focus = self._codex_focus_record(question)
        if scope == 'node' and not target and focus is not None:
            target = focus.display_name
        elif scope == 'namespace' and not target and focus is not None:
            target = self._namespace_for(focus)
        if scope == 'node' and isinstance(target, str) and len(target) == 1:
            referenced = self._codex_referenced_record(f'node {target}')
            if referenced is not None:
                target = referenced.display_name

        request = {'command': action}
        if scope == 'node':
            if not target:
                return False, 'A node target is required.'
            request['node'] = str(target)
        elif scope == 'namespace':
            if target is None:
                return False, 'A namespace target is required.'
            request['namespace'] = str(target)
        else:
            request['all'] = True

        try:
            if action == 'debug':
                if scope != 'node':
                    raise ControlError('debug requires one node target')
                records = self._selected_records(request)
                if len(records) != 1:
                    raise ControlError('debug requires exactly one matching node')
                self.debug(records[0])
                self.ui.redraw()
                self._emit_event(
                    'control_action',
                    action=action,
                    nodes=[self._record_dict(records[0])],
                )
                result = {
                    'action': action,
                    'matched': 1,
                    'nodes': [self._record_dict(records[0])],
                }
            else:
                result = await self.control_request(request)
        except (ControlError, OSError, ValueError) as exc:
            return False, str(exc)

        names = [
            node.get('name', 'unknown')
            for node in result.get('nodes', [])
            if isinstance(node, dict)
        ]
        return True, (
            f"{action.capitalize()} accepted for {result.get('matched', 0)} "
            f"node(s): {', '.join(names) or 'none'}."
        )

    async def _run_codex(
            self, question: str, *, mode: str = 'agent') -> None:
        """Run one interactive Agent or Diagnosis turn beside the live launch."""
        if mode not in ('agent', 'diagnosis'):
            raise ValueError(f'unknown Agent mode {mode!r}')
        ros_mutation_allowed = False
        ros_authorization_question = question
        if mode == 'agent':
            if CODEX_ROS_OPERATION_REQUEST_RE.search(question):
                ros_mutation_allowed = True
                self._codex_pending_ros_operation_question = question
            elif (
                    self._codex_pending_ros_operation_question
                    and CODEX_ROS_OPERATION_FOLLOWUP_RE.fullmatch(question)):
                ros_mutation_allowed = True
                ros_authorization_question = (
                    f'{self._codex_pending_ros_operation_question}\n'
                    f'Human clarification: {question}'
                )
                self._codex_pending_ros_operation_question = (
                    ros_authorization_question)
            else:
                self._codex_pending_ros_operation_question = None
        else:
            self._codex_pending_ros_operation_question = None
        if mode == 'agent':
            # Some ROS hosts disable unprivileged user namespaces, which makes
            # Codex's Bubblewrap workspace sandbox fail before any command can
            # run ("bwrap: setting up uid map: Permission denied"). Agent
            # mutations are still gated by the direct-request checks and
            # validated Rosmon/ROS tools below.
            sandbox = 'danger-full-access'
            sandbox_policy = {
                'type': 'dangerFullAccess',
            }
            if self.ui.codex_access_mode == 'approve-for-me':
                approval_policy = 'on-request'
                approvals_reviewer = 'auto_review'
            else:
                approval_policy = 'never'
                approvals_reviewer = None
        else:
            sandbox = 'read-only'
            sandbox_policy = {
                'type': 'readOnly',
                'networkAccess': True,
            }
            approval_policy = 'never'
            approvals_reviewer = None
        history = (
            self._diagnosis_chat_history
            if mode == 'diagnosis' else
            self._codex_history
        )
        history.append(('User', question))
        self._codex_mode = mode
        command = [
            self.codex_command,
            'app-server',
            '--stdio',
        ]
        dynamic_tools = self._codex_control_tools(question)
        python_node_tools = []
        python_node_write_requested = False
        if mode == 'agent':
            dynamic_tools.extend(self._codex_ros_tools(
                question, mutation_allowed=ros_mutation_allowed))
            python_node_tools = self._codex_python_node_tools(question)
            dynamic_tools.extend(python_node_tools)
            python_node_write_requested = bool(
                CODEX_PYTHON_NODE_WRITE_REQUEST_RE.search(question))
        turn_workspace = (
            self.agent_node_workspace
            if python_node_write_requested else self.codex_workspace
        )
        process = None
        stderr_task = None
        try:
            if self._codex_cancel_requested:
                self._chat_add_message(mode, 'Codex', 'Request cancelled.')
                self._chat_set_running(mode, False)
                return
            if not self.codex_workspace.is_dir():
                raise FileNotFoundError(
                    f"Codex workspace '{self.codex_workspace}' is not a directory"
                )
            if python_node_tools or python_node_write_requested:
                self.agent_node_workspace.mkdir(parents=True, exist_ok=True)
            if shutil.which(self.codex_command) is None:
                raise FileNotFoundError(
                    f"Codex CLI command '{self.codex_command}' was not found on PATH"
                )
            self._chat_set_running(mode, True)
            process = await asyncio.create_subprocess_exec(
                *command,
                cwd=str(turn_workspace),
                stdin=asyncio.subprocess.PIPE,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                limit=CODEX_APP_SERVER_STREAM_LIMIT,
            )
            self._codex_process = process
            stderr_task = asyncio.create_task(process.stderr.read())
            if self._codex_cancel_requested:
                try:
                    process.terminate()
                except ProcessLookupError:
                    pass

            initialize_params = {
                'clientInfo': {
                    'name': 'rosmon2',
                    'title': 'rosmon2 Agent',
                    'version': '0.1.0',
                },
            }
            if dynamic_tools:
                initialize_params['capabilities'] = {
                    'experimentalApi': True,
                }
            await self._write_app_server_message(process, {
                'id': 1,
                'method': 'initialize',
                'params': initialize_params,
            })
            await asyncio.wait_for(
                self._read_app_server_response(process.stdout, 1),
                timeout=5.0,
            )
            await self._write_app_server_message(
                process, {'method': 'initialized', 'params': {}})
            thread_params = {
                'cwd': str(turn_workspace),
                'approvalPolicy': approval_policy,
                'sandbox': sandbox,
                'ephemeral': True,
                'serviceName': 'rosmon2',
            }
            if approvals_reviewer is not None:
                thread_params['approvalsReviewer'] = approvals_reviewer
            if self.ui.codex_selected_model is not None:
                thread_params['model'] = self.ui.codex_selected_model
            if dynamic_tools:
                thread_params['dynamicTools'] = dynamic_tools
            await self._write_app_server_message(process, {
                'id': 2,
                'method': 'thread/start',
                'params': thread_params,
            })
            thread_result = await asyncio.wait_for(
                self._read_app_server_response(process.stdout, 2),
                timeout=30.0,
            )
            thread = thread_result.get('thread')
            thread_id = thread.get('id') if isinstance(thread, dict) else None
            if not isinstance(thread_id, str) or not thread_id:
                raise OSError('Codex did not return a thread id')

            await self._write_app_server_message(process, {
                'id': 3,
                'method': 'turn/start',
                'params': {
                    'threadId': thread_id,
                    'effort': self.ui.codex_reasoning_effort(),
                        'summary': 'detailed',
                        'sandboxPolicy': sandbox_policy,
                        'input': [{
                            'type': 'text',
                            'text': self._codex_context(
                                question,
                                mode=mode,
                                ros_operation_allowed=ros_mutation_allowed,
                            ),
                        }],
                },
            })

            item_phases = {}
            active_executions = {}
            reasoning_summaries = {}

            def refresh_execution() -> None:
                label = (
                    next(reversed(active_executions.values()))
                    if active_executions else None
                )
                self._chat_set_execution(mode, label)

            active_stream_item = None
            streamed_text = ''
            final_messages = []
            turn_status = None
            turn_error = None
            while True:
                try:
                    line = await process.stdout.readline()
                except ValueError as exc:
                    raise OSError(
                        'Codex returned an app-server event larger than '
                        f'{CODEX_APP_SERVER_STREAM_LIMIT // (1024 * 1024)} MiB. '
                        'The request was stopped safely.'
                    ) from exc
                if not line:
                    raise OSError(
                        'Codex app server closed before completing the response')
                try:
                    message = json.loads(line)
                except (json.JSONDecodeError, UnicodeDecodeError):
                    continue

                if message.get('id') == 3 and 'error' in message:
                    raise OSError(
                        f"Codex turn failed to start: {message['error']}")

                method = message.get('method')
                params = message.get('params')
                if not isinstance(params, dict):
                    params = {}
                if method == 'item/tool/call' and 'id' in message:
                    execution_key = (
                        'tool-request',
                        params.get('callId', message['id']),
                    )
                    active_executions[execution_key] = (
                        self._tool_execution_label(params.get('tool')))
                    refresh_execution()
                    try:
                        if params.get('tool') == 'ros2_interface':
                            success, result_text = await self._run_codex_ros_tool(
                                params,
                                question,
                                mutation_allowed=ros_mutation_allowed,
                                authorization_question=(
                                    ros_authorization_question),
                            )
                        elif params.get('tool') == 'rosmon_python_node':
                            success, result_text = (
                                await self._run_codex_python_node_tool(
                                    params, question))
                        else:
                            success, result_text = (
                                await self._run_codex_control_tool(
                                    params, question))
                        await self._write_app_server_message(process, {
                            'id': message['id'],
                            'result': {
                                'contentItems': [{
                                    'type': 'inputText',
                                    'text': result_text,
                                }],
                                'success': success,
                            },
                        })
                    finally:
                        active_executions.pop(execution_key, None)
                        refresh_execution()
                elif method == 'item/started':
                    item = params.get('item')
                    item_id = item.get('id') if isinstance(item, dict) else None
                    execution_label = self._execution_label_from_item(item)
                    if isinstance(item_id, str) and execution_label is not None:
                        active_executions[('item', item_id)] = execution_label
                        refresh_execution()
                    if isinstance(item, dict) and item.get('type') == 'agentMessage':
                        item_id = item.get('id')
                        if isinstance(item_id, str):
                            phase = item.get('phase')
                            item_phases[item_id] = phase
                            if phase != 'commentary':
                                active_stream_item = item_id
                                streamed_text = ''
                                self._chat_begin_stream(mode)
                elif method == 'item/agentMessage/delta':
                    item_id = params.get('itemId')
                    delta = params.get('delta')
                    if (isinstance(item_id, str) and isinstance(delta, str)
                            and item_phases.get(item_id) != 'commentary'):
                        if active_stream_item != item_id:
                            active_stream_item = item_id
                            streamed_text = ''
                            self._chat_begin_stream(mode)
                        streamed_text += delta
                        self._chat_append_stream(mode, delta)
                elif method == 'item/reasoning/summaryPartAdded':
                    item_id = params.get('itemId')
                    summary_index = params.get('summaryIndex')
                    if isinstance(item_id, str) and isinstance(summary_index, int):
                        reasoning_summaries[(item_id, summary_index)] = ''
                elif method == 'item/reasoning/summaryTextDelta':
                    item_id = params.get('itemId')
                    summary_index = params.get('summaryIndex')
                    delta = params.get('delta')
                    if (
                            isinstance(item_id, str)
                            and isinstance(summary_index, int)
                            and isinstance(delta, str)):
                        summary_key = (item_id, summary_index)
                        reasoning_summaries[summary_key] = (
                            reasoning_summaries.get(summary_key, '') + delta
                        )
                        label = self._reasoning_activity_label(
                            reasoning_summaries[summary_key])
                        if label is not None:
                            active_executions[('item', item_id)] = label
                            refresh_execution()
                elif method == 'item/completed':
                    item = params.get('item')
                    item_id = item.get('id') if isinstance(item, dict) else None
                    if isinstance(item_id, str):
                        active_executions.pop(('item', item_id), None)
                        refresh_execution()
                    text = self._agent_message_from_item(item)
                    if text is not None:
                        final_messages.append(text)
                elif method == 'turn/completed':
                    active_executions.clear()
                    refresh_execution()
                    turn = params.get('turn')
                    if isinstance(turn, dict):
                        turn_status = turn.get('status')
                        error = turn.get('error')
                        if isinstance(error, dict):
                            turn_error = error.get('message')
                        if not final_messages:
                            for item in turn.get('items', []):
                                text = self._agent_message_from_item(item)
                                if text is not None:
                                    final_messages.append(text)
                    break

            if self._codex_cancel_requested:
                self._chat_clear_stream(mode)
                self._chat_add_message(mode, 'Codex', 'Request cancelled.')
                self._chat_set_running(mode, False)
            elif turn_status in (None, 'completed'):
                answer = self._clean_codex_text(
                    final_messages[-1] if final_messages else streamed_text)
                history.append(('Codex', answer))
                if answer.lower().rstrip().endswith('[y/n]'):
                    self._codex_yes_no_pending = True
                    self._codex_yes_no_mode = mode
                    self._codex_pending_fix_question = question
                # Keep the compact footer useful and put the complete answer
                # into normal terminal scrollback for longer debugging plans.
                self.ui.log('rosmon', answer)
                self._chat_finish_stream(mode, 'Codex', answer)
                self._chat_set_running(mode, False)
                if (
                        (mode == 'agent' and self.ui.codex_active)
                        or (mode == 'diagnosis' and self.ui.diagnosis_active)):
                    self._request_codex_usage()
            else:
                detail = turn_error or f'turn ended with status {turn_status}'
                self._chat_clear_stream(mode)
                self._chat_add_message(
                    mode,
                    'Codex',
                    f'Rosmon could not complete the request: {detail}',
                )
                self._chat_set_running(mode, False)
        except asyncio.CancelledError:
            self._chat_clear_stream(mode)
            self._chat_add_message(mode, 'Codex', 'Request cancelled.')
            self._chat_set_running(mode, False)
            raise
        except (FileNotFoundError, OSError, asyncio.TimeoutError) as exc:
            self._chat_clear_stream(mode)
            if self._codex_cancel_requested:
                self._chat_add_message(mode, 'Codex', 'Request cancelled.')
            else:
                self._chat_add_message(mode, 'Codex', str(exc))
            self._chat_set_running(mode, False)
        except Exception as exc:
            # Background Agent tasks must report failures through the panel;
            # otherwise asyncio prints "Task exception was never retrieved"
            # over the live terminal UI.
            self._chat_clear_stream(mode)
            self._chat_add_message(
                mode,
                'Codex',
                f'Rosmon Agent stopped after an unexpected error: {exc}',
            )
            self._chat_set_running(mode, False)
        finally:
            self._chat_set_execution(mode, None)
            if process is not None:
                if process.stdin is not None:
                    process.stdin.close()
                try:
                    await asyncio.wait_for(process.wait(), timeout=1.0)
                except asyncio.TimeoutError:
                    if process.returncode is None:
                        try:
                            process.terminate()
                        except ProcessLookupError:
                            pass
                    await process.wait()
            if stderr_task is not None:
                try:
                    await stderr_task
                except (asyncio.CancelledError, OSError):
                    pass
            self._codex_process = None
            self._codex_task = None
            self._codex_mode = None

    def _cancel_codex(self) -> None:
        """Stop the child CLI without interrupting the ROS launch."""
        self._codex_cancel_requested = True
        if self._codex_mode in ('agent', 'diagnosis'):
            self._chat_set_execution(
                self._codex_mode, 'Stopping current query')
        process = self._codex_process
        if process is not None and process.returncode is None:
            try:
                process.terminate()
            except ProcessLookupError:
                pass
            if self._codex_mode == 'agent':
                self.ui.set_codex_running(True, 'Cancelling Codex…')
        ros_process = self._ros_tool_process
        if ros_process is not None and ros_process.returncode is None:
            try:
                ros_process.terminate()
            except ProcessLookupError:
                pass

    async def _stop_codex(self) -> None:
        """Clean up a diagnostic subprocess before terminal restoration."""
        task = self._codex_task
        if task is None:
            return
        self._cancel_codex()
        try:
            await asyncio.wait_for(asyncio.shield(task), timeout=3.0)
        except asyncio.TimeoutError:
            process = self._codex_process
            if process is not None and process.returncode is None:
                try:
                    process.kill()
                except ProcessLookupError:
                    pass
            await task
        except asyncio.CancelledError:
            pass

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
            self._diagnosis_record_changed(record, 'node stopped')
            return
        target = record.action
        # The keyboard reader runs in the launch event loop.  Calling the
        # thread-safe LaunchService.emit_event() here would wait on this same
        # loop and deadlock it.
        self._context.emit_event_sync(
            ShutdownProcess(process_matcher=lambda action: action is target)
        )

    def start(
            self, record: ProcessRecord,
            *, count_restart: bool = True) -> Optional[object]:
        """Start a stopped process again from its fully expanded command."""
        if record.pid is not None or not record.cmd or self._context is None:
            return
        record.manually_stopped = False
        record.state = State.WAITING
        if count_restart:
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
        try:
            action.execute(self._context)
        except Exception:
            self._by_action.pop(action, None)
            raise
        self.ui.redraw()
        self._diagnosis_record_changed(record, 'node restart requested')
        return action

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
            observed_at = time.monotonic()
            if severity in ('ERROR', 'FATAL'):
                self._diagnosis_error_times.setdefault(
                    source, deque()).append(observed_at)
            if DIAGNOSIS_STALL_RE.search(line):
                self._diagnosis_stall_times.setdefault(
                    source, deque()).append(observed_at)
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
            'agent_created': record.agent_created,
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
