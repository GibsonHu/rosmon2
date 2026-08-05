import asyncio
import json
from datetime import datetime

from launch_ros.actions import Node

from rosmon2.model import ProcessRecord, State
from rosmon2.supervisor import Supervisor


class _FakeCodexReadStream:
    def __init__(self, messages=()):
        self._lines = [
            (json.dumps(message) + '\n').encode() for message in messages
        ]

    async def readline(self):
        return self._lines.pop(0) if self._lines else b''

    async def read(self):
        return b''


class _FakeCodexWriteStream:
    def __init__(self):
        self.messages = []

    def write(self, data):
        self.messages.extend(
            json.loads(line) for line in data.decode().splitlines())

    async def drain(self):
        return None

    def close(self):
        return None


class _FakeCodexProcess:
    def __init__(
            self, answer, chunks=None, tool_request=None,
            reasoning_chunks=None):
        chunks = chunks or [answer]
        item = {
            'id': 'item-1',
            'type': 'agentMessage',
            'phase': 'final_answer',
            'text': answer,
        }
        reasoning_item = {
            'id': 'reasoning-1',
            'type': 'reasoning',
            'summary': [],
            'content': [],
        }
        reasoning_messages = []
        if reasoning_chunks:
            reasoning_messages = [
                {
                    'method': 'item/started',
                    'params': {'item': reasoning_item},
                },
                {
                    'method': 'item/reasoning/summaryPartAdded',
                    'params': {
                        'itemId': 'reasoning-1',
                        'summaryIndex': 0,
                    },
                },
                *[
                    {
                        'method': 'item/reasoning/summaryTextDelta',
                        'params': {
                            'itemId': 'reasoning-1',
                            'summaryIndex': 0,
                            'delta': chunk,
                        },
                    }
                    for chunk in reasoning_chunks
                ],
                {
                    'method': 'item/completed',
                    'params': {'item': reasoning_item},
                },
            ]
        messages = [
            {'id': 1, 'result': {}},
            {'id': 2, 'result': {'thread': {'id': 'thread-1'}}},
            {'id': 3, 'result': {
                'turn': {'id': 'turn-1', 'status': 'inProgress', 'items': []},
            }},
            *([tool_request] if tool_request is not None else []),
            *reasoning_messages,
            {'method': 'item/started', 'params': {'item': {
                **item, 'text': '',
            }}},
            *[
                {
                    'method': 'item/agentMessage/delta',
                    'params': {'itemId': 'item-1', 'delta': chunk},
                }
                for chunk in chunks
            ],
            {'method': 'item/completed', 'params': {'item': item}},
            {'method': 'turn/completed', 'params': {
                'turn': {
                    'id': 'turn-1',
                    'status': 'completed',
                    'items': [item],
                },
            }},
        ]
        self.stdin = _FakeCodexWriteStream()
        self.stdout = _FakeCodexReadStream(messages)
        self.stderr = _FakeCodexReadStream()
        self.returncode = None

    async def wait(self):
        self.returncode = 0
        return 0

    def terminate(self):
        self.returncode = -15


class _UnnamedNode(Node):
    @property
    def node_name(self):
        return '/ur10e/<node_name_unspecified>'


def test_display_names_do_not_include_the_root_slash():
    assert Supervisor._normalize_display_name('/talker') == 'talker'
    assert Supervisor._normalize_display_name('/robot/talker') == 'robot/talker'
    assert Supervisor._normalize_display_name('talker') == 'talker'


def test_unnamed_node_uses_its_process_name():
    action = object.__new__(_UnnamedNode)
    assert Supervisor._display_name(action, 'move_group-5') == 'ur10e/move_group'


def test_process_counter_removal_preserves_hyphens_in_names():
    assert Supervisor._process_name_without_counter('camera-driver-12') == 'camera-driver'
    assert Supervisor._process_name_without_counter('camera-driver') == 'camera-driver'


def test_namespace_mode_can_inspect_and_stop_a_group(monkeypatch):
    supervisor = Supervisor('', [], ui=False)
    root = ProcessRecord(key=0, display_name='hardware_setup')
    move_group = ProcessRecord(key=1, display_name='ur10e/move_group')
    command_server = ProcessRecord(
        key=2, display_name='ur10e/ur_ros_rtde/command_server')
    supervisor.records.extend([root, move_group, command_server])
    supervisor.ui.set_records(supervisor.records)

    supervisor.handle_key('F5')
    assert supervisor.ui.namespace_mode
    # Root is key a; ur10e is key b.
    supervisor.handle_key('b')
    supervisor.handle_key('i')
    assert supervisor.ui.namespace_inspect == 'ur10e'
    assert supervisor.ui.visible_records() == [move_group, command_server]

    supervisor.handle_key('b')
    supervisor.handle_key('m')
    assert command_server.muted
    assert supervisor.ui.namespace_inspect == 'ur10e'

    supervisor.handle_key('\x7f')
    stopped = []
    monkeypatch.setattr(supervisor, 'stop', stopped.append)
    supervisor.handle_key('b')
    supervisor.handle_key('k')
    assert stopped == [move_group, command_server]


def test_namespace_mode_can_mute_and_unmute_a_group():
    supervisor = Supervisor('', [], ui=False)
    root = ProcessRecord(key=0, display_name='hardware_setup')
    move_group = ProcessRecord(key=1, display_name='ur10e/move_group')
    command_server = ProcessRecord(
        key=2, display_name='ur10e/ur_ros_rtde/command_server')
    supervisor.records.extend([root, move_group, command_server])
    supervisor.ui.set_records(supervisor.records)
    supervisor.handle_key('F5')

    # Root is key a; ur10e is key b.
    supervisor.handle_key('b')
    supervisor.handle_key('m')
    assert not root.muted
    assert move_group.muted
    assert command_server.muted

    supervisor.handle_key('b')
    supervisor.handle_key('u')
    assert not move_group.muted
    assert not command_server.muted


def test_node_search_filters_navigates_and_selects_full_names():
    supervisor = Supervisor('', [], ui=False)
    receiver = ProcessRecord(
        key=0, display_name='ur10e/ur_ros_rtde/robot_state_receiver')
    server = ProcessRecord(
        key=1, display_name='ur10e/ur_ros_rtde/command_server')
    camera = ProcessRecord(key=2, display_name='external/camera')
    supervisor.records.extend([receiver, server, camera])
    supervisor.ui.set_records(supervisor.records)

    supervisor.handle_key('/')
    for key in 'ur_ros_rtde':
        supervisor.handle_key(key)

    assert supervisor.ui.search_active
    assert supervisor.ui.search_matches() == [receiver, server]
    supervisor.handle_key('\t')
    supervisor.handle_key('\n')
    assert not supervisor.ui.search_active
    assert supervisor.ui.selected == 1

    supervisor.handle_key('m')
    assert server.muted
    assert not receiver.muted


def test_node_search_backspace_and_escape_cancel():
    supervisor = Supervisor('', [], ui=False)
    supervisor.records.append(ProcessRecord(key=0, display_name='robot/driver'))
    supervisor.ui.set_records(supervisor.records)

    supervisor.handle_key('/')
    supervisor.handle_key('x')
    supervisor.handle_key('\x7f')
    assert supervisor.ui.search_query == ''

    supervisor.handle_key('ESC')
    assert not supervisor.ui.search_active
    assert supervisor.ui.selected is None


def test_f4_opens_and_closes_codex_without_losing_selected_node():
    supervisor = Supervisor('', [], ui=False, control=False)
    driver = ProcessRecord(key=0, display_name='ur10e/driver')
    supervisor.records.append(driver)
    supervisor.ui.set_records(supervisor.records)
    supervisor.ui.selected = 0

    supervisor.handle_key('F4')

    assert supervisor.ui.codex_active
    assert supervisor._codex_focus_record() is driver

    supervisor.handle_key('F4')

    assert not supervisor.ui.codex_active
    assert supervisor.ui.selected == 0


def test_f4_logged_out_highlights_enter_to_login(monkeypatch):
    supervisor = Supervisor('', [], ui=False, control=False)
    supervisor._codex_logged_in = False
    supervisor.ui.set_codex_login_state(False)
    actions = []
    monkeypatch.setattr(
        supervisor,
        '_start_codex_auth',
        lambda action, *, mode: actions.append((action, mode)),
    )

    supervisor.handle_key('F4')

    assert supervisor.ui.codex_active
    assert supervisor.ui.codex_model_picker_active
    selected = supervisor.ui._codex_picker_choices()[
        supervisor.ui.codex_model_picker_selected]
    assert selected['action'] == 'login'

    supervisor.handle_key('\n')

    assert actions == [('login', 'agent')]
    assert not supervisor.ui.codex_model_picker_active


def test_agent_arrow_keys_scroll_while_query_is_running():
    supervisor = Supervisor('', [], ui=False, control=False)
    supervisor.ui.codex_active = True
    for index in range(30):
        supervisor.ui.add_codex_message('Codex', f'answer line {index}')
    supervisor.ui._codex_panel_lines(100)
    supervisor.ui.set_codex_running(True, 'Inspecting…')

    supervisor.handle_key('UP')
    assert supervisor.ui.codex_scroll_offset == 1
    supervisor.handle_key('UP')
    assert supervisor.ui.codex_scroll_offset == 2
    supervisor.handle_key('DOWN')
    assert supervisor.ui.codex_scroll_offset == 1


def test_agent_f2_picker_and_model_command_choose_a_model():
    supervisor = Supervisor('', [], ui=False, control=False)
    supervisor.ui.codex_active = True
    supervisor.ui.set_codex_models([
        {
            'model': 'gpt-5.4',
            'display_name': 'GPT-5.4',
            'is_default': True,
        },
        {
            'model': 'gpt-5.3-codex',
            'display_name': 'GPT-5.3-Codex',
            'is_default': False,
        },
    ])

    supervisor.handle_key('F2')
    assert supervisor.ui.codex_model_picker_active
    supervisor.handle_key('\n')
    assert supervisor.ui.codex_model_picker_stage == 'model'
    supervisor.handle_key('DOWN')
    supervisor.handle_key('\n')
    assert supervisor.ui.codex_model_picker_stage == 'reasoning'
    supervisor.handle_key('\n')
    assert supervisor.ui.codex_selected_model == 'gpt-5.4'
    assert supervisor.ui.codex_model_picker_stage == 'root'

    supervisor.handle_key('DOWN')
    supervisor.handle_key('\n')
    assert supervisor.ui.codex_model_picker_stage == 'access'
    supervisor.handle_key('UP')
    supervisor.handle_key('\n')
    assert supervisor.ui.codex_access_mode == 'approve-for-me'
    assert supervisor.ui.codex_model_picker_stage == 'root'
    supervisor.handle_key('ESC')
    assert not supervisor.ui.codex_model_picker_active

    for character in '/model':
        supervisor.handle_key(character)
    supervisor.handle_key('\n')
    assert supervisor.ui.codex_model_picker_active
    supervisor.handle_key('ESC')
    assert supervisor.ui.codex_active
    assert not supervisor.ui.codex_model_picker_active


def test_f2_account_picker_routes_login_action(monkeypatch):
    supervisor = Supervisor('', [], ui=False, control=False)
    supervisor.ui.codex_active = True
    actions = []
    monkeypatch.setattr(
        supervisor,
        '_start_codex_auth',
        lambda action, *, mode: actions.append((action, mode)),
    )

    supervisor.handle_key('F2')
    supervisor.ui.codex_model_picker_selected = next(
        index
        for index, choice in enumerate(
            supervisor.ui._codex_picker_choices())
        if choice.get('action') == 'login'
    )
    supervisor.handle_key('\n')

    assert actions == [('login', 'agent')]
    assert not supervisor.ui.codex_model_picker_active


def test_codex_account_commands_stream_login_and_logout_output(
        monkeypatch, tmp_path):
    class RawStream:
        def __init__(self, lines):
            self.lines = [line.encode() + b'\n' for line in lines]

        async def readline(self):
            return self.lines.pop(0) if self.lines else b''

    class AuthProcess:
        def __init__(self, lines):
            self.stdout = RawStream(lines)
            self.returncode = None

        async def wait(self):
            self.returncode = 0
            return 0

        def terminate(self):
            self.returncode = -15

    calls = []
    outputs = iter((
        ['Opening browser for ChatGPT authentication'],
        ['Logged out'],
    ))

    async def fake_subprocess(*args, **kwargs):
        calls.append((args, kwargs))
        return AuthProcess(next(outputs))

    monkeypatch.setattr(
        'rosmon2.supervisor.shutil.which', lambda _command: '/usr/bin/codex')
    monkeypatch.setattr(
        'rosmon2.supervisor.asyncio.create_subprocess_exec', fake_subprocess)
    supervisor = Supervisor(
        '', [], ui=False, control=False, codex_workspace=str(tmp_path))
    monkeypatch.setattr(supervisor, '_request_codex_usage', lambda: None)

    asyncio.run(supervisor._run_codex_auth('login', mode='agent'))
    asyncio.run(supervisor._run_codex_auth('logout', mode='agent'))

    assert calls[0][0] == ('codex', 'login')
    assert calls[1][0] == ('codex', 'logout')
    assert calls[0][1]['stdin'] is asyncio.subprocess.DEVNULL
    transcript = '\n'.join(
        message for _speaker, message in supervisor.ui.codex_messages)
    assert 'Opening the browser for Codex login' in transcript
    assert 'Opening browser for ChatGPT authentication' in transcript
    assert 'Codex login completed' in transcript
    assert 'Codex logout completed' in transcript
    assert supervisor._codex_logged_in is False
    assert not supervisor.ui.codex_running
    assert supervisor._codex_auth_process is None


def test_codex_login_status_updates_cached_authentication(
        monkeypatch, tmp_path):
    class StatusProcess:
        def __init__(self, returncode):
            self.returncode = returncode

        async def communicate(self):
            return b'', b''

    calls = []
    returncodes = iter((0, 1))

    async def fake_subprocess(*args, **kwargs):
        calls.append((args, kwargs))
        return StatusProcess(next(returncodes))

    monkeypatch.setattr(
        'rosmon2.supervisor.shutil.which', lambda _command: '/usr/bin/codex')
    monkeypatch.setattr(
        'rosmon2.supervisor.asyncio.create_subprocess_exec', fake_subprocess)
    supervisor = Supervisor(
        '', [], ui=False, control=False, codex_workspace=str(tmp_path))

    assert asyncio.run(supervisor._refresh_codex_login_status()) is True
    assert asyncio.run(supervisor._refresh_codex_login_status()) is False

    assert [call[0] for call in calls] == [
        ('codex', 'login', 'status'),
        ('codex', 'login', 'status'),
    ]


def test_logged_out_agent_request_asks_for_login():
    supervisor = Supervisor('', [], ui=False, control=False)
    supervisor.ui.codex_active = True
    supervisor._codex_logged_in = False
    supervisor.ui.codex_prompt = 'inspect the robot driver'

    supervisor.handle_key('\n')

    assert supervisor._codex_task is None
    assert supervisor.ui.codex_prompt == 'inspect the robot driver'
    assert list(supervisor.ui.codex_messages)[-1] == (
        'Codex',
        '- Please log in before talking to the Agent. '
        'Press F2 and select Log in.',
    )


def test_logged_out_diagnosis_does_not_start_lifecycle_agent():
    supervisor = Supervisor('', [], ui=False, control=False)
    supervisor.ui.diagnosis_active = True
    supervisor._codex_logged_in = False

    supervisor._queue_diagnosis_agent('initial diagnosis check')

    assert supervisor._diagnosis_task is None
    assert list(supervisor.ui.diagnosis_messages)[-1] == (
        'Codex',
        '- Please log in before talking to the Agent. '
        'Press F2 and select Log in.',
    )


def test_diagnosis_f2_picker_changes_shared_model():
    supervisor = Supervisor('', [], ui=False, control=False)
    supervisor.ui.diagnosis_active = True
    supervisor.ui.set_codex_models([
        {
            'model': 'gpt-5.5',
            'display_name': 'GPT-5.5',
            'is_default': True,
        },
        {
            'model': 'gpt-5.4',
            'display_name': 'GPT-5.4',
            'is_default': False,
        },
    ])

    supervisor.handle_key('F2')
    assert supervisor.ui.codex_model_picker_active
    supervisor.handle_key('\n')
    assert supervisor.ui.codex_model_picker_stage == 'model'
    supervisor.handle_key('DOWN')
    supervisor.handle_key('\n')
    assert supervisor.ui.codex_model_picker_stage == 'reasoning'
    supervisor.handle_key('\n')

    assert supervisor.ui.codex_selected_model == 'gpt-5.4'
    assert supervisor.ui.diagnosis_active
    assert supervisor.ui.codex_model_picker_stage == 'root'
    supervisor.handle_key('ESC')
    assert not supervisor.ui.codex_model_picker_active


def test_f3_opens_diagnosis_with_initial_health_check(monkeypatch):
    supervisor = Supervisor('', [], ui=False, control=False)
    supervisor.records.extend([
        ProcessRecord(
            key=0, display_name='ur10e/driver',
            state=State.RUNNING, pid=100,
        ),
        ProcessRecord(
            key=1, display_name='external/camera',
            state=State.CRASHED, return_code=2,
        ),
    ])
    supervisor.ui.set_records(supervisor.records)
    supervisor._logs.append({
        'node': 'external/camera',
        'severity': 'ERROR',
        'message': '[ERROR] [camera]: No devices detected',
    })
    checks = []
    usage_refreshes = []
    monkeypatch.setattr(
        supervisor, '_queue_diagnosis_agent', checks.append)
    monkeypatch.setattr(
        supervisor, '_request_codex_usage',
        lambda: usage_refreshes.append(True))

    supervisor.handle_key('F3')

    assert supervisor.ui.diagnosis_active
    assert checks == ['initial diagnosis check']
    assert usage_refreshes == [True]
    assert [row['health'] for row in supervisor.ui.diagnosis_rows] == ['Down']
    assert supervisor.ui.diagnosis_rows[0]['selection_key'] == 'b'
    assert 'No devices detected' in supervisor.ui.diagnosis_rows[0]['detail']

    supervisor.handle_key('F3')
    assert not supervisor.ui.diagnosis_active


def test_f3_and_f4_switch_directly_between_agent_and_diagnosis(monkeypatch):
    supervisor = Supervisor('', [], ui=False, control=False)
    diagnosis_checks = []
    usage_refreshes = []
    monkeypatch.setattr(
        supervisor, '_queue_diagnosis_agent', diagnosis_checks.append)
    monkeypatch.setattr(
        supervisor, '_request_codex_usage',
        lambda: usage_refreshes.append(True))

    supervisor.handle_key('F4')
    assert supervisor.ui.codex_active
    assert not supervisor.ui.diagnosis_active

    supervisor.handle_key('F3')
    assert not supervisor.ui.codex_active
    assert supervisor.ui.diagnosis_active
    assert diagnosis_checks == ['initial diagnosis check']

    supervisor.handle_key('F4')
    assert supervisor.ui.codex_active
    assert not supervisor.ui.diagnosis_active
    assert usage_refreshes == [True, True, True]


def test_global_function_keys_remain_available_in_agent_and_diagnosis(
        monkeypatch):
    for mode in ('agent', 'diagnosis'):
        supervisor = Supervisor('', [], ui=False, control=False)
        first = ProcessRecord(key=0, display_name='robot/driver')
        second = ProcessRecord(key=1, display_name='robot/camera')
        supervisor.records.extend([first, second])
        supervisor.ui.set_records(supervisor.records)
        if mode == 'agent':
            supervisor.ui.codex_active = True
        else:
            supervisor.ui.diagnosis_active = True

        started = []
        stopped = []
        monkeypatch.setattr(supervisor, 'start', started.append)
        monkeypatch.setattr(supervisor, 'stop', stopped.append)

        supervisor.handle_key('F5')
        assert supervisor.ui.namespace_mode
        supervisor.handle_key('F6')
        supervisor.handle_key('F7')
        assert started == [first, second]
        assert stopped == [first, second]

        supervisor.handle_key('F8')
        assert supervisor.ui.warn_only
        supervisor.handle_key('F9')
        assert first.muted and second.muted
        supervisor.handle_key('F10')
        assert not first.muted and not second.muted

        assert supervisor.ui.codex_active == (mode == 'agent')
        assert supervisor.ui.diagnosis_active == (mode == 'diagnosis')


def test_cancel_codex_terminates_query_and_ros_tool_processes():
    class FakeProcess:
        def __init__(self):
            self.returncode = None
            self.terminated = False

        def terminate(self):
            self.terminated = True
            self.returncode = -15

    supervisor = Supervisor('', [], ui=False, control=False)
    query_process = FakeProcess()
    ros_tool_process = FakeProcess()
    supervisor._codex_mode = 'agent'
    supervisor._codex_process = query_process
    supervisor._ros_tool_process = ros_tool_process

    supervisor._cancel_codex()

    assert query_process.terminated
    assert ros_tool_process.terminated
    assert supervisor.ui.codex_execution_label == 'Stopping current query'


def test_diagnosis_mode_accepts_typed_questions(monkeypatch):
    supervisor = Supervisor('', [], ui=False, control=False)
    supervisor.ui.diagnosis_active = True
    requests = []

    async def fake_run(question, *, mode='agent'):
        requests.append((question, mode))

    monkeypatch.setattr(supervisor, '_run_codex', fake_run)

    async def ask():
        for character in 'what is wrong with b?':
            supervisor.handle_key(character)
        assert supervisor.ui.diagnosis_prompt == 'what is wrong with b?'
        supervisor.handle_key('\n')
        await supervisor._codex_task

    asyncio.run(ask())

    assert requests == [('what is wrong with b?', 'diagnosis')]
    assert list(supervisor.ui.diagnosis_messages)[-1] == (
        'You', 'what is wrong with b?')
    assert not supervisor.ui.codex_messages


def test_diagnosis_tab_switches_arrows_between_nodes_and_agent_text():
    supervisor = Supervisor('', [], ui=False, control=False)
    supervisor.ui.diagnosis_active = True
    supervisor.ui.set_diagnosis_rows([
        {
            'record_key': 0, 'selection_key': 'a',
            'name': 'robot/driver', 'namespace': 'robot',
            'state': 'crashed', 'health': 'Down', 'errors': 1,
            'detail': 'Exited',
        },
        {
            'record_key': 1, 'selection_key': 'b',
            'name': 'robot/camera', 'namespace': 'robot',
            'state': 'waiting', 'health': 'Waiting', 'errors': 0,
            'detail': 'Waiting for device',
        },
    ])
    for index in range(20):
        supervisor.ui.add_diagnosis_message(
            'Codex', f'diagnosis line {index}')
    supervisor.ui._diagnosis_panel_lines(100)

    supervisor.handle_key('DOWN')
    assert supervisor.ui.diagnosis_selected == 1
    assert supervisor.ui.diagnosis_chat_scroll_offset == 0

    supervisor.handle_key('\t')
    assert supervisor.ui.diagnosis_chat_focused
    supervisor.handle_key('UP')
    assert supervisor.ui.diagnosis_selected == 1
    assert supervisor.ui.diagnosis_chat_scroll_offset == 1
    supervisor.handle_key('DOWN')
    assert supervisor.ui.diagnosis_chat_scroll_offset == 0

    supervisor.handle_key('\t')
    assert not supervisor.ui.diagnosis_chat_focused
    supervisor.handle_key('UP')
    assert supervisor.ui.diagnosis_selected == 0


def test_diagnosis_agent_runs_only_when_health_class_changes(monkeypatch):
    supervisor = Supervisor('', [], ui=False, control=False)
    driver = ProcessRecord(
        key=0, display_name='ur10e/driver',
        state=State.RUNNING, pid=100,
    )
    supervisor.records.append(driver)
    supervisor.ui.set_records(supervisor.records)
    supervisor.ui.diagnosis_active = True
    supervisor._diagnosis_health[driver.key] = (
        State.RUNNING.value, 'Healthy')
    checks = []
    monkeypatch.setattr(
        supervisor, '_queue_diagnosis_agent', checks.append)

    for _index in range(4):
        supervisor._record_output(
            driver.display_name, '[ERROR] connection failed', False)
        supervisor._diagnosis_record_changed(driver, 'log health changed')

    assert checks == []
    assert supervisor.ui.diagnosis_rows == []

    supervisor._record_output(
        driver.display_name, '[ERROR] connection failed', False)
    supervisor._diagnosis_record_changed(driver, 'log health changed')

    assert len(checks) == 1
    assert 'running/Healthy to running/Erroring' in checks[0]
    assert supervisor.ui.diagnosis_rows[0]['health'] == 'Erroring'

    supervisor._record_output(
        driver.display_name, '[ERROR] connection failed', False)
    supervisor._diagnosis_record_changed(driver, 'log health changed')
    assert len(checks) == 1


def test_diagnosis_detects_repeated_stall_signals(monkeypatch):
    supervisor = Supervisor('', [], ui=False, control=False)
    driver = ProcessRecord(
        key=0, display_name='ur10e/driver',
        state=State.RUNNING, pid=100,
    )
    supervisor.records.append(driver)
    supervisor.ui.diagnosis_active = True
    supervisor._diagnosis_health[driver.key] = (
        State.RUNNING.value, 'Healthy')
    checks = []
    monkeypatch.setattr(
        supervisor, '_queue_diagnosis_agent', checks.append)

    for _index in range(3):
        supervisor._record_output(
            driver.display_name,
            '[INFO] waiting for service /controller_manager',
            False,
        )
        supervisor._diagnosis_record_changed(driver, 'log health changed')

    assert supervisor.ui.diagnosis_rows[0]['health'] == 'Stalled'
    assert len(checks) == 1


def test_diagnosis_can_restart_selected_node_or_namespace(monkeypatch):
    supervisor = Supervisor('', [], ui=False, control=False)
    setup = ProcessRecord(
        key=0, display_name='hardware_setup',
        state=State.RUNNING, pid=100,
    )
    driver = ProcessRecord(
        key=1, display_name='ur10e/driver',
        state=State.CRASHED, return_code=2,
    )
    controller = ProcessRecord(
        key=2, display_name='ur10e/controller',
        state=State.RUNNING, pid=101,
    )
    supervisor.records.extend([setup, driver, controller])
    supervisor.ui.diagnosis_active = True
    supervisor.ui.set_diagnosis_rows(supervisor._diagnosis_rows())
    assert [row['selection_key']
            for row in supervisor.ui.diagnosis_rows] == ['b']
    supervisor.ui.diagnosis_selected = 0
    # Keep the unhealthy row while simulating a process that is still alive
    # long enough for the stop controls to target it.
    driver.pid = 102
    restarted = []
    stopped = []
    monkeypatch.setattr(supervisor, 'restart', restarted.append)
    monkeypatch.setattr(supervisor, 'stop', stopped.append)

    supervisor.handle_key('R')
    assert restarted == [driver]

    supervisor.handle_key('K')
    assert stopped == [driver]

    restarted.clear()
    supervisor.handle_key('N')
    assert restarted == [driver, controller]

    stopped.clear()
    supervisor.handle_key('X')
    assert stopped == [driver, controller]


def test_stop_selected_node_uses_graceful_shutdown_event():
    class FakeContext:
        def __init__(self):
            self.events = []

        def emit_event_sync(self, event):
            self.events.append(event)

    supervisor = Supervisor('', [], ui=False, control=False)
    action = object()
    driver = ProcessRecord(
        key=0,
        display_name='robot/driver',
        state=State.RUNNING,
        action=action,
        pid=123,
    )
    supervisor._context = FakeContext()

    supervisor.stop(driver)

    assert driver.manually_stopped
    assert len(supervisor._context.events) == 1
    event = supervisor._context.events[0]
    assert event.process_matcher(action)


def test_diagnosis_agent_receives_initial_snapshot(monkeypatch, tmp_path):
    class FakeProcess:
        returncode = 0

        async def communicate(self):
            return (b'- /ur10e/driver is healthy.', b'')

    commands = []

    async def fake_subprocess(*args, **kwargs):
        commands.append((args, kwargs))
        return FakeProcess()

    monkeypatch.setattr(
        'rosmon2.supervisor.shutil.which',
        lambda _command: '/usr/bin/codex',
    )
    monkeypatch.setattr(
        'rosmon2.supervisor.asyncio.create_subprocess_exec',
        fake_subprocess,
    )
    supervisor = Supervisor(
        '', [], ui=False, control=False, codex_workspace=str(tmp_path))
    supervisor.records.append(ProcessRecord(
        key=0, display_name='ur10e/driver',
        state=State.RUNNING, pid=100,
    ))
    supervisor.ui.codex_reasoning_efforts['gpt-5.5'] = 'high'
    monkeypatch.setattr(supervisor, '_request_codex_usage', lambda: None)

    async def open_and_wait():
        supervisor.handle_key('F3')
        await supervisor._diagnosis_task

    asyncio.run(open_and_wait())

    assert commands
    assert '--model' in commands[0][0]
    assert commands[0][0][commands[0][0].index('--model') + 1] == 'gpt-5.5'
    assert '--config' in commands[0][0]
    assert commands[0][0][commands[0][0].index('--config') + 1] == (
        'model_reasoning_effort="high"')
    assert 'initial diagnosis check' in commands[0][0][-1]
    assert '/ur10e/driver: state=running, health=Healthy' in commands[0][0][-1]
    assert supervisor.ui.diagnosis_summary == [
        '- /ur10e/driver is healthy.',
    ]


def test_codex_greeting_returns_guidance_without_starting_a_cli_task():
    supervisor = Supervisor('', [], ui=False, control=False)
    supervisor.handle_key('F4')
    for character in 'hi':
        supervisor.handle_key(character)

    supervisor.handle_key('\n')

    assert supervisor._codex_task is None
    assert list(supervisor.ui.codex_messages)[-1][0] == 'Codex'
    assert any(
        'What would you like me to help with?' in message
        for _speaker, message in supervisor.ui.codex_messages
    )
    assert not any(
        'press F4' in message
        for _speaker, message in supervisor.ui.codex_messages
    )
    assert all(
        not message.startswith('- ')
        for speaker, message in supervisor.ui.codex_messages
        if speaker == 'Codex'
    )
    assert not supervisor.ui.codex_running


def test_codex_question_resolves_displayed_node_key_without_preselection():
    supervisor = Supervisor('', [], ui=False, control=False)
    setup = ProcessRecord(key=0, display_name='hardware_setup')
    receiver = ProcessRecord(
        key=1, display_name='ur10e/robot_state_receiver',
        state=State.CRASHED, return_code=-6,
    )
    command_server = ProcessRecord(
        key=2, display_name='ur10e/command_server')
    supervisor.records.extend([setup, receiver, command_server])
    supervisor.ui.set_records(supervisor.records)
    supervisor._logs.extend([
        {
            'node': 'hardware_setup', 'severity': 'INFO',
            'message': 'setup loaded',
        },
        {
            'node': 'ur10e/robot_state_receiver', 'severity': 'ERROR',
            'message': 'RTDE connection aborted',
        },
    ])

    context = supervisor._codex_context("whats wrong with 'b'")

    assert supervisor._codex_focus_record(
        "whats wrong with 'b'") is receiver
    assert 'Selected node: /ur10e/robot_state_receiver' in context
    assert 'RTDE connection aborted' in context
    assert 'setup loaded' not in context
    assert supervisor._codex_focus_record(
        'what is wrong with `b`?') is receiver


def test_codex_explicit_node_key_takes_precedence_over_ui_selection():
    supervisor = Supervisor('', [], ui=False, control=False)
    first = ProcessRecord(key=0, display_name='first')
    second = ProcessRecord(key=1, display_name='second')
    supervisor.records.extend([first, second])
    supervisor.ui.set_records(supervisor.records)
    supervisor.ui.selected = 0

    assert supervisor._codex_focus_record('diagnose node b') is second
    assert supervisor._codex_focus_record('b') is second


def test_codex_context_uses_selected_node_and_only_its_recent_logs(tmp_path):
    supervisor = Supervisor(
        'robot.launch.py', [], ui=False, control=False,
        codex_workspace=str(tmp_path),
    )
    camera = ProcessRecord(
        key=0, display_name='camera', state=State.RUNNING, pid=100)
    driver = ProcessRecord(
        key=1, display_name='ur10e/driver', state=State.CRASHED,
        return_code=2,
    )
    supervisor.records.extend([camera, driver])
    supervisor.ui.set_records(supervisor.records)
    supervisor.ui.selected = 1
    supervisor._logs.extend([
        {
            'node': 'camera', 'severity': 'WARNING',
            'message': 'frame delayed',
        },
        {
            'node': 'ur10e/driver', 'severity': 'ERROR',
            'message': 'cannot open /dev/robot',
        },
    ])

    context = supervisor._codex_context('what is wrong?')

    assert 'Selected node: /ur10e/driver' in context
    assert '[ERROR] /ur10e/driver: cannot open /dev/robot' in context
    assert 'frame delayed' not in context
    assert 'general-purpose Rosmon Agent' in context
    assert 'Do not force responses into fault-diagnosis headings' in context
    assert '## What might be wrong' not in context
    assert 'Do not edit files' in context


def test_diagnosis_context_uses_selected_unhealthy_table_row():
    supervisor = Supervisor('', [], ui=False, control=False)
    camera = ProcessRecord(
        key=0, display_name='camera', state=State.CRASHED, return_code=1)
    driver = ProcessRecord(
        key=1, display_name='ur10e/driver',
        state=State.CRASHED, return_code=2)
    supervisor.records.extend([camera, driver])
    supervisor.ui.set_records(supervisor.records)
    supervisor.ui.diagnosis_active = True
    supervisor.ui.set_diagnosis_rows(supervisor._diagnosis_rows())
    supervisor.ui.diagnosis_selected = 1

    context = supervisor._codex_context(
        'what is wrong with this node?', mode='diagnosis')

    assert 'interactive Diagnosis assistant' in context
    assert 'Selected node: /ur10e/driver' in context
    assert '## What might be wrong' in context


def test_agent_and_diagnosis_context_keep_separate_histories():
    supervisor = Supervisor('', [], ui=False, control=False)
    supervisor._codex_history.append(('Codex', 'general answer'))
    supervisor._diagnosis_chat_history.append(
        ('Codex', 'diagnostic answer'))

    agent = supervisor._codex_context('continue', mode='agent')
    diagnosis = supervisor._codex_context('continue', mode='diagnosis')

    assert 'general answer' in agent
    assert 'diagnostic answer' not in agent
    assert 'diagnostic answer' in diagnosis
    assert 'general answer' not in diagnosis


def test_codex_edit_permission_requires_an_explicit_fix_request(tmp_path):
    supervisor = Supervisor('', [], ui=False, control=False,
                            codex_workspace=str(tmp_path))

    diagnostic = supervisor._codex_context('why did it crash?')
    fix = supervisor._codex_context('please fix the driver')

    assert 'Do not edit files' in diagnostic
    assert 'You may edit only files inside the workspace' in fix


def test_codex_context_gates_live_ros_actions_and_motion_details(tmp_path):
    supervisor = Supervisor('', [], ui=False, control=False,
                            codex_workspace=str(tmp_path))

    inspect = supervisor._codex_context(
        'what actions are available?', mode='agent')
    motion = supervisor._codex_context(
        'move the robot arm up 5 mm', mode='agent')
    diagnosis = supervisor._codex_context(
        'move the robot arm up 5 mm', mode='diagnosis')

    assert 'You may inspect the ROS graph' in inspect
    assert 'ros2_interface' in motion
    assert 'configured defaults for omitted optional' in motion
    assert 'Never guess a missing target' in motion
    assert 'Shell commands are available in Agent mode' in motion
    assert 'temporary intermediate files under /tmp' in motion
    assert 'Speed and acceleration are optional tuning parameters' in motion
    assert 'do not ask for them' in motion
    assert 'existing configured defaults' in motion
    assert 'Explicit Human speed or acceleration values override' in motion
    assert 'Never bypass safety limits or interlocks' in motion
    assert 'immediately pending clarification' in motion
    assert 'use ros2_interface directly' in motion
    assert 'Do not create a Python motion script' in motion
    assert 'external rosmon2 tool as an alternate transport' in motion
    assert 'Diagnosis is read-only for ROS hardware' in diagnosis


def test_codex_execution_item_labels_are_short_and_user_facing():
    assert Supervisor._execution_label_from_item({
        'type': 'commandExecution',
        'command': 'pytest -q test/test_supervisor.py',
    }) == 'Running pytest -q test/test_supervisor.py'
    assert Supervisor._execution_label_from_item({
        'type': 'commandExecution',
        'command': 'sed -n 1,80p rosmon2/terminal.py',
        'commandActions': [{
            'type': 'read',
            'command': 'sed -n 1,80p rosmon2/terminal.py',
            'name': 'terminal.py',
            'path': '/workspace/rosmon2/terminal.py',
        }],
    }) == 'Reading terminal.py'
    assert Supervisor._execution_label_from_item({
        'type': 'commandExecution',
        'command': 'rg -n spinner rosmon2',
        'commandActions': [{
            'type': 'search',
            'command': 'rg -n spinner rosmon2',
            'query': 'spinner',
            'path': 'rosmon2',
        }],
    }) == 'Searching for spinner in rosmon2'
    assert Supervisor._execution_label_from_item({
        'type': 'fileChange',
        'changes': [{
            'path': 'rosmon2/terminal.py',
            'kind': 'update',
            'diff': '...',
        }],
    }) == 'Editing rosmon2/terminal.py'
    assert Supervisor._execution_label_from_item({
        'type': 'mcpToolCall',
        'server': 'ros',
        'tool': 'list_nodes',
    }) == 'Using ros/list_nodes'
    assert Supervisor._execution_label_from_item({
        'type': 'dynamicToolCall',
        'tool': 'ros2_interface',
    }) == 'Executing ROS operation'
    assert Supervisor._execution_label_from_item({
        'type': 'webSearch',
        'query': 'ROS 2 controller timeout',
    }) == 'Searching the web for ROS 2 controller timeout'
    assert Supervisor._execution_label_from_item({
        'type': 'reasoning',
    }) == 'Thinking'
    assert Supervisor._execution_label_from_item({
        'type': 'agentMessage',
        'text': 'answer',
    }) is None


def test_codex_python_node_tool_is_exposed_only_for_direct_start_request():
    assert Supervisor._codex_python_node_tools(
        'please write and start a Python ROS node')
    assert Supervisor._codex_python_node_tools(
        'run scripts/health_probe.py as a node')
    assert Supervisor._codex_python_node_tools(
        'write a node that prints hello world and run it')
    assert Supervisor._codex_python_node_tools(
        'write a node that prints hello world and runit')
    assert not Supervisor._codex_python_node_tools(
        'could a Python script start a ROS node?')
    assert not Supervisor._codex_python_node_tools(
        'please write a Python ROS node but do not run it')
    assert not Supervisor._codex_python_node_tools(
        'please write a node but do not runit')
    assert not Supervisor._codex_python_node_tools(
        'please write a hello-world node')


def test_codex_python_node_write_uses_dedicated_workspace(
        monkeypatch, tmp_path):
    node_workspace = tmp_path / 'home' / 'rosmon2'
    processes = []
    process_cwds = []

    async def fake_subprocess(*_args, **kwargs):
        process_cwds.append(kwargs['cwd'])
        process = _FakeCodexProcess('- Wrote hello_node.py.')
        processes.append(process)
        return process

    monkeypatch.setattr(
        'rosmon2.supervisor.shutil.which', lambda _command: '/usr/bin/codex')
    monkeypatch.setattr(
        'rosmon2.supervisor.asyncio.create_subprocess_exec', fake_subprocess)
    supervisor = Supervisor(
        '', [], ui=False, control=False, codex_workspace=str(tmp_path))
    supervisor.agent_node_workspace = node_workspace

    asyncio.run(supervisor._run_codex(
        'write a Python ROS node that prints hello', mode='agent'))

    assert node_workspace.is_dir()
    assert process_cwds == [str(node_workspace)]
    thread_start = next(
        message for message in processes[0].stdin.messages
        if message.get('method') == 'thread/start')
    assert thread_start['params']['cwd'] == str(node_workspace)
    assert all(
        tool['name'] != 'rosmon_python_node'
        for tool in thread_start['params']['dynamicTools']
    )
    turn_start = next(
        message for message in processes[0].stdin.messages
        if message.get('method') == 'turn/start')
    context = turn_start['params']['input'][0]['text']
    assert f'Active working directory: {node_workspace}' in context
    assert (
        f'do not write any part of it under the launch workspace {tmp_path}'
        in context
    )


def test_codex_python_node_write_detection_accepts_natural_phrasing():
    supervisor = Supervisor('', [], ui=False, control=False)

    for question in (
            'make a node that publishes a heartbeat',
            'build a Python ROS 2 node',
            'scaffold hello_world_node.py',
            'add two monitoring nodes'):
        context = supervisor._codex_context(question, mode='agent')
        assert (
            f'Active working directory: {supervisor.agent_node_workspace}'
            in context
        )


def test_codex_usage_uses_most_constrained_remaining_percentage():
    result = {
        'rateLimitsByLimitId': {
            'codex': {
                'primary': {'usedPercent': 19},
                'secondary': {'usedPercent': 42},
                'individualLimit': {'remainingPercent': 73},
            },
        },
    }

    assert Supervisor._codex_remaining_percent(result) == 58
    assert Supervisor._codex_remaining_percent({
        'rateLimits': {'primary': {'usedPercent': 100}},
    }) == 0
    assert Supervisor._codex_remaining_percent({}) is None


def test_codex_usage_is_read_from_app_server(monkeypatch, tmp_path):
    class FakeStdin:
        def __init__(self):
            self.writes = []
            self.closed = False

        def write(self, value):
            self.writes.append(value)

        async def drain(self):
            pass

        def close(self):
            self.closed = True

    class FakeStdout:
        def __init__(self):
            self.lines = iter([
                b'{"id":1,"result":{"userAgent":"codex"}}\n',
                b'{"method":"account/rateLimits/updated","params":{}}\n',
                (
                    b'{"id":2,"result":{"rateLimits":{"primary":'
                    b'{"usedPercent":27}}}}\n'
                ),
                (
                    b'{"id":3,"result":{"data":['
                        b'{"id":"gpt-5.4","model":"gpt-5.4",'
                        b'"displayName":"GPT-5.4","hidden":false,'
                        b'"isDefault":true,'
                        b'"supportedReasoningEfforts":['
                        b'{"reasoningEffort":"low"},'
                        b'{"reasoningEffort":"medium"},'
                        b'{"reasoningEffort":"high"}],'
                        b'"defaultReasoningEffort":"medium"},'
                    b'{"id":"hidden","model":"hidden",'
                    b'"displayName":"Hidden","hidden":true,'
                    b'"isDefault":false}]}}\n'
                ),
            ])

        async def readline(self):
            return next(self.lines, b'')

    class FakeProcess:
        def __init__(self):
            self.returncode = None
            self.stdin = FakeStdin()
            self.stdout = FakeStdout()
            self.stderr = None

        async def wait(self):
            self.returncode = 0
            return 0

        def terminate(self):
            self.returncode = -15

    process = FakeProcess()
    calls = []

    async def fake_subprocess(*args, **kwargs):
        calls.append((args, kwargs))
        return process

    monkeypatch.setattr(
        'rosmon2.supervisor.shutil.which',
        lambda _command: '/usr/bin/codex',
    )
    monkeypatch.setattr(
        'rosmon2.supervisor.asyncio.create_subprocess_exec',
        fake_subprocess,
    )
    supervisor = Supervisor(
        '', [], ui=False, control=False, codex_workspace=str(tmp_path))

    asyncio.run(supervisor._fetch_codex_usage())

    assert calls[0][0] == ('codex', 'app-server', '--stdio')
    written = b''.join(process.stdin.writes)
    assert b'"method": "initialize"' in written
    assert b'"method": "account/rateLimits/read"' in written
    assert b'"method": "model/list"' in written
    assert b'"includeHidden": false' in written
    assert supervisor.ui.codex_usage_remaining == 73
    assert not supervisor.ui.codex_usage_loading
    assert supervisor.ui.codex_models == [{
        'model': 'gpt-5.4',
        'display_name': 'GPT-5.4',
        'is_default': True,
        'supported_reasoning_efforts': ('low', 'medium', 'high'),
        'default_reasoning_effort': 'medium',
    }]
    assert not supervisor.ui.codex_models_loading
    assert process.stdin.closed


def test_stopped_node_prompt_requests_plain_recovery_sections_and_software_choice(
        tmp_path):
    supervisor = Supervisor('', [], ui=False, control=False,
                            codex_workspace=str(tmp_path))
    supervisor.records.append(ProcessRecord(
        key=0, display_name='driver', state=State.CRASHED, return_code=1))

    context = supervisor._codex_context(
        'what is wrong with a?', mode='diagnosis')

    assert '## What might be wrong' in context
    assert '### Hardware' in context
    assert '### Software' in context
    assert 'Never output a “What to try next” heading' in context
    assert 'which category is better supported' in context
    assert 'Every reason must begin with “- ”' in context
    assert 'For greetings, thanks, and casual conversation' in context
    assert '- Would you like me to try to fix this software issue? [y/n]' in context
    assert 'Do not offer this for hardware-only, mixed, or uncertain' in context


def test_codex_no_choice_declines_changes():
    supervisor = Supervisor('', [], ui=False, control=False)
    supervisor.ui.diagnosis_active = True
    supervisor._codex_yes_no_pending = True
    supervisor._codex_yes_no_mode = 'diagnosis'
    supervisor._codex_pending_fix_question = 'what is wrong with b?'

    supervisor.handle_key('n')

    assert not supervisor._codex_yes_no_pending
    assert supervisor._codex_task is None
    assert list(supervisor.ui.diagnosis_messages)[-1] == (
        'Rosmon', 'Okay. I will not make any changes.')


def test_codex_yes_choice_starts_explicit_software_fix(monkeypatch):
    supervisor = Supervisor('', [], ui=False, control=False)
    supervisor.ui.diagnosis_active = True
    supervisor._codex_yes_no_pending = True
    supervisor._codex_yes_no_mode = 'diagnosis'
    supervisor._codex_pending_fix_question = 'what is wrong with b?'
    questions = []

    async def fake_run(question, *, mode='agent'):
        questions.append((question, mode))

    monkeypatch.setattr(supervisor, '_run_codex', fake_run)

    async def choose_yes():
        supervisor.handle_key('y')
        await supervisor._codex_task

    asyncio.run(choose_yes())

    assert questions
    assert questions[0][0].startswith('Fix the software issue')
    assert 'Original question: what is wrong with b?' in questions[0][0]
    assert questions[0][1] == 'diagnosis'
    assert list(supervisor.ui.diagnosis_messages)[-1] == ('You', 'y')


def test_software_fix_marker_enables_yes_no_selection(monkeypatch, tmp_path):
    answer = (
        '## What might be wrong\n'
        '### Hardware\n- No hardware cause is currently indicated.\n'
        '### Software\n- The configuration is invalid.\n'
        '- Would you like me to try to fix this software issue? [y/n]'
    )

    async def fake_subprocess(*_args, **_kwargs):
        return _FakeCodexProcess(answer, [answer[:40], answer[40:]])

    monkeypatch.setattr('rosmon2.supervisor.shutil.which',
                        lambda _command: '/usr/bin/codex')
    monkeypatch.setattr('rosmon2.supervisor.asyncio.create_subprocess_exec',
                        fake_subprocess)
    supervisor = Supervisor('', [], ui=False, control=False,
                            codex_workspace=str(tmp_path))

    asyncio.run(supervisor._run_codex(
        'what is wrong with b?', mode='diagnosis'))

    assert supervisor._codex_yes_no_pending
    assert supervisor._codex_yes_no_mode == 'diagnosis'
    assert supervisor._codex_pending_fix_question == 'what is wrong with b?'
    assert list(supervisor.ui.diagnosis_messages)[-1][0] == 'Codex'
    assert not supervisor.ui.codex_messages


def test_codex_exec_avoids_bubblewrap_for_agent_shell_and_tmp_access(
        monkeypatch, tmp_path):
    calls = []
    processes = []

    async def fake_subprocess(*args, **kwargs):
        calls.append((args, kwargs))
        process = _FakeCodexProcess(
            'Software likely. Run the driver unit tests.',
            ['Software likely. ', 'Run the driver unit tests.'],
        )
        processes.append(process)
        return process

    monkeypatch.setattr('rosmon2.supervisor.shutil.which',
                        lambda _command: '/usr/bin/codex')
    monkeypatch.setattr('rosmon2.supervisor.asyncio.create_subprocess_exec',
                        fake_subprocess)
    supervisor = Supervisor(
        'robot.launch.py', [], ui=False, control=False,
        codex_workspace=str(tmp_path),
    )
    streamed_chunks = []
    original_append = supervisor.ui.append_codex_stream

    def capture_chunk(chunk):
        streamed_chunks.append(chunk)
        original_append(chunk)

    monkeypatch.setattr(supervisor.ui, 'append_codex_stream', capture_chunk)

    asyncio.run(supervisor._run_codex('what is wrong?'))
    asyncio.run(supervisor._run_codex('fix the software issue'))

    first, second = (call[0] for call in calls)
    assert first == ('codex', 'app-server', '--stdio')
    assert second == ('codex', 'app-server', '--stdio')
    first_thread = next(
        message for message in processes[0].stdin.messages
        if message.get('method') == 'thread/start')
    second_thread = next(
        message for message in processes[1].stdin.messages
        if message.get('method') == 'thread/start')
    assert first_thread['params']['sandbox'] == 'danger-full-access'
    assert second_thread['params']['sandbox'] == 'danger-full-access'
    assert first_thread['params']['approvalPolicy'] == 'never'
    assert first_thread['params']['ephemeral'] is True
    assert first_thread['params']['model'] == 'gpt-5.5'
    assert second_thread['params']['model'] == 'gpt-5.5'
    first_turn = next(
        message for message in processes[0].stdin.messages
        if message.get('method') == 'turn/start')
    second_turn = next(
        message for message in processes[1].stdin.messages
        if message.get('method') == 'turn/start')
    expected_policy = {'type': 'dangerFullAccess'}
    assert first_turn['params']['sandboxPolicy'] == expected_policy
    assert second_turn['params']['sandboxPolicy'] == expected_policy
    assert first_turn['params']['effort'] == 'medium'
    assert second_turn['params']['effort'] == 'medium'
    assert first_turn['params']['summary'] == 'detailed'
    assert second_turn['params']['summary'] == 'detailed'
    assert streamed_chunks == [
        'Software likely. ', 'Run the driver unit tests.',
        'Software likely. ', 'Run the driver unit tests.',
    ]
    assert calls[0][1]['cwd'] == str(tmp_path)
    assert calls[0][1]['limit'] == 16 * 1024 * 1024
    assert supervisor._codex_process is None
    assert not supervisor.ui.codex_running


def test_codex_streams_detailed_reasoning_summary_as_activity(
        monkeypatch, tmp_path):
    process = _FakeCodexProcess(
        'The controller is ready.',
        reasoning_chunks=[
            'Inspecting the live ROS graph ',
            'and checking the target-pose controller.',
        ],
    )
    activities = []

    async def fake_subprocess(*_args, **_kwargs):
        return process

    monkeypatch.setattr(
        'rosmon2.supervisor.shutil.which', lambda _command: '/usr/bin/codex')
    monkeypatch.setattr(
        'rosmon2.supervisor.asyncio.create_subprocess_exec', fake_subprocess)
    supervisor = Supervisor(
        '', [], ui=False, control=False, codex_workspace=str(tmp_path))
    monkeypatch.setattr(
        supervisor.ui,
        'set_agent_execution',
        lambda mode, label: activities.append((mode, label)),
    )

    asyncio.run(supervisor._run_codex(
        'inspect the controller before moving', mode='agent'))

    assert (
        'agent',
        'Thinking: Inspecting the live ROS graph and checking the '
        'target-pose controller',
    ) in activities
    assert activities[-1] == ('agent', None)


def test_codex_oversized_stream_event_is_reported_without_escaping_task(
        monkeypatch, tmp_path):
    class OversizedEventStream(_FakeCodexReadStream):
        async def readline(self):
            if self._lines:
                return self._lines.pop(0)
            raise ValueError(
                'Separator is not found, and chunk exceed the limit')

    process = _FakeCodexProcess('unused')
    process.stdout = OversizedEventStream([
        {'id': 1, 'result': {}},
        {'id': 2, 'result': {'thread': {'id': 'thread-1'}}},
        {'id': 3, 'result': {
            'turn': {'id': 'turn-1', 'status': 'inProgress', 'items': []},
        }},
    ])

    async def fake_subprocess(*_args, **_kwargs):
        return process

    monkeypatch.setattr(
        'rosmon2.supervisor.shutil.which', lambda _command: '/usr/bin/codex')
    monkeypatch.setattr(
        'rosmon2.supervisor.asyncio.create_subprocess_exec', fake_subprocess)
    supervisor = Supervisor(
        '', [], ui=False, control=False, codex_workspace=str(tmp_path))

    asyncio.run(supervisor._run_codex('inspect the workspace'))

    assert any(
        'event larger than 16 MiB' in message
        for _speaker, message in supervisor.ui.codex_messages
    )
    assert not supervisor.ui.codex_running
    assert supervisor._codex_task is None


def test_diagnosis_keeps_read_only_filesystem_with_loopback_access(
        monkeypatch, tmp_path):
    processes = []

    async def fake_subprocess(*_args, **_kwargs):
        process = _FakeCodexProcess('- Read-only inspection complete.')
        processes.append(process)
        return process

    monkeypatch.setattr(
        'rosmon2.supervisor.shutil.which', lambda _command: '/usr/bin/codex')
    monkeypatch.setattr(
        'rosmon2.supervisor.asyncio.create_subprocess_exec', fake_subprocess)
    supervisor = Supervisor(
        '', [], ui=False, control=False, codex_workspace=str(tmp_path))

    asyncio.run(supervisor._run_codex(
        'inspect the controller logs', mode='diagnosis'))

    thread_start = next(
        message for message in processes[0].stdin.messages
        if message.get('method') == 'thread/start')
    turn_start = next(
        message for message in processes[0].stdin.messages
        if message.get('method') == 'turn/start')
    assert thread_start['params']['sandbox'] == 'read-only'
    assert turn_start['params']['effort'] == 'medium'
    assert turn_start['params']['sandboxPolicy'] == {
        'type': 'readOnly',
        'networkAccess': True,
    }


def test_selected_model_applies_to_agent_and_diagnosis_turns(
        monkeypatch, tmp_path):
    processes = []

    async def fake_subprocess(*_args, **_kwargs):
        process = _FakeCodexProcess('Done.')
        processes.append(process)
        return process

    monkeypatch.setattr(
        'rosmon2.supervisor.shutil.which',
        lambda _command: '/usr/bin/codex',
    )
    monkeypatch.setattr(
        'rosmon2.supervisor.asyncio.create_subprocess_exec',
        fake_subprocess,
    )
    supervisor = Supervisor(
        '', [], ui=False, control=False, codex_workspace=str(tmp_path))
    supervisor.ui.set_codex_models([{
        'model': 'gpt-5.3-codex',
        'display_name': 'GPT-5.3-Codex',
        'is_default': False,
    }])
    supervisor.ui.codex_selected_model = 'gpt-5.3-codex'
    supervisor.ui.codex_reasoning_efforts['gpt-5.3-codex'] = 'high'
    supervisor.ui.codex_access_mode = 'approve-for-me'

    asyncio.run(supervisor._run_codex('inspect the workspace', mode='agent'))
    asyncio.run(supervisor._run_codex('what is wrong?', mode='diagnosis'))

    agent_thread, diagnosis_thread = [
        next(
            message for message in process.stdin.messages
            if message.get('method') == 'thread/start'
        )
        for process in processes
    ]
    assert agent_thread['params']['model'] == 'gpt-5.3-codex'
    assert diagnosis_thread['params']['model'] == 'gpt-5.3-codex'
    assert agent_thread['params']['approvalPolicy'] == 'on-request'
    assert agent_thread['params']['approvalsReviewer'] == 'auto_review'
    assert diagnosis_thread['params']['approvalPolicy'] == 'never'
    assert 'approvalsReviewer' not in diagnosis_thread['params']
    agent_turn, diagnosis_turn = [
        next(
            message for message in process.stdin.messages
            if message.get('method') == 'turn/start'
        )
        for process in processes
    ]
    assert agent_turn['params']['effort'] == 'high'
    assert diagnosis_turn['params']['effort'] == 'high'


def test_codex_control_tool_is_only_exposed_for_direct_node_actions():
    assert Supervisor._codex_control_tools('what is wrong with b?') == []
    assert Supervisor._codex_control_tools(
        'what would happen if I restarted b?') == []

    tools = Supervisor._codex_control_tools('please restart b')

    assert len(tools) == 1
    assert tools[0]['name'] == 'rosmon_control'
    assert tools[0]['inputSchema']['properties']['action']['enum'] == [
        'start', 'stop', 'restart', 'mute', 'unmute', 'debug',
    ]


def test_codex_ros_tool_exposes_mutations_only_for_direct_requests():
    inspect_tool = Supervisor._codex_ros_tools(
        'what services and actions are available?')[0]
    inspect_operations = inspect_tool[
        'inputSchema']['properties']['operation']['enum']

    assert 'list_services' in inspect_operations
    assert 'list_actions' in inspect_operations
    assert 'list_parameters' in inspect_operations
    assert 'get_parameter' in inspect_operations
    assert 'call_service' not in inspect_operations
    assert 'send_action_goal' not in inspect_operations
    assert 'call_service' not in Supervisor._codex_ros_tools(
        'run the relevant tests')[0][
            'inputSchema']['properties']['operation']['enum']

    action_tool = Supervisor._codex_ros_tools(
        'move the robot arm up 5 mm')[0]
    action_operations = action_tool[
        'inputSchema']['properties']['operation']['enum']

    assert 'call_service' in action_operations
    assert 'send_action_goal' in action_operations
    assert 'send_action_goal' in Supervisor._codex_ros_tools(
        'move 10 mm in x')[0][
            'inputSchema']['properties']['operation']['enum']
    assert 'send_action_goal' in Supervisor._codex_ros_tools(
        'base', mutation_allowed=True)[0][
            'inputSchema']['properties']['operation']['enum']


def test_codex_ros_motion_authorization_survives_short_clarification_only(
        monkeypatch, tmp_path):
    processes = []

    async def fake_subprocess(*_args, **_kwargs):
        process = _FakeCodexProcess('- Ready.')
        processes.append(process)
        return process

    monkeypatch.setattr(
        'rosmon2.supervisor.shutil.which', lambda _command: '/usr/bin/codex')
    monkeypatch.setattr(
        'rosmon2.supervisor.asyncio.create_subprocess_exec', fake_subprocess)
    supervisor = Supervisor(
        '', [], ui=False, control=False, codex_workspace=str(tmp_path))

    asyncio.run(supervisor._run_codex(
        'move the robot tcp 10 mm +X', mode='agent'))
    asyncio.run(supervisor._run_codex('base', mode='agent'))
    asyncio.run(supervisor._run_codex(
        'what nodes are running?', mode='agent'))

    operation_sets = []
    turn_contexts = []
    for process in processes:
        thread_start = next(
            message for message in process.stdin.messages
            if message.get('method') == 'thread/start')
        ros_tool = next(
            tool for tool in thread_start['params']['dynamicTools']
            if tool['name'] == 'ros2_interface')
        operation_sets.append(
            ros_tool['inputSchema']['properties']['operation']['enum'])
        turn_start = next(
            message for message in process.stdin.messages
            if message.get('method') == 'turn/start')
        turn_contexts.append(turn_start['params']['input'][0]['text'])

    assert 'send_action_goal' in operation_sets[0]
    assert 'send_action_goal' in operation_sets[1]
    assert 'send_action_goal' not in operation_sets[2]
    assert 'immediately pending request' in turn_contexts[1]
    assert supervisor._codex_pending_ros_operation_question is None


def test_codex_ros_tool_inspects_graph_and_calls_service(
        monkeypatch, tmp_path):
    class FakeRosProcess:
        def __init__(self, output):
            self.output = output
            self.returncode = 0

        async def communicate(self):
            return self.output, b''

    calls = []
    outputs = iter((
        b'/reset_controller [std_srvs/srv/Trigger]\n',
        b'response: success=true\n',
    ))

    async def fake_subprocess(*args, **kwargs):
        calls.append((args, kwargs))
        return FakeRosProcess(next(outputs))

    monkeypatch.setattr(
        'rosmon2.supervisor.shutil.which',
        lambda command: f'/usr/bin/{command}',
    )
    monkeypatch.setattr(
        'rosmon2.supervisor.asyncio.create_subprocess_exec',
        fake_subprocess,
    )
    supervisor = Supervisor(
        '', [], ui=False, control=False, codex_workspace=str(tmp_path))

    success, output = asyncio.run(supervisor._run_codex_ros_tool({
        'tool': 'ros2_interface',
        'arguments': {'operation': 'list_services'},
    }, 'what services are available?'))

    assert success
    assert calls[0][0] == ('ros2', 'service', 'list', '-t')
    assert '/reset_controller' in output

    success, output = asyncio.run(supervisor._run_codex_ros_tool({
        'tool': 'ros2_interface',
        'arguments': {
            'operation': 'call_service',
            'name': '/reset_controller',
            'interface_type': 'std_srvs/srv/Trigger',
            'values': '{}',
            'timeout_seconds': 8,
        },
    }, 'please call the /reset_controller service'))

    assert success
    assert calls[1][0] == (
        'ros2', 'service', 'call',
        '/reset_controller', 'std_srvs/srv/Trigger', '{}',
    )
    assert 'success=true' in output


def test_codex_ros_tool_reads_controller_parameter_defaults(
        monkeypatch, tmp_path):
    class FakeRosProcess:
        returncode = 0

        def __init__(self, output):
            self.output = output

        async def communicate(self):
            return self.output, b''

    calls = []
    outputs = iter((
        b'default_speed\ndefault_acceleration\n',
        b'Double value is: 0.2\n',
    ))

    async def fake_subprocess(*args, **kwargs):
        calls.append((args, kwargs))
        return FakeRosProcess(next(outputs))

    monkeypatch.setattr(
        'rosmon2.supervisor.shutil.which',
        lambda command: f'/usr/bin/{command}',
    )
    monkeypatch.setattr(
        'rosmon2.supervisor.asyncio.create_subprocess_exec',
        fake_subprocess,
    )
    supervisor = Supervisor(
        '', [], ui=False, control=False, codex_workspace=str(tmp_path))

    success, output = asyncio.run(supervisor._run_codex_ros_tool({
        'tool': 'ros2_interface',
        'arguments': {
            'operation': 'list_parameters',
            'name': '/arm_controller',
        },
    }, 'move 10 mm in x'))
    assert success
    assert calls[0][0] == (
        'ros2', 'param', 'list', '/arm_controller',
    )
    assert 'default_acceleration' in output

    success, output = asyncio.run(supervisor._run_codex_ros_tool({
        'tool': 'ros2_interface',
        'arguments': {
            'operation': 'get_parameter',
            'name': '/arm_controller',
            'parameter': 'motion.default_speed',
        },
    }, 'move 10 mm in x'))
    assert success
    assert calls[1][0] == (
        'ros2', 'param', 'get',
        '/arm_controller', 'motion.default_speed',
    )
    assert '0.2' in output


def test_codex_ros_tool_validates_motion_and_action_goal(
        monkeypatch, tmp_path):
    class FakeRosProcess:
        returncode = 0

        async def communicate(self):
            return b'Goal accepted\\nResult: success=true\\n', b''

    calls = []

    async def fake_subprocess(*args, **kwargs):
        calls.append((args, kwargs))
        return FakeRosProcess()

    monkeypatch.setattr(
        'rosmon2.supervisor.shutil.which',
        lambda command: f'/usr/bin/{command}',
    )
    monkeypatch.setattr(
        'rosmon2.supervisor.asyncio.create_subprocess_exec',
        fake_subprocess,
    )
    supervisor = Supervisor(
        '', [], ui=False, control=False, codex_workspace=str(tmp_path))
    action_call = {
        'tool': 'ros2_interface',
        'arguments': {
            'operation': 'send_action_goal',
            'name': '/move_relative',
            'interface_type': 'robot_msgs/action/MoveRelative',
            'values': '{axis: x, distance_m: 0.01}',
            'timeout_seconds': 20,
            'feedback': True,
        },
    }

    success, message = asyncio.run(supervisor._run_codex_ros_tool(
        action_call, 'move 10 mm'))
    assert not success
    assert 'missing an explicit target or a direction' in message
    assert calls == []

    success, message = asyncio.run(supervisor._run_codex_ros_tool(
        action_call, 'move 10 mm in x'))
    assert success
    assert calls[0][0] == (
        'ros2', 'action', 'send_goal', '--feedback', '--timeout', '20',
        '/move_relative', 'robot_msgs/action/MoveRelative',
        '{axis: x, distance_m: 0.01}',
    )
    assert 'Goal accepted' in message


def test_codex_ros_tool_executes_authorized_motion_clarification(
        monkeypatch, tmp_path):
    class FakeRosProcess:
        returncode = 0

        async def communicate(self):
            return b'Goal accepted\\nResult: success=true\\n', b''

    calls = []

    async def fake_subprocess(*args, **kwargs):
        calls.append((args, kwargs))
        return FakeRosProcess()

    monkeypatch.setattr(
        'rosmon2.supervisor.shutil.which',
        lambda command: f'/usr/bin/{command}',
    )
    monkeypatch.setattr(
        'rosmon2.supervisor.asyncio.create_subprocess_exec',
        fake_subprocess,
    )
    supervisor = Supervisor(
        '', [], ui=False, control=False, codex_workspace=str(tmp_path))
    supervisor._codex_pending_ros_operation_question = (
        'move the robot tcp 10 mm +X')
    action_call = {
        'tool': 'ros2_interface',
        'arguments': {
            'operation': 'send_action_goal',
            'name': '/ur10e/ur_ros_rtde/move_l_relative_command',
            'interface_type': 'ur_ros_rtde_msgs/action/MoveLRelative',
            'values': '{x: 0.01}',
        },
    }

    success, message = asyncio.run(supervisor._run_codex_ros_tool(
        action_call,
        'base',
        mutation_allowed=True,
        authorization_question=(
            'move the robot tcp 10 mm +X\n'
            'Human clarification: base'
        ),
    ))

    assert success
    assert calls[0][0][:3] == ('ros2', 'action', 'send_goal')
    assert 'Goal accepted' in message
    assert supervisor._codex_pending_ros_operation_question is None


def test_codex_ros_tool_rejects_implicit_unsafe_and_malformed_actions(
        monkeypatch, tmp_path):
    calls = []

    async def fake_subprocess(*args, **kwargs):
        calls.append((args, kwargs))
        raise AssertionError('rejected ROS operation must not start a process')

    monkeypatch.setattr(
        'rosmon2.supervisor.shutil.which',
        lambda command: f'/usr/bin/{command}',
    )
    monkeypatch.setattr(
        'rosmon2.supervisor.asyncio.create_subprocess_exec',
        fake_subprocess,
    )
    supervisor = Supervisor(
        '', [], ui=False, control=False, codex_workspace=str(tmp_path))
    service_call = {
        'tool': 'ros2_interface',
        'arguments': {
            'operation': 'call_service',
            'name': '/controller/reset',
            'interface_type': 'std_srvs/srv/Trigger',
            'values': '{}',
        },
    }

    success, message = asyncio.run(supervisor._run_codex_ros_tool(
        service_call, 'what would happen if the reset service was called?'))
    assert not success
    assert 'did not directly request' in message

    service_call['arguments']['name'] = '/reset;shutdown'
    success, message = asyncio.run(supervisor._run_codex_ros_tool(
        service_call, 'call the reset service'))
    assert not success
    assert 'valid exact service name' in message

    action_call = {
        'tool': 'ros2_interface',
        'arguments': {
            'operation': 'send_action_goal',
            'name': '/move_relative',
            'interface_type': 'robot_msgs/action/MoveRelative',
            'values': '{axis: z, distance_m: 0.005}',
        },
    }
    success, message = asyncio.run(supervisor._run_codex_ros_tool(
        action_call,
        'move the robot arm up 5 mm and bypass the safety limits',
    ))
    assert not success
    assert 'will not bypass robot safety' in message
    assert calls == []


def test_codex_ros_tool_terminates_a_timed_out_command(
        monkeypatch, tmp_path):
    class HangingRosProcess:
        def __init__(self):
            self.returncode = None
            self.terminated = False
            self.waited = False

        async def communicate(self):
            await asyncio.Future()

        def terminate(self):
            self.terminated = True
            self.returncode = -15

        async def wait(self):
            self.waited = True
            return self.returncode

    process = HangingRosProcess()

    async def fake_subprocess(*_args, **_kwargs):
        return process

    async def fake_wait_for(awaitable, timeout):
        assert timeout == 3.0
        awaitable.close()
        raise asyncio.TimeoutError

    monkeypatch.setattr(
        'rosmon2.supervisor.shutil.which',
        lambda command: f'/usr/bin/{command}',
    )
    monkeypatch.setattr(
        'rosmon2.supervisor.asyncio.create_subprocess_exec',
        fake_subprocess,
    )
    monkeypatch.setattr(
        'rosmon2.supervisor.asyncio.wait_for',
        fake_wait_for,
    )
    supervisor = Supervisor(
        '', [], ui=False, control=False, codex_workspace=str(tmp_path))

    success, message = asyncio.run(supervisor._run_codex_ros_tool({
        'tool': 'ros2_interface',
        'arguments': {
            'operation': 'list_actions',
            'timeout_seconds': 1,
        },
    }, 'what actions are available?'))

    assert not success
    assert 'timed out after 1 seconds' in message
    assert process.terminated
    assert process.waited
    assert supervisor._ros_tool_process is None


def test_codex_control_tool_executes_validated_node_and_namespace_actions(
        monkeypatch):
    supervisor = Supervisor('', [], ui=False, control=False)
    driver = ProcessRecord(
        key=0, display_name='ur10e/driver', state=State.RUNNING, pid=100)
    helper = ProcessRecord(
        key=1, display_name='ur10e/helper', state=State.RUNNING, pid=101)
    supervisor.records.extend([driver, helper])
    supervisor.ui.set_records(supervisor.records)
    restarts = []
    monkeypatch.setattr(supervisor, 'restart', restarts.append)

    success, message = asyncio.run(supervisor._run_codex_control_tool({
        'tool': 'rosmon_control',
        'arguments': {
            'action': 'restart',
            'scope': 'node',
            'target': 'a',
        },
    }, 'restart a'))

    assert success
    assert restarts == [driver]
    assert '/ur10e/driver' in message

    success, message = asyncio.run(supervisor._run_codex_control_tool({
        'tool': 'rosmon_control',
        'arguments': {
            'action': 'mute',
            'scope': 'namespace',
            'target': 'ur10e',
        },
    }, 'mute the ur10e namespace'))

    assert success
    assert driver.muted and helper.muted
    assert '2 node(s)' in message


def test_codex_control_tool_rejects_implicit_or_invalid_actions(monkeypatch):
    supervisor = Supervisor('', [], ui=False, control=False)
    driver = ProcessRecord(key=0, display_name='driver', state=State.RUNNING)
    supervisor.records.append(driver)
    stops = []
    monkeypatch.setattr(supervisor, 'stop', stops.append)
    call = {
        'tool': 'rosmon_control',
        'arguments': {
            'action': 'stop',
            'scope': 'node',
            'target': 'driver',
        },
    }

    success, message = asyncio.run(
        supervisor._run_codex_control_tool(call, 'is the driver healthy?'))
    assert not success
    assert 'No direct node action' in message
    assert stops == []

    call['arguments']['target'] = 'missing'
    success, message = asyncio.run(
        supervisor._run_codex_control_tool(call, 'stop missing'))
    assert not success
    assert 'no processes match target' in message
    assert stops == []


def test_codex_app_server_routes_dynamic_control_tool_call(
        monkeypatch, tmp_path):
    processes = []
    tool_request = {
        'id': 40,
        'method': 'item/tool/call',
        'params': {
            'callId': 'call-1',
            'threadId': 'thread-1',
            'turnId': 'turn-1',
            'tool': 'rosmon_control',
            'arguments': {
                'action': 'mute',
                'scope': 'node',
                'target': '/ur10e/driver',
            },
        },
    }

    async def fake_subprocess(*_args, **_kwargs):
        process = _FakeCodexProcess(
            '- Muted /ur10e/driver.',
            tool_request=tool_request,
        )
        processes.append(process)
        return process

    monkeypatch.setattr(
        'rosmon2.supervisor.shutil.which', lambda _command: '/usr/bin/codex')
    monkeypatch.setattr(
        'rosmon2.supervisor.asyncio.create_subprocess_exec', fake_subprocess)
    supervisor = Supervisor(
        '', [], ui=False, control=False, codex_workspace=str(tmp_path))
    driver = ProcessRecord(
        key=0, display_name='ur10e/driver', state=State.RUNNING, pid=100)
    supervisor.records.append(driver)
    supervisor.ui.set_records(supervisor.records)

    asyncio.run(supervisor._run_codex('mute a'))

    assert driver.muted
    messages = processes[0].stdin.messages
    initialize = next(
        message for message in messages if message.get('method') == 'initialize')
    thread_start = next(
        message for message in messages if message.get('method') == 'thread/start')
    tool_response = next(message for message in messages if message.get('id') == 40)
    assert initialize['params']['capabilities']['experimentalApi'] is True
    assert thread_start['params']['dynamicTools'][0]['name'] == 'rosmon_control'
    assert tool_response['result']['success'] is True
    assert 'Mute accepted for 1 node(s)' in (
        tool_response['result']['contentItems'][0]['text'])


def test_codex_app_server_routes_managed_python_node_tool(
        monkeypatch, tmp_path):
    node_workspace = tmp_path / 'home' / 'rosmon2'
    processes = []
    tool_request = {
        'id': 41,
        'method': 'item/tool/call',
        'params': {
            'callId': 'call-python-node',
            'threadId': 'thread-1',
            'turnId': 'turn-1',
            'tool': 'rosmon_python_node',
            'arguments': {
                'path': 'probe.py',
                'name': 'tools/probe',
            },
        },
    }

    async def fake_subprocess(*_args, **kwargs):
        assert node_workspace.is_dir()
        assert kwargs['cwd'] == str(node_workspace)
        (node_workspace / 'probe.py').write_text(
            'print("probe")\n', encoding='utf-8')
        process = _FakeCodexProcess(
            '- Started /tools/probe.',
            tool_request=tool_request,
        )
        processes.append(process)
        return process

    monkeypatch.setattr(
        'rosmon2.supervisor.shutil.which', lambda _command: '/usr/bin/codex')
    monkeypatch.setattr(
        'rosmon2.supervisor.asyncio.create_subprocess_exec', fake_subprocess)
    supervisor = Supervisor(
        '', [], ui=False, control=False, codex_workspace=str(tmp_path))
    supervisor.agent_node_workspace = node_workspace
    supervisor._context = object()
    starts = []

    def fake_start(record, *, count_restart=True):
        assert supervisor.ui.codex_execution_label == 'Starting Python node'
        starts.append((record, count_restart))

    monkeypatch.setattr(supervisor, 'start', fake_start)

    asyncio.run(supervisor._run_codex(
        'write and start the Python script probe.py as a ROS node'))

    assert len(supervisor.records) == 1
    assert supervisor.records[0].agent_created
    assert starts == [(supervisor.records[0], False)]
    thread_start = next(
        message for message in processes[0].stdin.messages
        if message.get('method') == 'thread/start'
    )
    tool_names = [
        tool['name'] for tool in thread_start['params']['dynamicTools']
    ]
    assert 'rosmon_python_node' in tool_names
    assert thread_start['params']['cwd'] == str(node_workspace)
    turn_start = next(
        message for message in processes[0].stdin.messages
        if message.get('method') == 'turn/start'
    )
    assert (
        f'Agent-created node workspace: {node_workspace}'
        in turn_start['params']['input'][0]['text']
    )
    tool_response = next(
        message for message in processes[0].stdin.messages
        if message.get('id') == 41
    )
    assert tool_response['result']['success'] is True
    assert 'orange background' in (
        tool_response['result']['contentItems'][0]['text'])
    assert supervisor.ui.codex_execution_label is None


def test_codex_app_server_tracks_command_execution_lifecycle(
        monkeypatch, tmp_path):
    processes = []
    command = {
        'id': 'command-1',
        'type': 'commandExecution',
        'command': 'pytest -q',
        'status': 'inProgress',
    }

    async def fake_subprocess(*_args, **_kwargs):
        process = _FakeCodexProcess('- Tests passed.')
        process.stdout._lines[3:3] = [
            (json.dumps({
                'method': 'item/started',
                'params': {'item': command},
            }) + '\n').encode(),
            (json.dumps({
                'method': 'item/completed',
                'params': {'item': {
                    **command,
                    'status': 'completed',
                    'exitCode': 0,
                }},
            }) + '\n').encode(),
        ]
        processes.append(process)
        return process

    monkeypatch.setattr(
        'rosmon2.supervisor.shutil.which', lambda _command: '/usr/bin/codex')
    monkeypatch.setattr(
        'rosmon2.supervisor.asyncio.create_subprocess_exec', fake_subprocess)
    supervisor = Supervisor(
        '', [], ui=False, control=False, codex_workspace=str(tmp_path))
    execution_states = []
    original = supervisor.ui.set_agent_execution

    def track_execution(mode, label):
        execution_states.append((mode, label))
        original(mode, label)

    monkeypatch.setattr(
        supervisor.ui, 'set_agent_execution', track_execution)

    asyncio.run(supervisor._run_codex('inspect the test results'))

    assert ('agent', 'Running pytest -q') in execution_states
    assert execution_states[-1] == ('agent', None)
    assert supervisor.ui.codex_execution_label is None


def test_control_status_and_namespace_mute_are_structured():
    supervisor = Supervisor('', [], ui=False, control=False)
    root = ProcessRecord(
        key=0, display_name='hardware_setup', state=State.RUNNING, pid=100)
    driver = ProcessRecord(
        key=1, display_name='ur10e/driver', state=State.CRASHED, return_code=2)
    helper = ProcessRecord(
        key=2, display_name='ur10e/helper', state=State.RUNNING, pid=101)
    supervisor.records.extend([root, driver, helper])

    status = asyncio.run(supervisor.control_request({'command': 'status'}))
    assert status['summary'] == {
        'total': 3,
        'idle': 0,
        'running': 2,
        'crashed': 1,
        'waiting': 0,
    }
    ur10e = next(item for item in status['namespaces'] if item['name'] == 'ur10e')
    assert (ur10e['alive'], ur10e['dead']) == (1, 1)

    result = asyncio.run(supervisor.control_request({
        'command': 'mute',
        'namespace': '/ur10e',
    }))
    assert result['matched'] == 2
    assert not root.muted
    assert driver.muted and helper.muted


def test_control_wait_returns_when_target_is_already_in_state():
    supervisor = Supervisor('', [], ui=False, control=False)
    supervisor.records.append(ProcessRecord(
        key=0, display_name='ur10e/driver', state=State.RUNNING, pid=100))

    result = asyncio.run(supervisor.control_request({
        'command': 'wait',
        'node': '/ur10e/driver',
        'state': 'running',
        'timeout': 0,
    }))

    assert result['matched'] == 1
    assert result['nodes'][0]['name'] == '/ur10e/driver'


def test_codex_python_node_tool_registers_managed_orange_node(
        monkeypatch, tmp_path):
    script = tmp_path / 'health_probe.py'
    script.write_text(
        'import rclpy\n'
        'from rclpy.node import Node\n'
        'rclpy.init()\n'
        'node = Node("health_probe")\n',
        encoding='utf-8',
    )
    supervisor = Supervisor(
        '', [], ui=False, control=False, codex_workspace=str(tmp_path))
    supervisor.agent_node_workspace = tmp_path
    supervisor._context = object()
    starts = []

    def fake_start(record, *, count_restart=True):
        starts.append((record, count_restart))

    monkeypatch.setattr(supervisor, 'start', fake_start)

    success, message = asyncio.run(
        supervisor._run_codex_python_node_tool({
            'tool': 'rosmon_python_node',
            'arguments': {
                'path': 'health_probe.py',
                'name': 'tools/health_probe',
                'script_arguments': ['--rate', '2'],
            },
        }, 'please write and start a Python ROS node'))

    assert success
    assert 'orange background' in message
    assert len(supervisor.records) == 1
    record = supervisor.records[0]
    assert record.agent_created
    assert record.display_name == 'tools/health_probe'
    assert record.cmd[0]
    assert record.cmd[1:] == [
        str(script.resolve()), '--rate', '2',
    ]
    assert record.cwd == str(tmp_path)
    assert starts == [(record, False)]
    assert supervisor._record_dict(record)['agent_created'] is True


def test_codex_python_node_tool_rejects_implicit_external_and_duplicate(
        monkeypatch, tmp_path):
    script = tmp_path / 'probe.py'
    script.write_text('print("ok")\n', encoding='utf-8')
    outside = tmp_path.parent / 'outside_agent_node.py'
    outside.write_text('print("outside")\n', encoding='utf-8')
    supervisor = Supervisor(
        '', [], ui=False, control=False, codex_workspace=str(tmp_path))
    supervisor.agent_node_workspace = tmp_path
    supervisor._context = object()
    monkeypatch.setattr(
        supervisor,
        'start',
        lambda _record, *, count_restart=True: None,
    )
    call = {
        'tool': 'rosmon_python_node',
        'arguments': {'path': 'probe.py', 'name': 'probe'},
    }

    success, message = asyncio.run(
        supervisor._run_codex_python_node_tool(
            call, 'would a Python node be useful?'))
    assert not success
    assert 'did not directly request' in message

    call['arguments']['path'] = str(outside)
    success, message = asyncio.run(
        supervisor._run_codex_python_node_tool(
            call, 'start this Python script as a node'))
    assert not success
    assert 'inside ~/rosmon2' in message

    call['arguments']['path'] = 'probe.py'
    success, _message = asyncio.run(
        supervisor._run_codex_python_node_tool(
            call, 'start this Python script as a node'))
    assert success
    success, message = asyncio.run(
        supervisor._run_codex_python_node_tool(
            call, 'start this Python script as a node'))
    assert not success
    assert 'already exists' in message


def test_log_file_lines_are_timestamped(tmp_path):
    """The --log file outlives the session; the in-memory ring does not.

    rosmon2_logs carries a timestamp per entry but keeps only the last
    _logs.maxlen across every process, so any launch of real size loses its
    early history and the whole thing vanishes when the session exits. This
    file is what a post-mortem actually has, and without a clock on it
    "node A went quiet while node B was still retrying" cannot be answered.
    """
    log_file = tmp_path / 'rosmon2.log'
    supervisor = Supervisor(
        '', [], ui=False, control=False, log_file=str(log_file))

    supervisor._write_log('track_3d', '[INFO] tracking initialized', False)
    supervisor._write_log('camera', 'device disconnected', True)
    supervisor._log_handle.flush()

    lines = log_file.read_text().splitlines()
    assert len(lines) == 2
    stamp, rest = lines[0].split('] ', 1)
    datetime.strptime(stamp.lstrip('['), '%Y-%m-%d %H:%M:%S.%f')
    # The old format stays intact as a suffix so existing greps keep working.
    assert rest == '[stdout] track_3d: [INFO] tracking initialized'
    assert lines[1].endswith('[stderr] camera: device disconnected')


def test_log_file_splits_multiline_output_into_stamped_lines(tmp_path):
    """One read can carry several lines; each needs its own clock."""
    log_file = tmp_path / 'rosmon2.log'
    supervisor = Supervisor(
        '', [], ui=False, control=False, log_file=str(log_file))

    supervisor._write_log('planner', 'first\nsecond', False)
    supervisor._log_handle.flush()

    lines = log_file.read_text().splitlines()
    assert len(lines) == 2
    for line, expected in zip(lines, ('first', 'second')):
        stamp, rest = line.split('] ', 1)
        datetime.strptime(stamp.lstrip('['), '%Y-%m-%d %H:%M:%S.%f')
        assert rest == f'[stdout] planner: {expected}'


def test_no_log_file_is_not_an_error():
    """--log is optional; the tee is simply skipped."""
    supervisor = Supervisor('', [], ui=False, control=False)
    supervisor._write_log('track_3d', 'hello', False)
