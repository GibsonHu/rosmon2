import os

from rosmon2.model import ProcessRecord, State
from rosmon2.terminal import ANSI_RE, _hsluv_label_color, TerminalUI


def test_ros_severity_takes_precedence_over_stderr_channel():
    assert TerminalUI._severity('[INFO] node started', None, True) == 'INFO'
    assert TerminalUI._severity('[WARN] delayed', None, False) == 'WARNING'
    assert TerminalUI._severity('plain stderr', None, True) == 'ERROR'


def test_processes_get_distinct_stable_label_colors():
    ui = TerminalUI(False, lambda _key: None)
    ui.records = [
        ProcessRecord(key=0, display_name='/talker'),
        ProcessRecord(key=1, display_name='/listener'),
    ]
    assert ui._label_color('/talker') != ui._label_color('/listener')
    assert ui._label_color('/talker') == ui._label_color('/talker')
    assert ui._label_color('launch') is None


def test_hsluv_colors_match_rosmon_reference_palette():
    assert _hsluv_label_color(0) == (102, 0, 39)
    assert _hsluv_label_color(120) == (21, 55, 0)
    assert _hsluv_label_color(240) == (0, 51, 78)


def test_status_colors_match_rosmon_reference_palette():
    assert '\x1b[48;2;0;64;64m' in TerminalUI.BAR
    assert '\x1b[48;2;0;96;96m' in TerminalUI.BAR_KEY
    assert '\x1b[48;2;200;200;200m' in TerminalUI.KEY
    assert '\x1b[48;2;24;178;24m' in TerminalUI.RUNNING
    assert '\x1b[48;2;200;200;0m' in TerminalUI.PARTIAL
    assert '\x1b[48;2;135;206;250m' in TerminalUI.NODE_SELECTED


def test_bottom_bar_uses_rosmon_reference_colors():
    assert '48;2;0;64;64' in TerminalUI.BAR
    assert '48;2;0;96;96' in TerminalUI.BAR_KEY
    assert '48;2;200;200;200' in TerminalUI.KEY
    assert '48;2;24;178;24' in TerminalUI.RUNNING


def test_narrow_menu_preserves_key_background_colors():
    ui = TerminalUI(False, lambda _key: None)
    menu = ui._menu_item('A-Z', 'Node actions') + ui._menu_item('F6', 'Start all')
    fitted = ui._fit(menu, 20)
    assert TerminalUI.BAR_KEY in fitted
    assert TerminalUI.BAR in fitted
    assert ui._visible_len(fitted) == 20


def test_status_bar_shows_complete_process_names(monkeypatch, capsys):
    ui = TerminalUI(False, lambda _key: None)
    ui.enabled = True
    ui._started = True
    ui.records = [
        ProcessRecord(key=0, display_name='hardware_setup'),
        ProcessRecord(key=1, display_name='ur10e/ur_ros_rtde/robot_state_receiver'),
    ]
    monkeypatch.setattr('rosmon2.terminal.shutil.get_terminal_size',
                        lambda _fallback: os.terminal_size((50, 24)))

    ui.redraw()

    output = ANSI_RE.sub('', capsys.readouterr().out)
    assert 'hardware_setup' in output
    assert 'ur10e/ur_ros_rtde/robot_state_receiver' in output
    assert 'ardware_setup' not in output.replace('hardware_setup', '')


def test_selected_node_uses_light_blue_background(monkeypatch, capsys):
    ui = TerminalUI(False, lambda _key: None)
    ui.enabled = True
    ui._started = True
    ui.selected = 0
    ui.records = [
        ProcessRecord(key=0, display_name='ur10e/command_server',
                      state=State.RUNNING),
    ]
    monkeypatch.setattr('rosmon2.terminal.shutil.get_terminal_size',
                        lambda _fallback: os.terminal_size((100, 24)))

    ui.redraw()

    output = capsys.readouterr().out
    assert TerminalUI.NODE_SELECTED + '[ur10e/command_server]' in output


def test_namespace_mode_groups_child_namespaces_under_the_top_level():
    ui = TerminalUI(False, lambda _key: None)
    ui.records = [
        ProcessRecord(key=0, display_name='hardware_setup'),
        ProcessRecord(key=1, display_name='ur10e/move_group'),
        ProcessRecord(key=2, display_name='ur10e/ur_ros_rtde/command_server'),
        ProcessRecord(key=3, display_name='camera/image_publisher'),
    ]

    assert ui.namespaces() == ['/', 'camera', 'ur10e']
    assert [record.key for record in ui.records_in_namespace('ur10e')] == [1, 2]


def test_search_matches_full_names_including_namespaces():
    ui = TerminalUI(False, lambda _key: None)
    move_group = ProcessRecord(key=0, display_name='ur10e/move_group')
    command_server = ProcessRecord(
        key=1, display_name='ur10e/ur_ros_rtde/command_server')
    camera = ProcessRecord(key=2, display_name='camera/image_publisher')
    ui.records = [move_group, command_server, camera]

    ui.search_query = 'ur_ros_rtde'

    assert ui.search_matches() == [command_server]


def test_search_is_scoped_to_inspected_namespace():
    ui = TerminalUI(False, lambda _key: None)
    ur_camera = ProcessRecord(key=0, display_name='ur10e/camera')
    external_camera = ProcessRecord(key=1, display_name='external/camera')
    ui.records = [ur_camera, external_camera]
    ui.namespace_mode = True
    ui.namespace_inspect = 'ur10e'
    ui.search_query = 'camera'

    assert ui.search_matches() == [ur_camera]


def test_namespace_colors_reflect_alive_and_dead_counts():
    running = ProcessRecord(key=0, display_name='robot/driver', state=State.RUNNING)
    idle = ProcessRecord(key=1, display_name='robot/helper', state=State.IDLE)
    crashed = ProcessRecord(key=2, display_name='robot/camera', state=State.CRASHED)

    assert TerminalUI.namespace_counts([running, idle, crashed]) == (1, 2)
    assert TerminalUI.namespace_style([running]) == TerminalUI.RUNNING
    assert TerminalUI.namespace_style([running, idle]) == TerminalUI.PARTIAL
    assert TerminalUI.namespace_style([idle, crashed]) == TerminalUI.CRASHED


def test_namespace_status_bar_shows_root_group(monkeypatch, capsys):
    ui = TerminalUI(False, lambda _key: None)
    ui.enabled = True
    ui._started = True
    ui.namespace_mode = True
    ui.records = [
        ProcessRecord(key=0, display_name='hardware_setup'),
        ProcessRecord(key=1, display_name='ur10e/move_group'),
    ]
    monkeypatch.setattr('rosmon2.terminal.shutil.get_terminal_size',
                        lambda _fallback: os.terminal_size((80, 24)))

    ui.redraw()

    output = ANSI_RE.sub('', capsys.readouterr().out)
    assert '/ [0:1]' in output
    assert 'ur10e [0:1]' in output
    assert 'hardware_setup' not in output


def test_selected_namespace_does_not_wrap_its_name_in_brackets(monkeypatch, capsys):
    ui = TerminalUI(False, lambda _key: None)
    ui.enabled = True
    ui._started = True
    ui.namespace_mode = True
    ui.selected = 0
    ui.records = [
        ProcessRecord(key=0, display_name='ur10e/move_group', state=State.RUNNING),
        ProcessRecord(key=1, display_name='ur10e/driver', state=State.CRASHED),
    ]
    monkeypatch.setattr('rosmon2.terminal.shutil.get_terminal_size',
                        lambda _fallback: os.terminal_size((100, 24)))

    ui.redraw()

    output = ANSI_RE.sub('', capsys.readouterr().out)
    assert 'ur10e [1:1]' in output
    assert '[ur10e [1:1]]' not in output


def test_search_status_shows_query_and_only_matching_nodes(monkeypatch, capsys):
    ui = TerminalUI(False, lambda _key: None)
    ui.enabled = True
    ui._started = True
    ui.search_active = True
    ui.search_query = 'receiver'
    ui.records = [
        ProcessRecord(key=0, display_name='ur10e/robot_state_receiver'),
        ProcessRecord(key=1, display_name='ur10e/command_server'),
    ]
    monkeypatch.setattr('rosmon2.terminal.shutil.get_terminal_size',
                        lambda _fallback: os.terminal_size((100, 24)))

    ui.redraw()

    output = ANSI_RE.sub('', capsys.readouterr().out)
    assert 'Searching for: receiver' in output
    assert 'ur10e/robot_state_receiver' in output
    assert 'ur10e/command_server' not in output


def test_input_reader_decodes_search_navigation_keys(monkeypatch):
    class FakeStdin:
        @staticmethod
        def fileno():
            return 10

    pressed = []
    chunks = iter((b'\x1b[A', b'\x1b'))
    monkeypatch.setattr('rosmon2.terminal.sys.stdin', FakeStdin())
    monkeypatch.setattr('rosmon2.terminal.os.read', lambda _fd, _size: next(chunks))
    ui = TerminalUI(False, pressed.append)

    ui._read_input()
    ui._read_input()

    assert pressed == ['UP', 'ESC']


def test_input_reader_waits_for_split_escape_sequence(monkeypatch):
    class FakeStdin:
        @staticmethod
        def fileno():
            return 10

    class FakeTimer:
        def __init__(self, callback):
            self.callback = callback
            self.cancelled = False

        def cancel(self):
            self.cancelled = True

    class FakeLoop:
        def __init__(self):
            self.timers = []

        def call_later(self, _delay, callback):
            timer = FakeTimer(callback)
            self.timers.append(timer)
            return timer

    pressed = []
    chunks = iter((b'\x1b', b'[A'))
    monkeypatch.setattr('rosmon2.terminal.sys.stdin', FakeStdin())
    monkeypatch.setattr('rosmon2.terminal.os.read', lambda _fd, _size: next(chunks))
    ui = TerminalUI(False, pressed.append)
    ui._loop = FakeLoop()

    ui._read_input()
    assert pressed == []
    ui._read_input()

    assert ui._loop.timers[0].cancelled
    assert pressed == ['UP']


def test_start_keeps_shared_terminal_output_blocking(monkeypatch):
    class FakeStream:
        def __init__(self, fd):
            self.fd = fd
            self.output = ''

        @staticmethod
        def isatty():
            return True

        def fileno(self):
            return self.fd

        def write(self, text):
            self.output += text

        @staticmethod
        def flush():
            pass

    class FakeLoop:
        def __init__(self):
            self.readers = []

        def add_reader(self, fd, callback):
            self.readers.append((fd, callback))

    stdin = FakeStream(10)
    stdout = FakeStream(11)
    blocking_calls = []
    monkeypatch.setattr('rosmon2.terminal.sys.stdin', stdin)
    monkeypatch.setattr('rosmon2.terminal.sys.stdout', stdout)
    monkeypatch.setattr('rosmon2.terminal.termios.tcgetattr', lambda _fd: [])
    monkeypatch.setattr('rosmon2.terminal.tty.setcbreak', lambda _fd: None)
    monkeypatch.setattr(
        'rosmon2.terminal.os.set_blocking',
        lambda fd, enabled: blocking_calls.append((fd, enabled)),
    )
    loop = FakeLoop()

    ui = TerminalUI(True, lambda _key: None)
    ui.start(loop)

    assert blocking_calls == [(stdout.fd, True)]
    assert loop.readers == [(stdin.fd, ui._read_input)]
