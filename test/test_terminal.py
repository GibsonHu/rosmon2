from rosmon2.model import ProcessRecord
from rosmon2.terminal import _hsluv_label_color, TerminalUI


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
