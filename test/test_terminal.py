from rosmon2.terminal import TerminalUI


def test_ros_severity_takes_precedence_over_stderr_channel():
    assert TerminalUI._severity('[INFO] node started', None, True) == 'INFO'
    assert TerminalUI._severity('[WARN] delayed', None, False) == 'WARNING'
    assert TerminalUI._severity('plain stderr', None, True) == 'ERROR'
