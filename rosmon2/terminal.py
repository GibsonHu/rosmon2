"""Small ANSI terminal UI patterned after rosmon's interface."""

import os
import re
import shutil
import sys
import termios
import tty
from typing import Callable, Iterable, Optional

from .model import ProcessRecord, selection_key, State


ANSI_RE = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
SEVERITY_RE = re.compile(r'\[(DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\]')


class TerminalUI:
    """Render streaming logs with a persistent rosmon-style status bar."""

    RESET = '\x1b[0m'
    BAR = '\x1b[48;5;58m\x1b[97m'
    BAR_KEY = '\x1b[48;5;60m\x1b[97m'
    RUNNING = '\x1b[30;42m'
    CRASHED = '\x1b[30;41m'
    WAITING = '\x1b[30;43m'
    IDLE = '\x1b[97;40m'
    KEY = '\x1b[30;47m'
    MUTED_KEY = '\x1b[97;44m'

    def __init__(self, enabled: bool, on_key: Callable[[str], None]):
        self.enabled = bool(enabled and sys.stdin.isatty() and sys.stdout.isatty())
        self.on_key = on_key
        self.records: Iterable[ProcessRecord] = []
        self.selected: Optional[int] = None
        self.warn_only = False
        self._saved_termios = None
        self._status_lines = 0
        self._buffer = ''
        self._started = False

    def start(self, loop) -> None:
        """Enter raw input mode and register the keyboard reader."""
        if not self.enabled or self._started:
            return
        self._saved_termios = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())
        os.set_blocking(sys.stdin.fileno(), False)
        loop.add_reader(sys.stdin.fileno(), self._read_input)
        sys.stdout.write('\x1b[?25l')
        sys.stdout.flush()
        self._started = True

    def close(self, loop=None) -> None:
        """Restore the user's terminal even when launch was interrupted."""
        if not self._started:
            return
        if loop is not None:
            try:
                loop.remove_reader(sys.stdin.fileno())
            except Exception:
                pass
        self._erase_status()
        sys.stdout.write(self.RESET + '\x1b[?25h')
        sys.stdout.flush()
        if self._saved_termios is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._saved_termios)
        self._started = False

    def set_records(self, records: Iterable[ProcessRecord]) -> None:
        self.records = records
        self.redraw()

    def log(self, source: str, text: str, is_stderr: bool = False,
            severity: Optional[str] = None) -> None:
        """Print one or more process output lines above the status bar."""
        self._erase_status()
        width = max([len(r.display_name) for r in self.records] + [len(source), 8])
        clean = text.replace('\r\n', '\n').replace('\r', '\n')
        for line in clean.splitlines():
            line_severity = self._severity(line, severity, is_stderr)
            if self.warn_only and line_severity not in ('WARNING', 'ERROR', 'FATAL'):
                continue
            label = f'{source:>{width}}:'
            if self.enabled:
                label = '\x1b[48;5;24m\x1b[97m' + label + self.RESET
                style = {
                    'DEBUG': '\x1b[32m',
                    'WARNING': '\x1b[33m',
                    'ERROR': '\x1b[31m',
                    'FATAL': '\x1b[1;31m',
                }.get(line_severity, '')
                if style:
                    line = style + line + self.RESET
            sys.stdout.write(f'{label} {line}\n')
        sys.stdout.flush()
        self.redraw()

    def notice(self, text: str, error: bool = False) -> None:
        self.log('rosmon2', text, severity='ERROR' if error else 'INFO')

    @staticmethod
    def _severity(line: str, explicit: Optional[str], is_stderr: bool) -> str:
        """Determine severity without assuming all ROS stderr output is an error."""
        match = SEVERITY_RE.search(ANSI_RE.sub('', line))
        value = match.group(1) if match else explicit
        if value == 'WARN':
            value = 'WARNING'
        if value:
            return value.upper()
        return 'ERROR' if is_stderr else 'INFO'

    def redraw(self) -> None:
        if not self.enabled or not self._started:
            return
        self._erase_status()
        columns = max(40, shutil.get_terminal_size((100, 24)).columns)
        sep = '\x1b[38;5;58m' + ('▂' * columns) + self.RESET
        if self.selected is None:
            menu = self._menu_item('A-Z', 'Node actions')
            menu += self._menu_item('F6', 'Start all')
            menu += self._menu_item('F7', 'Stop all')
            menu += self._menu_item('F8', 'Toggle WARN+ only')
            menu += self._menu_item('F9', 'Mute all')
            menu += self._menu_item('F10', 'Unmute all')
            if self.warn_only:
                menu += ' \x1b[30;45m ! WARN+ output only ! ' + self.RESET
            if any(r.muted for r in self.records):
                menu += ' \x1b[30;43m ! Caution: Nodes muted ! ' + self.RESET
        else:
            records = list(self.records)
            if self.selected >= len(records):
                self.selected = None
                return self.redraw()
            record = records[self.selected]
            menu = self.BAR + f" Node '{record.display_name}' is {record.state.value}. Actions:"
            menu += self._menu_item('s', 'start')
            menu += self._menu_item('k', 'stop')
            menu += self._menu_item('d', 'debug')
            menu += self._menu_item('u' if record.muted else 'm',
                                    'unmute' if record.muted else 'mute')
        menu = self._fit(menu, columns)

        blocks = []
        line = ''
        for index, record in enumerate(self.records):
            key = selection_key(index)
            key_text = key if key is not None else ' '
            key_style = self.MUTED_KEY if record.muted else self.KEY
            state_style = {
                State.RUNNING: self.RUNNING,
                State.CRASHED: self.CRASHED,
                State.WAITING: self.WAITING,
                State.IDLE: self.IDLE,
            }[record.state]
            name = record.display_name.lstrip('/')[-13:]
            selected = self.selected == index
            label = f'[{name:^13}]' if selected else f' {name:^13} '
            block = key_style + key_text + state_style + label + self.RESET
            plain_len = 16
            if self._visible_len(line) + plain_len + 1 > columns and line:
                blocks.append(line)
                line = block
            else:
                line += (' ' if line else '') + block
        if line:
            blocks.append(line)
        if not blocks:
            blocks = [self.IDLE + ' waiting for processes ' + self.RESET]

        lines = [sep, menu] + blocks
        sys.stdout.write('\n'.join(lines) + '\n')
        sys.stdout.write(f'\x1b[{len(lines)}A\r')
        sys.stdout.flush()
        self._status_lines = len(lines)

    def _menu_item(self, key: str, label: str) -> str:
        return f'{self.BAR_KEY} {key}:{self.BAR} {label} {self.RESET}'

    def _erase_status(self) -> None:
        if self.enabled and self._status_lines:
            sys.stdout.write('\r\x1b[J')
            self._status_lines = 0

    def _read_input(self) -> None:
        try:
            data = os.read(sys.stdin.fileno(), 64).decode(errors='ignore')
        except (BlockingIOError, OSError):
            return
        self._buffer += data
        keys = {
            '\x1b[17~': 'F6', '\x1b[18~': 'F7', '\x1b[19~': 'F8',
            '\x1b[20~': 'F9', '\x1b[21~': 'F10',
        }
        while self._buffer:
            matched = False
            for sequence, name in keys.items():
                if self._buffer.startswith(sequence):
                    self._buffer = self._buffer[len(sequence):]
                    self.on_key(name)
                    matched = True
                    break
            if matched:
                continue
            if self._buffer.startswith('\x1b') and len(self._buffer) < 3:
                break
            char, self._buffer = self._buffer[0], self._buffer[1:]
            self.on_key(char)

    @staticmethod
    def _visible_len(text: str) -> int:
        return len(ANSI_RE.sub('', text))

    def _fit(self, text: str, columns: int) -> str:
        plain = ANSI_RE.sub('', text)
        if len(plain) <= columns:
            return text + self.BAR + (' ' * (columns - len(plain))) + self.RESET
        # The status remains useful on narrow terminals even without every action.
        return plain[:columns]
