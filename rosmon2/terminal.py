"""Small ANSI terminal UI patterned after rosmon's interface."""

from math import cos, pi, sin
import os
import re
import shutil
import sys
import termios
import time
import tty
from typing import Callable, Iterable, Optional

from .model import ProcessRecord, selection_key, State


ANSI_RE = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
SEVERITY_RE = re.compile(r'\[(DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\]')
ROS_CONSOLE_PREFIX_RE = re.compile(
    r'^\s*\[(?:DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\]'
    r'(?:\s+\[[^\]\r\n]*\])*\s+\[(?P<context>[^\]\r\n]*)\]'
    r'\s*:\s*(?P<message>.*)$'
)
RGB_MATRIX = (
    (3.2406, -1.5372, -0.4986),
    (-0.9689, 1.8758, 0.0415),
    (0.0557, -0.2040, 1.0570),
)


def _hsluv_label_color(hue: float):
    """Return rosmon's HSLuv(H, 100, 20) process-label color."""
    lightness = 20.0
    hue_radians = hue / 360.0 * 2.0 * pi
    sin_hue = sin(hue_radians)
    cos_hue = cos(hue_radians)
    sub1 = (lightness + 16.0) ** 3 / 1560896.0
    sub2 = sub1 if sub1 > 0.008856 else lightness / 903.3
    max_chroma = float('inf')
    for m1, m2, m3 in RGB_MATRIX:
        top = (0.99915 * m1 + 1.05122 * m2 + 1.14460 * m3) * sub2
        right = 0.86330 * m3 - 0.17266 * m2
        left = 0.12949 * m3 - 0.38848 * m1
        bottom = (right * sin_hue + left * cos_hue) * sub2
        for boundary in (0.0, 1.0):
            chroma = lightness * (top - 1.05122 * boundary)
            chroma /= bottom + 0.17266 * sin_hue * boundary
            if 0.0 < chroma < max_chroma:
                max_chroma = chroma

    u_value = cos_hue * max_chroma
    v_value = sin_hue * max_chroma
    y_value = ((lightness + 16.0) / 116.0) ** 3
    var_u = u_value / (13.0 * lightness) + 0.19784
    var_v = v_value / (13.0 * lightness) + 0.46834
    x_value = -(9.0 * y_value * var_u)
    x_value /= (var_u - 4.0) * var_v - var_u * var_v
    z_value = (9.0 * y_value - 15.0 * var_v * y_value - var_v * x_value)
    z_value /= 3.0 * var_v

    def from_linear(component):
        if component <= 0.0031308:
            return 12.92 * component
        return 1.055 * component ** (1.0 / 2.4) - 0.055

    xyz = (x_value, y_value, z_value)
    rgb = [from_linear(sum(row[i] * xyz[i] for i in range(3)))
           for row in RGB_MATRIX]
    return tuple(max(0, min(255, int(component * 255.0))) for component in rgb)


class TerminalUI:
    """Render streaming logs with a persistent rosmon-style status bar."""

    OUTPUT_FLUSH_INTERVAL = 1.0 / 60.0
    OUTPUT_BUFFER_LIMIT = 64 * 1024
    REDRAW_INTERVAL = 1.0 / 30.0
    RESET = '\x1b[0m'
    # Exact true-color styles from rosmon's UI. Its packed 0xBBGGRR values
    # 0x404000, 0x606000, and 0xC8C8C8 become the RGB values below.
    BAR = '\x1b[48;2;0;64;64m\x1b[38;2;255;255;255m'
    BAR_KEY = '\x1b[48;2;0;96;96m\x1b[38;2;255;255;255m'
    RUNNING = '\x1b[38;2;0;0;0m\x1b[48;2;24;178;24m'
    CRASHED = '\x1b[38;2;0;0;0m\x1b[48;2;178;24;24m'
    PARTIAL = '\x1b[38;2;0;0;0m\x1b[48;2;200;200;0m'
    WAITING = '\x1b[38;2;0;0;0m\x1b[48;2;178;104;24m'
    IDLE = '\x1b[38;2;255;255;255m\x1b[48;2;0;0;0m'
    NODE_SELECTED = '\x1b[38;2;0;0;0m\x1b[48;2;135;206;250m'
    SEARCH_SELECTED = '\x1b[38;2;0;0;0m\x1b[48;2;0;178;178m'
    KEY = '\x1b[38;2;0;0;0m\x1b[48;2;200;200;200m'
    MUTED_KEY = '\x1b[38;2;255;255;255m\x1b[48;2;165;0;0m'

    def __init__(self, enabled: bool, on_key: Callable[[str], None],
                 output_enabled: bool = True):
        self.enabled = bool(enabled and sys.stdin.isatty() and sys.stdout.isatty())
        self.output_enabled = output_enabled
        self.on_key = on_key
        self.records: Iterable[ProcessRecord] = []
        self.selected: Optional[int] = None
        self.namespace_mode = False
        self.namespace_inspect: Optional[str] = None
        self.search_active = False
        self.search_query = ''
        self.search_selected = 0
        self.warn_only = False
        self._saved_termios = None
        self._status_lines = 0
        self._buffer = ''
        self._loop = None
        self._escape_timer = None
        self._output_timer = None
        self._output_buffer = []
        self._output_buffer_size = 0
        self._redraw_timer = None
        self._last_redraw_at = 0.0
        self._render_cache_key = None
        self._render_cache_lines = None
        self._label_names = None
        self._label_width = 8
        self._label_colors = {}
        self._plain_labels = {}
        self._styled_labels = {}
        self._started = False

    def start(self, loop) -> None:
        """Enter raw input mode and register the keyboard reader."""
        if self._started:
            return
        self._loop = loop
        if not self.enabled:
            return
        self._saved_termios = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())
        # stdin/stdout/stderr commonly share one open file description on a
        # pseudo-terminal.  Making stdin nonblocking therefore also makes
        # launch's output writes nonblocking, which can fail with EAGAIN and
        # shut down the complete LaunchService.  add_reader() only invokes
        # _read_input when data is ready, so blocking terminal I/O is safe.
        os.set_blocking(sys.stdout.fileno(), True)
        loop.add_reader(sys.stdin.fileno(), self._read_input)
        sys.stdout.write('\x1b[?25l')
        sys.stdout.flush()
        self._started = True

    def close(self, loop=None) -> None:
        """Restore the user's terminal even when launch was interrupted."""
        if self._output_timer is not None:
            self._output_timer.cancel()
            self._output_timer = None
        self._flush_output(redraw=False)
        if not self._started:
            self._loop = None
            return
        if loop is not None:
            try:
                loop.remove_reader(sys.stdin.fileno())
            except Exception:
                pass
        if self._escape_timer is not None:
            self._escape_timer.cancel()
            self._escape_timer = None
        if self._redraw_timer is not None:
            self._redraw_timer.cancel()
            self._redraw_timer = None
        self._erase_status()
        sys.stdout.write(self.RESET + '\x1b[?25h')
        sys.stdout.flush()
        if self._saved_termios is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._saved_termios)
        self._started = False
        self._loop = None

    def set_records(self, records: Iterable[ProcessRecord]) -> None:
        self.records = records
        names = tuple(record.display_name for record in records)
        if names != self._label_names:
            self._rebuild_label_cache(names)
        self.redraw()

    @staticmethod
    def namespace_for(record: ProcessRecord) -> str:
        """Return the top-level namespace containing a process."""
        parts = [part for part in record.display_name.strip('/').split('/') if part]
        return parts[0] if len(parts) > 1 else '/'

    def namespaces(self):
        """Return stable namespace groups represented by the current records."""
        values = {self.namespace_for(record) for record in self.records}
        return sorted(values, key=lambda value: (value != '/', value))

    def records_in_namespace(self, namespace: str):
        """Return every process recursively grouped under a top-level namespace."""
        return [record for record in self.records
                if self.namespace_for(record) == namespace]

    def visible_records(self):
        """Return nodes visible in normal or namespace inspection mode."""
        if self.namespace_mode and self.namespace_inspect is not None:
            return self.records_in_namespace(self.namespace_inspect)
        return list(self.records)

    def search_matches(self):
        """Return visible nodes whose full names contain the search query."""
        return [record for record in self.visible_records()
                if self.search_query in record.display_name]

    @staticmethod
    def namespace_counts(records):
        """Return running and non-running process counts for a namespace."""
        alive = sum(record.state is State.RUNNING for record in records)
        return alive, len(records) - alive

    @classmethod
    def namespace_style(cls, records):
        """Color a namespace green, yellow, or red from its live/dead counts."""
        alive, dead = cls.namespace_counts(records)
        if dead == 0:
            return cls.RUNNING
        if alive == 0:
            return cls.CRASHED
        return cls.PARTIAL

    @classmethod
    def state_style(cls, state: State):
        """Return the status color for one process state."""
        return {
            State.RUNNING: cls.RUNNING,
            State.CRASHED: cls.CRASHED,
            State.WAITING: cls.WAITING,
            State.IDLE: cls.IDLE,
        }[state]

    def log(self, source: str, text: str, is_stderr: bool = False,
            severity: Optional[str] = None) -> None:
        """Print one or more process output lines above the status bar."""
        if not self.output_enabled:
            return
        self._ensure_label_cache()
        width = max(self._label_width, len(source))
        clean = text.replace('\r\n', '\n').replace('\r', '\n')
        output = []
        for line in clean.splitlines():
            line_severity = self._severity(line, severity, is_stderr)
            if self.warn_only and line_severity not in ('WARNING', 'ERROR', 'FATAL'):
                continue
            line = self._message_body(line)
            label = self._plain_label(source, width)
            if self.enabled:
                label = self._styled_label(source, width)
                style = {
                    'DEBUG': '\x1b[32m',
                    'WARNING': '\x1b[33m',
                    'ERROR': '\x1b[31m',
                    'FATAL': '\x1b[1;31m',
                }.get(line_severity, '')
                if style:
                    line = style + line + self.RESET
            output.append(f'{label} {line}\n')
        if output:
            self._queue_output(output)

    def notice(self, text: str, error: bool = False) -> None:
        self.log('rosmon2', text, severity='ERROR' if error else 'INFO')

    def flush(self) -> None:
        """Immediately write pending process output."""
        if self._output_timer is not None:
            self._output_timer.cancel()
            self._output_timer = None
        had_output = bool(self._output_buffer)
        self._flush_output()
        if not had_output:
            sys.stdout.flush()

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

    @staticmethod
    def _message_body(line: str) -> str:
        """Keep rosmon's function/logger field while removing severity and time."""
        plain = ANSI_RE.sub('', line)
        match = ROS_CONSOLE_PREFIX_RE.match(plain)
        if match is None:
            return line
        return f'[{match.group("context")}]: {match.group("message")}'

    def _label_color(self, source: str):
        """Return the cached color for a process label."""
        self._ensure_label_cache()
        return self._label_colors.get(source)

    def _ensure_label_cache(self) -> None:
        if self._label_names is None:
            names = tuple(record.display_name for record in self.records)
            self._rebuild_label_cache(names)

    def _rebuild_label_cache(self, names) -> None:
        """Cache widths and colors that only change with the process list."""
        self._label_names = names
        self._label_width = max([len(name) for name in names] + [8])
        self._label_colors = {}
        process_count = max(1, len(names))
        for index, name in enumerate(names):
            if name not in self._label_colors:
                hue = index * 360.0 / process_count
                self._label_colors[name] = _hsluv_label_color(hue)
        self._plain_labels = {}
        self._styled_labels = {}

    def _plain_label(self, source: str, width: int) -> str:
        key = (source, width)
        cached = self._plain_labels.get(key)
        if cached is None:
            cached = f'{source:>{width}}:'
            self._plain_labels[key] = cached
        return cached

    def _styled_label(self, source: str, width: int) -> str:
        key = (source, width)
        cached = self._styled_labels.get(key)
        if cached is not None:
            return cached
        label = self._plain_label(source, width)
        background = self._label_colors.get(source)
        if background is None:
            styled = '\x1b[38;2;178;178;178m' + label + self.RESET
        else:
            red, green, blue = background
            styled = (
                f'\x1b[48;2;{red};{green};{blue}m'
                '\x1b[38;2;255;255;255m' + label + self.RESET
            )
        self._styled_labels[key] = styled
        return styled

    def _queue_output(self, output) -> None:
        self._output_buffer.extend(output)
        self._output_buffer_size += sum(len(item) for item in output)
        if self._loop is None or self._output_buffer_size >= self.OUTPUT_BUFFER_LIMIT:
            if self._output_timer is not None:
                self._output_timer.cancel()
                self._output_timer = None
            self._flush_output()
        elif self._output_timer is None:
            self._output_timer = self._loop.call_later(
                self.OUTPUT_FLUSH_INTERVAL, self._run_output_flush)

    def _run_output_flush(self) -> None:
        self._output_timer = None
        self._flush_output()

    def _flush_output(self, *, redraw: bool = True) -> None:
        if not self._output_buffer:
            return
        output = ''.join(self._output_buffer)
        self._output_buffer.clear()
        self._output_buffer_size = 0
        if (redraw and self.enabled and self._started
                and self._render_cache_lines is not None):
            # Keep the status bar in the same terminal write as the new logs.
            # Writing the erase sequence, logs, and status separately makes
            # the status area visibly flash while output is streaming.
            if self._redraw_timer is not None:
                self._redraw_timer.cancel()
                self._redraw_timer = None
            erase = self._take_status_erase()
            lines = self._render_cache_lines
            sys.stdout.write(erase + output + self._status_text(lines))
            self._status_lines = len(lines)
            self._last_redraw_at = time.monotonic()
            sys.stdout.flush()
            return

        self._erase_status()
        sys.stdout.write(output)
        redrawn = self._request_redraw() if redraw else False
        if not redrawn:
            sys.stdout.flush()

    def redraw(self) -> None:
        if self._redraw_timer is not None:
            self._redraw_timer.cancel()
            self._redraw_timer = None
        if not self.enabled or not self._started:
            return
        # Keep erasing the previous status and drawing its replacement in one
        # terminal write.  Selection keys redraw immediately; sending ESC[J
        # first can otherwise leave a visible blank frame in some terminals.
        erase = self._take_status_erase()
        columns = max(40, shutil.get_terminal_size((100, 24)).columns)
        render_key = self._status_render_key(columns)
        if self._render_cache_key == render_key:
            self._draw_status_lines(self._render_cache_lines, prefix=erase)
            return
        sep = '\x1b[38;2;0;64;64m' + ('▂' * columns) + self.RESET
        showing_namespaces = self.namespace_mode and self.namespace_inspect is None
        if self.search_active:
            menu = self.BAR + f' Searching for: {self.search_query}'
        elif self.selected is None:
            menu = self._menu_item(
                'A-Z', 'Namespace actions' if showing_namespaces else 'Node actions')
            menu += self._menu_item(
                'F5', 'Node mode' if self.namespace_mode else 'Namespace mode')
            if self.namespace_mode and self.namespace_inspect is not None:
                menu += self._menu_item('Backspace', 'Namespaces')
            menu += self._menu_item('F6', 'Start all')
            menu += self._menu_item('F7', 'Stop all')
            menu += self._menu_item('F8', 'Toggle WARN+ only')
            menu += self._menu_item('F9', 'Mute all')
            menu += self._menu_item('F10', 'Unmute all')
            menu += self._menu_item('/', 'Node search')
            if self.warn_only:
                menu += ' \x1b[30;45m ! WARN+ output only ! ' + self.RESET
            if any(r.muted for r in self.records):
                menu += ' \x1b[30;43m ! Caution: Nodes muted ! ' + self.RESET
        else:
            if showing_namespaces:
                namespaces = self.namespaces()
                if self.selected >= len(namespaces):
                    self.selected = None
                    return self.redraw()
                namespace = namespaces[self.selected]
                count = len(self.records_in_namespace(namespace))
                menu = self.BAR + f" Namespace '{namespace}' has {count} node(s). Actions:"
                menu += self._menu_item('s', 'start all')
                menu += self._menu_item('k', 'stop all')
                menu += self._menu_item('i', 'inspect')
                menu += self._menu_item('m', 'mute namespace')
                menu += self._menu_item('u', 'unmute namespace')
            else:
                records = self.visible_records()
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

        if self.search_active:
            entries = [(record.display_name, self.state_style(record.state), record.muted)
                       for record in self.search_matches()]
        elif showing_namespaces:
            entries = []
            for namespace in self.namespaces():
                members = self.records_in_namespace(namespace)
                alive, dead = self.namespace_counts(members)
                entries.append((
                    f'{namespace} [{alive}:{dead}]',
                    self.namespace_style(members),
                    bool(members) and all(record.muted for record in members),
                ))
        else:
            entries = [(record.display_name, self.state_style(record.state), record.muted)
                       for record in self.visible_records()]

        blocks = []
        line = ''
        for index, (display_name, state_style, muted) in enumerate(entries):
            if self.search_active:
                name = display_name.lstrip('/')
                max_name_length = max(1, columns - 2)
                if len(name) > max_name_length:
                    name = name[:max_name_length - 1] + '…'
                label = f' {name} '
                style = self.SEARCH_SELECTED if self.search_selected == index else ''
                block = style + label + self.RESET
                plain_len = len(label)
                if self._visible_len(line) + plain_len + 1 > columns and line:
                    blocks.append(line)
                    line = block
                else:
                    line += (' ' if line else '') + block
                continue

            key = selection_key(index)
            key_text = key if key is not None else ' '
            key_style = self.MUTED_KEY if muted else self.KEY
            name = display_name if showing_namespaces else display_name.lstrip('/')
            # Keep the complete process name whenever it fits.  The status
            # area already wraps blocks onto additional rows, so shortening
            # every name to 13 characters only hides useful node identity.
            max_name_length = max(1, columns - 3)
            if len(name) > max_name_length:
                name = name[:max_name_length - 1] + '…'
            selected = self.selected == index
            label = f'[{name}]' if selected and not showing_namespaces else f' {name} '
            label_style = (
                self.NODE_SELECTED
                if selected and not showing_namespaces else state_style
            )
            block = key_style + key_text + label_style + label + self.RESET
            plain_len = 1 + len(label)
            if self._visible_len(line) + plain_len + 1 > columns and line:
                blocks.append(line)
                line = block
            else:
                line += (' ' if line else '') + block
        if line:
            blocks.append(line)
        if not blocks:
            message = ' no matching nodes ' if self.search_active else ' waiting for processes '
            blocks = [self.IDLE + message + self.RESET]

        lines = [sep, menu] + blocks
        self._render_cache_key = render_key
        self._render_cache_lines = tuple(lines)
        self._draw_status_lines(lines, prefix=erase)

    def _request_redraw(self) -> bool:
        """Coalesce log-driven status updates to avoid redrawing per message."""
        if not self.enabled or not self._started:
            return False
        if self._loop is None:
            self.redraw()
            return True
        elapsed = time.monotonic() - self._last_redraw_at
        delay = self.REDRAW_INTERVAL - elapsed
        if delay <= 0:
            self.redraw()
            return True
        elif self._redraw_timer is None:
            self._redraw_timer = self._loop.call_later(
                delay, self._run_scheduled_redraw)
        return False

    def _run_scheduled_redraw(self) -> None:
        self._redraw_timer = None
        self.redraw()

    def _status_render_key(self, columns: int):
        records = tuple(
            (record.display_name, record.state, record.muted)
            for record in self.records
        )
        return (
            columns,
            records,
            self.selected,
            self.namespace_mode,
            self.namespace_inspect,
            self.search_active,
            self.search_query,
            self.search_selected,
            self.warn_only,
        )

    def _draw_status_lines(self, lines, *, prefix: str = '') -> None:
        sys.stdout.write(prefix + self._status_text(lines))
        sys.stdout.flush()
        self._status_lines = len(lines)
        self._last_redraw_at = time.monotonic()

    @staticmethod
    def _status_text(lines) -> str:
        return '\n'.join(lines) + '\n' + f'\x1b[{len(lines)}A\r'

    def _menu_item(self, key: str, label: str) -> str:
        return f'{self.BAR_KEY} {key}:{self.BAR} {label} {self.RESET}'

    def _erase_status(self) -> None:
        erase = self._take_status_erase()
        if erase:
            sys.stdout.write(erase)

    def _take_status_erase(self) -> str:
        if self.enabled and self._status_lines:
            self._status_lines = 0
            return '\r\x1b[J'
        return ''

    def _read_input(self) -> None:
        try:
            data = os.read(sys.stdin.fileno(), 64).decode(errors='ignore')
        except (BlockingIOError, OSError):
            return
        if self._escape_timer is not None:
            self._escape_timer.cancel()
            self._escape_timer = None
        self._buffer += data
        keys = {
            '\x1b[15~': 'F5', '\x1b[17~': 'F6', '\x1b[18~': 'F7', '\x1b[19~': 'F8',
            '\x1b[20~': 'F9', '\x1b[21~': 'F10',
            '\x1b[A': 'UP', '\x1b[B': 'DOWN',
            '\x1b[C': 'RIGHT', '\x1b[D': 'LEFT',
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
            if self._buffer == '\x1b':
                if self._loop is None:
                    self._flush_escape()
                    continue
                self._escape_timer = self._loop.call_later(0.03, self._flush_escape)
                break
            if self._buffer.startswith('\x1b') and len(self._buffer) < 3:
                break
            char, self._buffer = self._buffer[0], self._buffer[1:]
            self.on_key(char)

    def _flush_escape(self) -> None:
        """Emit a standalone Escape after allowing time for an arrow sequence."""
        self._escape_timer = None
        if self._buffer == '\x1b':
            self._buffer = ''
            self.on_key('ESC')

    @staticmethod
    def _visible_len(text: str) -> int:
        return len(ANSI_RE.sub('', text))

    @staticmethod
    def _truncate_ansi(text: str, columns: int) -> str:
        """Truncate visible text while retaining embedded ANSI styles."""
        output = []
        position = 0
        visible = 0
        for match in ANSI_RE.finditer(text):
            chunk = text[position:match.start()]
            remaining = columns - visible
            if remaining <= 0:
                break
            output.append(chunk[:remaining])
            visible += min(len(chunk), remaining)
            if len(chunk) > remaining:
                break
            output.append(match.group())
            position = match.end()
        else:
            remaining = columns - visible
            if remaining > 0:
                output.append(text[position:position + remaining])
        return ''.join(output)

    def _fit(self, text: str, columns: int) -> str:
        plain = ANSI_RE.sub('', text)
        if len(plain) <= columns:
            return text + self.BAR + (' ' * (columns - len(plain))) + self.RESET
        # The status remains useful on narrow terminals even without every action.
        return self._truncate_ansi(text, columns) + self.RESET
