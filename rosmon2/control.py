"""Local Unix-socket transport shared by rosmon2's CLI and MCP adapter."""

import asyncio
import json
import os
import re
import socket
import stat
from pathlib import Path
from typing import Dict, Iterator


SESSION_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$')
CONTROL_PROTOCOL_VERSION = 1


class ControlError(RuntimeError):
    """An error returned by, or encountered while contacting, rosmon2."""


def validate_session_name(session: str) -> str:
    """Validate a session before using it as part of a socket path."""
    if not SESSION_RE.fullmatch(session):
        raise ValueError(
            'session names must start with a letter or number and contain only '
            'letters, numbers, ".", "_", or "-" (maximum 64 characters)'
        )
    return session


def runtime_directory() -> Path:
    """Return a private runtime directory for this user's control sockets."""
    override = os.environ.get('ROSMON2_RUNTIME_DIR')
    if override:
        return Path(override).expanduser()
    xdg_runtime = os.environ.get('XDG_RUNTIME_DIR')
    if xdg_runtime:
        return Path(xdg_runtime) / 'rosmon2'
    return Path('/tmp') / f'rosmon2-{os.getuid()}'


def session_socket_path(session: str) -> Path:
    """Return the socket path for a named rosmon2 session."""
    validate_session_name(session)
    return runtime_directory() / f'{session}.sock'


def _encoded(message: Dict) -> bytes:
    return (json.dumps(message, separators=(',', ':'), sort_keys=True) + '\n').encode()


class ControlClient:
    """Synchronous client used by short-lived CLI and MCP commands."""

    def __init__(self, session: str = 'default', timeout: float = 10.0):
        self.session = validate_session_name(session)
        self.timeout = timeout
        self.path = session_socket_path(session)

    def request(self, request: Dict) -> Dict:
        """Send one request and return its response."""
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
                connection.settimeout(self.timeout)
                connection.connect(str(self.path))
                connection.sendall(_encoded(request))
                with connection.makefile('rb') as stream:
                    response = self._read_line(stream)
        except (FileNotFoundError, ConnectionRefusedError) as exc:
            raise ControlError(
                f"rosmon2 session '{self.session}' is not running "
                f'(socket: {self.path})'
            ) from exc
        except socket.timeout as exc:
            raise ControlError(
                f"timed out waiting for rosmon2 session '{self.session}'"
            ) from exc
        except OSError as exc:
            raise ControlError(
                f"could not contact rosmon2 session '{self.session}': {exc}"
            ) from exc
        if not response.get('ok', False):
            raise ControlError(response.get('error', 'rosmon2 control request failed'))
        return response

    def events(self) -> Iterator[Dict]:
        """Subscribe to events and yield them until the session exits."""
        connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            connection.connect(str(self.path))
            connection.sendall(_encoded({'command': 'events'}))
            with connection.makefile('rb') as stream:
                acknowledgement = self._read_line(stream)
                if not acknowledgement.get('ok', False):
                    raise ControlError(
                        acknowledgement.get('error', 'event subscription failed')
                    )
                yield acknowledgement
                while True:
                    try:
                        yield self._read_line(stream)
                    except EOFError:
                        return
        except (FileNotFoundError, ConnectionRefusedError, OSError) as exc:
            raise ControlError(
                f"rosmon2 session '{self.session}' is not running "
                f'(socket: {self.path})'
            ) from exc
        finally:
            connection.close()

    @staticmethod
    def _read_line(stream) -> Dict:
        raw = stream.readline(4 * 1024 * 1024 + 1)
        if not raw:
            raise EOFError('rosmon2 closed the control connection')
        if len(raw) > 4 * 1024 * 1024:
            raise ControlError('rosmon2 control response is too large')
        try:
            return json.loads(raw.decode())
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ControlError('rosmon2 returned an invalid JSON response') from exc


class ControlServer:
    """Expose one Supervisor through a private local Unix socket."""

    def __init__(self, supervisor, session: str):
        self.supervisor = supervisor
        self.session = validate_session_name(session)
        self.path = session_socket_path(session)
        self._server = None
        self._subscribers = set()

    async def start(self) -> None:
        """Create the session socket, rejecting active name collisions."""
        self.path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        try:
            self.path.parent.chmod(0o700)
        except OSError:
            pass
        if self.path.exists():
            if not stat.S_ISSOCK(self.path.lstat().st_mode):
                raise ControlError(
                    f"refusing to replace non-socket path '{self.path}'"
                )
            if await self._socket_is_active():
                raise ControlError(
                    f"rosmon2 session '{self.session}' is already running "
                    f'(socket: {self.path})'
                )
            self.path.unlink()
        self._server = await asyncio.start_unix_server(
            self._handle_client, path=str(self.path)
        )
        self.path.chmod(0o600)
        self.supervisor.add_event_listener(self.publish)

    async def close(self) -> None:
        """Stop accepting requests and remove the session socket."""
        self.supervisor.remove_event_listener(self.publish)
        subscribers = list(self._subscribers)
        self._subscribers.clear()
        for writer in subscribers:
            writer.close()
        if subscribers:
            await asyncio.gather(
                *(writer.wait_closed() for writer in subscribers),
                return_exceptions=True,
            )
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
            self._server = None
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass

    async def _socket_is_active(self) -> bool:
        try:
            _reader, writer = await asyncio.open_unix_connection(str(self.path))
        except (ConnectionRefusedError, FileNotFoundError, OSError):
            return False
        writer.close()
        await writer.wait_closed()
        return True

    async def _handle_client(self, reader, writer) -> None:
        try:
            raw = await reader.readline()
            if not raw:
                return
            try:
                request = json.loads(raw)
                if not isinstance(request, dict):
                    raise ValueError('request must be a JSON object')
            except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
                await self._send(writer, {
                    'ok': False,
                    'protocol_version': CONTROL_PROTOCOL_VERSION,
                    'error': str(exc),
                })
                return

            if request.get('command') == 'events':
                self._subscribers.add(writer)
                await self._send(writer, {
                    'ok': True,
                    'protocol_version': CONTROL_PROTOCOL_VERSION,
                    'session': self.session,
                    'subscribed': True,
                })
                await reader.read()
                return

            try:
                response = await self.supervisor.control_request(request)
                response.setdefault('ok', True)
            except (ControlError, ValueError) as exc:
                response = {'ok': False, 'error': str(exc)}
            except Exception as exc:  # keep malformed clients from killing supervision
                response = {
                    'ok': False,
                    'error': f'internal rosmon2 control error: {exc}',
                }
            response.setdefault('protocol_version', CONTROL_PROTOCOL_VERSION)
            await self._send(writer, response)
        finally:
            self._subscribers.discard(writer)
            writer.close()
            try:
                await writer.wait_closed()
            except (BrokenPipeError, ConnectionResetError):
                pass

    def publish(self, event: Dict) -> None:
        """Queue an event for every active event-stream subscriber."""
        if not self._subscribers:
            return
        payload = _encoded(event)
        for writer in list(self._subscribers):
            try:
                transport = writer.transport
                if transport.get_write_buffer_size() > 1024 * 1024:
                    self._subscribers.discard(writer)
                    writer.close()
                    continue
                writer.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                self._subscribers.discard(writer)

    @staticmethod
    async def _send(writer, message: Dict) -> None:
        writer.write(_encoded(message))
        await writer.drain()
