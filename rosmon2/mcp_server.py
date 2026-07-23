"""Dependency-free MCP stdio adapter for the rosmon2 control socket."""

import json
import sys
from typing import Dict

from . import __version__
from .control import ControlClient, ControlError


PROTOCOL_VERSION = '2025-06-18'

SESSION_PROPERTY = {
    'type': 'string',
    'default': 'default',
    'description': 'Name passed to mon2 launch --session.',
}
TARGET_PROPERTIES = {
    'node': {
        'type': 'string',
        'description': 'Exact fully-qualified node/process name.',
    },
    'namespace': {
        'type': 'string',
        'description': 'Namespace prefix, such as /ur10e.',
    },
}


def _object_schema(properties=None, required=None):
    return {
        'type': 'object',
        'properties': properties or {},
        'required': required or [],
        'additionalProperties': False,
    }


def _tools():
    inspection_target = {'session': SESSION_PROPERTY, **TARGET_PROPERTIES}
    action_target = {
        **inspection_target,
        'all': {
            'type': 'boolean',
            'default': False,
            'description': 'Target every process. Use deliberately.',
        },
    }
    tools = [
        {
            'name': 'rosmon2_status',
            'title': 'Get rosmon2 status',
            'description': (
                'Get structured states, PIDs, exit codes, mute states, and '
                'namespace summaries from a running rosmon2 session.'
            ),
            'inputSchema': _object_schema(inspection_target),
            'annotations': {'readOnlyHint': True, 'idempotentHint': True},
        },
        {
            'name': 'rosmon2_logs',
            'title': 'Query rosmon2 logs',
            'description': 'Read recent structured process logs from a rosmon2 session.',
            'inputSchema': _object_schema({
                **inspection_target,
                'severity': {
                    'type': 'string',
                    'enum': ['DEBUG', 'INFO', 'WARNING', 'ERROR', 'FATAL'],
                },
                'since_seconds': {
                    'type': 'number',
                    'minimum': 0,
                    'default': 0,
                },
                'limit': {
                    'type': 'integer',
                    'minimum': 1,
                    'maximum': 5000,
                    'default': 200,
                },
            }),
            'annotations': {'readOnlyHint': True, 'idempotentHint': True},
        },
        {
            'name': 'rosmon2_wait',
            'title': 'Wait for rosmon2 state',
            'description': (
                'Wait until all matching processes reach the requested state. '
                'With no target, waits for every discovered process.'
            ),
            'inputSchema': _object_schema({
                **inspection_target,
                'state': {
                    'type': 'string',
                    'enum': ['running', 'idle', 'crashed', 'waiting'],
                    'default': 'running',
                },
                'timeout': {
                    'type': 'number',
                    'minimum': 0,
                    'default': 30,
                },
            }),
            'annotations': {'readOnlyHint': True, 'idempotentHint': True},
        },
    ]
    for action in ('start', 'stop', 'restart', 'mute', 'unmute'):
        action_schema = _object_schema(action_target)
        action_schema['oneOf'] = [
            {'required': ['node']},
            {'required': ['namespace']},
            {'required': ['all'], 'properties': {'all': {'const': True}}},
        ]
        tools.append({
            'name': f'rosmon2_{action}',
            'title': f'{action.title()} rosmon2 processes',
            'description': (
                f'{action.title()} one process, a namespace, or all processes. '
                'Exactly one of node, namespace, or all=true must be supplied.'
            ),
            'inputSchema': action_schema,
            'annotations': {
                'readOnlyHint': False,
                'destructiveHint': action in ('stop', 'restart'),
                'idempotentHint': action in ('start', 'stop', 'mute', 'unmute'),
            },
        })
    return tools


TOOLS = {tool['name']: tool for tool in _tools()}


def _call_tool(name: str, arguments: Dict) -> Dict:
    if name not in TOOLS:
        raise KeyError(f'unknown tool: {name}')
    if not isinstance(arguments, dict):
        raise ValueError('tool arguments must be an object')
    session = arguments.get('session', 'default')
    request = dict(arguments)
    request.pop('session', None)
    command = name.removeprefix('rosmon2_')
    if command in ('start', 'stop', 'restart', 'mute', 'unmute'):
        target_count = sum(bool(request.get(key)) for key in ('node', 'namespace', 'all'))
        if target_count != 1:
            raise ValueError(
                'exactly one of node, namespace, or all=true must be supplied'
            )
    request['command'] = command
    timeout = float(request.get('timeout', 30)) + 2 if command == 'wait' else 10
    return ControlClient(session, timeout=timeout).request(request)


def _tool_result(result: Dict, *, is_error=False) -> Dict:
    return {
        'content': [{
            'type': 'text',
            'text': json.dumps(result, indent=2, sort_keys=True),
        }],
        'structuredContent': result,
        'isError': is_error,
    }


def _handle(request: Dict):
    method = request.get('method')
    if method == 'initialize':
        return {
            'protocolVersion': PROTOCOL_VERSION,
            'capabilities': {'tools': {'listChanged': False}},
            'serverInfo': {'name': 'rosmon2', 'version': __version__},
        }
    if method == 'ping':
        return {}
    if method == 'tools/list':
        return {'tools': list(TOOLS.values())}
    if method == 'tools/call':
        params = request.get('params') or {}
        name = params.get('name')
        try:
            return _tool_result(_call_tool(name, params.get('arguments') or {}))
        except (ControlError, ValueError) as exc:
            return _tool_result({'ok': False, 'error': str(exc)}, is_error=True)
    if method and method.startswith('notifications/'):
        return None
    raise KeyError(f'unsupported MCP method: {method}')


def _response(request: Dict) -> Dict:
    request_id = request.get('id')
    try:
        if request.get('jsonrpc') != '2.0':
            raise ValueError('jsonrpc must be "2.0"')
        result = _handle(request)
        if request_id is None:
            return None
        return {'jsonrpc': '2.0', 'id': request_id, 'result': result}
    except KeyError as exc:
        if request_id is None:
            return None
        return {
            'jsonrpc': '2.0',
            'id': request_id,
            'error': {'code': -32601, 'message': str(exc)},
        }
    except (TypeError, ValueError) as exc:
        if request_id is None:
            return None
        return {
            'jsonrpc': '2.0',
            'id': request_id,
            'error': {'code': -32602, 'message': str(exc)},
        }


def main() -> int:
    """Run an MCP server using newline-delimited JSON-RPC over stdio."""
    for line in sys.stdin:
        try:
            request = json.loads(line)
            if not isinstance(request, dict):
                raise ValueError('request must be a JSON object')
            response = _response(request)
        except (json.JSONDecodeError, ValueError) as exc:
            response = {
                'jsonrpc': '2.0',
                'id': None,
                'error': {'code': -32700, 'message': str(exc)},
            }
        if response is not None:
            sys.stdout.write(json.dumps(response, separators=(',', ':')) + '\n')
            sys.stdout.flush()
    return 0


if __name__ == '__main__':
    sys.exit(main())
