from launch_ros.actions import Node

from rosmon2.model import ProcessRecord
from rosmon2.supervisor import Supervisor


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
