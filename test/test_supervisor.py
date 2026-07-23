from rosmon2.supervisor import Supervisor


def test_display_names_do_not_include_the_root_slash():
    assert Supervisor._normalize_display_name('/talker') == 'talker'
    assert Supervisor._normalize_display_name('/robot/talker') == 'robot/talker'
    assert Supervisor._normalize_display_name('talker') == 'talker'
