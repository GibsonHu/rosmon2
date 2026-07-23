import pytest

from rosmon2.cli import resolve_launch_spec


def test_resolve_file_and_arguments(tmp_path):
    launch_file = tmp_path / 'example.launch.py'
    launch_file.write_text('')
    path, arguments = resolve_launch_spec([str(launch_file), 'answer:=42'])
    assert path == str(launch_file.resolve())
    assert arguments == ['answer:=42']


def test_rejects_too_many_specifiers():
    with pytest.raises(ValueError):
        resolve_launch_spec(['one', 'two', 'three'])
