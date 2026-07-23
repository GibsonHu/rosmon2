from rosmon2.model import selection_key


def test_selection_keys_match_rosmon_order():
    assert selection_key(0) == 'a'
    assert selection_key(25) == 'z'
    assert selection_key(26) == 'A'
    assert selection_key(51) == 'Z'
    assert selection_key(52) == '0'
    assert selection_key(61) == '9'
    assert selection_key(62) is None
