from dbvf_autonomy.tag_detector_adapter_node import DebounceFilter


def test_first_detection_becomes_active():
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    assert f.update(0) == 0


def test_same_tag_stays_active():
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    f.update(0)
    f.update(0)
    assert f.update(0) == 0


def test_none_keeps_previous_active():
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    f.update(0)
    assert f.update(None) == 0


def test_no_switch_below_threshold():
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    for _ in range(5):
        f.update(0)
    # Need 4/5 = 80% for switch. After 3 updates of 1: buffer=[0,0,1,1,1] = 60%
    f.update(1)
    f.update(1)
    result = f.update(1)  # buffer: [0,0,1,1,1] -> 3/5=60% < 80%
    assert result == 0  # Still tag 0


def test_switch_at_threshold():
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    f.update(0)  # buffer=[0], active=0
    f.update(1)  # buffer=[0,1], 1: 1/2=50% < 80%
    f.update(1)  # buffer=[0,1,1], 1: 2/3=67% < 80%
    f.update(1)  # buffer=[0,1,1,1], 1: 3/4=75% < 80%
    result = f.update(1)  # buffer=[0,1,1,1,1], 1: 4/5=80% >= 80%
    assert result == 1


def test_switch_back():
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    # Start with tag 1
    for _ in range(5):
        f.update(1)
    assert f.active_id == 1
    # Switch to tag 0
    for _ in range(5):
        f.update(0)
    assert f.active_id == 0


def test_none_does_not_affect_switching():
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    for _ in range(5):
        f.update(0)
    # Insert some Nones and 1s
    f.update(None)  # buffer=[0,0,0,0,None]
    f.update(1)     # buffer=[0,0,0,None,1]
    assert f.active_id == 0  # 1 has 1/5=20%
