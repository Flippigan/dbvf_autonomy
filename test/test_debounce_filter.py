from dbvf_autonomy.tag_detector_adapter_node import DebounceFilter, select_best_tag


def test_first_detection_becomes_active():
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    assert f.update(1) == 1


def test_same_tag_stays_active():
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    f.update(1)
    f.update(1)
    assert f.update(1) == 1


def test_none_keeps_previous_active():
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    f.update(1)
    assert f.update(None) == 1


def test_no_switch_below_threshold():
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    for _ in range(5):
        f.update(1)
    # Need 4/5 = 80% for switch. After 3 updates of 2: buffer=[1,1,2,2,2] = 60%
    f.update(2)
    f.update(2)
    result = f.update(2)  # buffer: [1,1,2,2,2] -> 3/5=60% < 80%
    assert result == 1  # Still tag 1


def test_switch_at_threshold():
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    f.update(1)  # buffer=[1], active=1
    f.update(2)  # buffer=[1,2], 2: 1/2=50% < 80%
    f.update(2)  # buffer=[1,2,2], 2: 2/3=67% < 80%
    f.update(2)  # buffer=[1,2,2,2], 2: 3/4=75% < 80%
    result = f.update(2)  # buffer=[1,2,2,2,2], 2: 4/5=80% >= 80%
    assert result == 2


def test_switch_back():
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    # Start with tag 2
    for _ in range(5):
        f.update(2)
    assert f.active_id == 2
    # Switch to tag 1
    for _ in range(5):
        f.update(1)
    assert f.active_id == 1


def test_none_does_not_affect_switching():
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    for _ in range(5):
        f.update(1)
    # Insert some Nones and 2s
    f.update(None)  # buffer=[1,1,1,1,None]
    f.update(2)     # buffer=[1,1,1,None,2]
    assert f.active_id == 1  # 2 has 1/5=20%


def test_preferred_id_immediate_switch():
    """Buffer full of tag 1, preferred_id=2 -> immediate switch to 2."""
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    for _ in range(5):
        f.update(1)
    assert f.active_id == 1
    result = f.update(2, preferred_id=2)
    assert result == 2


def test_preferred_id_none_no_bypass():
    """Buffer full of tag 1, preferred_id=None -> normal debounce, stays 1."""
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    for _ in range(5):
        f.update(1)
    result = f.update(2, preferred_id=None)
    assert result == 1


def test_non_preferred_still_debounced():
    """Buffer full of tag 1, preferred_id=1, candidate=2 -> stays 1 (2 != preferred)."""
    f = DebounceFilter(buffer_size=5, threshold=0.8)
    for _ in range(5):
        f.update(1)
    result = f.update(2, preferred_id=1)
    assert result == 1


def _adapter_detect(debounce, detections_by_id, primary_id, secondary_id, preferred_id):
    """Replicate the adapter's _detection_cb logic for testing."""
    candidate_id, _ = select_best_tag(
        detections_by_id, primary_id, secondary_id, preferred_id)
    active_id = debounce.update(candidate_id, preferred_id=preferred_id)

    # Fallback: if debounced tag is gone but another known tag is visible
    if (active_id not in detections_by_id
            and candidate_id is not None
            and candidate_id in detections_by_id):
        debounce.active_id = candidate_id
        active_id = candidate_id

    detected = active_id is not None and active_id in detections_by_id
    return active_id, detected


def test_fallback_when_active_tag_disappears():
    """Primary tag leaves FOV, secondary visible, preferred still primary.

    Without fallback the debounce keeps active_id=1 which is not in frame,
    causing a false detected=False even though tag 2 is right there.
    """
    f = DebounceFilter(buffer_size=30, threshold=0.8)
    for _ in range(30):
        f.update(1)

    active_id, detected = _adapter_detect(
        f, detections_by_id={2: 'det'}, primary_id=1, secondary_id=2,
        preferred_id=1)

    assert active_id == 2
    assert detected is True


def test_fallback_not_triggered_when_active_tag_visible():
    """When debounced active tag is still in frame, no fallback needed."""
    f = DebounceFilter(buffer_size=30, threshold=0.8)
    for _ in range(30):
        f.update(1)

    active_id, detected = _adapter_detect(
        f, detections_by_id={1: 'det1', 2: 'det2'}, primary_id=1,
        secondary_id=2, preferred_id=1)

    assert active_id == 1
    assert detected is True


def test_fallback_no_tags_visible():
    """When no tags are visible, detected remains False."""
    f = DebounceFilter(buffer_size=30, threshold=0.8)
    for _ in range(30):
        f.update(1)

    active_id, detected = _adapter_detect(
        f, detections_by_id={}, primary_id=1, secondary_id=2,
        preferred_id=1)

    assert detected is False
