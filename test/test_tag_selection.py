from dbvf_autonomy.tag_detector_adapter_node import select_best_tag


def test_both_tags_prefer_primary():
    detections = {0: 'det_0', 1: 'det_1'}
    tag_id, det = select_best_tag(detections, primary_id=0, secondary_id=1)
    assert tag_id == 0
    assert det == 'det_0'


def test_only_primary():
    detections = {0: 'det_0'}
    tag_id, det = select_best_tag(detections, primary_id=0, secondary_id=1)
    assert tag_id == 0


def test_only_secondary():
    detections = {1: 'det_1'}
    tag_id, det = select_best_tag(detections, primary_id=0, secondary_id=1)
    assert tag_id == 1
    assert det == 'det_1'


def test_no_tags():
    detections = {}
    tag_id, det = select_best_tag(detections, primary_id=0, secondary_id=1)
    assert tag_id is None
    assert det is None


def test_unknown_tag_ignored():
    detections = {5: 'det_5'}
    tag_id, det = select_best_tag(detections, primary_id=0, secondary_id=1)
    assert tag_id is None
    assert det is None


# ---------------------------------------------------------------------------
# Preferred tag selection tests (ISS-012)
# ---------------------------------------------------------------------------

def test_preferred_secondary_when_both_visible():
    detections = {0: 'det_0', 1: 'det_1'}
    tag_id, det = select_best_tag(detections, primary_id=0, secondary_id=1,
                                  preferred_id=1)
    assert tag_id == 1
    assert det == 'det_1'


def test_preferred_primary_when_both_visible():
    detections = {0: 'det_0', 1: 'det_1'}
    tag_id, det = select_best_tag(detections, primary_id=0, secondary_id=1,
                                  preferred_id=0)
    assert tag_id == 0
    assert det == 'det_0'


def test_preferred_not_detected_falls_back():
    detections = {0: 'det_0'}
    tag_id, det = select_best_tag(detections, primary_id=0, secondary_id=1,
                                  preferred_id=1)
    assert tag_id == 0
    assert det == 'det_0'


def test_preferred_none_uses_default():
    detections = {0: 'det_0', 1: 'det_1'}
    tag_id, det = select_best_tag(detections, primary_id=0, secondary_id=1,
                                  preferred_id=None)
    assert tag_id == 0
    assert det == 'det_0'
