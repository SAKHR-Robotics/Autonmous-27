from marker_detection.quality_flags import QualityFlag, names


def test_valid_flag_has_no_names():
    assert names(int(QualityFlag.VALID)) == []


def test_single_flag_round_trips_to_its_name():
    assert names(int(QualityFlag.HIGH_VIEWING_ANGLE)) == ["HIGH_VIEWING_ANGLE"]


def test_combined_flags_all_present_and_order_matches_declaration():
    combined = int(QualityFlag.LOW_IMAGE_QUALITY | QualityFlag.TRACK_LOST | QualityFlag.DEPTH_MISMATCH)
    result = names(combined)
    assert set(result) == {"LOW_IMAGE_QUALITY", "TRACK_LOST", "DEPTH_MISMATCH"}
    assert result.index("LOW_IMAGE_QUALITY") < result.index("DEPTH_MISMATCH") < result.index("TRACK_LOST")


def test_flags_are_independent_bits():
    a, b = QualityFlag.STALE_DATA, QualityFlag.POSE_AMBIGUOUS
    assert int(a) != int(b)
    assert int(a) & int(b) == 0
    assert int(a | b) == int(a) + int(b)
