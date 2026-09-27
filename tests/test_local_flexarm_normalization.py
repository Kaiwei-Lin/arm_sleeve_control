from __future__ import annotations

import json

import numpy as np
import pytest

from sleeve_arm.estimation.flexarm.calibration import FlexCalibration
from sleeve_arm.estimation.flexarm.normalization import FlexNormalizer


def test_calibration_uses_median_baseline() -> None:
    calibration = FlexCalibration.fit(
        [[10, 20, 30], [11, 21, 31], [999, 999, 999]],
        scales=[10, 20, 40],
    )
    assert calibration.baseline == (11.0, 21.0, 31.0)
    assert calibration.scale == (10.0, 20.0, 40.0)


def test_normalization_is_independent_of_future_values() -> None:
    calibration = FlexCalibration.fit(
        [[100, 200, 300], [102, 198, 302]],
        scales=[10, 20, 10],
    )
    normalizer = FlexNormalizer(calibration)
    before = normalizer.normalize([105, 190, 310])
    _future = [[900, 900, 900], [1, 1, 1]]
    after = normalizer.normalize([105, 190, 310])
    np.testing.assert_allclose(before, [0.4, -0.45, 0.9])
    np.testing.assert_allclose(after, before)


def test_fit_requires_explicit_training_scales() -> None:
    with pytest.raises(ValueError, match="scales"):
        FlexCalibration.fit([[1, 2, 3], [2, 3, 4]])


@pytest.mark.parametrize(
    "samples",
    [
        [[1, 2, 3]],
        [[1, 2, float("nan")], [2, 3, 4]],
        [[1, 2], [2, 3]],
    ],
)
def test_fit_rejects_invalid_samples(samples: list[list[float]]) -> None:
    with pytest.raises(ValueError):
        FlexCalibration.fit(samples, scales=[1, 1, 1])


@pytest.mark.parametrize("raw", [[1, 2], [1, 2, float("inf")]])
def test_normalizer_rejects_invalid_input(raw: list[float]) -> None:
    calibration = FlexCalibration.fit([[1, 2, 3], [2, 3, 4]], scales=[1, 1, 1])
    with pytest.raises(ValueError):
        FlexNormalizer(calibration).normalize(raw)


def test_calibration_json_round_trip(tmp_path) -> None:
    calibration = FlexCalibration.fit(
        [[100, 200, 300], [101, 199, 301], [99, 201, 299]],
        scales=[20, 30, 40],
    )
    path = tmp_path / "calibration.json"
    calibration.save(path)
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["flex1"] == {"baseline": 100.0, "scale": 20.0}
    assert FlexCalibration.load(path) == calibration

