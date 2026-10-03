from openpilot.common.test import OpenpilotTestCase
from unittest.mock import MagicMock
from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.accel_boost import (
  AccelBoost,
  ACCEL_BOOST_MAX,
  get_starting_boost,
)


class TestAccelBoost(OpenpilotTestCase):
  def test_get_starting_boost(self):
    assert get_starting_boost(log.LongitudinalPersonality.relaxed) == 0.25 * ACCEL_BOOST_MAX
    assert get_starting_boost('relaxed') == 0.25 * ACCEL_BOOST_MAX
    assert get_starting_boost(log.LongitudinalPersonality.standard) == 0.50 * ACCEL_BOOST_MAX
    assert get_starting_boost('standard') == 0.50 * ACCEL_BOOST_MAX
    assert get_starting_boost(log.LongitudinalPersonality.aggressive) == 0.75 * ACCEL_BOOST_MAX
    assert get_starting_boost('aggressive') == 0.75 * ACCEL_BOOST_MAX

  def test_enabled_by_default(self):
    mock_params = MagicMock()
    mock_params.get.return_value = True
    ab = AccelBoost(params=mock_params)
    assert ab.enabled is True

    ab.update(enabled=True, gas_pressed=False, model_limited=False, active=True)
    assert ab.value == 0.50 * ACCEL_BOOST_MAX
    assert ab.apply(1.0) > 1.0

  def test_disabled_by_param(self):
    mock_params = MagicMock()
    mock_params.get.return_value = False
    ab = AccelBoost(params=mock_params)
    assert ab.enabled is False

    ab.update(enabled=True, gas_pressed=True, model_limited=True, active=True)
    assert ab.value == 0.0
    assert ab.override_boost == 0.0
    assert ab.apply(1.5) == 1.5

  def test_param_toggle_at_runtime(self):
    mock_params = MagicMock()
    mock_params.get.return_value = True
    ab = AccelBoost(dt=DT_MDL, params=mock_params)
    assert ab.enabled is True

    # Active and accumulating
    ab.update(enabled=True, gas_pressed=True, model_limited=True, active=True)
    assert ab.value > 0.0

    # Simulate disabling param
    mock_params.get.return_value = False
    ab.frame = 59  # Next frame will trigger PARAMS_UPDATE_PERIOD check (60 frames at 20Hz = 3s)
    ab.update(enabled=True, gas_pressed=True, model_limited=True, active=True)
    assert ab.enabled is False
    assert ab.value == 0.0
    assert ab.override_boost == 0.0
    assert ab.apply(2.0) == 2.0
