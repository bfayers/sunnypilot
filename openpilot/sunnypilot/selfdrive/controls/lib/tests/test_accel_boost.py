"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace
from unittest.mock import MagicMock

from openpilot.common.constants import CV
from openpilot.common.parameterized import parameterized
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase

from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.accel_boost import AccelBoostSP, ACCEL_BOOST_MAX


def make_sm(brake_prob=0.05, gas_prob=0.3, lat_accel=0.0, v_ego=30.0, personality=0, enabled=True, should_stop=False,
            speed_limit_ms=None, speed_limit_valid=True):
  meta = SimpleNamespace(disengagePredictions=SimpleNamespace(brakePressProbs=[0.0, brake_prob, 0.0, 0.0, 0.0, 0.0],
                                                              gasPressProbs=[0.0, gas_prob, 0.0, 0.0, 0.0, 0.0]))
  action = SimpleNamespace(shouldStop=should_stop, desiredCurvature=lat_accel / v_ego ** 2)
  sm = {'modelV2': SimpleNamespace(meta=meta, action=action), 'carState': SimpleNamespace(vEgo=v_ego),
        'selfdriveState': SimpleNamespace(enabled=enabled, personality=personality)}
  if speed_limit_ms is not None:
    sm['liveMapDataSP'] = SimpleNamespace(speedLimit=speed_limit_ms, speedLimitValid=speed_limit_valid)
  return sm


def settle(sm, frames=100):
  boost = AccelBoostSP()
  for _ in range(frames):
    boost.update(sm, 0.0, 9.0, 9.0)
  return boost.total_boost


class TestAccelBoostSP(OpenpilotTestCase):
  @parameterized.expand([(0,), (1,), (2,)])
  def test_personality_max(self, personality):
    assert abs(settle(make_sm(personality=personality)) - ACCEL_BOOST_MAX[personality]) < 1e-6

  def test_brake_gate_closes(self):
    assert settle(make_sm(brake_prob=0.5)) == 0.0

  def test_gas_reopens(self):
    assert abs(settle(make_sm(brake_prob=0.5, gas_prob=0.9)) - ACCEL_BOOST_MAX[0]) < 1e-6

  def test_should_stop(self):
    assert settle(make_sm(gas_prob=0.9, should_stop=True)) == 0.0

  @parameterized.expand([(1.5, 1.0), (2.0, 0.5), (2.5, 0.0), (-2.0, 0.5)])
  def test_curve_fade(self, lat_accel, scale):
    assert abs(settle(make_sm(lat_accel=lat_accel)) - scale * ACCEL_BOOST_MAX[0]) < 1e-6

  def test_low_speed(self):
    assert settle(make_sm(v_ego=0.5)) == 0.0

  def test_disabled_resets(self):
    boost = AccelBoostSP()
    for _ in range(100):
      boost.update(make_sm(), 0.0, 9.0, 9.0)
    assert boost.update(make_sm(enabled=False), 0.0, 9.0, 9.0) == 0.0
    assert boost.boost == 0.0 and boost.total_boost == 0.0

  def test_build_and_release_rates(self):
    boost = AccelBoostSP()
    boost.update(make_sm(personality=1), 0.0, 9.0, 9.0)
    assert abs(boost.total_boost - 0.01) < 1e-6
    for _ in range(100):
      boost.update(make_sm(personality=1), 0.0, 9.0, 9.0)
    boost.update(make_sm(personality=1, brake_prob=0.5), 0.0, 9.0, 9.0)
    assert abs(boost.total_boost - (ACCEL_BOOST_MAX[1] - 0.05)) < 1e-6

  def test_aggressive_starts_at_max_boost(self):
    boost = AccelBoostSP()
    boost.update(make_sm(personality=0), 0.0, 9.0, 9.0)
    assert abs(boost.total_boost - ACCEL_BOOST_MAX[0]) < 1e-6

  def test_personality_switch_to_aggressive_starts_at_max(self):
    boost = AccelBoostSP()
    boost.update(make_sm(personality=1), 0.0, 9.0, 9.0)
    self.assertAlmostEqual(boost.total_boost, 0.01)
    boost.update(make_sm(personality=0), 0.0, 9.0, 9.0)
    self.assertAlmostEqual(boost.total_boost, ACCEL_BOOST_MAX[0])

  def test_map_mode_disabled_boosts_regardless_of_speed_limit(self):
    mock_params = MagicMock()
    mock_params.get_bool.side_effect = lambda k: False
    mock_params.get.side_effect = lambda k, return_default=False: 50
    boost = AccelBoostSP(params=mock_params)
    self.assertFalse(boost.map_mode)

    sm = make_sm(personality=0, speed_limit_ms=20.0 * CV.MPH_TO_MS)
    boost.update(sm, 0.0, 9.0, 9.0)
    self.assertAlmostEqual(boost.total_boost, ACCEL_BOOST_MAX[0])

  def test_map_mode_below_min_speed_gates_boost_off(self):
    mock_params = MagicMock()
    mock_params.get_bool.side_effect = lambda k: True if k == "AccelBoostSPMapMode" else False
    mock_params.get.side_effect = lambda k, return_default=False: 50
    boost = AccelBoostSP(params=mock_params)
    self.assertTrue(boost.map_mode)
    self.assertEqual(boost.map_min_speed, 50)

    sm = make_sm(personality=0, speed_limit_ms=30.0 * CV.MPH_TO_MS)
    boost.update(sm, 0.0, 9.0, 9.0)
    self.assertEqual(boost.total_boost, 0.0)

  def test_map_mode_at_or_above_min_speed_allows_boost(self):
    mock_params = MagicMock()
    mock_params.get_bool.side_effect = lambda k: True if k == "AccelBoostSPMapMode" else False
    mock_params.get.side_effect = lambda k, return_default=False: 50
    boost = AccelBoostSP(params=mock_params)
    self.assertTrue(boost.map_mode)
    self.assertEqual(boost.map_min_speed, 50)

    sm = make_sm(personality=0, speed_limit_ms=55.0 * CV.MPH_TO_MS)
    boost.update(sm, 0.0, 9.0, 9.0)
    self.assertAlmostEqual(boost.total_boost, ACCEL_BOOST_MAX[0])

  def test_map_mode_unmapped_speed_limit_gates_boost_off(self):
    mock_params = MagicMock()
    mock_params.get_bool.side_effect = lambda k: True if k == "AccelBoostSPMapMode" else False
    mock_params.get.side_effect = lambda k, return_default=False: 50
    boost = AccelBoostSP(params=mock_params)

    sm = make_sm(personality=0, speed_limit_ms=0.0, speed_limit_valid=False)
    boost.update(sm, 0.0, 9.0, 9.0)
    self.assertEqual(boost.total_boost, 0.0)

  def test_map_mode_metric_conversion(self):
    mock_params = MagicMock()
    mock_params.get_bool.side_effect = lambda k: True if k in ("AccelBoostSPMapMode", "IsMetric") else False
    mock_params.get.side_effect = lambda k, return_default=False: 80
    boost = AccelBoostSP(params=mock_params)
    self.assertTrue(boost.is_metric)
    self.assertEqual(boost.map_min_speed, 80)

    sm_slow = make_sm(personality=0, speed_limit_ms=70.0 * CV.KPH_TO_MS)
    boost.update(sm_slow, 0.0, 9.0, 9.0)
    self.assertEqual(boost.total_boost, 0.0)

    sm_fast = make_sm(personality=0, speed_limit_ms=90.0 * CV.KPH_TO_MS)
    boost.update(sm_fast, 0.0, 9.0, 9.0)
    self.assertAlmostEqual(boost.total_boost, ACCEL_BOOST_MAX[0])

  def test_map_mode_param_toggle_at_runtime(self):
    map_mode = False
    mock_params = MagicMock()
    mock_params.get_bool.side_effect = lambda k: map_mode if k == "AccelBoostSPMapMode" else False
    mock_params.get.side_effect = lambda k, return_default=False: 50
    boost = AccelBoostSP(params=mock_params)
    self.assertFalse(boost.map_mode)

    sm = make_sm(personality=0, speed_limit_ms=30.0 * CV.MPH_TO_MS)
    boost.update(sm, 0.0, 9.0, 9.0)
    self.assertAlmostEqual(boost.total_boost, ACCEL_BOOST_MAX[0])

    map_mode = True
    boost.frame = int(PARAMS_UPDATE_PERIOD / DT_MDL) - 1
    boost.update_params()
    self.assertTrue(boost.map_mode)
    boost.update(sm, 0.0, 9.0, 9.0)
    self.assertEqual(boost.total_boost, 0.0)

  def test_param_switch(self):
    params = Params()
    boost = AccelBoostSP()
    params.put_bool("AccelBoostSP", True, block=True)
    assert boost.update_params() and boost.enabled
    for _ in range(100):
      boost.update(make_sm(), 0.0, 9.0, 9.0)
    params.put_bool("AccelBoostSP", False, block=True)
    for _ in range(int(PARAMS_UPDATE_PERIOD / DT_MDL) - 1):
      assert not boost.update_params()
    assert boost.update_params() and not boost.enabled
    assert boost.boost == 0.0 and boost.total_boost == 0.0
