import numpy as np
from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD

ACCEL_BOOST_MAX = 0.5
ACCEL_BOOST_RATE = 0.05
ACCEL_BOOST_PER_OVERRIDE = 0.25


def get_starting_boost(personality=log.LongitudinalPersonality.standard) -> float:
  if personality == log.LongitudinalPersonality.relaxed or str(personality) == 'relaxed':
    return 0.25 * ACCEL_BOOST_MAX
  elif personality == log.LongitudinalPersonality.aggressive or str(personality) == 'aggressive':
    return 0.75 * ACCEL_BOOST_MAX
  return 0.50 * ACCEL_BOOST_MAX


class AccelBoost:
  def __init__(self, dt=DT_MDL, params=None):
    self.dt = dt
    self.params = params or Params()
    self.enabled = bool(self.params.get("AccelBoost", return_default=True))
    self.frame = -1
    self.value = 0.0
    self.override_boost = 0.0
    self.prev_active = False
    self.prev_personality = None

  def _update_params(self):
    self.frame += 1
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = bool(self.params.get("AccelBoost", return_default=True))

  def update(self, enabled, gas_pressed, model_limited, active=None, personality=log.LongitudinalPersonality.standard):
    self._update_params()

    if not self.enabled:
      self.value = 0.0
      self.override_boost = 0.0
      self.prev_active = False
      self.prev_personality = None
      return

    if active is None:
      active = enabled

    if not enabled or not gas_pressed:
      self.override_boost = 0.0

    if not active:
      self.value = 0.0
    else:
      if not self.prev_active or personality != self.prev_personality:
        self.value = get_starting_boost(personality)
      if enabled and gas_pressed and model_limited:
        increase = min(ACCEL_BOOST_RATE * self.dt, ACCEL_BOOST_PER_OVERRIDE - self.override_boost, ACCEL_BOOST_MAX - self.value)
        self.value += increase
        self.override_boost += increase

    self.prev_active = active
    self.prev_personality = personality

  def apply(self, accel):
    if not self.enabled or self.value == 0.0:
      return accel
    return accel + np.interp(accel, [-1.0, -0.5, 5.0], [0.0, self.value, self.value], right=0.0)
