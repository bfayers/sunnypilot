import numpy as np
from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL

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
  def __init__(self, dt=DT_MDL):
    self.dt = dt
    self.value = 0.0
    self.override_boost = 0.0
    self.prev_active = False
    self.prev_personality = None

  def update(self, enabled, gas_pressed, model_limited, active=None, personality=log.LongitudinalPersonality.standard):
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
    return accel + np.interp(accel, [-1.0, -0.5, 5.0], [0.0, self.value, self.value], right=0.0)
