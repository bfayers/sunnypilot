"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.vision_controller import SmartCruiseControlVision
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.map_controller import SmartCruiseControlMap


class SmartCruiseControl:
  def __init__(self):
    self.vision = SmartCruiseControlVision()
    self.map = SmartCruiseControlMap()
    self.params = Params()
    self.disable_on_blended = bool(self.params.get("SmartCruiseControlDisableOnBlended", return_default=True))
    self.param_read_counter = 0

  def update_params(self) -> None:
    if self.param_read_counter % 50 == 0:
      self.disable_on_blended = bool(self.params.get("SmartCruiseControlDisableOnBlended", return_default=True))
    self.param_read_counter += 1

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool,
             v_ego: float, a_ego: float, v_cruise: float, e2e_mode: bool = False) -> None:
    self.update_params()
    scc_enabled = long_enabled and not (self.disable_on_blended and e2e_mode)
    self.map.update(scc_enabled, long_override, v_ego, a_ego, v_cruise)
    self.vision.update(sm, scc_enabled, long_override, v_ego, a_ego, v_cruise)
