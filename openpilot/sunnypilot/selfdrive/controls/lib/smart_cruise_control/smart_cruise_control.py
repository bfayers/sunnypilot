"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import openpilot.cereal.messaging as messaging
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.vision_controller import SmartCruiseControlVision
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.map_controller import SmartCruiseControlMap


class SmartCruiseControl:
  def __init__(self):
    self.vision = SmartCruiseControlVision()
    self.map = SmartCruiseControlMap()

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool,
             v_ego: float, a_ego: float, v_cruise: float, e2e_mode: bool = False) -> None:
    scc_enabled = long_enabled and not e2e_mode
    self.map.update(scc_enabled, long_override, v_ego, a_ego, v_cruise)
    self.vision.update(sm, scc_enabled, long_override, v_ego, a_ego, v_cruise)
