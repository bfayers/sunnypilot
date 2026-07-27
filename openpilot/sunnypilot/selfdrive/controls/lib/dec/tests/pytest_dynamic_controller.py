import pytest

from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController

class MockLeadOne:
  def __init__(self, present=False):
    self.present = present

class MockRadarState:
  def __init__(self, present=False):
    self.leadOne = MockLeadOne(present=present)

class MockCarState:
  def __init__(self, vEgo=0.0, vCruise=0.0, standstill=False):
    self.vEgo = vEgo
    self.vCruise = vCruise
    self.standstill = standstill

class MockModelData:
  def __init__(self, valid=True, stop=False):
    size = 33 if valid else 10  # incomplete if invalid
    pos_x = [0.0] * size if stop else [100.0] * size
    self.position = type("Pos", (), {"x": pos_x})()
    self.orientation = type("Ori", (), {"x": [0.0] * size})()

class MockSelfDriveState:
  def __init__(self, experimentalMode=False):
    self.experimentalMode = experimentalMode

class MockLiveMapDataSP:
  def __init__(self, valid=False, speed_limit=0.0):
    self.speedLimitValid = valid
    self.speedLimit = speed_limit

class MockParams:
  def __init__(self, mode=1, map_max_speed=60, is_metric=False):
    self.mode = mode
    self.map_max_speed = map_max_speed
    self.is_metric = is_metric

  def get_bool(self, name):
    if name == "IsMetric":
      return self.is_metric
    return True

  def get(self, name, return_default=False):
    if name == "DynamicExperimentalControl":
      return self.mode
    if name == "DynamicExperimentalControlMapMaxSpeed":
      return self.map_max_speed
    return None

@pytest.fixture
def default_sm():
  sm = {
    'carState': MockCarState(vEgo=10.0, vCruise=20.0),
    'radarState': MockRadarState(present=True),
    'modelV2': MockModelData(valid=True),
    'selfdriveState': MockSelfDriveState(experimentalMode=True),
    'liveMapDataSP': MockLiveMapDataSP(valid=False, speed_limit=0.0),
  }
  return sm

@pytest.fixture
def mock_cp():
  class CP:
    radarUnavailable = False
  return CP()

@pytest.fixture
def mock_mpc():
  class MPC:
    crash_cnt = 0
  return MPC()

# Fake Kalman Filter that always returns a given value
class FakeKalman:
  def __init__(self, value=1.0):
    self.value = value
  def add_data(self, v): pass
  def get_value(self): return self.value
  def get_confidence(self): return 1.0
  def reset_data(self): pass

def test_initial_mode_is_acc(mock_cp, mock_mpc):
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())
  assert controller.mode() == "acc"

def test_standstill_triggers_blended(mock_cp, mock_mpc, default_sm):
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())
  default_sm['carState'].standstill = True
  for _ in range(15):
    controller.update(default_sm)
  assert controller.mode() == "blended"

def test_emergency_blended_on_fcw(mock_cp, mock_mpc, default_sm):
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())
  mock_mpc.crash_cnt = 1  # simulate FCW
  for _ in range(2):
    controller.update(default_sm)
  assert controller.mode() == "blended"

def test_radarless_slowdown_triggers_blended(mock_cp, mock_mpc, default_sm):
  mock_cp.radarUnavailable = True
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=MockParams())

  # Force conditions to simulate slowdown
  controller._slow_down_filter = FakeKalman(value=1.0)  # ty: ignore[invalid-assignment]
  controller._v_ego_kph = 35.0
  default_sm['modelV2'] = MockModelData(valid=False)  # Incomplete trajectory

  for _ in range(3):
    controller.update(default_sm)

  assert controller.mode() == "blended"

def test_dec_map_unmapped_road_triggers_blended(mock_cp, mock_mpc, default_sm):
  params = MockParams(mode=2, map_max_speed=60, is_metric=False)
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=params)

  default_sm['liveMapDataSP'] = MockLiveMapDataSP(valid=False, speed_limit=0.0)
  for _ in range(10):
    controller.update(default_sm)
  assert controller.mode() == "blended"

def test_dec_map_below_threshold_triggers_blended(mock_cp, mock_mpc, default_sm):
  params = MockParams(mode=2, map_max_speed=60, is_metric=False)
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=params)

  # 45 mph in m/s is ~20.11 m/s
  default_sm['liveMapDataSP'] = MockLiveMapDataSP(valid=True, speed_limit=20.1168)
  for _ in range(10):
    controller.update(default_sm)
  assert controller.mode() == "blended"

def test_dec_map_above_threshold_triggers_acc(mock_cp, mock_mpc, default_sm):
  params = MockParams(mode=2, map_max_speed=60, is_metric=False)
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=params)

  # 65 mph in m/s is ~29.05 m/s
  default_sm['liveMapDataSP'] = MockLiveMapDataSP(valid=True, speed_limit=29.0576)
  for _ in range(10):
    controller.update(default_sm)
  assert controller.mode() == "acc"

def test_dec_map_custom_threshold(mock_cp, mock_mpc, default_sm):
  # Custom threshold = 45 mph
  params = MockParams(mode=2, map_max_speed=45, is_metric=False)
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=params)

  # 40 mph limit -> blended
  default_sm['liveMapDataSP'] = MockLiveMapDataSP(valid=True, speed_limit=17.8816)
  for _ in range(10):
    controller.update(default_sm)
  assert controller.mode() == "blended"

  # 50 mph limit -> acc
  default_sm['liveMapDataSP'] = MockLiveMapDataSP(valid=True, speed_limit=22.352)
  for _ in range(10):
    controller.update(default_sm)
  assert controller.mode() == "acc"

def test_dec_map_fcw_overrides_to_blended(mock_cp, mock_mpc, default_sm):
  params = MockParams(mode=2, map_max_speed=60, is_metric=False)
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=params)

  # Highway speed limit (65 mph) normally ACC
  default_sm['liveMapDataSP'] = MockLiveMapDataSP(valid=True, speed_limit=29.0576)
  mock_mpc.crash_cnt = 1  # FCW active
  for _ in range(2):
    controller.update(default_sm)

  assert controller.mode() == "blended"

def test_dec_map_standstill_overrides_to_blended(mock_cp, mock_mpc, default_sm):
  params = MockParams(mode=2, map_max_speed=60, is_metric=False)
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=params)

  default_sm['liveMapDataSP'] = MockLiveMapDataSP(valid=True, speed_limit=29.0576)
  default_sm['carState'].standstill = True
  for _ in range(10):
    controller.update(default_sm)

  assert controller.mode() == "blended"

def test_dec_map_high_urgency_slowdown_overrides_to_blended(mock_cp, mock_mpc, default_sm):
  params = MockParams(mode=2, map_max_speed=60, is_metric=False)
  controller = DynamicExperimentalController(mock_cp, mock_mpc, params=params)

  default_sm['liveMapDataSP'] = MockLiveMapDataSP(valid=True, speed_limit=29.0576)
  default_sm['modelV2'] = MockModelData(valid=True, stop=True)

  for _ in range(2):
    controller.update(default_sm)

  assert controller.mode() == "blended"


