"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import random
import time

from openpilot.common.parameterized import parameterized

from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import LIMIT_MAX_MAP_DATA_AGE

from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver, ALL_SOURCES
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode, Policy, OffsetType
from openpilot.common.test import OpenpilotTestCase

SpeedLimitSource = custom.LongitudinalPlanSP.SpeedLimit.Source


def create_mock(properties, mocker):
  mock = mocker.MagicMock()
  for _property, value in properties.items():
    setattr(mock, _property, value)
  return mock


def setup_sm_mock(mocker):
  cruise_speed_limit = random.uniform(0, 120)
  live_map_data_limit = random.uniform(0, 120)

  car_state = create_mock({
    'gasPressed': False,
    'brakePressed': False,
    'standstill': False,
  }, mocker)
  car_state_sp = create_mock({
    'speedLimit': cruise_speed_limit,
  }, mocker)
  live_map_data = create_mock({
    'speedLimit': live_map_data_limit,
    'speedLimitValid': True,
    'speedLimitAhead': 0.,
    'speedLimitAheadValid': 0.,
    'speedLimitAheadDistance': 0.,
  }, mocker)
  gps_data = create_mock({
    'unixTimestampMillis': time.monotonic() * 1e3,
  }, mocker)
  sm_mock = mocker.MagicMock()
  sm_mock.__getitem__.side_effect = lambda key: {
    'carState': car_state,
    'liveMapDataSP': live_map_data,
    'carStateSP': car_state_sp,
    'gpsLocation': gps_data,
  }[key]
  return sm_mock


parametrized_policies = parameterized.expand(
  [
    (Policy.car_state_only, 'carStateSP', SpeedLimitSource.car),
    (Policy.car_state_priority, 'carStateSP', SpeedLimitSource.car),
    (Policy.map_data_only, 'liveMapDataSP', SpeedLimitSource.map),
    (Policy.map_data_priority, 'liveMapDataSP', SpeedLimitSource.map),
  ],
  names=["policy", "sm_key", "function_key"]
)


def resolver_class():
  return SpeedLimitResolver


class TestSpeedLimitResolverValidation(OpenpilotTestCase):

  @parameterized.expand(list(Policy), names=["policy"])
  def test_initial_state(self, resolver_class, policy):
    resolver = resolver_class()
    resolver.policy = policy
    for source in ALL_SOURCES:
      if source in resolver.limit_solutions:
        assert resolver.limit_solutions[source] == 0.
        assert resolver.distance_solutions[source] == 0.

  @parametrized_policies
  def test_resolver(self, resolver_class, policy, sm_key, function_key, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = setup_sm_mock(mocker)
    source_speed_limit = sm_mock[sm_key].speedLimit

    # Assert the resolver
    resolver.update(source_speed_limit, sm_mock)
    assert resolver.speed_limit == source_speed_limit
    assert resolver.source == ALL_SOURCES[function_key]

  def test_resolver_combined(self, resolver_class, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.combined
    sm_mock = setup_sm_mock(mocker)
    socket_to_source = {'carStateSP': SpeedLimitSource.car, 'liveMapDataSP': SpeedLimitSource.map}
    minimum_key, minimum_speed_limit = min(
      ((key, sm_mock[key].speedLimit) for key in
       socket_to_source.keys()), key=lambda x: x[1])

    # Assert the resolver
    resolver.update(minimum_speed_limit, sm_mock)
    assert resolver.speed_limit == minimum_speed_limit
    assert resolver.source == socket_to_source[minimum_key]

  @parametrized_policies
  def test_parser(self, resolver_class, policy, sm_key, function_key, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = setup_sm_mock(mocker)
    source_speed_limit = sm_mock[sm_key].speedLimit

    # Assert the parsing
    resolver.update(source_speed_limit, sm_mock)
    assert resolver.limit_solutions[ALL_SOURCES[function_key]] == source_speed_limit
    assert resolver.distance_solutions[ALL_SOURCES[function_key]] == 0.

  @parameterized.expand(list(Policy), names=["policy"])
  def test_resolve_interaction_in_update(self, resolver_class, policy, mocker):
    v_ego = 50
    resolver = resolver_class()
    resolver.policy = policy

    sm_mock = setup_sm_mock(mocker)
    resolver.update(v_ego, sm_mock)

    # After resolution
    assert resolver.speed_limit is not None
    assert resolver.distance is not None
    assert resolver.source is not None

  @parameterized.expand(list(Policy), names=["policy"])
  def test_old_map_data_ignored(self, resolver_class, policy, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = mocker.MagicMock()
    sm_mock['gpsLocation'].unixTimestampMillis = (time.monotonic() - 2 * LIMIT_MAX_MAP_DATA_AGE) * 1e3
    resolver._get_from_map_data(sm_mock)
    assert resolver.limit_solutions[SpeedLimitSource.map] == 0.
    assert resolver.distance_solutions[SpeedLimitSource.map] == 0.


class TestSpeedLimitMaxSpeed(OpenpilotTestCase):
  """The maximum speed caps the target derived from the speed limit, so roads posted higher are driven at the maximum."""

  def setup_method(self):
    self.params = Params()
    self.params.put_bool("IsMetric", False, block=True)
    self.params.put("SpeedLimitMode", int(Mode.assist), block=True)
    self.params.put("SpeedLimitOffsetType", int(OffsetType.off), block=True)
    self.params.put("SpeedLimitValueOffset", 0, block=True)
    self.params.put_bool("SpeedLimitMaxSpeedEnabled", True, block=True)
    self.params.put("SpeedLimitMaxSpeed", 73, block=True)

  def _resolver(self, resolver_class, speed_limit_mph):
    resolver = resolver_class()
    resolver.speed_limit = speed_limit_mph * CV.MPH_TO_MS
    resolver.speed_limit_offset = resolver._get_speed_limit_offset()
    resolver.update_speed_limit_states()
    return resolver

  def test_speed_limit_above_max_is_capped(self, resolver_class):
    resolver = self._resolver(resolver_class, 80)
    assert round(resolver.speed_limit_final * CV.MS_TO_MPH) == 73
    assert round(resolver.speed_limit_final_last * CV.MS_TO_MPH) == 73
    # the speed limit itself is untouched, only the target we drive to
    assert round(resolver.speed_limit * CV.MS_TO_MPH) == 80
    assert round(resolver.speed_limit_last * CV.MS_TO_MPH) == 80
    # the reported offset reflects what is actually applied
    assert round(resolver.speed_limit_offset * CV.MS_TO_MPH) == -7

  def test_speed_limit_below_max_is_untouched(self, resolver_class):
    resolver = self._resolver(resolver_class, 55)
    assert round(resolver.speed_limit_final * CV.MS_TO_MPH) == 55
    assert resolver.speed_limit_offset == 0.

  def test_offset_is_applied_before_the_cap(self, resolver_class):
    self.params.put("SpeedLimitOffsetType", int(OffsetType.fixed), block=True)
    self.params.put("SpeedLimitValueOffset", 5, block=True)

    # 65 + 5 = 70, still under the maximum
    assert round(self._resolver(resolver_class, 65).speed_limit_final * CV.MS_TO_MPH) == 70
    # 70 + 5 = 75, capped back down to the maximum
    assert round(self._resolver(resolver_class, 70).speed_limit_final * CV.MS_TO_MPH) == 73

  def test_disabled_toggle_does_not_cap(self, resolver_class):
    self.params.put_bool("SpeedLimitMaxSpeedEnabled", False, block=True)
    assert round(self._resolver(resolver_class, 80).speed_limit_final * CV.MS_TO_MPH) == 80

  @parameterized.expand([Mode.off, Mode.information, Mode.warning], names=["mode"])
  def test_only_applies_in_assist_mode(self, resolver_class, mode):
    self.params.put("SpeedLimitMode", int(mode), block=True)
    assert round(self._resolver(resolver_class, 80).speed_limit_final * CV.MS_TO_MPH) == 80

  def test_metric_value_is_read_in_km_h(self, resolver_class):
    self.params.put_bool("IsMetric", True, block=True)
    self.params.put("SpeedLimitMaxSpeed", 110, block=True)

    resolver = resolver_class()
    resolver.speed_limit = 130 * CV.KPH_TO_MS
    resolver.speed_limit_offset = resolver._get_speed_limit_offset()
    resolver.update_speed_limit_states()
    assert round(resolver.speed_limit_final * CV.MS_TO_KPH) == 110
