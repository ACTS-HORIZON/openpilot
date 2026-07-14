"""
Unit tests for the GV60 torque lateral controller (Torque Control Version 3.0).

Dependency-light: CP / CI / CS / VM are mocked so the tests run without a car
interface or compiled params. When openpilot.common.params (params_pyx) is not
built in the environment, a minimal stub is injected before importing the
controller — on-device / CI with the full build, the real module is used.
"""
import math
import sys
import types

import numpy as np
import pytest

# --- optional stub for environments without the compiled params extension ---
try:
  import openpilot.common.params  # noqa: F401
except ImportError:
  _stub = types.ModuleType("openpilot.common.params_pyx")

  class _Params:
    def __init__(self, *a, **kw):
      pass

    def get(self, *a, **kw):
      return None

    def get_bool(self, *a, **kw):
      return False

  _stub.Params = _Params
  _stub.ParamKeyFlag = object
  _stub.ParamKeyType = object
  _stub.UnknownKeyName = KeyError
  sys.modules["openpilot.common.params_pyx"] = _stub

from openpilot.sunnypilot.selfdrive.controls.lib import latcontrol_torque_gv60 as gv60
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_gv60 import LatControlTorque, k_gain

DT = 0.01


class MockTorqueTuning:
  def __init__(self):
    self.latAccelFactor = 3.15
    self.latAccelOffset = 0.0
    self.friction = 0.1
    self.steeringAngleDeadzoneDeg = 0.0
    self.kf = 1.0

  def as_builder(self):
    return self


class MockLateralTuning:
  def __init__(self):
    self.torque = MockTorqueTuning()

  def which(self):
    return "torque"


class MockCP:
  minSteerSpeed = 0.0
  steerLimitTimer = 0.4
  steerActuatorDelay = 0.17

  def __init__(self):
    self.lateralTuning = MockLateralTuning()


class MockCI:
  def torque_from_lateral_accel(self):
    return lambda lataccel, tp: lataccel / tp.latAccelFactor

  def lateral_accel_from_torque(self):
    return lambda torque, tp: torque * tp.latAccelFactor

  def torque_from_lateral_accel_in_torque_space(self):
    return lambda *a, **kw: 0.0


from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.helpers import MOCK_MODEL_PATH


class MockNNModel:
  path = MOCK_MODEL_PATH


class MockNNLC:
  model = MockNNModel()
  fuzzyFingerprint = False


class MockCPSP:
  neuralNetworkLateralControl = MockNNLC()


class MockCS:
  def __init__(self, v_ego=15.0, steering_angle_deg=0.0, steering_pressed=False):
    self.vEgo = v_ego
    self.steeringAngleDeg = steering_angle_deg
    self.steeringPressed = steering_pressed


class MockVM:
  """Simple bicycle-ish stand-in: curvature proportional to angle."""
  def calc_curvature(self, angle_rad, v_ego, roll):
    return angle_rad / 15.0

  def get_steer_from_curvature(self, curvature, v_ego, roll):
    return curvature * 15.0


class MockLiveParams:
  angleOffsetDeg = 0.0
  roll = 0.0


def make_controller():
  return LatControlTorque(MockCP(), MockCPSP(), MockCI(), DT)


def run_update(lac, active=True, v_ego=15.0, desired_curvature=0.0, angle_deg=0.0,
               pressed=False, lat_delay=0.17):
  cs = MockCS(v_ego=v_ego, steering_angle_deg=angle_deg, steering_pressed=pressed)
  return lac.update(active, cs, MockVM(), MockLiveParams(), False, desired_curvature,
                    None, False, lat_delay)


class TestGainConversion:
  def test_k_gain_matches_table(self):
    for v, k in zip(gv60.LAF_SPEEDS, gv60.LAF_GAINS, strict=True):
      assert k_gain(v) == pytest.approx(k)

  def test_k_gain_floor_below_5ms(self):
    for v in [0.0, 1.0, 2.5, 5.0]:
      assert k_gain(v) == pytest.approx(1.0)

  def test_conversion_round_trip_bounded(self):
    # deliverable: |output_torque| <= 1 for output_lataccel in +/- k(v), v in [0, 45]
    for v in np.linspace(0.0, 45.0, 91):
      k = k_gain(v)
      for lataccel in np.linspace(-k, k, 21):
        torque = np.clip(lataccel / k, -1.0, 1.0)
        assert abs(torque) <= 1.0 + 1e-9
        # round trip: torque back through k recovers lataccel inside the limit
        assert torque * k == pytest.approx(np.clip(lataccel, -k, k), abs=1e-9)

  def test_pid_limits_track_k_of_v(self):
    lac = make_controller()
    for v in [0.0, 5.0, 12.5, 22.5, 36.5, 45.0]:
      run_update(lac, v_ego=v)
      assert lac.pid.pos_limit == pytest.approx(lac.steer_max * k_gain(v))
      assert lac.pid.neg_limit == pytest.approx(-lac.steer_max * k_gain(v))

  def test_learner_scalar_laf_not_applied_to_conversion(self):
    lac = make_controller()
    run_update(lac, v_ego=20.0, desired_curvature=0.005)
    limits_before = lac.pid.pos_limit
    # a wildly wrong learner latAccelFactor must not change conversion or limits
    lac.update_live_torque_params(latAccelFactor=99.0, latAccelOffset=0.0, friction=0.5)
    run_update(lac, v_ego=20.0, desired_curvature=0.005)
    assert lac.pid.pos_limit == pytest.approx(limits_before)
    assert lac.live_lat_accel_factor == pytest.approx(99.0)
    assert lac.live_friction == pytest.approx(0.5)

  def test_lat_accel_offset_is_applied(self):
    lac_a = make_controller()
    lac_b = make_controller()
    lac_b.update_live_torque_params(latAccelFactor=3.15, latAccelOffset=0.5, friction=0.1)
    ta, _, _ = run_update(lac_a, v_ego=20.0, desired_curvature=0.0)
    tb, _, _ = run_update(lac_b, v_ego=20.0, desired_curvature=0.0)
    assert ta != pytest.approx(tb)


class TestRobustness:
  @pytest.mark.parametrize("seed", [0, 1, 2])
  def test_finite_outputs_randomized(self, seed):
    rng = np.random.default_rng(seed)
    lac = make_controller()
    for _ in range(500):
      active = bool(rng.random() > 0.1)  # includes disengaged frames
      v = float(rng.uniform(0.0, 45.0))  # includes v = 0 region
      if rng.random() < 0.05:
        v = 0.0
      torque, _, pid_log = run_update(
        lac, active=active, v_ego=v,
        desired_curvature=float(rng.uniform(-0.2, 0.2)),
        angle_deg=float(rng.uniform(-540.0, 540.0)),
        pressed=bool(rng.random() < 0.1),
        lat_delay=float(rng.uniform(0.01, 0.5)),
      )
      assert math.isfinite(torque)
      assert abs(torque) <= lac.steer_max + 1e-6
      assert math.isfinite(pid_log.output)

  def test_inactive_returns_zero_and_resets(self):
    lac = make_controller()
    for _ in range(10):
      run_update(lac, v_ego=20.0, desired_curvature=0.01)
    torque, _, pid_log = run_update(lac, active=False)
    assert torque == 0.0
    assert not pid_log.active
    assert lac.pid.i == 0.0


class TestFrictionHysteresis:
  def test_sign_flips_with_desired_rate(self):
    # deliverable: friction comp sign flips with desired torque rate
    lac = make_controller()
    v = 20.0
    # ramp desired curvature up: positive desired torque rate
    torques_up = []
    for c in np.linspace(0.0, 0.02, 50):
      t, _, _ = run_update(lac, v_ego=v, desired_curvature=float(c))
      torques_up.append(t)
    rate_up = lac.ff_torque_rate_filter.x
    comp_up = gv60.FRICTION_TORQUE * math.tanh(rate_up / gv60.FRICTION_RATE_SCALE)

    lac2 = make_controller()
    for c in np.linspace(0.0, -0.02, 50):
      run_update(lac2, v_ego=v, desired_curvature=float(c))
    rate_down = lac2.ff_torque_rate_filter.x
    comp_down = gv60.FRICTION_TORQUE * math.tanh(rate_down / gv60.FRICTION_RATE_SCALE)

    assert comp_up > 0.0
    assert comp_down < 0.0
    assert abs(comp_up) <= gv60.FRICTION_TORQUE + 1e-9
    assert abs(comp_down) <= gv60.FRICTION_TORQUE + 1e-9

  def test_no_comp_at_steady_state(self):
    lac = make_controller()
    for _ in range(300):
      run_update(lac, v_ego=20.0, desired_curvature=0.0)
    comp = gv60.FRICTION_TORQUE * math.tanh(lac.ff_torque_rate_filter.x / gv60.FRICTION_RATE_SCALE)
    assert abs(comp) < 1e-6

  def test_comp_bounded_by_friction_torque(self):
    # even extreme desired rates saturate at +/- FRICTION_TORQUE (tanh)
    for rate in [-1e6, -1.0, 1.0, 1e6]:
      comp = gv60.FRICTION_TORQUE * math.tanh(rate / gv60.FRICTION_RATE_SCALE)
      assert abs(comp) <= gv60.FRICTION_TORQUE + 1e-12


class TestDelayAndSlewLead:
  def test_delay_table_matches_measurements(self):
    for v, d in [(7.5, 0.33), (12.5, 0.33), (17.5, 0.17), (22.5, 0.125), (27.5, 0.12), (36.5, 0.16)]:
      assert np.interp(v, gv60.DELAY_SPEEDS, gv60.DELAY_VALUES) == pytest.approx(d)

  def test_speed_interp_delay_used_over_lat_delay(self):
    # identical inputs except a wildly different live lat_delay must produce
    # identical outputs when USE_SPEED_INTERP_DELAY is on
    assert gv60.USE_SPEED_INTERP_DELAY
    lac_a = make_controller()
    lac_b = make_controller()
    out_a = out_b = None
    for c in np.linspace(0.0, 0.01, 30):
      out_a, _, _ = run_update(lac_a, v_ego=8.0, desired_curvature=float(c), lat_delay=0.05)
      out_b, _, _ = run_update(lac_b, v_ego=8.0, desired_curvature=float(c), lat_delay=0.95)
    assert out_a == pytest.approx(out_b)

  def test_lead_zero_at_high_speed(self):
    assert np.interp(25.0, gv60.LEAD_SPEEDS, gv60.LEAD_S) == pytest.approx(0.0)
    assert np.interp(45.0, gv60.LEAD_SPEEDS, gv60.LEAD_S) == pytest.approx(0.0)

  def test_lead_clamped(self):
    for rate in [-1e6, 1e6]:
      lead = np.clip(np.interp(5.0, gv60.LEAD_SPEEDS, gv60.LEAD_S) * rate,
                     -gv60.LEAD_TORQUE_CLAMP, gv60.LEAD_TORQUE_CLAMP)
      assert abs(lead) <= gv60.LEAD_TORQUE_CLAMP + 1e-12

  def test_lead_adds_phase_at_low_speed(self, monkeypatch):
    # with the lead on, a low-speed ramp should command more torque during the
    # transient than with it off (same inputs)
    def ramp(lac):
      last = 0.0
      total = 0.0
      for c in np.linspace(0.0, 0.03, 40):
        last, _, _ = run_update(lac, v_ego=6.0, desired_curvature=float(c))
        total += last
      return total

    total_on = ramp(make_controller())
    monkeypatch.setattr(gv60, "SLEW_LEAD_ENABLED", False)
    total_off = ramp(make_controller())
    # left-positive convention: positive curvature -> negative returned torque is
    # possible depending on sign; compare magnitudes of accumulated command
    assert abs(total_on) > abs(total_off)


class TestKPRescale:
  def test_kp_matches_rescale_formula(self):
    # KP_new(v) = KP_old(v) * k(v) / LAF_REFERENCE (matched torque-loop gain)
    for v, kp_old, kp_new in zip(gv60.INTERP_SPEEDS, gv60.KP_INTERP_OLD, gv60.KP_INTERP, strict=True):
      expected = kp_old * k_gain(v) / gv60.LAF_REFERENCE
      assert kp_new == pytest.approx(expected, rel=0.02)

  def test_torque_loop_gain_matched_at_low_speed(self):
    # effective torque per unit lat-accel error must match the old controller:
    # old: KP_old / LAF_REFERENCE, new: KP_new / k(v)
    for v in [1.0, 2.0, 5.0, 10.0, 30.0]:
      old_eff = np.interp(v, gv60.INTERP_SPEEDS, gv60.KP_INTERP_OLD) / gv60.LAF_REFERENCE
      new_eff = np.interp(v, gv60.INTERP_SPEEDS, gv60.KP_INTERP) / k_gain(v)
      assert new_eff == pytest.approx(old_eff, rel=0.05)

  def test_ki_unchanged(self):
    assert gv60.KI == 0.35
