import math

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.torque_speed_shape import (
  V_REF,
  friction_shape,
  lat_accel_factor_shape,
  scale_torque_params,
)


# Speed-binned learner output used to derive the shape. v=10 bin was treated as noise.
DATA_V              = [6.5, 10.0, 15.0, 21.0, 26.5, 32.0, 37.5]
DATA_LAT_ACCEL_FACT = [1.986, 2.611, 2.729, 2.962, 3.025, 3.064, 3.039]
DATA_FRICTION       = [0.168, 0.130, 0.108, 0.092, 0.096, 0.089, 0.099]
DATA_VALID          = [True, True, True, True, True, False, True]

# Pick a high-speed bin as the reference for ratios — the learner's scalar is
# what the controller would converge to at that speed.
REF_BIN = 4  # 26.5 m/s, closest valid bin to V_REF=25
LAT_ACCEL_FACT_AT_REF = DATA_LAT_ACCEL_FACT[REF_BIN]
FRICTION_AT_REF = DATA_FRICTION[REF_BIN]


class FakeTorqueParams:
  def __init__(self, lat_accel_factor, lat_accel_offset, friction):
    self.latAccelFactor = lat_accel_factor
    self.latAccelOffset = lat_accel_offset
    self.friction = friction


class TestTorqueSpeedShape:

  def test_unity_at_reference_speed(self):
    assert lat_accel_factor_shape(V_REF) == pytest.approx(1.0, abs=1e-12)
    assert friction_shape(V_REF) == pytest.approx(1.0, abs=1e-12)

  def test_lat_accel_factor_monotonic_increasing(self):
    speeds = np.linspace(0.0, 50.0, 101)
    shape = [lat_accel_factor_shape(v) for v in speeds]
    diffs = np.diff(shape)
    assert (diffs > 0).all(), "lat_accel_factor_shape must be strictly increasing"

  def test_friction_monotonic_decreasing(self):
    speeds = np.linspace(0.0, 50.0, 101)
    shape = [friction_shape(v) for v in speeds]
    diffs = np.diff(shape)
    assert (diffs < 0).all(), "friction_shape must be strictly decreasing"

  def test_low_speed_lat_accel_factor_is_attenuating(self):
    # At low speed the modifier must scale latAccelFactor DOWN (less lat accel per torque
    # means the controller commands MORE torque per unit desired lat accel).
    assert lat_accel_factor_shape(5.0) < 0.8
    assert lat_accel_factor_shape(10.0) < 1.0

  def test_low_speed_friction_is_amplifying(self):
    # At low speed the friction term must be boosted.
    assert friction_shape(5.0) > 1.5
    assert friction_shape(10.0) > 1.1

  def test_high_speed_shapes_are_near_unity(self):
    # Above ~30 m/s the modifier should be a near-no-op.
    for v in (30.0, 35.0, 40.0):
      assert abs(lat_accel_factor_shape(v) - 1.0) < 0.05
      assert abs(friction_shape(v) - 1.0) < 0.05

  def test_negative_speed_clamped(self):
    # vEgo can briefly be slightly negative; shape must remain finite and bounded.
    for v in (-5.0, -0.1, 0.0):
      assert math.isfinite(lat_accel_factor_shape(v))
      assert math.isfinite(friction_shape(v))
      assert lat_accel_factor_shape(v) == lat_accel_factor_shape(0.0)
      assert friction_shape(v) == friction_shape(0.0)

  def test_matches_speed_binned_learner_data(self):
    # Apply the modifier to the reference-speed scalar and compare against the
    # learner's per-bin output. This is the "does the shape match reality" check.
    for v, observed_factor, observed_friction, valid in zip(
        DATA_V, DATA_LAT_ACCEL_FACT, DATA_FRICTION, DATA_VALID, strict=True):
      if not valid:
        continue
      predicted_factor = LAT_ACCEL_FACT_AT_REF * lat_accel_factor_shape(v)
      predicted_friction = FRICTION_AT_REF * friction_shape(v)
      # 12% tolerance — the v=10 bin is a known outlier and dominates residuals.
      assert predicted_factor == pytest.approx(observed_factor, rel=0.12), \
        f"latAccelFactor at v={v}: predicted {predicted_factor:.3f}, observed {observed_factor:.3f}"
      assert predicted_friction == pytest.approx(observed_friction, rel=0.25), \
        f"friction at v={v}: predicted {predicted_friction:.3f}, observed {observed_friction:.3f}"

  def test_scale_torque_params_passes_offset_through(self):
    params = FakeTorqueParams(lat_accel_factor=2.5, lat_accel_offset=0.04, friction=0.1)
    scaled = scale_torque_params(params, V_REF)
    assert scaled.latAccelOffset == 0.04

  def test_scale_torque_params_no_op_at_reference_speed(self):
    params = FakeTorqueParams(lat_accel_factor=2.5, lat_accel_offset=0.0, friction=0.1)
    scaled = scale_torque_params(params, V_REF)
    assert scaled.latAccelFactor == pytest.approx(2.5, abs=1e-9)
    assert scaled.friction == pytest.approx(0.1, abs=1e-9)

  def test_scale_torque_params_scales_correctly(self):
    params = FakeTorqueParams(lat_accel_factor=2.5, lat_accel_offset=0.0, friction=0.1)
    scaled = scale_torque_params(params, 6.5)
    assert scaled.latAccelFactor == pytest.approx(2.5 * lat_accel_factor_shape(6.5))
    assert scaled.friction == pytest.approx(0.1 * friction_shape(6.5))
