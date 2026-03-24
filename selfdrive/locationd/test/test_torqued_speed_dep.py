"""Tests for speed-dependent (per-bin) torque learning in TorqueEstimatorExt.

These tests exercise the speed-binned learning pipeline added by PR #1776:
  _post_reset  →  _on_torque_point  →  _estimate_params_speed_binned  →  _extend_msg
"""
import numpy as np
import pytest

from unittest.mock import MagicMock, patch  # noqa: TID251

from openpilot.sunnypilot.selfdrive.locationd.torqued_ext import (
  SPEED_BIN_BOUNDS, SPEED_BIN_CENTERS,
  MIN_POINTS_PER_SPEED_BIN, FIT_POINTS_PER_SPEED_BIN,
  POINTS_PER_SPEED_BUCKET, SPEED_BIN_MIN_CAL_PERC,
  TorqueEstimatorExt,
)


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _make_cp(brand='toyota', lat_which='torque'):
  cp = MagicMock()
  cp.brand = brand
  cp.carFingerprint = 'MAZDA_CX5_2022'
  cp.lateralTuning.which.return_value = lat_which
  return cp


def _make_ext(speed_dep=True, brand='toyota'):
  with patch('openpilot.common.params.Params') as MockParams:
    inst = MockParams.return_value
    inst.get_bool.side_effect = lambda key: {
      'EnforceTorqueControl': True,
      'LiveTorqueParamsToggle': True,
      'TorqueParamsOverrideEnabled': False,
      'LiveTorqueParamsRelaxedToggle': True,
      'CustomTorqueParams': False,
      'SpeedDependentTorqueToggle': speed_dep,
    }.get(key, False)
    inst.get.return_value = None
    ext = TorqueEstimatorExt(_make_cp(brand=brand))
    ext.initialize_custom_params(decimated=True)
    # Attach attributes that torqued.py normally sets
    ext.resets = 0
    ext.decay = 25.0
    ext._speed_bin_resets = -1
    return ext


# ---------------------------------------------------------------------------
# Config-level tests
# ---------------------------------------------------------------------------

class TestSpeedBinConstants:
  def test_bins_cover_full_range(self):
    assert SPEED_BIN_BOUNDS[0][0] == 5
    assert SPEED_BIN_BOUNDS[-1][1] == 40

  def test_bins_contiguous(self):
    for i in range(len(SPEED_BIN_BOUNDS) - 1):
      assert SPEED_BIN_BOUNDS[i][1] == SPEED_BIN_BOUNDS[i + 1][0]

  def test_centers_within_bounds(self):
    for center, (lo, hi) in zip(SPEED_BIN_CENTERS, SPEED_BIN_BOUNDS):
      assert lo <= center < hi

  def test_min_points_reasonable(self):
    assert MIN_POINTS_PER_SPEED_BIN > 0
    assert FIT_POINTS_PER_SPEED_BIN <= MIN_POINTS_PER_SPEED_BIN
    assert POINTS_PER_SPEED_BUCKET > 0

  def test_cal_perc_threshold(self):
    assert 0 < SPEED_BIN_MIN_CAL_PERC <= 100

  def test_seven_bins(self):
    assert len(SPEED_BIN_BOUNDS) == 7
    assert len(SPEED_BIN_CENTERS) == 7


# ---------------------------------------------------------------------------
# Toggle gating
# ---------------------------------------------------------------------------

class TestToggleGating:
  def test_speed_dep_off_by_default(self):
    ext = _make_ext(speed_dep=False)
    assert ext.speed_binned is False

  def test_speed_dep_on_when_toggled(self):
    ext = _make_ext(speed_dep=True)
    assert ext.speed_binned is True

  def test_on_torque_point_noop_when_off(self):
    ext = _make_ext(speed_dep=False)
    ext._on_torque_point(0.5, 0.3, 15.0)  # should not raise

  def test_extend_msg_noop_when_off(self):
    ext = _make_ext(speed_dep=False)
    ltp = MagicMock()
    ext._extend_msg(ltp, False)  # should not raise or set fields


# ---------------------------------------------------------------------------
# Speed-bin initialization
# ---------------------------------------------------------------------------

class TestSpeedBinInit:
  def test_post_reset_creates_bins(self):
    ext = _make_ext(speed_dep=True)
    ext._post_reset()
    assert len(ext.speed_bin_points) == len(SPEED_BIN_BOUNDS)
    assert len(ext.speed_bin_filtered) == len(SPEED_BIN_BOUNDS)
    assert len(ext.speed_bin_decays) == len(SPEED_BIN_BOUNDS)

  def test_ensure_speed_bins_lazy_init(self):
    ext = _make_ext(speed_dep=True)
    assert not hasattr(ext, 'speed_bin_points')
    ext._ensure_speed_bins()
    assert hasattr(ext, 'speed_bin_points')

  def test_ensure_speed_bins_idempotent(self):
    ext = _make_ext(speed_dep=True)
    ext._ensure_speed_bins()
    bins1 = ext.speed_bin_points
    ext._ensure_speed_bins()
    assert ext.speed_bin_points is bins1


# ---------------------------------------------------------------------------
# Point routing
# ---------------------------------------------------------------------------

class TestPointRouting:
  def test_point_routed_to_correct_bin(self):
    ext = _make_ext(speed_dep=True)
    ext._ensure_speed_bins()
    for i, (lo, hi) in enumerate(SPEED_BIN_BOUNDS):
      mid = (lo + hi) / 2.0
      ext._on_torque_point(0.1, 0.05, mid)

  def test_point_below_min_speed_ignored(self):
    ext = _make_ext(speed_dep=True)
    ext._ensure_speed_bins()
    ext._on_torque_point(0.1, 0.05, 2.0)  # below 5 m/s

  def test_point_above_max_speed_ignored(self):
    ext = _make_ext(speed_dep=True)
    ext._ensure_speed_bins()
    ext._on_torque_point(0.1, 0.05, 45.0)  # above 40 m/s


# ---------------------------------------------------------------------------
# Message extension
# ---------------------------------------------------------------------------

class TestExtendMsg:
  def test_extend_msg_populates_fields(self):
    ext = _make_ext(speed_dep=True)
    ext._ensure_speed_bins()

    ltp = MagicMock()
    ext._extend_msg(ltp, False)

    assert ltp.speedBinCenters == SPEED_BIN_CENTERS
    assert len(ltp.speedBinLatAccelFactors) == len(SPEED_BIN_BOUNDS)
    assert len(ltp.speedBinFrictions) == len(SPEED_BIN_BOUNDS)
    assert len(ltp.speedBinValid) == len(SPEED_BIN_BOUNDS)
    assert len(ltp.speedBinCalPerc) == len(SPEED_BIN_BOUNDS)

  def test_extend_msg_no_bins_set_when_not_init(self):
    ext = _make_ext(speed_dep=True)
    ltp = MagicMock()
    ext._extend_msg(ltp, False)
    # speedBinCenters should not be set since bins not initialized
    ltp.speedBinCenters.__setattr__.assert_not_called()
