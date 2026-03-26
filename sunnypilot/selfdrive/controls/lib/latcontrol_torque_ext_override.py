"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from openpilot.common.params import Params


class LatControlTorqueExtOverride:
  def __init__(self, CP):
    self.CP = CP
    self.params = Params()
    self.enforce_torque_control_toggle = self.params.get_bool("EnforceTorqueControl")  # only during init
    self.torque_override_enabled = self.params.get_bool("TorqueParamsOverrideEnabled")
    self.frame = -1

    # Speed-dep state (set by LatControlTorqueExt subclass)
    self._speed_dep_active = False
    self._speed_dep_speed_bp = []
    self._speed_dep_lat_accel_factor_bp = []
    self._speed_dep_friction_bp = []
    self._last_vego = 0.0

  def update_speed_dep_torque(self, tp):
    """Apply speed-dependent learned values from torqued.
    Uses learned values for valid bins, global filtered as fallback."""
    speed_bp = list(tp.speedBinCenters)
    if not speed_bp:
      self._speed_dep_active = False
      return

    self._speed_dep_active = True
    self._speed_dep_speed_bp = speed_bp
    self._speed_dep_lat_accel_factor_bp = [tp.speedBinLatAccelFactors[i] if tp.speedBinValid[i] else tp.latAccelFactorFiltered for i in range(len(speed_bp))]
    self._speed_dep_friction_bp = [tp.speedBinFrictions[i] if tp.speedBinValid[i] else tp.frictionCoefficientFiltered for i in range(len(speed_bp))]
    self._speed_dep_cal_perc = min(tp.speedBinCalPerc) if len(tp.speedBinCalPerc) else 0.0

  def update_override_torque_params(self, torque_params) -> bool:
    changed = False

    # Speed-dep latAccelFactor and friction: interpolate by current speed each frame.
    # Must run here (before get_friction and torque_from_lateral_accel use
    # torque_params) because extension.update() runs after those calls.
    if self._speed_dep_active and self._speed_dep_speed_bp:
      new_lat_accel_factor = float(np.interp(self._last_vego, self._speed_dep_speed_bp, self._speed_dep_lat_accel_factor_bp))
      new_fric = float(np.interp(self._last_vego, self._speed_dep_speed_bp, self._speed_dep_friction_bp))
      if new_lat_accel_factor != torque_params.latAccelFactor or new_fric != torque_params.friction:
        torque_params.latAccelFactor = new_lat_accel_factor
        torque_params.friction = new_fric
        changed = True

    if not self.enforce_torque_control_toggle:
      return changed

    self.frame += 1
    if self.frame % 300 == 0:
      self.torque_override_enabled = self.params.get_bool("TorqueParamsOverrideEnabled")

      if not self.torque_override_enabled:
        return changed

      torque_params.latAccelFactor = float(self.params.get("TorqueParamsOverrideLatAccelFactor", return_default=True))
      torque_params.friction = float(self.params.get("TorqueParamsOverrideFriction", return_default=True))
      return True

    return changed
