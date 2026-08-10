"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
import numpy as np

from collections import deque

from opendbc.car.structs import car

from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD

RELAXED_MIN_BUCKET_POINTS = np.array([1, 200, 300, 500, 500, 300, 200, 1])

ALLOWED_CARS = ['toyota', 'hyundai', 'rivian', 'honda']

# liveTorqueParameters publishes at 4Hz, so ~240 samples ≈ 1 minute
RAW_AVG_WINDOW = 240
RAW_AVG_PARAM = "LiveTorqueParametersRawAvg"


class TorqueEstimatorExt:
  def __init__(self, CP: car.CarParams):
    self.CP = CP
    self._params = Params()
    self.frame = -1

    self.enforce_torque_control_toggle = self._params.get_bool("EnforceTorqueControl")  # only during init
    self.use_params = self.CP.brand in ALLOWED_CARS and self.CP.lateralTuning.which() == 'torque'
    self.use_live_torque_params = self._params.get_bool("LiveTorqueParamsToggle")
    self.custom_torque_params = self._params.get_bool("CustomTorqueParams")
    self.torque_override_enabled = self._params.get_bool("TorqueParamsOverrideEnabled")
    self.min_bucket_points = RELAXED_MIN_BUCKET_POINTS
    self.factor_sanity = 0.0
    self.friction_sanity = 0.0
    self.offline_latAccelFactor = 0.0
    self.offline_friction = 0.0

    # Rolling ~1-minute windows of the raw estimates, surfaced as a baseline while not liveValid
    self.raw_factor_window: deque[float] = deque(maxlen=RAW_AVG_WINDOW)
    self.raw_friction_window: deque[float] = deque(maxlen=RAW_AVG_WINDOW)
    self.raw_offset_window: deque[float] = deque(maxlen=RAW_AVG_WINDOW)

  def update_raw_average(self, liveTorqueParameters):
    # Only sample when raw estimates are actually being computed (otherwise they're a stale 0.0)
    if not self.filtered_points.is_calculable():
      return
    self.raw_factor_window.append(float(liveTorqueParameters.latAccelFactorRaw))
    self.raw_friction_window.append(float(liveTorqueParameters.frictionCoefficientRaw))
    self.raw_offset_window.append(float(liveTorqueParameters.latAccelOffsetRaw))

  def persist_raw_average(self):
    if not self.raw_factor_window:
      return
    data = {
      "latAccelFactorRawAvg": float(np.mean(self.raw_factor_window)),
      "frictionCoefficientRawAvg": float(np.mean(self.raw_friction_window)),
      "latAccelOffsetRawAvg": float(np.mean(self.raw_offset_window)),
      "samples": len(self.raw_factor_window),
    }
    self._params.put(RAW_AVG_PARAM, json.dumps(data).encode())

  def initialize_custom_params(self, decimated=False):
    self.update_use_params()

    if self.enforce_torque_control_toggle:
      if self._params.get_bool("LiveTorqueParamsRelaxedToggle"):
        self.min_bucket_points = RELAXED_MIN_BUCKET_POINTS / (10 if decimated else 1)
        self.factor_sanity = 0.5 if decimated else 1.0
        self.friction_sanity = 0.8 if decimated else 1.0

      if self._params.get_bool("CustomTorqueParams"):
        self.offline_latAccelFactor = float(self._params.get("TorqueParamsOverrideLatAccelFactor", return_default=True))
        self.offline_friction = float(self._params.get("TorqueParamsOverrideFriction", return_default=True))

  def _update_params(self):
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.use_live_torque_params = self._params.get_bool("LiveTorqueParamsToggle")
      self.custom_torque_params = self._params.get_bool("CustomTorqueParams")
      self.torque_override_enabled = self._params.get_bool("TorqueParamsOverrideEnabled")

  def update_use_params(self):
    self._update_params()

    if self.enforce_torque_control_toggle:
      if self.custom_torque_params and self.torque_override_enabled:
        self.use_params = False
      else:
        self.use_params = self.use_live_torque_params

    self.frame += 1
