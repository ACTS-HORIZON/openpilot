"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest

from opendbc.car.structs import car

from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext_override import LatControlTorqueExtOverride


class TestLatControlTorqueExtOverride:

  def setup_method(self):
    self.params = Params()
    self.params.put_bool("EnforceTorqueControl", True, block=True)
    self.params.put_bool("TorqueParamsOverrideEnabled", True, block=True)
    self.params.put("TorqueParamsOverrideLatAccelFactor", 2.5, block=True)
    self.params.put("TorqueParamsOverrideFriction", 0.1, block=True)
    self.params.put("TorqueParamsOverrideLatAccelOffset", -0.42, block=True)
    self.override = LatControlTorqueExtOverride(car.CarParams())

  def test_offset_passthrough(self):
    torque_params = car.CarParams.LateralTorqueTuning()
    # frame advances to a multiple of 300 (starts at -1, so 300 increments to reach frame % 300 == 0)
    applied = False
    for _ in range(301):
      applied = self.override.update_override_torque_params(torque_params)
      if applied:
        break

    assert applied
    # capnp stores these as Float32, so compare with float32 precision
    assert torque_params.latAccelFactor == pytest.approx(2.5)
    assert torque_params.friction == pytest.approx(0.1)
    assert torque_params.latAccelOffset == pytest.approx(-0.42)
