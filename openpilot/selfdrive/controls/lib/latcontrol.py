import numpy as np
from abc import abstractmethod, ABC
from openpilot.selfdrive.locationd.helpers import Pose


class LatControl(ABC):
  def __init__(self, CP, CP_SP, CI, dt):
    self.dt = dt
    self.sat_limit = CP.steerLimitTimer
    self.sat_time = 0.
    # BUGFIX(GV60 campaign): stock gate of 10 m/s (22 mph) silences the
    # "Turn Exceeds Steering Limit" alert exactly where low-speed authority
    # (~2.1 m/s^2 at 18 mph) is most easily exceeded. Route 0000089 seg 8:
    # 2 s pinned at full torque, understeering, zero warning. Keep a small
    # gate so parking-lot full-lock doesn't chime.
    self.sat_check_min_speed = 3.

    # we define the steer torque scale as [-1.0...1.0]
    self.steer_max = 1.0

  @abstractmethod
  def update(self, active: bool, CS, VM, params, steer_limited_by_safety: bool, desired_curvature: float, calibrated_pose: Pose | None,
             curvature_limited: bool, lat_delay: float):
    pass

  def reset(self):
    self.sat_time = 0.

  def _check_saturation(self, saturated, CS, steer_limited_by_safety, curvature_limited):
    # Saturated only if control output is not being limited by car torque/angle rate limits
    if (saturated or curvature_limited) and CS.vEgo > self.sat_check_min_speed and not steer_limited_by_safety and not CS.steeringPressed:
      self.sat_time += self.dt
    else:
      self.sat_time -= self.dt
    self.sat_time = np.clip(self.sat_time, 0.0, self.sat_limit)
    return self.sat_time > (self.sat_limit - 1e-3)
