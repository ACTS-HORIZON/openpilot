"""
StarPilot torque lateral controller — ported StarPilot tune (no per-car gate).

Faithful port of firestar5683/StarPilot branch `Dom` (commit f5a5b9e1, "lat control
refactor") selfdrive/controls/lib/latcontrol_torque.py, reduced to StarPilot's single tune
(seeded from his Ioniq 6 entry) and
adapted to sunnypilot's signatures (CP, CP_SP, CI, dt). Registered as an additional
"Torque Control Version" (see README_starpilot_tune.md). The tune is applied
unconditionally whenever this version is selected — there is no per-car fingerprint gate.

NOTE: this controller is standalone and does NOT use the sunnypilot LatControlTorqueExt
(NNLC / NNFF-lite) extension, matching StarPilot's design.
"""
import math
import numpy as np
from collections import deque

from cereal import log
from opendbc.car.lateral import get_friction
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.pid import PIDController
from openpilot.selfdrive.controls.lib.drive_helpers import MIN_SPEED
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext import LatControlTorqueExt

from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_vehicle_tunes import (
  STARPILOT_LOW_SPEED_PID_RESET_SPEED,
  get_friction_threshold,            # noqa: F401  (kept for parity / future use)
  get_starpilot_center_taper_scale,
  get_starpilot_ff_scale,
  get_starpilot_friction_scale,
  get_starpilot_friction_threshold,
  get_starpilot_highway_output_taper_scale,
  get_starpilot_highway_transition_output_taper_scale,
  get_starpilot_low_speed_angle_assist_torque,
)

# ============================================================================
# Tunable base parameters — dial these on the GV60.
# These are StarPilot's base multipliers (seeded from his Ioniq 6 entry), applied on top of the car's
# OWN latAccelFactor / friction (which come from values.py + the live tuner).
# The finer response curves live in latcontrol_vehicle_tunes.py.
# ============================================================================
STARPILOT_LAT_ACCEL_FACTOR_MULT = 1.22   # StarPilot base; multiplies the car's own latAccelFactor
STARPILOT_FF_MASTER_GAIN = 1.0           # global feedforward scale (<1 softens, >1 sharpens)
STARPILOT_FRICTION_MASTER_GAIN = 1.0     # global friction-response scale

KP = 0.6
KI = 0.35
KD = 0.0

INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, KP]

LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [12, 10.5, 8, 5]
MAX_LAT_JERK_UP = 2.5            # m/s^3

LP_FILTER_CUTOFF_HZ = 1.2
JERK_GAIN = 0.22
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
VERSION = 2
DEADZONE_BOOST_LAT_ACCEL = 0.15
UNWIND_D_DES_THRESHOLD = -1.0
UNWIND_LAT_ACCEL_NEAR_ZERO = 0.3
MIN_LATERAL_CONTROL_SPEED = 0.3


class LatControlTorque(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()
    self.pid = PIDController([INTERP_SPEEDS, KP_INTERP], KI, KD, rate=1/self.dt)
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.lat_accel_request_buffer_len = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / self.dt)
    self.lat_accel_request_buffer = deque([0.] * self.lat_accel_request_buffer_len, maxlen=self.lat_accel_request_buffer_len)
    self.jerk_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)
    self.previous_measurement = 0.0
    self.measurement_rate_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * (MAX_LAT_JERK_UP - 0.5)), self.dt)
    self.steer_release_i_decay = 0.8
    self.prev_steering_pressed = False
    self.prev_desired_lateral_accel = 0.0
    self.torque_deadzone_boost = float(getattr(self.torque_params, "kfDEPRECATED", 0.0))

    # StarPilot base parameter shaping (applied unconditionally for this version)
    self.torque_params.latAccelFactor *= STARPILOT_LAT_ACCEL_FACTOR_MULT
    self.low_speed_reset_threshold = max(CP.minSteerSpeed, MIN_LATERAL_CONTROL_SPEED)
    self.low_speed_reset_threshold = min(self.low_speed_reset_threshold, STARPILOT_LOW_SPEED_PID_RESET_SPEED)

    self.update_limits()

    # The sunnypilot control loop feeds the lateral extension every cycle
    # (extension.update_model_v2 / update_lateral_lag / update_limits). We instantiate it
    # for plumbing compatibility, but never call extension.update() in our update() loop,
    # so NNLC / NNFF-lite never alter the StarPilot output.
    self.extension = LatControlTorqueExt(self, CP, CP_SP, CI)

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    latAccelFactor *= STARPILOT_LAT_ACCEL_FACTOR_MULT
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction
    self.update_limits()

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION
    if not active:
      output_torque = 0.0
      pid_log.active = False
      self.pid.reset()
      self.previous_measurement = 0.0
      self.measurement_rate_filter.x = 0.0
      self.lat_accel_request_buffer = deque([0.] * self.lat_accel_request_buffer_len, maxlen=self.lat_accel_request_buffer_len)
      self.prev_desired_lateral_accel = 0.0
    else:
      if self.prev_steering_pressed and not CS.steeringPressed:
        self.pid.i *= self.steer_release_i_decay

      measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
      curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      delay_frames = int(np.clip(lat_delay / self.dt, 1, self.lat_accel_request_buffer_len))
      expected_lateral_accel = self.lat_accel_request_buffer[-delay_frames]
      future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
      self.lat_accel_request_buffer.append(future_desired_lateral_accel)
      raw_lateral_jerk = (future_desired_lateral_accel - expected_lateral_accel) / max(lat_delay, self.dt)
      raw_lateral_jerk = np.clip(raw_lateral_jerk, -MAX_LAT_JERK_UP, MAX_LAT_JERK_UP)
      desired_lateral_jerk = np.clip(self.jerk_filter.update(raw_lateral_jerk), -MAX_LAT_JERK_UP, MAX_LAT_JERK_UP)
      gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
      setpoint = expected_lateral_accel + desired_lateral_jerk * lat_delay
      desired_lateral_accel_rate = (setpoint - self.prev_desired_lateral_accel) / self.dt
      unwind_detected = (desired_lateral_accel_rate < UNWIND_D_DES_THRESHOLD and
                         abs(setpoint) < UNWIND_LAT_ACCEL_NEAR_ZERO)
      self.prev_desired_lateral_accel = setpoint

      measurement = measured_curvature * CS.vEgo ** 2
      measurement_rate = self.measurement_rate_filter.update((measurement - self.previous_measurement) / self.dt)
      measurement_rate = np.clip(measurement_rate, -MAX_LAT_JERK_UP, MAX_LAT_JERK_UP)
      self.previous_measurement = measurement

      low_speed_factor = (np.interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y) / max(CS.vEgo, MIN_SPEED)) ** 2
      current_kp = np.interp(CS.vEgo, self.pid._k_p[0], self.pid._k_p[1])
      error = setpoint - measurement
      error_with_lsf = error * (1 + low_speed_factor / max(current_kp, 1e-3))

      # do error correction in lateral acceleration space, convert at end to handle non-linear torque responses correctly
      pid_log.error = float(error_with_lsf)
      ff = gravity_adjusted_future_lateral_accel
      # latAccelOffset corrects roll compensation bias from device roll misalignment relative to car roll
      ff -= self.torque_params.latAccelOffset

      # --- StarPilot feedforward / friction shaping ---
      center_taper = get_starpilot_center_taper_scale(setpoint, CS.vEgo)
      ff *= get_starpilot_ff_scale(setpoint, desired_lateral_jerk, CS.vEgo) * center_taper
      friction_threshold = get_starpilot_friction_threshold(CS.vEgo, setpoint, desired_lateral_jerk) / max(center_taper, 1e-3)
      friction_scale = get_starpilot_friction_scale(CS.vEgo, setpoint, desired_lateral_jerk)
      friction_scale = 1.0 + ((friction_scale - 1.0) * center_taper)

      # global tunable dials
      ff *= STARPILOT_FF_MASTER_GAIN
      friction_scale *= STARPILOT_FRICTION_MASTER_GAIN

      ff += friction_scale * get_friction(error_with_lsf + JERK_GAIN * desired_lateral_jerk, lateral_accel_deadzone, friction_threshold, self.torque_params)

      if self.torque_deadzone_boost > 0.0 and abs(gravity_adjusted_future_lateral_accel) < DEADZONE_BOOST_LAT_ACCEL:
        boost_scale = np.interp(abs(gravity_adjusted_future_lateral_accel), [0.0, DEADZONE_BOOST_LAT_ACCEL], [1.0, 0.0])
        ff += np.sign(gravity_adjusted_future_lateral_accel) * self.torque_deadzone_boost * boost_scale

      if CS.vEgo < self.low_speed_reset_threshold:
        self.pid.reset()
      freeze_integrator = (steer_limited_by_safety or CS.steeringPressed or
                           CS.vEgo < self.low_speed_reset_threshold or unwind_detected)
      output_lataccel = self.pid.update(pid_log.error, -measurement_rate, speed=CS.vEgo, feedforward=ff, freeze_integrator=freeze_integrator)
      output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)

      # StarPilot low-speed angle assist (rewrites output torque below ~3.25 m/s)
      if not CS.steeringPressed:
        desired_angle_no_offset = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
        actual_angle_no_offset = CS.steeringAngleDeg - params.angleOffsetDeg
        output_torque = get_starpilot_low_speed_angle_assist_torque(desired_angle_no_offset, actual_angle_no_offset,
                                                                  output_torque, CS.vEgo)
      output_torque *= get_starpilot_highway_output_taper_scale(setpoint, CS.vEgo)
      output_torque *= get_starpilot_highway_transition_output_taper_scale(setpoint, desired_lateral_jerk, CS.vEgo)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque)  # TODO: log lat accel?
      pid_log.actualLateralAccel = float(measurement)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.desiredLateralJerk = float(desired_lateral_jerk)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    self.prev_steering_pressed = CS.steeringPressed

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
