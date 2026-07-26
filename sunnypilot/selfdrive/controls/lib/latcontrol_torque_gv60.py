"""
GV60 torque lateral controller — inverse-EPS feedforward built around the measured car.

Derived from latcontrol_torque_starpilot.py (StarPilot port, branch
claude/starpilot-lateral-controller-sitzu9): the phase machinery (delay-compensated
setpoint buffer, jerk feedforward + low-pass, derivative-on-measurement damping,
integrator release decay, unwind detection, low-speed PID reset) is kept. What changes
is the vehicle model: torque conversion, friction compensation, and gain schedule are
replaced with values measured from this specific 2023 Genesis GV60 Performance AWD.

Measured vehicle model (July 2026 rlog regression, 81 routes / ~6.3M frames):
  1. Gain k(v) — lateral accel per unit normalized torque — is strongly
     speed-dependent (k ~= 0.55 + 0.085*v), not the scalar latAccelFactor the stock
     torque controller assumes.
  2. Apparent lag is amplitude-dependent (EPS torque-rate limiting), pooling to
     ~168 ms but ~330 ms at low speed.
  3. Friction/hysteresis half-width is ~0.078 normalized torque.

Design: the feedforward converts desired lateral accel to torque through the measured
model (inverse-EPS FF), so feedback only handles disturbances instead of masking model
error. Online/real-time NN adaptation in the control loop was explicitly rejected;
if residuals remain after validation, the escalation path is sunnypilot's offline
NNFF (frozen weights), out of scope here.

Like the StarPilot port, this controller is standalone and does NOT use the sunnypilot
LatControlTorqueExt (NNLC / NNFF-lite) extension: it is instantiated for plumbing
compatibility but extension.update() is never called.
"""
import math
import numpy as np
from collections import deque

from cereal import log
from opendbc.car.lateral import get_friction
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.pid import PIDController
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext import LatControlTorqueExt

from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_vehicle_tunes import (
  STARPILOT_LOW_SPEED_PID_RESET_SPEED,
  get_starpilot_friction_threshold,
  get_starpilot_highway_output_taper_scale,
  get_starpilot_highway_transition_output_taper_scale,
  get_starpilot_low_speed_angle_assist_torque,
)

# =============================================================================
# Measured GV60 vehicle model — July 2026 rlog regression (81 routes, ~6.3M
# frames; laf_from_rlogs.py windowed gain regression + lag_and_viz.py).
# These constants ARE the controller: change them only against new measurements.
# =============================================================================

# --- Gain k(v): achieved lateral accel (m/s^2, roll-compensated) per unit
# normalized torque, steady state. Measured bins (v m/s -> k):
#   7.5 -> 1.18, 12.5 -> 2.22, 17.5 -> 2.26, 22.5 -> 2.62, 27.5 -> 3.10, 36.5 -> 3.56
# Linear fit k(v) ~= 0.55 + 0.085*v. Endpoints extrapolated; floored at 1.0 below
# 5 m/s to avoid divide-by-small where the low-speed angle assist should own the
# regime. Steady-state closed-loop measurement slightly underestimates pure gain
# (friction eats some); the 12.5 m/s bin was the noisiest in every run.
LAF_SPEEDS = [0.0, 5.0, 7.5, 12.5, 17.5, 22.5, 27.5, 36.5, 45.0]
LAF_GAINS  = [1.0, 1.0, 1.3,  2.0,  2.3,  2.6,  3.0,  3.6,  4.2]

# --- Friction / hysteresis: measured half-width in normalized torque
# (~0.19 m/s^2 lat-accel-equivalent at 20 m/s).
FRICTION_TORQUE = 0.078
# smooth_sign(rate) = tanh(rate / FRICTION_RATE_SCALE): full compensation once the
# desired torque rate clearly commits to a direction, near-zero at rest so the
# term cannot chatter on straight-road lane-keeping noise. 0.05 torque/s reaches
# ~76% compensation at a 0.1/s desired rate.
FRICTION_RATE_SCALE = 0.05    # normalized torque per second
# Low-pass on the desired-torque rate driving the hysteresis compensation (and,
# later, the slew lead) — raw d/dt of the FF at 100 Hz is too noisy to sign.
FF_RATE_FILTER_CUTOFF_HZ = 2.0
# A/B flag: True restores the error-driven get_friction term (StarPilot/stock
# style) instead of the hysteresis-model compensation.
USE_ERROR_FRICTION = False

# =============================================================================
# Controller constants
# =============================================================================

KP = 0.6
KI = 0.35
KD = 0.0

# Gain schedule. The StarPilot port's KP table was tuned against a torque conversion
# using the learner's scalar latAccelFactor (~3.15 on this car). The conversion now
# goes through the measured k(v), which is ~1.0 at low speed - roughly 3x smaller -
# so the same lat-accel-space KP applies ~3x more torque per unit error down there.
#
# Fix: define the schedule in TORQUE space (the quantity that physically sets loop
# gain) and scale by k(v) to get the lat-accel-space gain the PID expects. Single
# source of truth: when LAF_GAINS moves, KP follows and the torque-loop gain holds.
# Numerically identical to the hand-computed phase (d) table.
LAF_REFERENCE = 3.15   # learner's typical scalar latAccelFactor (July 2026 routes)
INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP_STARPILOT = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, KP]  # pre-rescale, A/B reference
KP_TORQUE_INTERP = [g / LAF_REFERENCE for g in KP_INTERP_STARPILOT]
KP_INTERP = [g * float(np.interp(v, LAF_SPEEDS, LAF_GAINS))
             for v, g in zip(INTERP_SPEEDS, KP_TORQUE_INTERP)]

MAX_LAT_JERK_UP = 2.5            # m/s^3

LP_FILTER_CUTOFF_HZ = 1.2
JERK_GAIN = 0.22
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
VERSION = 3
UNWIND_D_DES_THRESHOLD = -1.0
UNWIND_LAT_ACCEL_NEAR_ZERO = 0.3
MIN_LATERAL_CONTROL_SPEED = 0.3


def k_gain(v_ego: float) -> float:
  """Measured GV60 gain: lateral accel (m/s^2) per unit normalized torque at v_ego."""
  return float(np.interp(v_ego, LAF_SPEEDS, LAF_GAINS))


class LatControlTorque(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.torque_params = CP.lateralTuning.torque.as_builder()
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
    # Desired (FF) torque rate — drives the friction hysteresis compensation.
    self.prev_ff_torque = 0.0
    self.ff_torque_rate_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * FF_RATE_FILTER_CUTOFF_HZ), self.dt)

    # Live torque learner values — logged, not applied (see update_live_torque_params).
    self.live_lat_accel_factor = float(self.torque_params.latAccelFactor)
    self.live_friction = float(self.torque_params.friction)

    self.low_speed_reset_threshold = max(CP.minSteerSpeed, MIN_LATERAL_CONTROL_SPEED)
    self.low_speed_reset_threshold = min(self.low_speed_reset_threshold, STARPILOT_LOW_SPEED_PID_RESET_SPEED)

    self.set_speed_limits(0.0)

    # The sunnypilot control loop feeds the lateral extension every cycle
    # (extension.update_model_v2 / update_lateral_lag / update_limits). We instantiate it
    # for plumbing compatibility, but never call extension.update() in our update() loop,
    # so NNLC / NNFF-lite never alter this controller's output.
    self.extension = LatControlTorqueExt(self, CP, CP_SP, CI)

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    # latAccelOffset is applied live (roll-compensation bias correction, lat-accel space).
    self.torque_params.latAccelOffset = latAccelOffset
    # The learner's scalar latAccelFactor is IGNORED for the conversion path: it is
    # the speed-averaged fiction this controller replaces with the measured k(v)
    # table. The learner's friction is logged but not applied initially (the
    # measured FRICTION_TORQUE hysteresis model owns that term). Both are kept
    # here for pid_log/analysis visibility only.
    self.live_lat_accel_factor = latAccelFactor
    self.live_friction = friction

  def update_gain_learner(self, v_ego: float, measured_gain: float) -> None:
    """PHASE 2 stub — speed-binned gain learner (torqued-style).

    Intended design: one slow estimator per LAF_SPEEDS point, updating LAF_GAINS
    from steady-state (low-jerk, engaged, unsaturated) frames, clamped to +/-20%
    of the July 2026 measured values. Interface stubbed so the validation
    pipeline can call it; intentionally not implemented in the first commit
    series — the static measured table must be validated on-road first.
    """
    pass

  def set_speed_limits(self, v_ego: float):
    # PID operates in lateral-accel space; the lat accel reachable at full torque
    # is speed-dependent through the measured gain, so limits must track k(v).
    max_lataccel = self.steer_max * k_gain(v_ego)
    self.pid.set_limits(max_lataccel, -max_lataccel)

  def update_limits(self):
    # Kept for loop-plumbing compatibility (controlsd calls this via the
    # extension path on some branches). Real limit tracking happens per-cycle in
    # set_speed_limits(v_ego) inside update().
    self.set_speed_limits(self.pid.speed)

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
      self.prev_ff_torque = 0.0
      self.ff_torque_rate_filter.x = 0.0
    else:
      if self.prev_steering_pressed and not CS.steeringPressed:
        self.pid.i *= self.steer_release_i_decay

      k_v = k_gain(CS.vEgo)
      # PID limits must track the speed-dependent gain — set before pid.update.
      self.set_speed_limits(CS.vEgo)

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

      error = setpoint - measurement
      pid_log.error = float(error)

      # Inverse-EPS feedforward in lateral-accel space; conversion to torque via
      # the measured k(v) happens once, at the end.
      ff = gravity_adjusted_future_lateral_accel
      # latAccelOffset corrects roll compensation bias from device roll misalignment relative to car roll
      ff -= self.torque_params.latAccelOffset

      # Friction. Default: hysteresis-model compensation in TORQUE space, driven
      # by the direction of the DESIRED torque rate (measured half-width
      # FRICTION_TORQUE) — added after the lat-accel -> torque conversion below.
      # A/B fallback: the error-driven get_friction term in lat-accel space.
      if USE_ERROR_FRICTION:
        friction_threshold = get_starpilot_friction_threshold(CS.vEgo, setpoint, desired_lateral_jerk)
        ff += get_friction(error + JERK_GAIN * desired_lateral_jerk, lateral_accel_deadzone, friction_threshold, self.torque_params)

      # Desired torque and its (filtered) rate, from the FF path only — feedback
      # noise must not drive the hysteresis sign.
      ff_torque = ff / k_v
      ff_torque_rate = self.ff_torque_rate_filter.update((ff_torque - self.prev_ff_torque) / self.dt)
      self.prev_ff_torque = ff_torque

      if CS.vEgo < self.low_speed_reset_threshold:
        self.pid.reset()
      freeze_integrator = (steer_limited_by_safety or CS.steeringPressed or
                           CS.vEgo < self.low_speed_reset_threshold or unwind_detected)
      output_lataccel = self.pid.update(pid_log.error, -measurement_rate, speed=CS.vEgo, feedforward=ff, freeze_integrator=freeze_integrator)

      # Torque conversion through the measured gain table — replaces
      # torque_from_lateral_accel(latAccelFactor).
      output_torque = output_lataccel / k_v

      if not USE_ERROR_FRICTION:
        output_torque += FRICTION_TORQUE * math.tanh(ff_torque_rate / FRICTION_RATE_SCALE)

      output_torque = float(np.clip(output_torque, -self.steer_max, self.steer_max))

      # Low-speed angle assist hook (rewrites output torque below ~3.25 m/s)
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
      pid_log.output = float(-output_torque)  # normalized torque — downstream analysis scripts assume this
      pid_log.actualLateralAccel = float(measurement)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.desiredLateralJerk = float(desired_lateral_jerk)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    self.prev_steering_pressed = CS.steeringPressed

    # left is positive in this convention
    return -output_torque, 0.0, pid_log
