"""
StarPilot lateral tune — extracted subset (no per-car gate).

Ported from firestar5683/StarPilot branch `Dom` (commit f5a5b9e1, "lat control refactor"),
file selfdrive/controls/lib/latcontrol_vehicle_tunes.py. Only one tune is kept;
all other vehicles and the testing_grounds gating were stripped. Values were seeded from
StarPilot's Ioniq 6 entry (his closest E-GMP car) and are the starting point for tuning. Consumed by
latcontrol_torque_starpilot.py. See README_starpilot_tune.md for provenance and concerns.
"""
import math
import numpy as np

from openpilot.common.constants import CV


# === StarPilot tune constants (seeded from his Ioniq 6 entry) ===
STARPILOT_FF_GAIN_LEFT = 0.045
STARPILOT_FF_GAIN_RIGHT = 0.015
STARPILOT_BASE_LAT_ACCEL_FACTOR_MULT = 1.22
STARPILOT_BASE_FRICTION_THRESHOLD = 0.36
STARPILOT_FF_ONSET = 0.10
STARPILOT_FF_ONSET_WIDTH = 0.04
STARPILOT_FF_CUTOFF = 0.48
STARPILOT_FF_CUTOFF_WIDTH = 0.12
STARPILOT_TRANSITION_SPEED = 10.0
STARPILOT_PHASE_SCALE = 0.10
STARPILOT_TURN_IN_BOOST_LEFT = 1.64
STARPILOT_TURN_IN_BOOST_RIGHT = 2.10
STARPILOT_UNWIND_TAPER_LEFT = 3.18
STARPILOT_UNWIND_TAPER_RIGHT = 8.20
STARPILOT_FRICTION_MULT = 0.928
STARPILOT_FRICTION_LAT_RISE = 0.20
STARPILOT_FRICTION_JERK_RISE = 0.24
STARPILOT_TURN_IN_THRESHOLD_REDUCTION_LEFT = 0.78
STARPILOT_TURN_IN_THRESHOLD_REDUCTION_RIGHT = 1.42
STARPILOT_UNWIND_THRESHOLD_INCREASE_LEFT = 3.90
STARPILOT_UNWIND_THRESHOLD_INCREASE_RIGHT = 10.20
STARPILOT_TURN_IN_FRICTION_BOOST_LEFT = 0.44
STARPILOT_TURN_IN_FRICTION_BOOST_RIGHT = 0.94
STARPILOT_UNWIND_FRICTION_REDUCTION_LEFT = 3.55
STARPILOT_UNWIND_FRICTION_REDUCTION_RIGHT = 9.10
STARPILOT_CENTER_TAPER_MAX = 0.082
STARPILOT_CENTER_TAPER_LAT = 0.24
STARPILOT_CENTER_TAPER_LAT_WIDTH = 0.025
STARPILOT_CENTER_TAPER_SPEED = 18.0
STARPILOT_CENTER_TAPER_SPEED_WIDTH = 2.5
STARPILOT_HIGHWAY_CENTER_TAPER_MAX = 0.046
STARPILOT_HIGHWAY_CENTER_TAPER_LAT = 0.10
STARPILOT_HIGHWAY_CENTER_TAPER_LAT_WIDTH = 0.035
STARPILOT_HIGHWAY_CENTER_TAPER_SPEED = 24.5
STARPILOT_HIGHWAY_CENTER_TAPER_SPEED_WIDTH = 1.8
STARPILOT_HIGHWAY_OUTPUT_TAPER_MAX = 0.10
STARPILOT_HIGHWAY_OUTPUT_TAPER_LAT = 0.14
STARPILOT_HIGHWAY_OUTPUT_TAPER_LAT_WIDTH = 0.04
STARPILOT_HIGHWAY_OUTPUT_TAPER_SPEED = 23.5
STARPILOT_HIGHWAY_OUTPUT_TAPER_SPEED_WIDTH = 2.0
STARPILOT_HIGHWAY_TRANSITION_OUTPUT_TAPER_MAX = 0.18
STARPILOT_HIGHWAY_TRANSITION_OUTPUT_TAPER_LAT = 1.05
STARPILOT_HIGHWAY_TRANSITION_OUTPUT_TAPER_LAT_WIDTH = 0.22
STARPILOT_HIGHWAY_TRANSITION_OUTPUT_TAPER_JERK = 0.24
STARPILOT_HIGHWAY_TRANSITION_OUTPUT_TAPER_JERK_WIDTH = 0.14
STARPILOT_LOW_MID_CENTER_TAPER_MAX = 0.088
STARPILOT_LOW_MID_CENTER_TAPER_LAT = 0.28
STARPILOT_LOW_MID_CENTER_TAPER_LAT_WIDTH = 0.06
STARPILOT_LOW_MID_CENTER_TAPER_SPEED_MIN = 8.5
STARPILOT_LOW_MID_CENTER_TAPER_SPEED_MAX = 16.5
STARPILOT_LOW_MID_CENTER_TAPER_SPEED_WIDTH = 1.5
STARPILOT_DIRECTIONAL_TAPER_LAT_START = 0.19
STARPILOT_DIRECTIONAL_TAPER_LAT_END = 0.90
STARPILOT_DIRECTIONAL_TAPER_LAT_WIDTH = 0.06
STARPILOT_DIRECTIONAL_TAPER_BASE_LEFT = 0.11
STARPILOT_DIRECTIONAL_TAPER_BASE_RIGHT = 0.45
STARPILOT_DIRECTIONAL_TAPER_UNWIND_LEFT = 2.15
STARPILOT_DIRECTIONAL_TAPER_UNWIND_RIGHT = 4.25
STARPILOT_DIRECTIONAL_TAPER_FLOOR_LEFT = 0.48
STARPILOT_DIRECTIONAL_TAPER_FLOOR_RIGHT = 0.52
STARPILOT_DIRECTIONAL_TAPER_UNWIND_FLOOR_LEFT = 0.16
STARPILOT_DIRECTIONAL_TAPER_UNWIND_FLOOR_RIGHT = 0.04
STARPILOT_DIRECTIONAL_TAPER_JERK_ONSET = 0.60
STARPILOT_DIRECTIONAL_TAPER_JERK_WIDTH = 0.14
STARPILOT_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF = 0.98
STARPILOT_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF_SPEED = 11.2
STARPILOT_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF_SPEED_WIDTH = 1.5
STARPILOT_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF_LAT = 0.10
STARPILOT_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF_LAT_WIDTH = 0.06
STARPILOT_CRAWL_TURN_IN_FF_BOOST_LEFT = 0.18
STARPILOT_CRAWL_TURN_IN_FF_BOOST_RIGHT = 0.24
STARPILOT_CRAWL_TURN_IN_FF_SPEED = 5.3
STARPILOT_CRAWL_TURN_IN_FF_SPEED_WIDTH = 1.0
STARPILOT_CRAWL_TURN_IN_FF_LAT = 0.06
STARPILOT_CRAWL_TURN_IN_FF_LAT_WIDTH = 0.035
STARPILOT_LOW_SPEED_ANGLE_ASSIST_MAX_TORQUE = 0.46
STARPILOT_LOW_SPEED_ANGLE_ASSIST_SPEED = 3.25
STARPILOT_LOW_SPEED_ANGLE_ASSIST_SPEED_WIDTH = 0.45
STARPILOT_LOW_SPEED_ANGLE_ASSIST_ERROR = 1.9
STARPILOT_LOW_SPEED_ANGLE_ASSIST_ERROR_WIDTH = 1.20
STARPILOT_LOW_SPEED_ANGLE_ASSIST_DESIRED_ANGLE = 5.5
STARPILOT_LOW_SPEED_ANGLE_ASSIST_DESIRED_ANGLE_WIDTH = 2.4
STARPILOT_LOW_SPEED_ANGLE_ASSIST_TRACK_RATIO_START = 0.66
STARPILOT_LOW_SPEED_ANGLE_ASSIST_TRACK_RATIO_WIDTH = 0.12
STARPILOT_LOW_SPEED_ANGLE_ASSIST_TRACK_RATIO_FLOOR = 0.26
STARPILOT_LOW_SPEED_ANGLE_ASSIST_ADD_BP = [0.0, 0.35, 0.65, 1.0]
STARPILOT_LOW_SPEED_ANGLE_ASSIST_ADD_V = [1.0, 1.0, 0.88, 0.08]
STARPILOT_LOW_SPEED_UNWIND_ASSIST_MAX_TORQUE = 0.30
STARPILOT_LOW_SPEED_UNWIND_ASSIST_SPEED = 3.35
STARPILOT_LOW_SPEED_UNWIND_ASSIST_SPEED_WIDTH = 0.50
STARPILOT_LOW_SPEED_UNWIND_ASSIST_ERROR = 1.6
STARPILOT_LOW_SPEED_UNWIND_ASSIST_ERROR_WIDTH = 0.95
STARPILOT_LOW_SPEED_UNWIND_ASSIST_ACTUAL_ANGLE = 10.5
STARPILOT_LOW_SPEED_UNWIND_ASSIST_ACTUAL_ANGLE_WIDTH = 4.0
STARPILOT_LOW_SPEED_UNWIND_ASSIST_BLEND = 0.52
STARPILOT_HIGH_SPEED_RIGHT_TURN_IN_FF_BOOST = 0.10
STARPILOT_HIGH_SPEED_RIGHT_TURN_IN_FF_SPEED = 18.0
STARPILOT_HIGH_SPEED_RIGHT_TURN_IN_FF_SPEED_WIDTH = 2.5
STARPILOT_HIGH_SPEED_RIGHT_TURN_IN_FF_LAT_START = 0.06
STARPILOT_HIGH_SPEED_RIGHT_TURN_IN_FF_LAT_END = 0.22
STARPILOT_HIGH_SPEED_RIGHT_TURN_IN_FF_LAT_WIDTH = 0.035
STARPILOT_LOW_SPEED_PID_RESET_SPEED = 0.1 * CV.MPH_TO_MS
STARPILOT_HEAVY_DIRECTIONAL_TAPER_LAT_START = 0.90
STARPILOT_HEAVY_DIRECTIONAL_TAPER_LAT_WIDTH = 0.18
STARPILOT_HEAVY_DIRECTIONAL_TAPER_BASE_LEFT = 0.06
STARPILOT_HEAVY_DIRECTIONAL_TAPER_BASE_RIGHT = 0.17
STARPILOT_HEAVY_DIRECTIONAL_TAPER_UNWIND_LEFT = 0.78
STARPILOT_HEAVY_DIRECTIONAL_TAPER_UNWIND_RIGHT = 1.10
STARPILOT_OUTPUT_TAPER_SPEED = 8.5
STARPILOT_OUTPUT_TAPER_SPEED_WIDTH = 2.5
STARPILOT_OUTPUT_CENTER_TAPER_BLEND = 0.90
STARPILOT_OUTPUT_DIRECTIONAL_TAPER_BLEND = 0.97


# === Helpers + tune functions ===
def _sigmoid(x: float) -> float:
  if x >= 0.0:
    z = math.exp(-x)
    return 1.0 / (1.0 + z)

  z = math.exp(x)
  return z / (1.0 + z)


def get_friction_threshold(v_ego: float) -> float:
  # Keep the speed-scaled friction threshold behavior.
  return float(np.interp(v_ego, [1 * CV.MPH_TO_MS, 20 * CV.MPH_TO_MS, 75 * CV.MPH_TO_MS], [0.16, 0.19, 0.27]))


def _starpilot_sigmoid(x: float) -> float:
  return _sigmoid(x)


def _starpilot_low_speed_factor(v_ego: float) -> float:
  return 1.0 / (1.0 + (max(v_ego, 0.0) / STARPILOT_TRANSITION_SPEED) ** 2)


def _starpilot_transition_phase(desired_lateral_accel: float, desired_lateral_jerk: float) -> float:
  return math.tanh((desired_lateral_accel * desired_lateral_jerk) / STARPILOT_PHASE_SCALE)


def _starpilot_side_value(desired_lateral_accel: float, left_value: float, right_value: float) -> float:
  return left_value if desired_lateral_accel >= 0.0 else right_value


def _starpilot_transition_envelope(v_ego: float, desired_lateral_accel: float, desired_lateral_jerk: float) -> float:
  lat_factor = 1.0 - math.exp(-abs(desired_lateral_accel) / STARPILOT_FRICTION_LAT_RISE)
  jerk_factor = 1.0 - math.exp(-abs(desired_lateral_jerk) / STARPILOT_FRICTION_JERK_RISE)
  return _starpilot_low_speed_factor(v_ego) * lat_factor * jerk_factor


def get_starpilot_ff_scale(desired_lateral_accel: float, desired_lateral_jerk: float, v_ego: float) -> float:
  if desired_lateral_accel == 0.0:
    return 1.0

  gain = _starpilot_side_value(desired_lateral_accel, STARPILOT_FF_GAIN_LEFT, STARPILOT_FF_GAIN_RIGHT)
  abs_lateral_accel = abs(desired_lateral_accel)
  onset = _starpilot_sigmoid((abs_lateral_accel - STARPILOT_FF_ONSET) / STARPILOT_FF_ONSET_WIDTH)
  cutoff = _starpilot_sigmoid((STARPILOT_FF_CUTOFF - abs_lateral_accel) / STARPILOT_FF_CUTOFF_WIDTH)
  extra_scale = gain * onset * cutoff
  phase = _starpilot_transition_phase(desired_lateral_accel, desired_lateral_jerk)
  turn_in_weight = max(phase, 0.0)
  unwind_weight = max(-phase, 0.0)
  low_speed_factor = _starpilot_low_speed_factor(v_ego)
  turn_in_boost = 1.0 + (_starpilot_side_value(desired_lateral_accel, STARPILOT_TURN_IN_BOOST_LEFT, STARPILOT_TURN_IN_BOOST_RIGHT) *
                          turn_in_weight * low_speed_factor)
  unwind_taper = 1.0 - (_starpilot_side_value(desired_lateral_accel, STARPILOT_UNWIND_TAPER_LEFT, STARPILOT_UNWIND_TAPER_RIGHT) *
                         unwind_weight * (0.30 + 0.70 * low_speed_factor))
  crawl_turn_in_scale = 0.0
  if desired_lateral_accel * desired_lateral_jerk > 0.0:
    crawl_speed_weight = _starpilot_sigmoid((STARPILOT_CRAWL_TURN_IN_FF_SPEED - max(v_ego, 0.0)) /
                                          STARPILOT_CRAWL_TURN_IN_FF_SPEED_WIDTH)
    crawl_lat_weight = _starpilot_sigmoid((abs_lateral_accel - STARPILOT_CRAWL_TURN_IN_FF_LAT) /
                                        STARPILOT_CRAWL_TURN_IN_FF_LAT_WIDTH)
    crawl_turn_in_scale = _starpilot_side_value(desired_lateral_accel, STARPILOT_CRAWL_TURN_IN_FF_BOOST_LEFT,
                                              STARPILOT_CRAWL_TURN_IN_FF_BOOST_RIGHT) * crawl_speed_weight * crawl_lat_weight
  high_speed_right_turn_in_scale = 0.0
  if desired_lateral_accel < 0.0 and desired_lateral_accel * desired_lateral_jerk > 0.0:
    high_speed_weight = _starpilot_sigmoid((max(v_ego, 0.0) - STARPILOT_HIGH_SPEED_RIGHT_TURN_IN_FF_SPEED) /
                                         STARPILOT_HIGH_SPEED_RIGHT_TURN_IN_FF_SPEED_WIDTH)
    high_speed_lat_onset = _starpilot_sigmoid((abs_lateral_accel - STARPILOT_HIGH_SPEED_RIGHT_TURN_IN_FF_LAT_START) /
                                            STARPILOT_HIGH_SPEED_RIGHT_TURN_IN_FF_LAT_WIDTH)
    high_speed_lat_cutoff = _starpilot_sigmoid((STARPILOT_HIGH_SPEED_RIGHT_TURN_IN_FF_LAT_END - abs_lateral_accel) /
                                             STARPILOT_HIGH_SPEED_RIGHT_TURN_IN_FF_LAT_WIDTH)
    high_speed_right_turn_in_scale = STARPILOT_HIGH_SPEED_RIGHT_TURN_IN_FF_BOOST * high_speed_weight * high_speed_lat_onset * high_speed_lat_cutoff
  return (1.0 + crawl_turn_in_scale + high_speed_right_turn_in_scale +
          (extra_scale * turn_in_boost * max(unwind_taper, 0.0))) * get_starpilot_directional_taper_scale(desired_lateral_accel, desired_lateral_jerk, v_ego)


def get_starpilot_friction_threshold(v_ego: float, desired_lateral_accel: float = 0.0, desired_lateral_jerk: float = 0.0) -> float:
  base_threshold = max(get_friction_threshold(v_ego), STARPILOT_BASE_FRICTION_THRESHOLD)
  transition_envelope = _starpilot_transition_envelope(v_ego, desired_lateral_accel, desired_lateral_jerk)
  phase = _starpilot_transition_phase(desired_lateral_accel, desired_lateral_jerk)
  turn_in_weight = max(phase, 0.0)
  unwind_weight = max(-phase, 0.0)
  threshold_scale = 1.0 - (_starpilot_side_value(desired_lateral_accel, STARPILOT_TURN_IN_THRESHOLD_REDUCTION_LEFT, STARPILOT_TURN_IN_THRESHOLD_REDUCTION_RIGHT) *
                           transition_envelope * turn_in_weight)
  threshold_scale += (_starpilot_side_value(desired_lateral_accel, STARPILOT_UNWIND_THRESHOLD_INCREASE_LEFT, STARPILOT_UNWIND_THRESHOLD_INCREASE_RIGHT) *
                      transition_envelope * unwind_weight)
  return base_threshold * min(max(threshold_scale, 0.82), 1.18)


def get_starpilot_friction_scale(v_ego: float, desired_lateral_accel: float, desired_lateral_jerk: float) -> float:
  transition_envelope = _starpilot_transition_envelope(v_ego, desired_lateral_accel, desired_lateral_jerk)
  phase = _starpilot_transition_phase(desired_lateral_accel, desired_lateral_jerk)
  turn_in_weight = max(phase, 0.0)
  unwind_weight = max(-phase, 0.0)
  friction_scale = STARPILOT_FRICTION_MULT
  friction_scale += (_starpilot_side_value(desired_lateral_accel, STARPILOT_TURN_IN_FRICTION_BOOST_LEFT, STARPILOT_TURN_IN_FRICTION_BOOST_RIGHT) *
                     transition_envelope * turn_in_weight)
  friction_scale -= (_starpilot_side_value(desired_lateral_accel, STARPILOT_UNWIND_FRICTION_REDUCTION_LEFT, STARPILOT_UNWIND_FRICTION_REDUCTION_RIGHT) *
                     transition_envelope * unwind_weight)
  return min(max(friction_scale, 0.82), 1.08)


def get_starpilot_center_taper_scale(desired_lateral_accel: float, v_ego: float) -> float:
  speed_weight = _starpilot_sigmoid((v_ego - STARPILOT_CENTER_TAPER_SPEED) / STARPILOT_CENTER_TAPER_SPEED_WIDTH)
  center_weight = _starpilot_sigmoid((STARPILOT_CENTER_TAPER_LAT - abs(desired_lateral_accel)) / STARPILOT_CENTER_TAPER_LAT_WIDTH)
  high_speed_reduction = STARPILOT_CENTER_TAPER_MAX * speed_weight * center_weight

  highway_speed_weight = _starpilot_sigmoid((v_ego - STARPILOT_HIGHWAY_CENTER_TAPER_SPEED) / STARPILOT_HIGHWAY_CENTER_TAPER_SPEED_WIDTH)
  highway_center_weight = _starpilot_sigmoid((STARPILOT_HIGHWAY_CENTER_TAPER_LAT - abs(desired_lateral_accel)) /
                                           STARPILOT_HIGHWAY_CENTER_TAPER_LAT_WIDTH)
  highway_center_reduction = STARPILOT_HIGHWAY_CENTER_TAPER_MAX * highway_speed_weight * highway_center_weight

  low_mid_onset = _starpilot_sigmoid((v_ego - STARPILOT_LOW_MID_CENTER_TAPER_SPEED_MIN) / STARPILOT_LOW_MID_CENTER_TAPER_SPEED_WIDTH)
  low_mid_cutoff = _starpilot_sigmoid((STARPILOT_LOW_MID_CENTER_TAPER_SPEED_MAX - v_ego) / STARPILOT_LOW_MID_CENTER_TAPER_SPEED_WIDTH)
  low_mid_speed_weight = low_mid_onset * low_mid_cutoff
  low_mid_center_weight = _starpilot_sigmoid((STARPILOT_LOW_MID_CENTER_TAPER_LAT - abs(desired_lateral_accel)) /
                                           STARPILOT_LOW_MID_CENTER_TAPER_LAT_WIDTH)
  low_mid_reduction = STARPILOT_LOW_MID_CENTER_TAPER_MAX * low_mid_speed_weight * low_mid_center_weight

  return 1.0 - min(high_speed_reduction + highway_center_reduction + low_mid_reduction, 0.12)


def get_starpilot_directional_taper_scale(desired_lateral_accel: float, desired_lateral_jerk: float, v_ego: float | None = None) -> float:
  if desired_lateral_accel == 0.0:
    return 1.0

  abs_lateral_accel = abs(desired_lateral_accel)
  onset = _starpilot_sigmoid((abs_lateral_accel - STARPILOT_DIRECTIONAL_TAPER_LAT_START) / STARPILOT_DIRECTIONAL_TAPER_LAT_WIDTH)
  cutoff = _starpilot_sigmoid((STARPILOT_DIRECTIONAL_TAPER_LAT_END - abs_lateral_accel) / STARPILOT_DIRECTIONAL_TAPER_LAT_WIDTH)
  band_weight = onset * cutoff
  heavy_band_weight = _starpilot_sigmoid((abs_lateral_accel - STARPILOT_HEAVY_DIRECTIONAL_TAPER_LAT_START) / STARPILOT_HEAVY_DIRECTIONAL_TAPER_LAT_WIDTH)
  phase = _starpilot_transition_phase(desired_lateral_accel, desired_lateral_jerk)
  unwind_weight = max(-phase, 0.0) * _starpilot_sigmoid((abs(desired_lateral_jerk) - STARPILOT_DIRECTIONAL_TAPER_JERK_ONSET) /
                                                       STARPILOT_DIRECTIONAL_TAPER_JERK_WIDTH)
  low_speed_relief_weight = 0.0
  if v_ego is not None:
    low_speed_weight = _starpilot_sigmoid((STARPILOT_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF_SPEED - max(v_ego, 0.0)) /
                                        STARPILOT_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF_SPEED_WIDTH)
    tight_turn_weight = _starpilot_sigmoid((abs_lateral_accel - STARPILOT_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF_LAT) /
                                         STARPILOT_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF_LAT_WIDTH)
    low_speed_relief_weight = STARPILOT_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF * low_speed_weight * tight_turn_weight * (1.0 - unwind_weight)
  base_reduction = _starpilot_side_value(desired_lateral_accel, STARPILOT_DIRECTIONAL_TAPER_BASE_LEFT, STARPILOT_DIRECTIONAL_TAPER_BASE_RIGHT)
  unwind_reduction = _starpilot_side_value(desired_lateral_accel, STARPILOT_DIRECTIONAL_TAPER_UNWIND_LEFT, STARPILOT_DIRECTIONAL_TAPER_UNWIND_RIGHT)
  heavy_base_reduction = _starpilot_side_value(desired_lateral_accel, STARPILOT_HEAVY_DIRECTIONAL_TAPER_BASE_LEFT, STARPILOT_HEAVY_DIRECTIONAL_TAPER_BASE_RIGHT)
  heavy_unwind_reduction = _starpilot_side_value(desired_lateral_accel, STARPILOT_HEAVY_DIRECTIONAL_TAPER_UNWIND_LEFT, STARPILOT_HEAVY_DIRECTIONAL_TAPER_UNWIND_RIGHT)
  base_reduction *= 1.0 - low_speed_relief_weight
  heavy_base_reduction *= 1.0 - low_speed_relief_weight
  reduction = band_weight * (base_reduction + unwind_reduction * unwind_weight)
  reduction += heavy_band_weight * (heavy_base_reduction + heavy_unwind_reduction * unwind_weight)
  floor = _starpilot_side_value(desired_lateral_accel, STARPILOT_DIRECTIONAL_TAPER_FLOOR_LEFT, STARPILOT_DIRECTIONAL_TAPER_FLOOR_RIGHT)
  floor -= _starpilot_side_value(desired_lateral_accel, STARPILOT_DIRECTIONAL_TAPER_UNWIND_FLOOR_LEFT, STARPILOT_DIRECTIONAL_TAPER_UNWIND_FLOOR_RIGHT) * unwind_weight
  return max(1.0 - reduction, floor)


def get_starpilot_output_taper_scale(desired_lateral_accel: float, desired_lateral_jerk: float, v_ego: float) -> float:
  speed_weight = _starpilot_sigmoid((v_ego - STARPILOT_OUTPUT_TAPER_SPEED) / STARPILOT_OUTPUT_TAPER_SPEED_WIDTH)
  center_taper = get_starpilot_center_taper_scale(desired_lateral_accel, v_ego)
  directional_taper = get_starpilot_directional_taper_scale(desired_lateral_accel, desired_lateral_jerk, v_ego)
  center_scale = 1.0 - ((1.0 - center_taper) * STARPILOT_OUTPUT_CENTER_TAPER_BLEND * speed_weight)
  directional_scale = 1.0 - ((1.0 - directional_taper) * STARPILOT_OUTPUT_DIRECTIONAL_TAPER_BLEND * speed_weight)
  return center_scale * directional_scale


def get_starpilot_highway_output_taper_scale(desired_lateral_accel: float, v_ego: float) -> float:
  speed_weight = _starpilot_sigmoid((v_ego - STARPILOT_HIGHWAY_OUTPUT_TAPER_SPEED) / STARPILOT_HIGHWAY_OUTPUT_TAPER_SPEED_WIDTH)
  center_weight = _starpilot_sigmoid((STARPILOT_HIGHWAY_OUTPUT_TAPER_LAT - abs(desired_lateral_accel)) /
                                   STARPILOT_HIGHWAY_OUTPUT_TAPER_LAT_WIDTH)
  reduction = STARPILOT_HIGHWAY_OUTPUT_TAPER_MAX * speed_weight * center_weight
  return 1.0 - reduction


def get_starpilot_highway_transition_output_taper_scale(desired_lateral_accel: float, desired_lateral_jerk: float, v_ego: float) -> float:
  speed_weight = _starpilot_sigmoid((v_ego - STARPILOT_HIGHWAY_OUTPUT_TAPER_SPEED) / STARPILOT_HIGHWAY_OUTPUT_TAPER_SPEED_WIDTH)
  center_weight = _starpilot_sigmoid((STARPILOT_HIGHWAY_TRANSITION_OUTPUT_TAPER_LAT - abs(desired_lateral_accel)) /
                                   STARPILOT_HIGHWAY_TRANSITION_OUTPUT_TAPER_LAT_WIDTH)
  jerk_weight = _starpilot_sigmoid((abs(desired_lateral_jerk) - STARPILOT_HIGHWAY_TRANSITION_OUTPUT_TAPER_JERK) /
                                 STARPILOT_HIGHWAY_TRANSITION_OUTPUT_TAPER_JERK_WIDTH)
  reduction = STARPILOT_HIGHWAY_TRANSITION_OUTPUT_TAPER_MAX * speed_weight * center_weight * jerk_weight
  return 1.0 - reduction


def get_starpilot_low_speed_angle_assist_torque(desired_angle_deg: float, actual_angle_deg: float,
                                              current_output_torque: float, v_ego: float) -> float:
  angle_error = desired_angle_deg - actual_angle_deg
  if desired_angle_deg * angle_error > 0.0:
    speed_weight = _starpilot_sigmoid((STARPILOT_LOW_SPEED_ANGLE_ASSIST_SPEED - max(v_ego, 0.0)) /
                                    STARPILOT_LOW_SPEED_ANGLE_ASSIST_SPEED_WIDTH)
    error_weight = _starpilot_sigmoid((abs(angle_error) - STARPILOT_LOW_SPEED_ANGLE_ASSIST_ERROR) /
                                    STARPILOT_LOW_SPEED_ANGLE_ASSIST_ERROR_WIDTH)
    desired_angle_weight = _starpilot_sigmoid((abs(desired_angle_deg) - STARPILOT_LOW_SPEED_ANGLE_ASSIST_DESIRED_ANGLE) /
                                            STARPILOT_LOW_SPEED_ANGLE_ASSIST_DESIRED_ANGLE_WIDTH)
    tracking_ratio = abs(actual_angle_deg) / max(abs(desired_angle_deg), 1e-3)
    tracking_taper = _starpilot_sigmoid((tracking_ratio - STARPILOT_LOW_SPEED_ANGLE_ASSIST_TRACK_RATIO_START) /
                                      STARPILOT_LOW_SPEED_ANGLE_ASSIST_TRACK_RATIO_WIDTH)
    tracking_scale = max(1.0 - tracking_taper, STARPILOT_LOW_SPEED_ANGLE_ASSIST_TRACK_RATIO_FLOOR)
    assist_torque = math.copysign(STARPILOT_LOW_SPEED_ANGLE_ASSIST_MAX_TORQUE * speed_weight * error_weight * desired_angle_weight * tracking_scale, -angle_error)
    if abs(assist_torque) < 1e-4:
      return current_output_torque

    if current_output_torque * assist_torque >= 0.0:
      add_scale = float(np.interp(abs(current_output_torque),
                                  STARPILOT_LOW_SPEED_ANGLE_ASSIST_ADD_BP,
                                  STARPILOT_LOW_SPEED_ANGLE_ASSIST_ADD_V))
      return float(np.clip(current_output_torque + (assist_torque * add_scale), -1.0, 1.0))

    return float(np.clip(current_output_torque + assist_torque, -1.0, 1.0))

  speed_weight = _starpilot_sigmoid((STARPILOT_LOW_SPEED_UNWIND_ASSIST_SPEED - max(v_ego, 0.0)) /
                                  STARPILOT_LOW_SPEED_UNWIND_ASSIST_SPEED_WIDTH)
  error_weight = _starpilot_sigmoid((abs(angle_error) - STARPILOT_LOW_SPEED_UNWIND_ASSIST_ERROR) /
                                  STARPILOT_LOW_SPEED_UNWIND_ASSIST_ERROR_WIDTH)
  actual_angle_weight = _starpilot_sigmoid((abs(actual_angle_deg) - STARPILOT_LOW_SPEED_UNWIND_ASSIST_ACTUAL_ANGLE) /
                                         STARPILOT_LOW_SPEED_UNWIND_ASSIST_ACTUAL_ANGLE_WIDTH)
  assist_torque = math.copysign(STARPILOT_LOW_SPEED_UNWIND_ASSIST_MAX_TORQUE * speed_weight * error_weight * actual_angle_weight, -angle_error)
  if abs(assist_torque) < 1e-4:
    return current_output_torque

  if current_output_torque * assist_torque >= 0.0:
    assist_torque *= STARPILOT_LOW_SPEED_UNWIND_ASSIST_BLEND

  return float(np.clip(current_output_torque + assist_torque, -1.0, 1.0))
