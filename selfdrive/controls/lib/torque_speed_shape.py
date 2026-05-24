import math
from collections import namedtuple

from opendbc.car import structs


# Speed-dependent shape applied on top of the per-car learned scalars for
# latAccelFactor and friction. Each shape is normalized so its value at V_REF
# is exactly 1.0 — at the reference (highway) speed the modifier is a no-op
# and the per-car learner's output is used unchanged.
#
# Fit was derived from a speed-binned torque learner on one vehicle. The
# bins centered at 6.5..37.5 m/s show both signals follow a saturating
# exponential in speed:
#
#   latAccelFactor(v) ≈ A_inf · (1 − exp(−v/τ_A))      with τ_A ≈ 6.2 m/s
#   friction(v)      ≈ F_inf · (1 + A_F · exp(−v/τ_F)) with τ_F ≈ 5.3 m/s
#                                                          A_F ≈ 2.76
#
# These constants are starting values. They are expected to be retuned —
# and likely made per-car — once the speed-binned learner is run across
# more vehicles. The v=10 bin in the source data was treated as noise.

V_REF = 25.0   # m/s, ~56 mph. Reference speed where modifier == 1.0.
TAU_A = 6.2    # m/s, latAccelFactor saturation time constant.
TAU_F = 5.3    # m/s, friction saturation time constant.
F_AMP = 2.76   # friction amplitude relative to its high-speed asymptote.

_SHAPE_A_DENOM = 1.0 - math.exp(-V_REF / TAU_A)
_SHAPE_F_DENOM = 1.0 + F_AMP * math.exp(-V_REF / TAU_F)


ScaledTorqueParams = namedtuple("ScaledTorqueParams", ["latAccelFactor", "latAccelOffset", "friction"])


def lat_accel_factor_shape(v_ego: float) -> float:
  v = max(0.0, v_ego)
  return (1.0 - math.exp(-v / TAU_A)) / _SHAPE_A_DENOM


def friction_shape(v_ego: float) -> float:
  v = max(0.0, v_ego)
  return (1.0 + F_AMP * math.exp(-v / TAU_F)) / _SHAPE_F_DENOM


def scale_torque_params(torque_params: structs.CarParams.LateralTorqueTuning, v_ego: float) -> ScaledTorqueParams:
  shape_a = lat_accel_factor_shape(v_ego)
  shape_f = friction_shape(v_ego)
  return ScaledTorqueParams(
    latAccelFactor=float(torque_params.latAccelFactor) * shape_a,
    latAccelOffset=float(torque_params.latAccelOffset),
    friction=float(torque_params.friction) * shape_f,
  )
