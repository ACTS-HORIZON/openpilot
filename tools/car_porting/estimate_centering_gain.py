#!/usr/bin/env python3
"""
Estimate the MDPS centering torque gain (K_CENTERING_ANGLE) from driving logs.

The MDPS applies a restoring torque proportional to steering angle. During steady-state
cornering, the PID integrator compensates for this centering force. By measuring the
relationship between steering angle and integrator value, we can estimate the centering
gain and use it as a feedforward term to reduce integrator load and improve transient response.

Usage:
  python tools/tuning/estimate_centering_gain.py --route "route_id/segment"
  python tools/tuning/estimate_centering_gain.py --route "route_id/0:10"  # first 10 segments

Output:
  - K_CENTERING_ANGLE estimate (slope of pid.i vs steering_angle_deg)
  - EPS spring constant (slope of steeringTorqueEps vs steeringAngleDeg)
  - Scatter plots saved to /tmp/centering_analysis/ (if matplotlib available)
"""

import argparse
import os
import signal
import sys
import numpy as np

from openpilot.tools.lib.logreader import LogReader

# Steady-state filtering thresholds
MIN_VEGO = 15.0             # m/s, only highway data
MAX_ERROR = 0.15            # m/s^2, lateral accel error must be small
MAX_STEERING_RATE = 5.0     # deg/s, wheel must not be actively turning
MIN_ENGAGE_FRAMES = 500     # ~5 seconds after engage before collecting data


def sigint_handler(sig, frame):
  sys.exit(0)

signal.signal(signal.SIGINT, sigint_handler)


class CenteringGainEstimator:
  def __init__(self):
    self.engage_count = 0

    # Data collection arrays
    self.steering_angles = []
    self.integrator_values = []
    self.speeds = []
    self.errors = []

    # EPS characterization
    self.eps_angles = []
    self.eps_torques = []

  def update(self, sm):
    car_state = sm.get('carState')
    controls_state = sm.get('controlsState')
    if car_state is None or controls_state is None:
      return

    lat_state = controls_state.lateralControlState
    # Access the torque state
    try:
      torque_state = lat_state.torqueState
    except Exception:
      return

    if not torque_state.active:
      self.engage_count = 0
      return

    v_ego = car_state.vEgo
    steering_pressed = car_state.steeringPressed
    steering_angle = car_state.steeringAngleDeg
    steering_rate = car_state.steeringRateDeg
    steering_torque_eps = car_state.steeringTorqueEps

    error = torque_state.error
    integrator = torque_state.i

    self.engage_count += 1

    # Always collect EPS data (no steady-state requirement)
    if v_ego > MIN_VEGO and not steering_pressed and abs(steering_angle) > 0.5:
      self.eps_angles.append(steering_angle)
      self.eps_torques.append(steering_torque_eps)

    # Steady-state filter for centering gain estimation
    if self.engage_count < MIN_ENGAGE_FRAMES:
      return
    if v_ego < MIN_VEGO:
      return
    if abs(error) > MAX_ERROR:
      return
    if abs(steering_rate) > MAX_STEERING_RATE:
      return
    if steering_pressed:
      return
    if abs(steering_angle) < 0.5:  # skip near-zero angles (noise dominated)
      return

    self.steering_angles.append(steering_angle)
    self.integrator_values.append(integrator)
    self.speeds.append(v_ego)
    self.errors.append(error)

  def analyze(self):
    print(f"\n{'='*60}")
    print("CENTERING GAIN ANALYSIS")
    print(f"{'='*60}\n")

    # --- PID Integrator vs Steering Angle ---
    n = len(self.steering_angles)
    print(f"Steady-state data points collected: {n}")
    if n < 50:
      print("WARNING: Too few data points for reliable estimation.")
      print("Need more highway driving with steady cornering (curves, highway bends).")
      if n == 0:
        return
    else:
      print(f"Speed range: {min(self.speeds):.1f} - {max(self.speeds):.1f} m/s")
      print(f"Steering angle range: {min(self.steering_angles):.1f} - {max(self.steering_angles):.1f} deg")

    angles = np.array(self.steering_angles)
    integrators = np.array(self.integrator_values)

    # Linear fit: integrator = K_CENTERING_ANGLE * angle + offset
    coeffs = np.polyfit(angles, integrators, 1)
    k_centering = coeffs[0]
    offset = coeffs[1]
    residuals = integrators - (k_centering * angles + offset)
    r_squared = 1 - np.var(residuals) / np.var(integrators) if np.var(integrators) > 0 else 0

    print("\n--- K_CENTERING_ANGLE Estimation ---")
    print(f"  K_CENTERING_ANGLE = {k_centering:.6f}")
    print(f"  Offset = {offset:.6f}")
    print(f"  R² = {r_squared:.4f}")
    print(f"\n  To use: set K_CENTERING_ANGLE = {k_centering:.4f} in")
    print("  selfdrive/controls/lib/latcontrol_torque.py")

    if r_squared < 0.3:
      print("\n  NOTE: Low R² ({r_squared:.2f}) suggests the centering force")
      print("  may be weak or the data is noisy. Consider more data or")
      print("  setting K_CENTERING_ANGLE = 0.0 to disable.")

    # --- Speed-bucketed analysis ---
    speed_buckets = [(15, 20), (20, 25), (25, 30), (30, 40)]
    speeds_arr = np.array(self.speeds)
    print("\n--- Speed-Bucketed K_CENTERING_ANGLE ---")
    for lo, hi in speed_buckets:
      mask = (speeds_arr >= lo) & (speeds_arr < hi)
      if mask.sum() < 20:
        continue
      c = np.polyfit(angles[mask], integrators[mask], 1)
      print(f"  {lo:2d}-{hi:2d} m/s: K={c[0]:.6f}  (n={mask.sum()})")

    # --- EPS Spring Characterization ---
    n_eps = len(self.eps_angles)
    print("\n--- EPS Spring Characterization ---")
    print(f"  EPS data points: {n_eps}")
    if n_eps >= 50:
      eps_a = np.array(self.eps_angles)
      eps_t = np.array(self.eps_torques)
      eps_coeffs = np.polyfit(eps_a, eps_t, 1)
      eps_r2 = 1 - np.var(eps_t - np.polyval(eps_coeffs, eps_a)) / np.var(eps_t) if np.var(eps_t) > 0 else 0
      print(f"  EPS spring constant: {eps_coeffs[0]:.4f} torque_units/deg")
      print(f"  EPS offset: {eps_coeffs[1]:.4f}")
      print(f"  R² = {eps_r2:.4f}")

    # --- Save plots if matplotlib available ---
    self._save_plots(angles, integrators, k_centering, offset)

  def _save_plots(self, angles, integrators, k_centering, offset):
    try:
      import matplotlib
      matplotlib.use('Agg')
      import matplotlib.pyplot as plt
    except ImportError:
      print("\n  (Install matplotlib for scatter plots: pip install matplotlib)")
      return

    out_dir = "/tmp/centering_analysis"
    os.makedirs(out_dir, exist_ok=True)

    # Plot 1: Integrator vs Steering Angle
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.scatter(angles, integrators, alpha=0.3, s=2, label='Data')
    x_line = np.linspace(min(angles), max(angles), 100)
    ax.plot(x_line, k_centering * x_line + offset, 'r-', linewidth=2,
            label=f'Fit: K={k_centering:.4f}')
    ax.set_xlabel('Steering Angle (deg)')
    ax.set_ylabel('PID Integrator Value')
    ax.set_title('Centering Gain Estimation: PID Integrator vs Steering Angle')
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.savefig(os.path.join(out_dir, 'integrator_vs_angle.png'), dpi=150)
    plt.close(fig)

    # Plot 2: EPS Torque vs Steering Angle
    if len(self.eps_angles) >= 50:
      eps_a = np.array(self.eps_angles)
      eps_t = np.array(self.eps_torques)
      fig, ax = plt.subplots(figsize=(10, 6))
      ax.scatter(eps_a, eps_t, alpha=0.3, s=2, label='Data')
      eps_coeffs = np.polyfit(eps_a, eps_t, 1)
      ax.plot(x_line, np.polyval(eps_coeffs, x_line), 'r-', linewidth=2,
              label=f'Fit: k_eps={eps_coeffs[0]:.4f}')
      ax.set_xlabel('Steering Angle (deg)')
      ax.set_ylabel('EPS Output Torque (CAN units)')
      ax.set_title('EPS Spring Characterization: Output Torque vs Steering Angle')
      ax.legend()
      ax.grid(True, alpha=0.3)
      fig.savefig(os.path.join(out_dir, 'eps_torque_vs_angle.png'), dpi=150)
      plt.close(fig)

    print(f"\n  Plots saved to {out_dir}/")


def main():
  parser = argparse.ArgumentParser(description='Estimate MDPS centering torque gain from driving logs')
  parser.add_argument('--route', required=True, help='Route identifier (e.g., "route_id/0:10")')
  args = parser.parse_args()

  print(f"Loading route: {args.route}")
  lr = LogReader(args.route, sort_by_time=True)

  estimator = CenteringGainEstimator()
  sm = {}
  msg_count = 0

  for msg in lr:
    w = msg.which()
    if w == 'carState':
      sm['carState'] = msg.carState
    elif w == 'controlsState':
      sm['controlsState'] = msg.controlsState

    if w == 'controlsState' and 'carState' in sm:
      estimator.update(sm)
      msg_count += 1
      if msg_count % 10000 == 0:
        n = len(estimator.steering_angles)
        print(f"  Processed {msg_count} messages, {n} steady-state points collected...")

  estimator.analyze()


if __name__ == "__main__":
  main()
