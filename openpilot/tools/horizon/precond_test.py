#!/usr/bin/env python3
"""
GV60 battery preconditioning test: send the 0xC7 request on its own (no 0x4ED nav context) and watch the BMS.

Run on the comma over SSH while the car is on and openpilot is onroad (panda in hyundaiCanfd safety):

  python -m openpilot.tools.horizon.precond_test monitor          # watch only
  python -m openpilot.tools.horizon.precond_test start            # request, resend like nav does, then watch
  python -m openpilot.tools.horizon.precond_test cancel           # cancel burst, then watch

What it prints (all bus 0, byte offsets are 0-based):
  0xC7  request frames: ours (panda TX echo, src 128) and any from the head unit (src 0)
  0x2AA b17 & 0x1F   BMS precond state: 1 Off, 5 Preparing, 21 On
  0x360 b16          co-moves with precond (8 idle / 50 precond / 10 after?)
  0x2FA b10 / b11    pack min / max temp (degC, inferred)
  0x2FA b5           coolant inlet temp (degC, inferred)
  0x25A b18          heater active / power (0 or ~46)
  pandaStates safetyTxBlocked  climbs if panda rejected our frames

A CSV of every change is written to /data/media/0/precond_tests/ (or the current dir off-device).
"""
import argparse
import csv
import os
import sys
import time

import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params

PRECOND_STATES = {1: "Off", 5: "Preparing", 21: "On"}

# (name, addr, byte index, mask)
SIGNALS = [
  ("bms_state", 0x2AA, 17, 0x1F),
  ("x360_b16", 0x360, 16, 0xFF),
  ("pack_min_c", 0x2FA, 10, 0xFF),
  ("pack_max_c", 0x2FA, 11, 0xFF),
  ("inlet_c", 0x2FA, 5, 0xFF),
  ("heater", 0x25A, 18, 0xFF),
]
BUS = 0
TX_ECHO_BUS = 128 + BUS

# nav-like resend: up to 3 bursts, gaps taken from route 139 (103.3, 106.8, 113.3 s)
RESEND_AT_S = (3.5, 10.0)
PREPARING_TIMEOUT_S = 15.0


def frame_name(dat: bytes) -> str:
  b34 = bytes(dat[3:5])
  return {b"\x40\x03": "REQUEST", b"\x00\x00": "CANCEL", b"\xe0\x07": "idle"}.get(b34, f"?{dat.hex()}")


def open_csv():
  out_dir = "/data/media/0/precond_tests" if os.path.isdir("/data/media/0") else "."
  os.makedirs(out_dir, exist_ok=True)
  path = os.path.join(out_dir, time.strftime("precond_%Y%m%d_%H%M%S.csv"))
  f = open(path, "w", newline="")
  w = csv.writer(f)
  w.writerow(["t_s", "event", "value"])
  return path, f, w


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("action", choices=["monitor", "start", "cancel"])
  ap.add_argument("--no-resend", action="store_true", help="start: send a single burst only")
  ap.add_argument("--duration", type=float, default=0, help="stop after N seconds (default: until Ctrl-C)")
  args = ap.parse_args()

  params = Params()
  can_sock = messaging.sub_sock("can", conflate=False, timeout=100)
  sm = messaging.SubMaster(["pandaStates"])

  csv_path, csv_f, w = open_csv()
  t0 = time.monotonic()

  def log(event, value, note=""):
    t = time.monotonic() - t0
    w.writerow([f"{t:.3f}", event, value])
    csv_f.flush()
    print(f"[{t:8.3f}s] {event:<14} {value}{('  ' + note) if note else ''}", flush=True)

  print(f"logging to {csv_path}")

  # wait for panda state so we can tell the user if TX can't work
  deadline = time.monotonic() + 3.0
  while time.monotonic() < deadline and not sm.seen["pandaStates"]:
    sm.update(100)
  if sm.seen["pandaStates"] and len(sm["pandaStates"]):
    ps = sm["pandaStates"][0]
    log("panda", f"safetyModel={ps.safetyModel} safetyParam={ps.safetyParam} ignition={ps.ignitionLine or ps.ignitionCan}")
    if str(ps.safetyModel) != "hyundaiCanfd" and args.action != "monitor":
      print("WARNING: panda is not in hyundaiCanfd safety (not onroad?); the request will not be transmitted", file=sys.stderr)
  else:
    print("WARNING: no pandaStates; is openpilot running?", file=sys.stderr)
  tx_blocked0 = sm["pandaStates"][0].safetyTxBlocked if sm.seen["pandaStates"] and len(sm["pandaStates"]) else None

  bursts_sent = 0
  burst_times: list[float] = []

  def send(cmd):
    nonlocal bursts_sent
    params.put("HyundaiPrecondCmd", cmd)
    bursts_sent += 1
    burst_times.append(time.monotonic() - t0)
    log("cmd", cmd, f"(burst {bursts_sent})")

  if args.action in ("start", "cancel"):
    send(args.action)

  last: dict[str, int] = {}
  echo_seen_for_burst = 0
  bms_reached = None
  done_resending = args.action != "start" or args.no_resend

  try:
    while True:
      now = time.monotonic() - t0
      if args.duration and now > args.duration:
        break

      for msg in messaging.drain_sock(can_sock, wait_for_one=True):
        for c in msg.can:
          if c.address == 0xC7 and c.src in (BUS, TX_ECHO_BUS):
            who = "ours" if c.src == TX_ECHO_BUS else "car"
            if who == "ours":
              echo_seen_for_burst = bursts_sent
            log(f"0xC7 {who}", frame_name(c.dat))
            continue
          if c.src != BUS:
            continue
          for name, addr, idx, mask in SIGNALS:
            if c.address == addr and len(c.dat) > idx:
              v = c.dat[idx] & mask
              if last.get(name) != v:
                last[name] = v
                note = PRECOND_STATES.get(v, "?") if name == "bms_state" else ""
                log(name, v, note)
                if name == "bms_state" and v in (5, 21) and bms_reached is None and bursts_sent:
                  bms_reached = now
                  dt = now - burst_times[0]
                  log("RESULT", f"BMS -> {PRECOND_STATES[v]} {dt:.1f}s after first request ({bursts_sent} burst(s) sent)")

      # panda rejected frames?
      sm.update(0)
      if tx_blocked0 is not None and sm.updated["pandaStates"] and len(sm["pandaStates"]):
        blocked = sm["pandaStates"][0].safetyTxBlocked
        if blocked != tx_blocked0:
          log("txBlocked", blocked, "panda safety rejected a frame")
          tx_blocked0 = blocked

      # TX sanity check one second after each burst
      if bursts_sent and echo_seen_for_burst < bursts_sent and now - burst_times[-1] > 1.0:
        log("WARN", "no TX echo for last burst", "(card not running / not onroad / panda blocked / old panda fw?)")
        echo_seen_for_burst = bursts_sent

      # nav-like resends until the BMS reacts
      if not done_resending:
        if bms_reached is not None:
          done_resending = True
        elif bursts_sent <= len(RESEND_AT_S) and now - burst_times[0] >= RESEND_AT_S[bursts_sent - 1]:
          send("start")
        elif bursts_sent > len(RESEND_AT_S) and now - burst_times[0] > PREPARING_TIMEOUT_S:
          log("RESULT", f"no Preparing within {PREPARING_TIMEOUT_S:.0f}s of first request (declined?)")
          done_resending = True
  except KeyboardInterrupt:
    pass
  finally:
    csv_f.close()
    print(f"\nsaved {csv_path}")


if __name__ == "__main__":
  main()
