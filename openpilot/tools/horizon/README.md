# GV60 battery preconditioning test (0xC7 alone)

This is the gate test before any precond UI work. Question: **does the 0xC7 request on bus 0 start
preconditioning by itself, without the 0x4ED nav-context message?**

## What's in the build

- **Panda safety** (opendbc `hyundai_canfd.h`): 0xC7 on E-CAN (bus 0 on the GV60 camera harness) is
  allowed only when the sunnypilot safety flag `HyundaiSafetyFlagsSP.PRECOND` (16) is set, and only
  these three payloads: request `00 00 00 40 03 00 00 00`, cancel `00 ×8`, idle `00 00 00 E0 07 00 00 00`.
  The flag is set only for `GENESIS_GV60_EV_1ST_GEN` with LFA steering and E-CAN on bus 0.
- **card**: when the `HyundaiPrecondCmd` param is set to `start` or `cancel`, card clears it and sends one burst:
  - start: request ×3 then idle ×3, 40 ms apart (what nav sent on route `00000139--d597be60c5`)
  - cancel: zeros ×2 then idle ×3 (Ioniq 5 M-CAN capture; never seen on the GV60)
- **`precond_test.py`**: sets the param, resends like nav does, and prints/logs the BMS response.

## Running it

1. Pull this branch on the comma and reboot. The panda firmware changed, so pandad reflashes it on
   start. Check `pandaStates` / the boot log shows a fresh flash.
2. Car on (READY), parked, no nav destination set. Wait for openpilot to go onroad.
3. SSH in (`ssh comma@10.0.0.88`), then:
   ```
   cd /data/openpilot
   python -m openpilot.tools.horizon.precond_test monitor --duration 20   # baseline, expect bms_state 1
   python -m openpilot.tools.horizon.precond_test start                   # the test
   ```
4. Leave it running. In the nav run, Preparing showed up ~7.6 s after the first request and On ~68 s
   after. Ctrl-C when done. If it started, try `... precond_test cancel` to see whether the cancel burst works.

A CSV lands in `/data/media/0/precond_tests/`. The rlog has everything too (our frames show as bus 128 echoes).

## Reading the output

| You see | Meaning |
|---|---|
| `0xC7 ours REQUEST` ×3 + `idle` ×3 per burst | panda transmitted it |
| `WARN no TX echo` / `txBlocked` climbing | not onroad, old panda firmware, or flag not set (check `safetyParam` / CarParamsSP) |
| `bms_state 5 Preparing` → `21 On`, `heater` 0 → ~46, `x360_b16` 8 → 50 | **0xC7 alone works** |
| `RESULT no Preparing within 15s` | BMS ignored it. Either it wants 0x4ED nav context, or the gateway doesn't route 0xC7 from bus 0 back to the BMS |

If it's ignored, these are the next things to separate:
- Repeat on a cold morning. On route 139 the BMS ignored a later request when the pack was already warm,
  so a warm pack could be what declined it, not a missing message.
- Gateway direction: on the GV60, 0xC7 comes from the head unit (M-CAN) and the gateway forwards it to
  bus 0. Nothing says the gateway forwards bus 0 → BMS. If cold-pack tests still fail, this is the likely
  blocker, and the request would have to go in on M-CAN (head-unit side), as tylerharvey/dragz do.
