# GV60 battery preconditioning test (0xC7 alone)

This is the gate test before any precond UI work. Question: **does the 0xC7 request on bus 0 start
preconditioning by itself, without the 0x4ED nav-context message?**

## What's in the build

- **Panda safety** (opendbc `hyundai_canfd.h`): 0xC7 on E-CAN (bus 0 on the GV60 camera harness) is
  allowed only when the sunnypilot safety flag `HyundaiSafetyFlagsSP.PRECOND` (16) is set, and only
  these three payloads: request `00 00 00 40 03 00 00 00`, cancel `00 ×8`, idle `00 00 00 E0 07 00 00 00`.
  The flag is set only for `GENESIS_GV60_EV_1ST_GEN` with LFA steering and E-CAN on bus 0.
- **card**: when the `HyundaiPrecondCmd` param is set to `start` or `cancel`, card clears it and sends:
  - start: request ×3 then idle ×3, 40 ms apart (what nav sent on route `00000139--d597be60c5`), resent at
    +3.5 s and +10 s like nav, until the BMS reports Preparing/On. No response after 15 s = declined.
  - cancel: zeros ×2 then idle ×3 (Ioniq 5 M-CAN capture; never seen on the GV60)

  card also decodes the BMS state and temps and writes them to `HyundaiPrecondStatus`.
- **Settings > Vehicle** (GV60 only): a Battery Preconditioning row with the live status, and Start / Cancel buttons.
- **`precond_test.py`** (optional, SSH): same Start/Cancel, plus a timestamped CSV of every change.

## Running it

1. Pull this branch on the comma and reboot. The panda firmware changed, so pandad reflashes it on
   start. Check `pandaStates` / the boot log shows a fresh flash.
2. Car on (READY), parked, no nav destination set. Wait for openpilot to go onroad.
3. On the comma: tap the screen to bring up the sidebar, open **Settings > Vehicle** and scroll to
   **Battery Preconditioning**. It should say **Ready** with BMS `off (1)` and temps filled in.
4. Tap **Start**. The status follows along: Requested (burst n of 3) → Preparing → Heating, or Declined.
   In the nav run, Preparing showed up ~7.6 s after the first request and On ~68 s after.
   If it starts, try **Cancel** to see whether the cancel burst works.

Everything is in the rlog either way (our frames show as bus 128 echoes), so the route is the record.
For a CSV with timestamps instead, over SSH (`ssh comma@10.0.0.88`, `cd /data/openpilot`):
```
python -m openpilot.tools.horizon.precond_test monitor   # watch; works alongside the buttons
python -m openpilot.tools.horizon.precond_test start     # or start from here
```
CSVs land in `/data/media/0/precond_tests/`.

## Reading the output

| You see | Meaning |
|---|---|
| "Waiting for car..." while onroad | card isn't publishing status: not a GV60 fingerprint, LKA-steering harness, or the build didn't take |
| "Sent N frames (panda TX confirmed)" / `0xC7 ours` lines | panda transmitted it |
| "WARNING: nothing was transmitted" / `txBlocked` climbing | old panda firmware, or the safety flag isn't set (check `safetyParam` in CarParamsSP = 16) |
| Preparing → Heating, heater 0 → ~46, 0x360 8 → 50 | **0xC7 alone works** |
| Declined | BMS ignored it. Either it wants 0x4ED nav context, or the gateway doesn't route 0xC7 from bus 0 back to the BMS |

If it's ignored, these are the next things to separate:
- Repeat on a cold morning. On route 139 the BMS ignored a later request when the pack was already warm,
  so a warm pack could be what declined it, not a missing message.
- Gateway direction: on the GV60, 0xC7 comes from the head unit (M-CAN) and the gateway forwards it to
  bus 0. Nothing says the gateway forwards bus 0 → BMS. If cold-pack tests still fail, this is the likely
  blocker, and the request would have to go in on M-CAN (head-unit side), as tylerharvey/dragz do.
