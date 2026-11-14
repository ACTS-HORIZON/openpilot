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
  - cancel: zeros ×3 then idle ×3 (what nav sent on the GV60, route `000001c8--8fa07408a4`)

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

## Result: route `000001c8--8fa07408a4` (2026-11-14, parked, pack 7–11 °C)

**Sending 0xC7 on bus 0 from the comma, as CAN-FD and with no nav destination set, did not start preconditioning.**
Nav did, minutes later, same session. (Both conditions were found in review afterwards; see "Open after review".)

| t (s) | What | BMS response |
|---|---|---|
| 170.7, 174.2, 180.7 | Our 3 bursts (panda TX echo on bus 128), same ID and payload as nav's | none. 0x2AA stayed 1, nothing else on buses 0/1/2 reacted |
| 316.05 | Nav request, one burst (`40 03` ×3, `E0 07` ×3) | 0x2B5 b9 pulses 8 and b8/b16–18 change at +1.0 s; 0x2AA → 5 Preparing at +7.6 s (same as route 139) |
| 384.0 | | 0x2AA → 21 On, heater 0x25A b18 0 → 52, inlet climbs 8 → 15 °C in 25 s |
| 410.0 | Nav cancel: `00 00` ×3, `E0 07` ×3 | 0x2AA → 1, heater off, 0x2B5 pulses again |

- Ruled out: pack temp, car state, payload bytes, panda blocking. **Not** ruled out: frame format (see below).
- 0x2B5 (bus 0, 100 ms) is **not** an acknowledgment. On route 139 its b9 pulses to 8 for 2.1 s every 4–15 s
  during nav to a charger with no 0xC7 in between, and once 30 ms *before* a nav request (1488.47 s). It looks
  like nav/head-unit context. Candidate for the GV60's equivalent of 0x4ED (meaning unconfirmed).
- Nothing new appeared on buses 0/1/2 between setting the destination and the request (no nav-context message
  reaches the comma; 0x4ED absent again).
- 0x360 b16: 12 before, 50 from Preparing, 12 after cancel, 62 at +30 s. The pre-request value isn't fixed (8 on route 139).

Possible explanations (none tested yet):
1. The gateway doesn't route 0xC7 from bus 0 toward the BMS.
2. The BMS also needs nav context (a destination was never set during our bursts).
3. Frame format: pandad turns on `canfd_auto` for every bus, and with it the panda sends every frame on a bus
   that has CAN-FD traffic as CAN-FD (`panda/board/drivers/fdcan.h`). So our 8-byte 0xC7 went out as CAN-FD.
   This is from the source, not seen on the wire. Nav's copy may be classic CAN, and a receiver can filter on that.

## Open after review: next parked session

The build now has:
- **Frame-format readout.** pandad records the classic/CAN-FD flag of every 0xC7 on bus 0, both the car's and
  our own (from the panda's TX echo). It shows under **Send Request as Classic CAN** in Settings > Vehicle, and is
  logged as `precond: 0xC7 frame format car=… ours=…` (1 = CAN-FD, 0 = classic).
- **Send Request as Classic CAN** toggle. It turns `canfd_auto` off on bus 0 and has pandad flag each frame
  itself: 0xC7 classic, everything else CAN-FD (as before). Frames the panda forwards from the camera to bus 0
  keep their own format in this mode (normally they are all converted to CAN-FD). It only switches while
  openpilot isn't engaged, and it's off after every reboot.

Order (parked, car on, onroad):
1. Leave the toggle off and tap **Start** once. Check that "ours" reads CAN-FD. That confirms the source reading.
2. Turn the toggle on and wait about a second for Mode: classic. Tap **Start**. "Ours" should read classic.
   If preconditioning starts, frame format was the problem (explanation 3).
3. If not: set a DC charger as the nav destination. Nav sends its request, and "car (nav)" now shows nav's format.
   Once preconditioning is on, tap **Cancel** (in whichever format matches nav's). If 0x2AA b17 drops 21 → 1, bus 0
   does reach the BMS, which rules out explanation 1.
4. If the car lets you stop preconditioning while keeping the destination, tap **Start** with the destination
   still active (tests explanation 2). The BMS has ignored a repeat request before (route 139, 1488 s), so a
   no-response here is weak evidence.
5. Turn the toggle off when done (or reboot).
