# Horizon Dev — on-device advanced knobs

Personal dev menu for testing tuning + CAN values live on the comma without switching
branches or rebuilding. Branch: `claude/horizon-dev-menu` (off `claude/fork-sync-sunny-master-mfg1m3`).

Everything defaults to **stock** when its toggle is off. Nothing here widens the panda safety
envelope on its own — the two experiments that would (max steer above the cap, raw CAN sends)
are gated behind a deliberate panda reflash described at the bottom.

---

## What's live now (no panda change, safe to daily-drive at defaults)

### 1. Max Steer knob — `Settings → Steering → Customize Torque Params`
- Toggle **Max Steer Override (Horizon Dev)**, then set **Max Steer (STEER_MAX)** (150–409, step 5).
- Overrides the `STEER_MAX` count that `actuators.torque` is scaled to, applied consistently to
  the scale, the driver-torque limit, and the output normalization.
- **Capped at 409** in the UI and again in code (`PANDA_STEER_CAP`), because the panda hard-caps
  requested CAN-FD steer torque at 409. So this knob can only **lower** steer authority from stock
  unless you do the reflash in §A below.
- Offroad-only to change; reads live (~3 s) so it takes effect without a restart. Disable → stock 409.
- Use it to test the darty/limit-cycle hypotheses at *lower* authority, or to A/B a reduced cap.

### 2. Steer Damp Override — `Settings → Horizon Dev`
- Toggle **Steer Damp Override**, then set **Damping Gain** (3–200).
- Overrides the LFA `Damping_Gain` signal (stock is the speed-interp table
  `_DAMP_FACTOR_BP/_V` = 5 at 0 mph → 200 at ~85 mph in `hyundaicanfd.py`).
- Rides the LFA message openpilot already sends → **no panda change**, applies mid-drive.
- Disable → stock speed table.

### Params (registered in `openpilot/common/params_keys.h`)
| Param | Type | Default | Meaning |
|---|---|---|---|
| `HorizonSteerMaxEnabled` | BOOL | 0 | enable the max-steer override |
| `HorizonSteerMax` | INT | 409 | effective STEER_MAX (clamped ≤ `PANDA_STEER_CAP`) |
| `HorizonSteerDampEnabled` | BOOL | 0 | enable the damp override |
| `HorizonSteerDampGain` | INT | 100 | LFA `Damping_Gain` [3–200] |
| `HorizonCanLabArmed` | BOOL | 0 | master arm for CAN-send experiments (inert until §B) |

You can also drive these over SSH without the UI, e.g.:
```
echo -n 1   > /data/params/d/HorizonSteerDampEnabled
echo -n 140 > /data/params/d/HorizonSteerDampGain
```
(param names are case-sensitive; the reader picks them up within ~3 s.)

### Code touchpoints (live path)
- `opendbc/sunnypilot/car/hyundai/horizon_dev.py` — throttled param reader, fails closed to stock.
- `opendbc/car/hyundai/carcontroller.py` — `self.horizon.update()`; `steer_max` threaded through
  the torque scale, `apply_driver_steer_torque_limits(..., steer_max=steer_max)`, and the output.
- `opendbc/car/hyundai/hyundaicanfd.py` — `create_steering_messages(..., damp_override=...)`.
- `selfdrive/ui/.../steering_sub_layouts/torque_settings.py` — max-steer knob.
- `selfdrive/ui/.../settings/horizon_dev.py` + `settings.py` — the Horizon Dev panel.

---

## §A. Max steer ABOVE 409 (requires a panda reflash — bench/parked test first)

The GV60's stock `STEER_MAX` (409) already equals the panda's hard cap
(`HYUNDAI_CANFD_STEERING_LIMITS.max_torque = 409`). To request more, raise the firmware cap and
reflash. **This widens the actual hardware safety limit — do it deliberately, bump conservatively,
and test parked before any road use.**

1. `opendbc/safety/modes/hyundai_canfd.h` → `HYUNDAI_CANFD_STEERING_LIMITS`:
   ```c
   .max_torque = 450,   // was 409 — pick the smallest bump you actually need
   ```
   Consider whether `.max_rate_up/.max_rate_down` (10) and `.max_rt_delta` (325) also need to move;
   for a modest cap bump they usually don't, but a higher cap reaches larger steps sooner.
2. `opendbc/safety/tests/test_hyundai_canfd.py` → `MAX_TORQUE_LOOKUP = [0], [409]` → set to your
   new value, or `make -C opendbc/safety` tests will fail.
3. `opendbc/sunnypilot/car/hyundai/horizon_dev.py` → `PANDA_STEER_CAP = 450` to match.
4. `selfdrive/ui/.../torque_settings.py` → raise the stepper `max_value=409` to your new cap.
5. Reboot the comma. openpilot rebuilds and reflashes panda firmware when it changes; confirm no
   panda fault, then test steer authority **parked** (wheel against light resistance) before driving.

Note: `STEER_DELTA_UP/DOWN` in `values.py` (10 for CAN-FD) still bounds how fast torque ramps —
raising the cap doesn't make it snap; it lets sustained demand reach higher. This is the lever for
the "tight low-speed corners bounded by STEER_MAX" item in the tuning notes.

---

## §B. CAN-send experiments — preconditioning (0xC7) & ccIC (CCNC_0x161)

These send addresses the panda does **not** currently allow, so they're **blocked (inert)** until
you patch the TX allowlist and reflash. The `Arm CAN-Send Experiments` toggle exists now but does
nothing until this is applied. **Parked, offroad, engine-off-or-ready only. Wrong frames can fault
the EPS or throw cluster errors — this is your daily driver.**

### B1. Allow the addresses in the panda TX list
In `opendbc/safety/modes/hyundai_canfd.h`, add entries to the tx_msgs list your GV60 config
resolves to (it uses the LFA-steering path, not `CANFD_LKA_STEER_MSG`; grep `hyundai_canfd_init`
to confirm which `*_TX_MSGS` array is selected for your flags), e.g.:
```c
{0xC7,  0, 8,  .check_relay = false},  // Toggle_Battery_Preconditioning (dev)
{0x161, 0, 32, .check_relay = false},  // CCNC_0x161 ccIC probe (dev) — try bus 0 then 1
```
Also relax/append the tx_hook if you want the frames unchecked, or leave them to pass through as
"unknown" (default `tx = true` when no per-address check matches). Update the safety tests to expect
the new tx_msgs. Reflash by rebooting.

### B2. Wire momentary sends in the carcontroller
Add pulse params (register in `params_keys.h`): `HorizonPulsePrecondOn`, `HorizonPulsePrecondOff`,
`HorizonPulseCcicBus`. In `horizon_dev.py`, read them (they're one-shot: read → act → clear).
In `carcontroller.py`, gated by `HorizonCanLabArmed`, `not CC.enabled`, and `CS.out.vEgo < 0.1`:
```python
# battery preconditioning (Ioniq6-confirmed recipe: bytes 3-4 = 0x40 0x03 ON / 0xE0 0x07 OFF)
if self.horizon.pulse_precond_on:
    dat = b"\x00\x00\x00\x40\x03\x00\x00\x00"
    can_sends += [make_can_msg(0xC7, dat, self.CAN.ECAN)] * 5   # a few frames
# ccIC probe: send CCNC_0x161 on the selected bus and watch the cluster
```
Send raw bytes with `make_can_msg` (no DBC entry needed) for 0xC7; for CCNC_0x161 you'll want the
`CCNC_0x161` definition from sunnypilot's `hyundai_canfd.dbc` in your packer, or send raw bytes.
Then add momentary buttons to the Horizon Dev panel that set the pulse params.

### B3. Reference values (from the openpilot skill notes)
- `0xC7` Toggle_Battery_Preconditioning: bytes 3–4 `0x40 0x03` = ON, `0xE0 0x07` = OFF/idle.
- Read-back: `0x2AD` `Battery_Precond_State` 21=on / 5=preparing / 1=off (event-driven).
- `CCNC_0x161` = ID 353; probe **bus 0 first, then bus 1**, watching for cluster response — this is
  the open ccIC question (write-only hypothesis: the cluster may just need the frame sent).

---

## Rollback
- Live knobs: flip each toggle off (or `git checkout` the branch you were on).
- §A/§B panda changes: revert the safety edits and reboot to reflash stock firmware. Confirm
  `HYUNDAI_CANFD_STEERING_LIMITS.max_torque` is back to 409 and tx_msgs no longer lists 0xC7/0x161.
