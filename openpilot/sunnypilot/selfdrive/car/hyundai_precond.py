"""
horizonpilot: battery preconditioning request for E-GMP (GV60).

Replays the 0xC7 frame burst the head unit sends when navigating to a DC charger
(GV60 route 00000139--d597be60c5): three request frames then three idle frames, ~40 ms apart.
Like nav, a start resends the burst at +3.5 s and +10 s until the BMS reports Preparing or On.
Cancel follows the Ioniq 5 M-CAN capture (tylerharvey/Ioniq5_CAN): two zero frames then three idle.

Triggered by writing the HyundaiPrecondCmd param ("start" or "cancel"), from the Vehicle settings
buttons or tools/horizon/precond_test.py. card consumes it, sends the bursts and publishes what the
car reports in the HyundaiPrecondStatus param. Panda safety only allows these exact frames, on E-CAN,
when the car's safety flags enable it (see opendbc HyundaiSafetyFlagsSP.PRECOND).
"""
from opendbc.car.can_definitions import CanData
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP

PRECOND_ADDR = 0xC7
REQUEST = bytes([0x00, 0x00, 0x00, 0x40, 0x03, 0x00, 0x00, 0x00])
CANCEL = bytes(8)
IDLE = bytes([0x00, 0x00, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x00])

BURSTS = {
  "start": [REQUEST] * 3 + [IDLE] * 3,
  "cancel": [CANCEL] * 2 + [IDLE] * 3,
}
FRAME_GAP_NS = 40_000_000  # 40 ms, matches the head unit
RESEND_AT_NS = (3_500_000_000, 10_000_000_000)  # nav on route 139: bursts at +0, +3.5, +10 s
DECLINE_TIMEOUT_NS = 15_000_000_000  # no Preparing/On this long after the first burst -> declined

BMS_OFF, BMS_PREPARING, BMS_ON = 1, 5, 21
BMS_STATE_NAMES = {BMS_OFF: "off", BMS_PREPARING: "preparing", BMS_ON: "on"}

# name -> (addr, byte, mask); all on E-CAN. Temps are 1 degC/bit, no offset (inferred, not OBD-confirmed)
SIGNALS = {
  "bms": (0x2AA, 17, 0x1F),
  "x360": (0x360, 16, 0xFF),
  "pack_min": (0x2FA, 10, 0xFF),
  "pack_max": (0x2FA, 11, 0xFF),
  "inlet": (0x2FA, 5, 0xFF),
  "heater": (0x25A, 18, 0xFF),
}
WATCH_ADDRS = {addr for addr, _, _ in SIGNALS.values()}


def precond_available(CP, CP_SP) -> bool:
  return CP.brand == "hyundai" and bool(CP_SP.safetyParam & HyundaiSafetyFlagsSP.PRECOND)


class PrecondRequester:
  """
  phase: idle -> requesting -> preparing -> on -> done (BMS back to off)
                            \\-> declined (no response in time)
         any -> cancelled (cancel burst sent)
  """
  def __init__(self, CP, CP_SP):
    self.available = precond_available(CP, CP_SP)
    self.bus = CanBus(CP).ECAN if self.available else 0
    self.tx_echo_bus = self.bus + 128

    self.queue: list[bytes] = []
    self.next_send_ns = 0

    self.phase = "idle"
    self.first_burst_ns: int | None = None
    self.bursts_sent = 0
    self.tx_frames_echoed = 0
    self.car_requests_seen = 0
    self.values: dict[str, int] = {}
    self.changed = True

  def command(self, cmd: str, now_nanos: int) -> bool:
    """Start or cancel. Returns True if accepted. A new command replaces an unfinished one."""
    if not self.available or cmd not in BURSTS:
      return False
    self._queue_burst(cmd, now_nanos)
    self.first_burst_ns = now_nanos
    self.bursts_sent = 1
    self.phase = "requesting" if cmd == "start" else "cancelled"
    self.changed = True
    return True

  def _queue_burst(self, cmd: str, now_nanos: int) -> None:
    self.queue = list(BURSTS[cmd])
    self.next_send_ns = now_nanos

  @property
  def busy(self) -> bool:
    return len(self.queue) > 0

  def update_can(self, can_list) -> None:
    """Pick the status signals out of raw CAN: [(nanos, [(addr, dat, src), ...]), ...]"""
    if not self.available:
      return
    for _, frames in can_list:
      for addr, dat, src in frames:
        if addr == PRECOND_ADDR:
          if src == self.tx_echo_bus:
            self.tx_frames_echoed += 1
            self.changed = True
          elif src == self.bus and dat[3:5] == REQUEST[3:5]:
            self.car_requests_seen += 1
            self.changed = True
        elif src == self.bus and addr in WATCH_ADDRS:
          for name, (s_addr, idx, mask) in SIGNALS.items():
            if s_addr == addr and len(dat) > idx:
              v = dat[idx] & mask
              if self.values.get(name) != v:
                self.values[name] = v
                self.changed = True

  def update(self, now_nanos: int) -> list[CanData]:
    bms = self.values.get("bms")

    if self.phase == "requesting":
      if bms in (BMS_PREPARING, BMS_ON):
        self.phase = BMS_STATE_NAMES[bms]
        self.changed = True
      elif not self.busy and self.bursts_sent <= len(RESEND_AT_NS) and \
           now_nanos - self.first_burst_ns >= RESEND_AT_NS[self.bursts_sent - 1]:
        self._queue_burst("start", now_nanos)
        self.bursts_sent += 1
        self.changed = True
      elif now_nanos - self.first_burst_ns > DECLINE_TIMEOUT_NS:
        self.phase = "declined"
        self.changed = True
    elif self.phase == "preparing" and bms == BMS_ON:
      self.phase = "on"
      self.changed = True
    elif self.phase in ("preparing", "on") and bms == BMS_OFF:
      self.phase = "done"
      self.changed = True

    if not self.queue or now_nanos < self.next_send_ns:
      return []
    dat = self.queue.pop(0)
    self.next_send_ns = now_nanos + FRAME_GAP_NS
    return [CanData(PRECOND_ADDR, dat, self.bus)]

  def status(self, now_nanos: int) -> dict:
    self.changed = False
    bms = self.values.get("bms")
    return {
      "phase": self.phase,
      "bms": bms,
      "bms_name": BMS_STATE_NAMES.get(bms, "unknown") if bms is not None else None,
      "since_request_s": round((now_nanos - self.first_burst_ns) / 1e9, 1) if self.first_burst_ns is not None else None,
      "bursts": self.bursts_sent,
      "tx_echoed": self.tx_frames_echoed,
      "car_requests": self.car_requests_seen,
      **{k: v for k, v in self.values.items() if k != "bms"},
    }
