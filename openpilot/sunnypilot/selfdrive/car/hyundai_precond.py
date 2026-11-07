"""
horizonpilot: battery preconditioning request for E-GMP (GV60).

Replays the 0xC7 frame burst the head unit sends when navigating to a DC charger
(GV60 route 00000139--d597be60c5): three request frames then three idle frames, ~40 ms apart.
Cancel follows the Ioniq 5 M-CAN capture (tylerharvey/Ioniq5_CAN): two zero frames then three idle.

Triggered by writing the HyundaiPrecondCmd param ("start" or "cancel"); card picks it up,
clears it and sends one burst. Panda safety only allows these exact frames, on E-CAN, when the
car's safety flags enable it (see opendbc HyundaiSafetyFlagsSP.PRECOND).
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


def precond_available(CP, CP_SP) -> bool:
  return CP.brand == "hyundai" and bool(CP_SP.safetyParam & HyundaiSafetyFlagsSP.PRECOND)


class PrecondRequester:
  def __init__(self, CP, CP_SP):
    self.available = precond_available(CP, CP_SP)
    self.bus = CanBus(CP).ECAN if self.available else 0
    self.queue: list[bytes] = []
    self.next_send_ns = 0

  def command(self, cmd: str, now_nanos: int) -> bool:
    """Queue a burst. Returns True if accepted. A new command replaces an unfinished burst."""
    if not self.available or cmd not in BURSTS:
      return False
    self.queue = list(BURSTS[cmd])
    self.next_send_ns = now_nanos
    return True

  @property
  def busy(self) -> bool:
    return len(self.queue) > 0

  def update(self, now_nanos: int) -> list[CanData]:
    if not self.queue or now_nanos < self.next_send_ns:
      return []
    dat = self.queue.pop(0)
    self.next_send_ns = now_nanos + FRAME_GAP_NS
    return [CanData(PRECOND_ADDR, dat, self.bus)]
