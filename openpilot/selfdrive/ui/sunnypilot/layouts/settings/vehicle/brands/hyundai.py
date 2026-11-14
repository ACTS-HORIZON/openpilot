"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import ListItemSP, dual_button_item_sp, multiple_button_item_sp
from opendbc.car.hyundai.values import CAR, UNSUPPORTED_LONGITUDINAL_CAR


class HyundaiSettings(BrandSettings):
  def __init__(self):
    super().__init__()
    self.alpha_long_available = False

    tuning_texts = [tr("Off"), tr("Dynamic"), tr("Predictive")]
    self.longitudinal_tuning_item = multiple_button_item_sp(tr("Custom Longitudinal Tuning"), "", tuning_texts,
                                                            button_width=300, callback=self._on_tuning_selected,
                                                            param="HyundaiLongitudinalTuning", inline=False)
    # horizonpilot: battery preconditioning test (GV60). card sends the request and writes HyundaiPrecondStatus
    self.precond_available = False
    self.precond_status: dict | None = None
    self.precond_status_read_t = 0.
    self.precond_status_item = ListItemSP(title=lambda: tr("Battery Preconditioning"), description=self._precond_description)
    self.precond_status_item.set_right_value(self._precond_right_value)
    self.precond_buttons = dual_button_item_sp(
      left_text=lambda: tr("Start"),
      left_callback=lambda: self._precond_command("start"),
      right_text=lambda: tr("Cancel"),
      right_callback=lambda: self._precond_command("cancel"),
    )

    self.items = [self.longitudinal_tuning_item, self.precond_status_item, self.precond_buttons]

  def _precond_command(self, cmd: str):
    ui_state.params.put("HyundaiPrecondCmd", cmd)
    self.precond_status_read_t = 0.  # refresh right away

  def _precond_right_value(self) -> str:
    st = self.precond_status
    if st is None:
      return ""
    return {
      "idle": tr("Ready"),
      "requesting": tr("Requested"),
      "preparing": tr("Preparing"),
      "on": tr("Heating"),
      "done": tr("Done"),
      "declined": tr("Declined"),
      "cancelled": tr("Cancel sent"),
    }.get(st.get("phase"), "")

  def _precond_description(self) -> str:
    st = self.precond_status
    if st is None:
      if ui_state.is_offroad():
        return tr("Turn the car on and wait for onroad. Start sends the same request nav sends when routing to a DC charger.")
      return tr("Waiting for car... (not seen yet: needs the updated panda firmware and a GV60 camera harness)")

    def temp(key):
      v = st.get(key)
      return "--" if v is None else f"{v} C / {round(v * 9 / 5 + 32)} F"

    phase = st.get("phase")
    t = st.get("since_request_s")
    lines = []
    if phase == "requesting":
      lines.append(tr("Request burst {} of 3 sent, waiting for the BMS ({} s)").format(st.get("bursts"), t))
    elif phase == "declined":
      lines.append(tr("No response from the BMS within 15 s of the first request."))
    elif phase in ("preparing", "on", "done") and t is not None:
      lines.append(tr("BMS reacted. {} s since first request.").format(t))

    bms = st.get("bms")
    bms_text = "--" if bms is None else f"{st.get('bms_name')} ({bms})"
    lines.append(tr("BMS: {}   Heater: {}   0x360: {}").format(bms_text, st.get("heater", "--"), st.get("x360", "--")))
    lines.append(tr("Coolant in: {}   Pack min: {}   max: {}").format(temp("inlet"), temp("pack_min"), temp("pack_max")))

    if st.get("bursts", 0) and phase != "idle":
      echoed = st.get("tx_echoed", 0)
      if echoed:
        lines.append(tr("Sent {} frames (panda TX confirmed).").format(echoed))
      elif t is not None and t > 1.0:
        lines.append(tr("WARNING: nothing was transmitted. Panda blocked it or has old firmware."))
    if st.get("car_requests"):
      lines.append(tr("Head unit also sent {} request frames.").format(st.get("car_requests")))
    return "\n".join(lines)

  def _update_precond(self):
    platform = None
    if bundle := ui_state.params.get("CarPlatformBundle"):
      platform = bundle.get("platform")
    elif ui_state.CP is not None:
      platform = ui_state.CP.carFingerprint
    self.precond_available = platform == CAR.GENESIS_GV60_EV_1ST_GEN

    if self.precond_available and time.monotonic() - self.precond_status_read_t > 0.5:
      self.precond_status_read_t = time.monotonic()
      self.precond_status = None if ui_state.is_offroad() else ui_state.params.get("HyundaiPrecondStatus")

    self.precond_status_item.set_visible(self.precond_available)
    self.precond_status_item.show_description(True)
    self.precond_buttons.set_visible(self.precond_available)
    precond_ready = self.precond_status is not None
    self.precond_buttons.action_item.left_button.set_enabled(precond_ready)
    self.precond_buttons.action_item.right_button.set_enabled(precond_ready)

  @staticmethod
  def _on_tuning_selected(index):
    ui_state.params.put("HyundaiLongitudinalTuning", index)

  def update_settings(self):
    self._update_precond()

    self.alpha_long_available = False
    bundle = ui_state.params.get("CarPlatformBundle")
    if bundle:
      platform = bundle.get("platform")
      self.alpha_long_available = CAR[platform] not in set().union(*UNSUPPORTED_LONGITUDINAL_CAR.values())
    elif ui_state.CP is not None:
      self.alpha_long_available = ui_state.CP.alphaLongitudinalAvailable

    tuning_param = int(ui_state.params.get("HyundaiLongitudinalTuning") or "0")
    long_enabled = ui_state.has_longitudinal_control

    long_tuning_descs = [
      tr("Your vehicle will use the Default longitudinal tuning."),
      tr("Your vehicle will use the Dynamic longitudinal tuning."),
      tr("Your vehicle will use the Predictive longitudinal tuning."),
    ]
    long_tuning_desc = long_tuning_descs[tuning_param] if tuning_param < len(long_tuning_descs) else long_tuning_descs[0]

    longitudinal_tuning_disabled = not ui_state.is_offroad() or not long_enabled
    if longitudinal_tuning_disabled:
      if not ui_state.is_offroad():
        long_tuning_desc = tr("This feature is unavailable while the car is onroad.")
      elif not long_enabled:
        long_tuning_desc = tr("This feature is unavailable because sunnypilot Longitudinal Control (Alpha) is not enabled.")

    self.longitudinal_tuning_item.action_item.set_enabled(not longitudinal_tuning_disabled)
    self.longitudinal_tuning_item.set_description(long_tuning_desc)
    self.longitudinal_tuning_item.show_description(True)
    self.longitudinal_tuning_item.action_item.set_selected_button(tuning_param)
    self.longitudinal_tuning_item.set_visible(self.alpha_long_available)
