"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Horizon Dev — Collin's on-device advanced knobs / CAN experiment bench (horizonpilot).
Lets values be tested live on the comma without switching branches or rebuilding.

What's live in this build:
  * Steer Damp Override — overrides the LFA `Damping_Gain` signal (stock is a speed-interp table).
    Rides a message openpilot already sends, so it needs no panda change. Applies mid-drive.

Staged behind a deliberate panda reflash (see HORIZON_DEV_MENU.md at the repo root):
  * Max Steer above the 409 panda cap — the live Max Steer knob lives in
    Steering -> Customize Torque Params and works up to 409 with no reflash.
  * CAN-send experiments (battery preconditioning 0xC7, ccIC CCNC_0x161 bus probe) — the panda
    TX allowlist blocks these addresses until patched, so the master arm below is inert until then.
"""
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp, option_item_sp, LineSeparatorSP
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.scroller_tici import Scroller


class HorizonDevLayout(Widget):
  def __init__(self):
    super().__init__()
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self):
    # --- Steer Damp Override (live) ---
    self._damp_toggle = toggle_item_sp(
      param="HorizonSteerDampEnabled",
      title=lambda: tr("Steer Damp Override"),
      description=lambda: tr("Override the LFA Damping_Gain signal with a fixed value instead of the stock " +
                             "speed-interpolated table (5 at 0 mph rising to 200 at ~85 mph). Higher = more " +
                             "damping / heavier wheel. Rides a message openpilot already sends, so no panda " +
                             "change is needed and it applies while driving. Disable to return to the stock table."),
    )
    self._damp_gain = option_item_sp(
      title=lambda: tr("Damping Gain"),
      param="HorizonSteerDampGain",
      description="",
      min_value=3,
      max_value=200,
      value_change_step=1,
      label_callback=(lambda x: f"{x}  (stock 5-200 by speed)"),
    )

    # --- CAN-send experiments (staged; requires panda TX patch) ---
    self._can_lab_toggle = toggle_item_sp(
      param="HorizonCanLabArmed",
      title=lambda: tr("Arm CAN-Send Experiments"),
      description=lambda: tr("Master arm for the parked CAN-send probes (battery preconditioning 0xC7, " +
                             "ccIC CCNC_0x161 bus test). INERT until you flash the panda TX allowlist patch " +
                             "and apply the carcontroller send patch documented in HORIZON_DEV_MENU.md. " +
                             "Until then the panda drops these frames. Bench/parked test only."),
    )

    return [
      self._damp_toggle,
      self._damp_gain,
      LineSeparatorSP(40),
      self._can_lab_toggle,
    ]

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._scroller.show_event()
    self._damp_toggle.show_description(True)
    self._can_lab_toggle.show_description(True)

  def _update_state(self):
    super()._update_state()

    # Damp override: offroad-only toggle; gain stepper visible only when enabled.
    self._damp_toggle.action_item.set_enabled(ui_state.is_offroad())
    damp_enabled = self._damp_toggle.action_item.get_state()
    self._damp_gain.set_visible(damp_enabled)
    self._damp_gain.action_item.set_enabled(ui_state.is_offroad())

    # CAN-send arm: offroad-only. Harmless without the panda patch (panda blocks the addresses).
    self._can_lab_toggle.action_item.set_enabled(ui_state.is_offroad())
