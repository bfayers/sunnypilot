"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

``carry_over_picks`` keeps each device's pick across a selector bump (``REQUIRED_JSON_VERSION``) or a
catalog that rebuilt the model, which would otherwise drop every device back to the stock model.
"""

from openpilot.cereal import custom
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot import jetlink_adapter
from openpilot.sunnypilot.models.helpers import get_selected_bundle

CARRY_OVER_KEY = "ModelManager_CarryOver"
_carry_over_queued: set[str] = set()


def carry_over_picks(params, source_bundles: dict[str, list["custom.ModelManagerSP.ModelBundle"]],
                     cleared: dict[str, dict]) -> None:
  """Run after validate_active_bundles, with the picks it cleared: one stored under an older selector
  version no longer parses, and one whose catalog entry changed (a new catalog, or the big-model list
  changing hands between sunnypilot's and jetlink's) no longer matches. Once the catalog lists the
  same model (by ref, else for a big model by its ONNX, which jetlink lists once under its newest ref,
  else by short name) queue that download, which fills the slot again.
  Kept until the slot is filled, so an offline first boot or a download cleared by going onroad tries
  again; queued once per model per process, so a download that keeps failing does not spin."""
  pending = params.get(CARRY_OVER_KEY) or {}
  changed = False
  for source, raw in cleared.items():
    if isinstance(raw, dict) and raw:
      pending[source] = {"ref": raw.get("ref", ""), "internalName": raw.get("internalName", "")}
      changed = True

  for source, pick in list(pending.items()):
    bundles = source_bundles.get(source) or []
    if get_selected_bundle(params, source) is not None:  # downloaded, or the user picked another
      del pending[source]
      changed = True
      continue
    if not bundles:  # offline, or the catalog is not fetched yet
      continue
    match = next((b for b in bundles if pick["ref"] and b.ref == pick["ref"]), None) or \
      _same_onnx(bundles, pick["ref"]) or \
      max((b for b in bundles if b.ref and b.internalName == pick["internalName"]), key=lambda b: b.index, default=None)
    if match is None:
      cloudlog.warning(f"Model pick {pick} is not in the {source} catalog, leaving the default")
      del pending[source]
      changed = True
    elif match.ref not in _carry_over_queued and params.get("ModelManager_DownloadRef") is None:
      _carry_over_queued.add(match.ref)
      params.put("ModelManager_DownloadRef", match.ref)
      cloudlog.warning(f"Carrying the {source} pick {match.displayName} over to the new catalog (queued ref {match.ref})")

  if changed:
    if pending:
      params.put(CARRY_OVER_KEY, pending)
    else:
      params.remove(CARRY_OVER_KEY)


def _same_onnx(bundles: list["custom.ModelManagerSP.ModelBundle"], ref: str) -> "custom.ModelManagerSP.ModelBundle | None":
  """The bundle jetlink's list names a big pick's ONNX under: a recompile is listed once, under its
  newest ref. None while the list is not jetlink's, or for a model it does not list."""
  listed = jetlink_adapter.listed_ref(ref)
  return next((b for b in bundles if b.ref == listed), None) if listed else None
