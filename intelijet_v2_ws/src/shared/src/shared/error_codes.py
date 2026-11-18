"""Central registry of operator-facing error/warning codes.

The actual codes (message/level/guide per code) live in
config/error_codes.yaml - not here - so editing the troubleshooting guide
text is a plain YAML edit, no code change/restart needed (same convention
as machine_info.yaml/storage_cleanup.yaml: read directly, not merged into
last_used.yaml, so there's no stale cache to clear). This module just
loads and looks it up.

Adding a new code: add an entry to config/error_codes.yaml, then pass
code="XXX-NNN" to the matching notify() call (or notification_center.push())
- no other file needs to change.
"""
from shared.config_loader import get_config_dir

import os
import yaml

ERROR_CODES_FILE = "error_codes.yaml"


def _load():
    path = os.path.join(get_config_dir(), ERROR_CODES_FILE)
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def lookup(code):
    """Return the registry entry (dict with message/level/guide) for
    `code`, or None if unknown. Re-reads the YAML file every call
    (cheap, tiny file) so a guide-text edit takes effect immediately."""
    return _load().get(code)
