#!/usr/bin/env python3
"""Auto-deletes old scan data under <BASE_DIR>/<DATA_DIR>/Projects to keep
disk usage bounded, per config/storage_cleanup.yaml (max_data_gb,
max_projects, check_interval_seconds).

Two independent caps, both enforced every cycle:
  1. Total size of the Projects folder <= max_data_gb. If over, strip the
     Pre-Scan/Post-Scan .ply files (the bulky raw clouds) from the oldest
     eligible project, re-check, and move on to the next-oldest project
     if still over. job_info.json, compared-cloud results and PDF
     reports are left alone - only the raw pre/post-scan clouds are
     deleted, since those are what's reproducible-but-large and what
     compare results/reports were already generated from.
  2. Number of projects <= max_projects. If over, delete the oldest
     eligible project's folder entirely.

"Oldest" = the project folder's own creation time on disk (os.stat().
st_ctime) - simplest, doesn't require reading every job's job_info.json.

"Eligible" excludes any project that has at least one job present in
active_jobs.json (Work Schedule) - same protection
project_repository.delete_project()'s callers already apply for manual
deletes, so this auto-cleanup can never touch data someone is actively
using. If every remaining project is active, cleanup stops early (logs
a warning) rather than violating that rule.

Standalone node (not the UI process) - independent of app.py running,
same as compare_cloud_action_server.py etc. Deliberately does NOT import
anything from the `ui` package (that dependency only ever goes the other
way - ui imports from pps/shared) - active_jobs.json's simple
[{"project":..., "job":...}] format is read directly here instead of
via ui.services.job_store.JobStore.
"""
import json
import os
import re
import shutil
import time

import rospy

from shared.config_loader import CONFIG as cfg, load_config
from shared.notify import notify

PROJECT_DIR = os.path.join(cfg.BASE_DIR, cfg.DATA_DIR, "Projects")
ACTIVE_JOB_FILE = os.path.join(PROJECT_DIR, "active_jobs.json")

_SYNC_JUNK_RE = re.compile(r"\.sync-conflict-\d{8}-\d{6}-[A-Z0-9]+", re.IGNORECASE)


def _is_sync_junk(name):
    """Mirrors ui.models.file_name.is_sync_junk() - duplicated (not
    imported) to keep this node independent of the `ui` package."""
    lname = name.lower()
    if lname in (".stfolder", ".stversions", ".stignore"):
        return True
    if _SYNC_JUNK_RE.search(name):
        return True
    if lname.startswith("~syncthing~") or lname.endswith(".tmp"):
        return True
    return False


def _is_prescan_or_postscan_ply(filename):
    """True for a raw Pre-Scan/Post-Scan cloud file - NOT a compared
    result (ui.models.file_name.FILENAME_TEMPLATE embeds the scan type
    right after the timestamp, e.g. "...#pre_scan_cloud_01#SCAN001.ply"
    vs "...#cloud_compared_01#SCAN001.ply" - matching on that substring
    avoids needing the full filename parser)."""
    lower = filename.lower()
    if not lower.endswith(".ply"):
        return False
    return "#pre_scan_cloud_" in lower or "#post_scan_cloud_" in lower


def _list_projects():
    if not os.path.isdir(PROJECT_DIR):
        return []
    return [
        name for name in os.listdir(PROJECT_DIR)
        if not _is_sync_junk(name) and os.path.isdir(os.path.join(PROJECT_DIR, name))
    ]


def _project_ctime(project):
    try:
        return os.stat(os.path.join(PROJECT_DIR, project)).st_ctime
    except OSError:
        return float("inf")  # can't stat it - sort last, don't pick it first


def _oldest_first(projects):
    return sorted(projects, key=_project_ctime)


def _active_projects():
    """Project names with >=1 entry in active_jobs.json - never touched
    by this node."""
    if not os.path.exists(ACTIVE_JOB_FILE):
        return set()
    try:
        with open(ACTIVE_JOB_FILE, "r") as f:
            jobs = json.load(f)
    except Exception as e:
        rospy.logwarn(f"[storage_cleanup] Failed to read {ACTIVE_JOB_FILE}: {e}")
        return set()
    if not isinstance(jobs, list):
        return set()
    return {j.get("project") for j in jobs if isinstance(j, dict) and j.get("project")}


def _dir_size_bytes(path):
    total = 0
    for dirpath, _dirnames, filenames in os.walk(path):
        for name in filenames:
            fp = os.path.join(dirpath, name)
            try:
                if not os.path.islink(fp):
                    total += os.path.getsize(fp)
            except OSError:
                pass  # file removed/inaccessible mid-walk - skip, don't crash the cycle
    return total


def _delete_prescan_postscan_ply_files(project):
    """Delete every Pre-Scan/Post-Scan .ply under `project` (all jobs).
    Returns bytes freed."""
    project_path = os.path.join(PROJECT_DIR, project)
    freed = 0
    deleted = 0
    for dirpath, _dirnames, filenames in os.walk(project_path):
        for name in filenames:
            if not _is_prescan_or_postscan_ply(name):
                continue
            fp = os.path.join(dirpath, name)
            try:
                freed += os.path.getsize(fp)
                os.remove(fp)
                deleted += 1
            except OSError as e:
                rospy.logwarn(f"[storage_cleanup] Failed to delete {fp}: {e}")
    return freed, deleted


def _enforce_max_data_size(max_bytes):
    """Repeatedly strip the oldest eligible project's Pre-Scan/Post-Scan
    .ply files until the Projects folder is back under max_bytes, or
    there's nothing left that's safe to touch. Returns True if the cap
    is satisfied by the time this returns, False if it had to give up
    early (every remaining project is active, or stripping is stuck) -
    callers must check this, not just loop and hope, or a permanently
    over-cap folder (every project active) would spin this forever."""
    active = _active_projects()
    already_stripped = set()  # avoid re-picking a project that freed 0 bytes (infinite loop)

    while True:
        size = _dir_size_bytes(PROJECT_DIR)
        if size <= max_bytes:
            return True

        candidates = _oldest_first(
            p for p in _list_projects()
            if p not in active and p not in already_stripped
        )
        if not candidates:
            rospy.logwarn(
                f"[storage_cleanup] Projects folder is {size / 1e9:.1f} GB "
                f"(cap {max_bytes / 1e9:.1f} GB) but every remaining project "
                "either has an active job or was already stripped - stopping."
            )
            notify(
                message=(
                    f"[WARN] Data folder over {max_bytes / 1e9:.0f} GB cap and no more "
                    "space can be freed automatically (remaining projects are active or empty)."
                ),
                level="warning",
                source="StorageCleanup",
            )
            return False

        oldest = candidates[0]
        freed, deleted = _delete_prescan_postscan_ply_files(oldest)
        if freed == 0:
            already_stripped.add(oldest)
            continue

        rospy.loginfo(
            f"[storage_cleanup] Freed {freed / 1e9:.2f} GB ({deleted} files) "
            f"deleting Pre-Scan/Post-Scan clouds from oldest project '{oldest}'"
        )
        notify(
            message=(
                f"[INFO] Data folder over {max_bytes / 1e9:.0f} GB cap - deleted "
                f"{deleted} Pre-Scan/Post-Scan file(s) ({freed / 1e9:.2f} GB) from "
                f"oldest project '{oldest}'."
            ),
            level="info",
            source="StorageCleanup",
        )


def _enforce_max_project_count(max_projects):
    """Repeatedly delete the oldest eligible project entirely until the
    project count is back under max_projects, or there's nothing left
    that's safe to delete. Returns True/False - see
    _enforce_max_data_size's docstring, same contract."""
    active = _active_projects()

    while True:
        projects = _list_projects()
        if len(projects) <= max_projects:
            return True

        candidates = _oldest_first(p for p in projects if p not in active)
        if not candidates:
            rospy.logwarn(
                f"[storage_cleanup] {len(projects)} projects (cap {max_projects}) "
                "but every remaining project has an active job - stopping."
            )
            notify(
                message=(
                    f"[WARN] Project count over the {max_projects}-project cap and no "
                    "more can be deleted automatically (remaining projects are active)."
                ),
                level="warning",
                source="StorageCleanup",
            )
            return False

        oldest = candidates[0]
        try:
            shutil.rmtree(os.path.join(PROJECT_DIR, oldest))
        except OSError as e:
            rospy.logwarn(f"[storage_cleanup] Failed to delete project '{oldest}': {e}")
            return False  # don't loop forever retrying the same failure
        rospy.loginfo(f"[storage_cleanup] Deleted oldest project '{oldest}' (over max_projects cap)")
        notify(
            message=f"[INFO] Project count over {max_projects} cap - deleted oldest project '{oldest}'.",
            level="info",
            source="StorageCleanup",
        )


def run_cleanup_cycle(max_data_gb, max_projects):
    """Both caps are enforced every cycle: size first, then count -
    deleting a whole project (count cap) also frees size, so running
    size-cleanup first and count-cleanup second means a project deleted
    for being over the count cap can never leave the size cap need-
    lessly re-triggered on the very next cycle. Each _enforce_* call
    already loops internally to completion (satisfied, or gives up when
    nothing eligible remains - see their docstrings), so this function
    itself never loops - rospy.Timer re-invokes it on its own interval,
    which is retry enough."""
    max_bytes = max_data_gb * (1024 ** 3)
    _enforce_max_data_size(max_bytes)
    _enforce_max_project_count(max_projects)


def main():
    rospy.init_node("storage_cleanup_node")

    def _tick(_event=None):
        try:
            settings = load_config("storage_cleanup.yaml")
            run_cleanup_cycle(settings.max_data_gb, settings.max_projects)
        except Exception as e:
            rospy.logerr(f"[storage_cleanup] Cleanup cycle failed: {e}")

    settings = load_config("storage_cleanup.yaml")
    rospy.loginfo(
        f"[storage_cleanup] Started - max_data_gb={settings.max_data_gb}, "
        f"max_projects={settings.max_projects}, "
        f"check_interval_seconds={settings.check_interval_seconds}"
    )
    _tick()  # run once at startup, don't wait a full interval first
    rospy.Timer(rospy.Duration(settings.check_interval_seconds), _tick)
    rospy.spin()


if __name__ == "__main__":
    main()
