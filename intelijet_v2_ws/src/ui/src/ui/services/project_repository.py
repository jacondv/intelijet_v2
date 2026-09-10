# ui/services/project_repository.py
"""Single source of truth for the Projects/Jobs filesystem layout.

A project is a folder under PROJECT_DIR; a job is a subfolder of a
project containing job_info.json (see ui.models.job_info.JobInfo).
Before this module existed, PROJECT_DIR/ACTIVE_JOB_FILE and the
"list projects by scanning the directory" loop were copy-pasted
independently into app.py, project_dlg_manager.py, compare_dlg_manager.py,
report_view_dlg_manager.py and history_page_manager.py - this is the
one place that should own that logic now; the others should import
from here instead of re-declaring it.

No PyQt/widget dependency - raises ProjectError for user-facing failures
(already exists, not found, ...); callers (UI code) decide how to show
that (QMessageBox, notification, etc).
"""
import os
import shutil

from shared.config_loader import CONFIG as cfg
from ui.models.job_info import JobInfo
from ui.models.file_name import is_sync_junk

BASE_DIR = cfg.BASE_DIR
DATA_DIR = cfg.DATA_DIR
PROJECT_DIR = os.path.join(BASE_DIR, DATA_DIR, "Projects")
ACTIVE_JOB_FILE = os.path.join(PROJECT_DIR, "active_jobs.json")
CURRENT_JOB_FILE = os.path.join(PROJECT_DIR, "current_job.json")


class ProjectError(Exception):
    """User-facing failure (already exists, not found, ...) - the message
    is written to be shown to the operator as-is."""


def project_path(project):
    return os.path.join(PROJECT_DIR, project)


def job_path(project, job):
    return os.path.join(PROJECT_DIR, project, job)


def parse_job_ref(text, sep="/"):
    """Split a 'Project / Job' display string into (project, job) -
    tolerant of a missing job half (returned as None). Centralizes what
    used to be split("/") in some call sites and split("/", 1) in
    others, which silently disagreed on how a "/" inside a job name
    would be handled."""
    parts = [p.strip() for p in text.split(sep, 1)]
    parts += [None] * (2 - len(parts))
    return parts[0], parts[1]


SORT_NAME_ASC = "name_asc"
SORT_NAME_DESC = "name_desc"
SORT_DATE_DESC = "date_desc"  # newest created first
SORT_DATE_ASC = "date_asc"    # oldest created first

# Kept as the plain alphabetical order every existing caller (report
# picker, tests) already relies on. The Job tab's project list defaults
# to SORT_DATE_DESC instead, but does so explicitly (see
# project_dlg_manager.py) rather than by changing this module default.
DEFAULT_SORT_MODE = SORT_NAME_ASC


def list_projects(sort_mode=DEFAULT_SORT_MODE):
    """Project names on disk (Syncthing junk filtered out), ordered per
    `sort_mode`. Creates PROJECT_DIR if it doesn't exist yet.

    Projects carry no creation-date metadata of their own (unlike jobs -
    see JobInfo.created), so "creation date" uses the project folder's
    filesystem ctime, same convention already used for date sort in
    report_page_manager.py.
    """
    os.makedirs(PROJECT_DIR, exist_ok=True)
    names = [
        name for name in os.listdir(PROJECT_DIR)
        if not is_sync_junk(name) and os.path.isdir(os.path.join(PROJECT_DIR, name))
    ]
    if sort_mode == SORT_NAME_ASC:
        return sorted(names, key=str.lower)
    if sort_mode == SORT_NAME_DESC:
        return sorted(names, key=str.lower, reverse=True)
    by_ctime = sorted(names, key=lambda n: os.path.getctime(os.path.join(PROJECT_DIR, n)))
    if sort_mode == SORT_DATE_ASC:
        return by_ctime
    return list(reversed(by_ctime))  # SORT_DATE_DESC (default)


def project_dir_size_bytes():
    """Total size in bytes of everything under PROJECT_DIR. Walks the
    whole tree (os.walk + os.path.getsize) - can take a while against a
    data-heavy install, so callers should run this off the GUI thread and
    poll it infrequently (see RosThread._update_storage_stats, every 10
    minutes) rather than on every UI refresh."""
    total = 0
    for root, _dirs, files in os.walk(PROJECT_DIR):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(root, name))
            except OSError:
                pass  # file removed/renamed mid-walk - skip it, not fatal
    return total


def list_jobs(project):
    """Sorted job names under `project` (Syncthing junk filtered out).
    Returns [] if the project doesn't exist."""
    p = project_path(project)
    if not os.path.isdir(p):
        return []
    return sorted(
        name for name in os.listdir(p)
        if not is_sync_junk(name) and os.path.isdir(os.path.join(p, name))
    )


def create_project(name):
    p = project_path(name)
    if os.path.exists(p):
        raise ProjectError(f"Project '{name}' already exists.")
    os.makedirs(p)


def rename_project(old_name, new_name):
    old_p, new_p = project_path(old_name), project_path(new_name)
    if os.path.exists(new_p):
        raise ProjectError(f"Project '{new_name}' already exists.")
    os.rename(old_p, new_p)


def delete_project(name):
    p = project_path(name)
    if os.path.exists(p):
        shutil.rmtree(p)


def create_job(project, job_info):
    """job_info: a JobInfo instance (job_info.name is the folder name).
    Creates the job folder and writes job_info.json into it."""
    p = job_path(project, job_info.name)
    if os.path.exists(p):
        raise ProjectError(f"Job '{job_info.name}' already exists.")
    os.makedirs(p)
    if not job_info.save(p):
        raise ProjectError(f"Unable to save job info for '{job_info.name}'.")


def rename_job(project, old_name, new_name):
    old_p, new_p = job_path(project, old_name), job_path(project, new_name)
    if os.path.exists(new_p):
        raise ProjectError(f"Job '{new_name}' already exists.")
    os.rename(old_p, new_p)
    job_info = JobInfo.load(new_p)
    if job_info is not None:
        job_info.name = new_name
        job_info.save(new_p)


def delete_job(project, job):
    p = job_path(project, job)
    if os.path.exists(p):
        shutil.rmtree(p)


def load_job_info(project, job):
    """Returns a JobInfo, or None if job_info.json is missing/corrupt."""
    return JobInfo.load(job_path(project, job))


def save_job_info(project, job_info):
    return job_info.save(job_path(project, job_info.name))
