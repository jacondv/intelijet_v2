# ui/services/job_store.py
"""Single source of truth for active_jobs.json / current_job.json.

No PyQt/widget dependency - App owns the QComboBox, JobStore only reads
and writes the two JSON files. Writes are atomic (write to a temp file,
then os.replace) so a power loss mid-write can't corrupt the file.
"""
import json
import os

try:
    import rospy
    _log_warn = rospy.logwarn
except ImportError:  # pragma: no cover - keeps this importable outside ROS
    _log_warn = print


def _atomic_write_json(path, data):
    tmp_path = f"{path}.tmp"
    with open(tmp_path, "w") as f:
        json.dump(data, f, indent=4)
    os.replace(tmp_path, path)


class JobStore:
    def __init__(self, active_job_file, current_job_file):
        self._active_job_file = active_job_file
        self._current_job_file = current_job_file
        self._active_jobs_cache = []
        self.reload()

    def reload(self):
        """Re-read active_jobs.json from disk into the in-memory cache."""
        self._active_jobs_cache = self._read_active_jobs()

    def _read_active_jobs(self):
        if not os.path.exists(self._active_job_file):
            return []
        try:
            with open(self._active_job_file, "r") as f:
                jobs = json.load(f)
        except Exception as e:
            _log_warn(f"[JobStore] Failed to read {self._active_job_file}: {e}")
            return []
        if not isinstance(jobs, list):
            _log_warn(f"[JobStore] {self._active_job_file} does not contain a list, ignoring")
            return []
        return jobs

    def list_active_jobs(self):
        """Return the cached list of {'project': ..., 'job': ...} dicts."""
        return list(self._active_jobs_cache)

    def get_current_job(self):
        """Return the 'project/job' string of the currently selected job,
        or None if unset/unreadable."""
        if not os.path.exists(self._current_job_file):
            return None
        try:
            with open(self._current_job_file, "r") as f:
                data = json.load(f)
            return data.get("current_job")
        except Exception as e:
            _log_warn(f"[JobStore] Failed to read {self._current_job_file}: {e}")
            return None

    def set_current_job(self, value):
        """Persist `value` ('project/job' string) as the current job."""
        _atomic_write_json(self._current_job_file, {"current_job": value})
