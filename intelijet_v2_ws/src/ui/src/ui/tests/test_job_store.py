# ui/tests/test_job_store.py
"""Standalone tests for JobStore - no ROS/Qt needed.

Run with:  python3 -m pytest ui/src/ui/tests/test_job_store.py
       or:  python3 ui/src/ui/tests/test_job_store.py
"""
import json
import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from ui.services.job_store import JobStore  # noqa: E402


class TempDir:
    def __enter__(self):
        self.path = tempfile.mkdtemp(prefix="job_store_test_")
        return self.path

    def __exit__(self, *exc):
        shutil.rmtree(self.path, ignore_errors=True)


def _paths(tmp_dir):
    return (
        os.path.join(tmp_dir, "active_jobs.json"),
        os.path.join(tmp_dir, "current_job.json"),
    )


def test_missing_files_return_safe_defaults():
    with TempDir() as tmp:
        active_file, current_file = _paths(tmp)
        store = JobStore(active_file, current_file)
        assert store.list_active_jobs() == []
        assert store.get_current_job() is None


def test_list_active_jobs_reads_existing_file():
    with TempDir() as tmp:
        active_file, current_file = _paths(tmp)
        with open(active_file, "w") as f:
            json.dump([{"project": "P1", "job": "J1"}, {"project": "P1", "job": "J2"}], f)

        store = JobStore(active_file, current_file)
        jobs = store.list_active_jobs()
        assert len(jobs) == 2
        assert jobs[0]["job"] == "J1"


def test_corrupt_active_jobs_file_does_not_crash():
    with TempDir() as tmp:
        active_file, current_file = _paths(tmp)
        with open(active_file, "w") as f:
            f.write("{not valid json")

        store = JobStore(active_file, current_file)
        assert store.list_active_jobs() == []


def test_active_jobs_not_a_list_is_ignored():
    with TempDir() as tmp:
        active_file, current_file = _paths(tmp)
        with open(active_file, "w") as f:
            json.dump({"unexpected": "shape"}, f)

        store = JobStore(active_file, current_file)
        assert store.list_active_jobs() == []


def test_set_and_get_current_job_roundtrip():
    with TempDir() as tmp:
        active_file, current_file = _paths(tmp)
        store = JobStore(active_file, current_file)

        store.set_current_job("ProjectA/Job1")
        assert store.get_current_job() == "ProjectA/Job1"

        # A second JobStore instance pointed at the same file must see it too.
        store2 = JobStore(active_file, current_file)
        assert store2.get_current_job() == "ProjectA/Job1"


def test_set_current_job_is_atomic_no_leftover_tmp_file():
    with TempDir() as tmp:
        active_file, current_file = _paths(tmp)
        store = JobStore(active_file, current_file)
        store.set_current_job("ProjectA/Job1")

        assert os.path.exists(current_file)
        assert not os.path.exists(current_file + ".tmp")


def test_reload_picks_up_external_changes():
    with TempDir() as tmp:
        active_file, current_file = _paths(tmp)
        with open(active_file, "w") as f:
            json.dump([{"project": "P1", "job": "J1"}], f)

        store = JobStore(active_file, current_file)
        assert len(store.list_active_jobs()) == 1

        # Simulate another process (e.g. Syncthing from the other tablet)
        # rewriting the file.
        with open(active_file, "w") as f:
            json.dump([{"project": "P1", "job": "J1"}, {"project": "P2", "job": "J2"}], f)

        # Cache should NOT change until reload() is called.
        assert len(store.list_active_jobs()) == 1

        store.reload()
        assert len(store.list_active_jobs()) == 2


def test_add_active_job_appends_and_persists():
    with TempDir() as tmp:
        active_file, current_file = _paths(tmp)
        store = JobStore(active_file, current_file)

        assert store.add_active_job("P1", "J1") is True
        assert store.list_active_jobs() == [{"project": "P1", "job": "J1"}]

        store2 = JobStore(active_file, current_file)
        assert store2.list_active_jobs() == [{"project": "P1", "job": "J1"}]


def test_add_active_job_is_idempotent():
    with TempDir() as tmp:
        active_file, current_file = _paths(tmp)
        store = JobStore(active_file, current_file)

        assert store.add_active_job("P1", "J1") is True
        assert store.add_active_job("P1", "J1") is False
        assert len(store.list_active_jobs()) == 1


def test_remove_active_job():
    with TempDir() as tmp:
        active_file, current_file = _paths(tmp)
        store = JobStore(active_file, current_file)
        store.add_active_job("P1", "J1")
        store.add_active_job("P1", "J2")

        assert store.remove_active_job("P1", "J1") is True
        assert store.list_active_jobs() == [{"project": "P1", "job": "J2"}]
        # Removing again is a no-op, not an error.
        assert store.remove_active_job("P1", "J1") is False


def test_rename_active_job_updates_matching_entry_only():
    with TempDir() as tmp:
        active_file, current_file = _paths(tmp)
        store = JobStore(active_file, current_file)
        store.add_active_job("P1", "J1")
        store.add_active_job("P1", "J2")

        assert store.rename_active_job("P1", "J1", "J1-renamed") is True
        jobs = store.list_active_jobs()
        assert {"project": "P1", "job": "J1-renamed"} in jobs
        assert {"project": "P1", "job": "J2"} in jobs
        assert len(jobs) == 2

        # No matching entry -> no-op, doesn't touch unrelated jobs.
        assert store.rename_active_job("P1", "does-not-exist", "x") is False


def test_rename_active_job_project_updates_all_matching_entries():
    with TempDir() as tmp:
        active_file, current_file = _paths(tmp)
        store = JobStore(active_file, current_file)
        store.add_active_job("P1", "J1")
        store.add_active_job("P1", "J2")
        store.add_active_job("P2", "J3")

        assert store.rename_active_job_project("P1", "P1-renamed") is True
        jobs = store.list_active_jobs()
        assert {"project": "P1-renamed", "job": "J1"} in jobs
        assert {"project": "P1-renamed", "job": "J2"} in jobs
        assert {"project": "P2", "job": "J3"} in jobs


def run_all():
    tests = [v for k, v in globals().items() if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print(f"OK: {t.__name__}")
    print(f"\n{len(tests)} tests passed.")


if __name__ == "__main__":
    run_all()
