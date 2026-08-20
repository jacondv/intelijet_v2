# ui/tests/test_project_repository.py
"""Standalone tests for project_repository - no ROS/Qt needed.

Run with:  python3 -m pytest ui/src/ui/tests/test_project_repository.py
       or:  python3 ui/src/ui/tests/test_project_repository.py
"""
import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

import ui.services.project_repository as repo  # noqa: E402
from ui.models.job_info import JobInfo  # noqa: E402


class TempProjectDir:
    """Points repo.PROJECT_DIR at a fresh temp dir for the duration of
    the `with` block, and restores it afterwards - the module's
    functions all read the PROJECT_DIR global at call time, so this is
    enough to isolate each test without touching real data."""

    def __enter__(self):
        self._orig = repo.PROJECT_DIR
        self.path = tempfile.mkdtemp(prefix="project_repo_test_")
        repo.PROJECT_DIR = self.path
        return self.path

    def __exit__(self, *exc):
        repo.PROJECT_DIR = self._orig
        shutil.rmtree(self.path, ignore_errors=True)


def _job_info(name):
    return JobInfo(name=name, status=JobInfo.PENDING)


def test_list_projects_empty_creates_dir():
    with TempProjectDir() as tmp:
        assert repo.list_projects() == []
        assert os.path.isdir(tmp)


def test_create_and_list_projects():
    with TempProjectDir():
        repo.create_project("Alpha")
        repo.create_project("Beta")
        assert repo.list_projects() == ["Alpha", "Beta"]


def test_create_project_duplicate_raises():
    with TempProjectDir():
        repo.create_project("Alpha")
        try:
            repo.create_project("Alpha")
            assert False, "expected ProjectError"
        except repo.ProjectError:
            pass


def test_rename_project():
    with TempProjectDir():
        repo.create_project("Alpha")
        repo.rename_project("Alpha", "Alpha2")
        assert repo.list_projects() == ["Alpha2"]


def test_delete_project():
    with TempProjectDir():
        repo.create_project("Alpha")
        repo.delete_project("Alpha")
        assert repo.list_projects() == []


def test_create_and_list_jobs():
    with TempProjectDir():
        repo.create_project("Alpha")
        repo.create_job("Alpha", _job_info("Job1"))
        assert repo.list_jobs("Alpha") == ["Job1"]
        loaded = repo.load_job_info("Alpha", "Job1")
        assert loaded is not None
        assert loaded.name == "Job1"
        assert loaded.status == JobInfo.PENDING


def test_create_job_duplicate_raises():
    with TempProjectDir():
        repo.create_project("Alpha")
        repo.create_job("Alpha", _job_info("Job1"))
        try:
            repo.create_job("Alpha", _job_info("Job1"))
            assert False, "expected ProjectError"
        except repo.ProjectError:
            pass


def test_rename_job_updates_folder_and_job_info_name():
    with TempProjectDir():
        repo.create_project("Alpha")
        repo.create_job("Alpha", _job_info("Job1"))
        repo.rename_job("Alpha", "Job1", "Job1-renamed")

        assert repo.list_jobs("Alpha") == ["Job1-renamed"]
        loaded = repo.load_job_info("Alpha", "Job1-renamed")
        assert loaded.name == "Job1-renamed"


def test_delete_job():
    with TempProjectDir():
        repo.create_project("Alpha")
        repo.create_job("Alpha", _job_info("Job1"))
        repo.delete_job("Alpha", "Job1")
        assert repo.list_jobs("Alpha") == []


def test_list_jobs_missing_project_returns_empty():
    with TempProjectDir():
        assert repo.list_jobs("does-not-exist") == []


def test_parse_job_ref():
    assert repo.parse_job_ref("Alpha / Job1") == ("Alpha", "Job1")
    assert repo.parse_job_ref("Alpha/Job1") == ("Alpha", "Job1")
    assert repo.parse_job_ref("Alpha") == ("Alpha", None)
    # A "/" inside the job half should not be split further.
    assert repo.parse_job_ref("Alpha / Job/With/Slash") == ("Alpha", "Job/With/Slash")


def run_all():
    tests = [v for k, v in globals().items() if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print(f"OK: {t.__name__}")
    print(f"\n{len(tests)} tests passed.")


if __name__ == "__main__":
    run_all()
