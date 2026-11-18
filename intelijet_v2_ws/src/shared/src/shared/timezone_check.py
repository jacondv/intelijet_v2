"""Check that this container's local time matches the host's.

Scan filenames (ui/models/file_name.py), report times (services/report) and
the header clock all use the container's local time. The clock itself is
the host's kernel clock, but the time zone is per-container - the image
defaults to UTC, and only run_docker.sh/docker-compose.yml set it to the
host's. run_docker.sh passes the host's UTC offset in HOST_UTC_OFFSET so
this can verify it actually worked.
"""
import os
import time


def check_timezone_matches_host():
    """Return None if the container's UTC offset matches the host's, else a
    human-readable problem description."""
    host_offset = os.environ.get("HOST_UTC_OFFSET", "").strip()
    local_offset = time.strftime("%z")
    if not host_offset:
        return (f"Host time zone unknown (app not started via run_docker.sh) - "
                f"app time zone is UTC{local_offset}, may not match the tablet's clock")
    if host_offset != local_offset:
        return f"App time zone UTC{local_offset} does not match the tablet's UTC{host_offset}"
    return None
