# 9. Maintenance & Software Updates

This chapter is for technicians.

## Updating the software

```bash
git pull
docker compose build
```

Do this only when the machine is not in active use — not in the middle of a job. The next tap of the desktop icon after the build finishes will use the new version. See Chapter 3 for the first-time install.

## What to back up

- **Job and report data** — under `data/Projects` in the installation folder. This is the most important thing to back up; it holds every job's scans and reports.
- **Configuration files** — the `.yaml` files under `intelijet_v2_ws/src/config/` in the installation folder.

Both of these are also kept in sync between the two tablets automatically via Syncthing — see Chapter 10 to check that sync is healthy, which is itself a form of backup.

## Housekeeping

- Old run logs accumulate under `data/logs` — these can be cleared periodically; only the two most recent are needed day-to-day (see Chapter 4).
- Scan data can take up a lot of disk space over time — check free disk space periodically.

> ⚠️ **NEEDS CONFIRMATION** — is there a recommended hardware maintenance schedule (cleaning the scanner lens, checking CAN cabling, inspecting the hydraulic arm/cylinder, etc.)? This is not something that can be determined from the software and needs to be provided.

> ⚠️ **NEEDS CONFIRMATION** — how long should old job data be kept, and who decides when it can be deleted?
