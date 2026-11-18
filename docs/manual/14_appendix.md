# 14. Appendix

## 14.1 Glossary

| Term | Meaning |
|------|---------|
| **Shotcrete** | Sprayed concrete applied to the tunnel wall. The machine measures its thickness. |
| **Pre-Scan** | Scan of the wall before spraying. Starts a segment. |
| **Post-Scan** | Scan after spraying. Compared against the segment's Pre-Scan. |
| **Segment** | One Pre-Scan plus all Post-Scans taken after it (`SCAN 1`, `SCAN 2`…). |
| **Compare** | Calculating the thickness = distance between Post-Scan and Pre-Scan surfaces. |
| **Heatmap** | The compared cloud coloured by thickness (green / red / blue). |
| **Report** | PDF "Shotcrete Thickness Report" generated from a heatmap. |
| **Project / Job** | Folder hierarchy for scans: project (site) → job (area). |
| **Work Schedule** | Jobs marked ready to scan; only these can be the CURRENT JOB. |
| **Housing** | Motorised cover carrying the scanner; rotates 12°→135° during a scan. |
| **Encoder** | Angle sensor of the Housing; 0° = fully closed (Set Zero Position). |
| **PLC / PCAN** | Machine controller and the CAN-to-Ethernet gateway between it and the router. |
| **Router** | The machine's own network; the tablet reaches the Lidar and PCAN through it. |
| **PPS** | The Intelijet scanning unit as a whole (Jacon PPS). |
| **Target Thickness / Tolerance** | Per-job design thickness and allowed deviation (mm). |
| **Restart App** | Exit with the power button, then start again from the Intelijet icon. |

## 14.2 File names

`<Job>#<YYYYMMDD_HHMMSS>#<type>_<nn>#SCAN<sss>.<ext>`

| `type` | File |
|--------|------|
| `pre_scan_cloud` | Pre-Scan cloud (`.ply`) |
| `post_scan_cloud` | Post-Scan cloud (`.ply`) |
| `cloud_compared` | Heatmap cloud (`.ply`) and its report (`.pdf`) — same time and `nn` as its Post-Scan |
| `finalreport` | Copy of the newest report of the segment (`.pdf`) |

`nn` = Post-Scan number in the segment, `sss` = segment number. The date/time is the tablet's local time when the scan was taken (header clock).

## 14.3 Folders on the tablet (repository folder, e.g. `~/intelijet_v2`)

```
data/Projects/<Project>/<Job>/        scans, heatmaps, reports, job_info.json
data/Projects/active_jobs.json        the Work Schedule
data/Projects/current_job.json        the CURRENT JOB
data/logs/YYYYMMDD.jsonl              alarm/event log per day
data/logs/last_run.log                start-up log of the current run
intelijet_v2_ws/src/config/*.yaml     configuration (Chapter 11.4)
docs/manual/                          this manual (sources, screenshots, tools)
```

## 14.4 Network

| Device | Address |
|--------|---------|
| Tablet | Wi-Fi or LAN on the machine's router (header indicator) |
| SICK LMS511 scanner | 192.168.82.121 (set in `devices.yaml` / scanner launch file) |
| PCAN Ethernet gateway | 192.168.82.10 |

## 14.5 Error codes (quick list)

| Code | Level | Message |
|------|-------|---------|
| SCAN-001 | Error | Failed to assemble point cloud |
| SCAN-002 | Warning | Encoder did not reach target angle while opening the housing |
| SCAN-003 | Warning | Encoder did not reach target angle while closing the housing |
| SCAN-004 | Error | Scan failed |
| SCAN-005 | Error | Housing move failed |
| SCAN-006 | Error | Unsupported active_lidar value in lidar.yaml |
| SCAN-007 | Error | Pre-Scan cloud preprocessing failed |
| SCAN-008 | Error | Post-Scan cloud preprocessing failed |
| SCAN-009 | Error | Scan pipeline error while processing a scan cloud |
| SCAN-010 | Warning | Failed to color point cloud for display |
| COMPARE-001 | Error | Compare finished with success=False |
| COMPARE-002 | Error | Compare timed out |
| COMPARE-003 | Warning | Post-process crop step failed, using the uncropped cloud instead |
| COMPARE-004 | Error | Compare failed (auto-compare) |
| COMPARE-005 | Error | Compare failed (manual compare) |
| COMPARE-006 | Error | Compare failed |
| DEVICE-LIDAR-001 | Error | Lidar disconnected (network unreachable) |
| DEVICE-LIDAR-002 | Error | Lidar network is reachable but no scan data is coming in |
| DEVICE-ENCODER-001 | Error | Encoder disconnected |
| DEVICE-PLC-001 | Error | PLC disconnected |
| DEVICE-PCAN-001 | Error | PCAN Ethernet Gateway disconnected |
| SYSTEM-TIME-001 | Warning | App time zone does not match the tablet's time zone |

Full fix guides: Chapter 10; on the tablet: ALARM tab → **View**.

## 14.6 Key numbers

| Item | Value |
|------|-------|
| Housing scan sweep | 12° → 135° (opening up to 150 s, closing up to 30 s) |
| Compare time-out | 120 s |
| Alarm log retention | 365 days |
| Automatic clean-up caps | 100 GB / 50 projects |
| Device light time-out | 3–5 s |
| Network indicator refresh | 4 s; Wi-Fi bars: ≥ −60 dBm = 3, ≥ −70 dBm = 2, below = 1 |
| Thickness treated as "no shotcrete" | < 12 mm |
| Thickness treated as noise / out of range | > 300 mm (report and heatmap, `thickness.max_measurable`) |

## 14.7 Support

> ⚠️ **NEEDS CONFIRMATION** — technical support contact.

## 14.8 Manual change log

| Date | Change |
|------|--------|
| 2026-08-18 | First draft (previous 3-tab interface). |
| 2026-09-09 | Rewritten for the v3.3 interface (side-navigation tabs, JOB/REPORT/ALARM pages, error codes, kiosk mode); screenshots added. |
| 2026-11-18 | v3.8 interface: all screenshots re-captured (`tools/capture_screenshots.sh`); header network indicator and clock (5.1.1, 5.1.2); SYSTEM tab with 7 processing switches and Storage & Data card; device alarms DEVICE-* and SYSTEM-TIME-001; Chapter 10 rewritten as step-by-step fix guides; report noise limit 300 mm and Source file row. |
