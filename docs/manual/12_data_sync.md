# 12. Data Sync Between Tablets

When two tablets are used, the job folder (`data/Projects`) and the configuration folder are kept identical on both by a background service (**Syncthing**). A job created or scanned on one tablet appears on the other within seconds to minutes. Both tablets must be on the network (header indicator, Chapter 5.1.1).

## 12.1 For operators

- The **CURRENT JOB** list refreshes itself every 30 s; the **JOB** and **REPORT** tabs refresh when you open them. If a job is missing, wait a moment and re-open the tab.
- Do not edit the **same job on both tablets at the same time**. If it happens, a copy named `…sync-conflict-<date>-<device>…` is created next to the original so nothing is lost — tell the technician which one is correct.
- Syncthing's own status page (technician) shows **Up to Date** (green), **Syncing** (blue) or an error (red).

## 12.2 For technicians — set-up (both tablets)

On each tablet, from the repository folder:

```bash
./install_syncthing.sh
```

It installs Syncthing from its official repository, starts it as a per-user service, creates the two folders to be synced, and prints this tablet's **Device ID** and the exact folder paths to enter in the web UI. Then, by hand on both tablets:

1. Web UI: `http://127.0.0.1:8384` (if exposed on the LAN, set a GUI password first).
2. **Remote Devices → Add Remote Device** with the other tablet's ID, on both sides.
3. Add the two shared folders, using the paths the script printed:
   - **config** → `intelijet_v2_ws/src/config/` — enable **Simple File Versioning**, rescan interval ~10 s.
   - **projects** → `data/Projects/` — versioning optional, default rescan.
4. Confirm **Connected** and **Up to Date** on both.

Full step-by-step pairing: `intallation_gui.md`, section *Folder Sync Setup with Syncthing*.

Firewall: TCP/UDP 22000 (sync), UDP 21027 (discovery), TCP 8384 (web UI, LAN only).

Conflict files (`.sync-conflict-…`) are ignored by the app's lists; resolve them by keeping the correct copy and deleting the other.
