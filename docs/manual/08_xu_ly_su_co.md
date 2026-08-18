# 8. Troubleshooting

## The app doesn't start

**What you see:** nothing happens after tapping the Intelijet icon, or the screen stays blank.

**What to do:**
1. Wait — the first startup after installation can take a few minutes (see Chapter 4).
2. If it still doesn't appear, restart the tablet/computer and tap the icon again.
3. If it still fails, contact a technician — they can check for a specific display/connection error.

## A device status light is red

**What you see:** the Lidar, Encoder, PCAN, or PLC light (Chapter 7) turns red.

**What to do:** see the table in Chapter 7 for what that specific light means and what to check (cabling, power, network). If it stays red after checking, contact a technician.

## The app seems frozen / not responding

**What you see:** buttons don't respond, the screen doesn't update.

**What to do:**
1. Wait a minute — some steps take time to finish, and the app is not supposed to lock up while they run.
2. If it's still unresponsive after a few minutes, restart it using the Intelijet desktop icon — this starts a clean, fresh session (see Chapter 4). This may lose any unsaved progress on the job you were working on.
3. If this happens often, contact a technician and mention exactly what you were doing when it happened.

## "Compare Failed" message

**What you see:** a red "Compare Failed" notification after pressing Compare.

**What to do:**
1. Make sure both a Pre-Scan and a Post-Scan have been completed for the current job.
2. Try Compare again.
3. If it keeps failing, contact a technician.

## No PDF report appears

**What you see:** Compare finishes but no report shows up, or View Report has nothing to show.

**What to do:** check that **Auto Report** (Setting tab) was turned on *before* you pressed Compare — the report is only generated automatically when it's on (see Chapter 6). If you need a report after the fact and this doesn't help, contact a technician.

## Housing or hydraulic arm won't move, or gets stuck mid-motion

**What you see:** pressing Open/Close Scanner does nothing, or the arm/Housing stops moving on its own.

**What to do:** do **not** force it by hand. Follow the safety steps in Chapter 2 for the part involved (Housing vs. hydraulic arm — they are not the same) before anyone touches it.

## Job/data files look out of sync between tablets

**What you see:** a file with `sync-conflict` in its name, or a sync status warning.

**What to do:** see Chapter 10 (Data Sync).

## Before contacting a technician

Have this ready:
- What you were doing when the problem happened (which step in Chapter 6).
- Whether the app needed to be restarted.
- The log files under `data/logs` in the installation directory (see Chapter 4) — a technician can use these to diagnose the issue.
