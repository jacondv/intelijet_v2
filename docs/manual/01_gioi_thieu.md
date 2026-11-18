# 1. System Introduction

Intelijet is a 3D scanning system used to measure **concrete thickness**. It captures a "before" scan (pre-scan) and an "after" scan (post-scan) of a concrete surface, then automatically compares the two to produce a **heatmap** — a color-coded 3D view showing concrete thickness across the scanned area, so thin or out-of-tolerance zones can be spotted visually.

> ⚠️ **NEEDS CONFIRMATION** — the specific structure/industry this is used for (e.g., tunnel lining, another type of concrete structure).

## Main hardware parts

- **Housing** — a motor-driven cover over the scanner. It opens to expose the scanner during a scan and closes to protect it afterward.
- **Hydraulic arm** — a 2-joint arm with 1 cylinder. It positions the Housing close to the surface being scanned, for better scan quality. It moves independently of the Housing.
- **Scanner (SICK LMS511)** — captures the 3D scan data.
- **Encoder, PLC, and CAN gateway** — hardware that tracks arm/Housing position and reports device status to the software.
- **Touchscreen computer** — runs the software you use to control a scan and view results.

See Chapter 2 for the safety warnings tied to the Housing and hydraulic arm — read it before operating the machine for the first time.

## Basic workflow (overview only — full steps in Chapter 6)

Select or create a job → position the arm and open the Housing → run a pre-scan → (the work being inspected happens) → run a post-scan → the system compares the two scans automatically and shows a heatmap → export a PDF report.

## Starting the software

The software starts with the **Intelijet** desktop icon — see Chapter 4 for daily startup and shutdown.
