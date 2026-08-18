# 12. Appendix

## Glossary

| Term | Meaning |
|---|---|
| **Housing** | The motor-driven cover over the scanner. Opens to expose the scanner during a scan, closes to protect it afterward. |
| **Hydraulic arm** | The 2-joint, 1-cylinder arm that positions the Housing close to the surface being scanned. Moves independently of the Housing. |
| **Pre-scan** | The "before" scan of a job. |
| **Post-scan** | The "after" scan of a job. |
| **Compare** | The step that compares the pre-scan and post-scan to show what changed. |
| **Job** | A single scan task, with its own pre-scan, post-scan, and report. Belongs to a Project. |
| **Project** | A container that groups related Jobs. |
| **PPS** | The system's own control/software unit (the part that has no dedicated physical emergency-stop — see Chapter 2). |

## Hardware reference

- Scanner: SICK LMS511, Laser Class 1.

> ⚠️ **NEEDS CONFIRMATION** — other hardware specifications (Housing motor, hydraulic system, PLC, CAN gateway model numbers, operating temperature/humidity range, etc.) are not available from the software and need to be provided from the manufacturer documentation or the installer.

## Network addresses (for technicians)

The scanner, encoder, CAN gateway, and PLC each have their own network/CAN address, configured per machine. These can differ between installations — check the actual configuration on the specific machine rather than assuming the same values apply everywhere.

## Error messages

The system does not use a fixed list of numbered error codes — messages shown in the notification bar (Chapter 7) are free-text descriptions. See Chapter 8 for what to do about the most common ones.

## Support contact

> ⚠️ **NEEDS CONFIRMATION** — contact name/phone/email for technical support, to be added here (also referenced in Chapter 8).

## Change log

| Date | Change |
|---|---|
| 2026-08-18 | First draft of this manual. |
