# 2. Safety Warnings

Read this chapter before operating the machine for the first time.

**Symbols used:** ⚠️ general hazard · ⚡ electrical · 🔦 laser · ⚙️ moving parts

## The two controls you must know

| Control | Where | What it does | What it does NOT do |
|---------|-------|--------------|---------------------|
| **Emergency** control | On the host vehicle | Stops the engine and cuts **hydraulic** power (arm stops). | Does **not** cut electrical power — the Housing motor and tablet stay live. |
| **Ignition OFF / battery master ("mass") switch** | On the host vehicle | Cuts **all electrical power** — Housing motor, PLC, tablet. Required before any maintenance. | Does not release pressure trapped in the hydraulic arm. |

The on-screen **Exit** button and the **CANCEL JOB** button only stop the *software*. They are not emergency stops.

**Quick reference**

| Situation | Do this |
|-----------|---------|
| Hydraulic arm moving unexpectedly, or someone caught by the arm | Press the vehicle **emergency** control at once. Do **not** touch the arm — see below. |
| Hand or object caught in the Housing | Turn **ignition OFF** or open the **mass** switch. The Housing can then be moved by hand. |
| Housing moving when you did not expect it during a scan | Release any Housing button you are holding; press **CANCEL JOB**; if it keeps moving, turn ignition OFF. |
| Any maintenance, cleaning or disassembly | Ignition OFF / mass switch open **first, always**. |

## ⚙️ Housing — pinch hazard

The Housing rotates on its own during every Pre-Scan and Post-Scan (about 12° → 135° and back, up to ~2.5 minutes), and whenever **OPEN HOUSING** / **CLOSE HOUSING** is held down on the screen.

- Keep hands and tools out of the Housing's swing path whenever a scan is running (the **PRE-SCAN** or **POST-SCAN** button has a green frame) or a Housing button is pressed.
- **OPEN HOUSING** and **CLOSE HOUSING** move only while your finger is on the button. Lifting your finger stops the Housing.
- If something is caught: ignition OFF / mass switch open, then move the Housing by hand to free it. There is no separate Housing power switch.

No fixed stand-off distance is required — just stay out of the swing path.

## ⚙️ Hydraulic arm — crush hazard (NOT the same as the Housing)

- The arm positions the Housing at the wall. It moves independently of the Housing.
- Pressing the vehicle **emergency** control stops the engine and the hydraulic supply, but **the arm does not release its own pressure**. It can still hold force after power is off.
- Never try to move or free anything from the arm by hand. Call a technician / authorised person to relieve the pressure first.

> ⚠️ **NEEDS CONFIRMATION** — the pressure-release procedure (steps, valve location). To be supplied by the machine owner. Do not improvise.
>
> ⚠️ **NEEDS CONFIRMATION** — how the arm is positioned (vehicle joystick? which control?). Chapter 7 assumes the arm is positioned from the vehicle's own controls, not from the tablet.

## 🔦 Laser

The scanner is a SICK LMS511, **Laser Class 1** — safe for the eyes in normal use. No eye protection is required for the scanner.

## ⚡ Electrical

Before any maintenance, cleaning, or disconnecting cables (encoder, CAN, scanner, router, tablet): turn the vehicle **ignition OFF** or open the battery **mass** switch. This is the machine's only full power cut-off.

Several fix guides in Chapter 10 say *power-cycle* a device (Lidar, PLC, router). Do that only with the Housing closed and nobody near the Housing or arm.

## Personal protective equipment

No PPE beyond the site's normal requirements is needed to operate the tablet and scanner.

## Working in tunnels / confined spaces

The machine is used in confined spaces. Follow your site's confined-space rules (air, ventilation, communication). No machine-specific extra rule has been identified.

## If an accident happens

> ⚠️ **NEEDS CONFIRMATION** — name and phone number to call immediately.

---

> This chapter must be reviewed by someone with hands-on knowledge of the machine before the manual is issued. Lines marked NEEDS CONFIRMATION are not yet instructions.
