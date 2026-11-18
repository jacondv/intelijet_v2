# 3. Initial Installation

This chapter is for technicians setting up the software on a new machine. It assumes the machine hardware (Housing, hydraulic arm, scanner, etc.) is already installed and wired.

## Requirements

- A Linux (Ubuntu/Debian-based) machine with a touchscreen and an X server.
- This repository already cloned onto the machine (the installer does not clone it for you).

## Steps

```bash
cd intelijet_v3      # the folder you just cloned
./install.sh
```

`install.sh` will:
1. Install Docker and Docker Compose if not already present (requires `sudo`, uses Docker's official install script).
2. Add the current user to the `docker` group if not already a member.
3. Build the application image (a one-time build — later runs via the desktop icon reuse this image and do **not** rebuild).
4. Create an "Intelijet" icon (in the application menu and on the Desktop) pointing at this machine's copy of the repository.

If step 2 just added the user to the `docker` group for the first time, that user must **log out and back in (or restart the machine)** before the icon will work — the new group membership does not apply to the current session.

## After installing

Tap the **Intelijet** icon to run the app. Every tap stops whatever is currently running and starts a fresh copy from the already-built image — it does not rebuild, so this only takes a few seconds (except the very first run, see Chapter 4).

## Updating the software later

`install.sh` does not update the code or rebuild automatically. After pulling new code (`git pull`), rebuild the image manually:

```bash
docker compose build
```

The next tap of the desktop icon will then use the newly built image. See Chapter 9 for more on updates.

> ⚠️ **NEEDS CONFIRMATION** — is there a separate hardware installation procedure (mounting the Housing/hydraulic arm, wiring CAN/PLC, etc.) for a brand-new machine, or is hardware always pre-installed before the software setup above? If one exists, it should be documented as its own section here.
