# 3. Installation & Kiosk Mode (technicians)

The machine hardware is assumed to be installed and wired. This chapter covers the tablet software only.

## 3.1 Requirements

- Linux tablet/PC (Ubuntu/Debian family) with a touchscreen.
- Internet access during installation (Docker and the application image are downloaded/built once).
- The Intelijet repository cloned onto the tablet, e.g. `~/intelijet_v2` (called *the repository folder* below).
- The tablet's **date, time and time zone** set correctly in Ubuntu **Settings → Date & Time** — scan file names, report times and the header clock all use them.

## 3.2 First-time install

```bash
cd ~/intelijet_v2
./install.sh
```

The script:

1. Installs `curl`, Docker and Docker Compose if missing (asks for `sudo`).
2. Adds the current user to the `docker` group.
3. Builds the application image (one time, several minutes).
4. Creates the **Intelijet** icon (desktop + application menu) with the PPS logo, pointing at this copy of the repository.

If the user was just added to the `docker` group, **log out and back in (or reboot)** before using the icon.

Tap **Intelijet** to start. See Chapter 4 for what happens next.

Always start the app from the **Intelijet** icon (or kiosk mode). The icon passes the tablet's time zone to the app; started any other way the app may run on the wrong time zone and raises **SYSTEM-TIME-001** (Chapter 10).

## 3.3 Kiosk mode (boot straight into the app)

On a delivered machine the tablet should start the app by itself, with no login screen. That is **kiosk mode**:

```bash
sudo ./kiosk_mode.sh status          # is it on?
sudo ./kiosk_mode.sh enable          # turn on (autologin on tty1 → minimal X → Intelijet), then: sudo reboot
sudo ./kiosk_mode.sh restore         # back to the normal desktop/login screen
sudo ./kiosk_mode.sh                 # interactive menu
```

What `enable` changes: disables the desktop login manager, installs `openbox`, sets tty1 autologin for the chosen user, and adds a start-up block to that user's `~/.bash_profile` and `~/.xinitrc` (marked `intelijet-kiosk-mode`). `restore` undoes all of it. The state is recorded in `/etc/intelijet-kiosk.conf`.

### If the tablet is stuck in a boot loop or shows only a black screen

Switch to a text console (Ctrl+Alt+F2, log in) or connect over SSH, then:

```bash
cd ~/intelijet_v2
sudo ./kiosk_mode.sh restore
sudo reboot
```

This brings back the normal graphical desktop.

## 3.4 Screen rotation

The launcher rotates a tablet panel named `DSI-1`/`DSI1` to the right and locks GNOME's auto-rotate, so the app is always landscape. Nothing to configure on other monitors.

## 3.5 Network

The tablet must be on the machine's router — over its **Wi-Fi** or a **LAN cable**. The header shows which one is in use (Chapter 5.1); a red **NO NETWORK** means neither, and then the Lidar and PLC cannot be reached.

## 3.6 Updating the software later

`install.sh` does **not** update code. After pulling a new version:

```bash
cd ~/intelijet_v2
git pull
docker compose build
```

The next tap of the icon runs the new version. See Chapter 11.

> ⚠️ **NEEDS CONFIRMATION** — whether a hardware installation procedure (mounting, wiring, PLC setup) needs to be documented for new machines.
