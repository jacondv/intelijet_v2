# Install vscode

```bash
sudo snap install code --classic
```

# Run bash file  link_to_leica_blkarc.sh
```bash
chmod +x link_to_leica_blkarc.sh
./link_to_leica_blkarc.sh
```

# Install catkin

```bash
sudo apt update
sudo apt install ros-noetic-catkin python3-catkin-tools
```

# Intall python3-venv
```bash
sudo apt update
sudo apt install python3-venv
```

# Create new venv in inteliject_v2 folder and install packages missing
```bash
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install --upgrade pip setuptools wheel
pip install open3d
python3 -m pip install --upgrade open3d==0.17.0
pip install git+https://github.com/eric-wieser/ros_numpy.git
pip install python-box

```

# Build project
```bash
```

# Report package
```bash
pip install weasyprint jinja2
pip install "weasyprint==59.0" "pydyf==0.9.0" --force-reinstall
sudo apt install xdg-utils

# Install evince, PDF viewer on Linux
sudo apt install okular
sudo apt install evince
```

<!-- # Config loaded
```bash
pip3 install watchdog
``` -->


# USB Data Copier - Installation Guide

## 1. Copy Project Folder

Copy the folder:

```text
intelijet_usbcopier
```

into your home directory:

```text
/home/<your_user>/
```


---

## 2. Open Terminal

Open a terminal window and go to the project folder:

```bash
cd ~/intelijet_usbcopier
```

---

## 3. Run Installation Script

Execute:

```bash
chmod +x install.sh
./install.sh
```

---

## 4. Verify Installation

Plug in a USB drive.

If the USB Data Copier window appears automatically, the installation is successful.

---

## 5. Check Service Status

To verify that the service is running:

```bash
sudo systemctl status usb-copier.service
```

You should see:

```text
active (running)
```

---

# Useful Commands

## Restart Service

```bash
sudo systemctl restart usb-copier.service
```

## Stop Service

```bash
sudo systemctl stop usb-copier.service
```

## Start Service

```bash
sudo systemctl start usb-copier.service
```

## View Logs

```bash
journalctl -u usb-copier.service -f
```

---

# Uninstall

To remove the application service:

```bash
cd ~/intelijet_usbcopier
chmod +x uninstall.sh
./uninstall.sh
```

This will:
- stop the service
- disable auto start
- remove the systemd service

---

# Run Manually (Without Service)

```bash
python3 usb_copier.py
```

or:

```bash
python3 usb_copier.py /path/to/source/data
```

---

# Notes

## If GUI Does Not Appear

Run:

```bash
xhost +local:
```

Then restart the service:

```bash
sudo systemctl restart usb-copier.service
```

---

# Service File Location

```text
/etc/systemd/system/usb-copier.service
```


# Folder Sync Setup with Syncthing (Tablet A ↔ Tablet B)

## Overview

- Tablet A and Tablet B are peers (no more VIP/keepalived — each has its own fixed IP).
- Two folders need syncing between them:
  - `/data/scanner_config` — small YAML config files, needs near-instant sync, versioned for safety.
  - `/data/scanner_results` — large scan result files, can tolerate more delay, lower priority.
- Syncthing runs as a background service on both tablets and syncs bidirectionally via its own P2P protocol (not plain rsync).

## 1. Install Syncthing (run on BOTH tablets)

```bash
curl -s https://syncthing.net/release-key.txt | sudo apt-key add -
echo "deb https://apt.syncthing.net/ syncthing stable" | sudo tee /etc/apt/sources.list.d/syncthing.list
sudo apt update
sudo apt install syncthing -y
```

Enable it as a systemd service for the current user (not root) so it starts on boot:

```bash
sudo systemctl enable syncthing@$USER --now
```

Check it's running:

```bash
systemctl status syncthing@$USER
```

## 3. Open the Web GUI

By default the GUI only listens on `127.0.0.1:8384` (local access only).

- **Access directly from the tablet's own browser:**
  ```
  http://127.0.0.1:8384
  ```
- **To access remotely from another machine on the LAN/WiFi**, on each tablet go to:
  `Actions (top right) → Settings → GUI`
  - Change **GUI Listen Address** from `127.0.0.1:8384` to `0.0.0.0:8384`
  - **Set a GUI username/password immediately** — once exposed to `0.0.0.0`, anyone on the network can reach the config UI without one.
  - Save and restart Syncthing when prompted.

Then access via:
```
http://<tablet-IP>:8384
```

## 4. Get the Device ID on each tablet

On **Tablet A**: open the web UI → **Actions → Show ID** → copy the long Device ID string.

Repeat on **Tablet B** to get its Device ID.

## 5. Pair the two devices (do this on BOTH tablets)

On **Tablet A**:
1. Go to **Remote Devices → Add Remote Device**.
2. Paste Tablet B's Device ID.
3. Give it a friendly name (e.g. `Tablet-B`).
4. Save.

On **Tablet B**:
1. A connection request from Tablet A should appear automatically (or add manually the same way, pasting Tablet A's Device ID).
2. Accept / confirm the pairing.

Once paired, both devices show as **Connected** (green) in the **Remote Devices** panel.

## 6. Add the Config folder (do this on BOTH tablets)

On **Tablet A**:
1. Go to **Folders → Add Folder**.
2. **Folder Label**: `scanner_config`
3. **Folder Path**: `/home/jacon/intelijet_v2/intelijet_v2_ws/src/config/`
4. Go to the **Sharing** tab → tick `Tablet-B` to share this folder with it.
5. Go to the **File Versioning** tab → select **Simple File Versioning** (keeps old versions if a file gets overwritten — important for config safety). Set **Keep Versions** to e.g. `5`.
6. Go to **Advanced** → set **Rescan Interval** to a low value (e.g. `10` seconds) so config changes propagate quickly.
7. Save.

On **Tablet B**: a folder-share request for `scanner_config` will appear — accept it, and confirm the local path is `/data/scanner_config`.

## 7. Add the Results folder (do this on BOTH tablets)

On **Tablet A**:
1. **Add Folder** again.
2. **Folder Label**: `scanner_results`
3. **Folder Path**: `/home/jacon/data/Projects`
4. **Sharing** tab → tick `Tablet-B`.
5. **File Versioning**: choose **None** or **Staggered File Versioning** depending on whether you want history kept for result files.
6. **Advanced** → Rescan Interval can stay at default (e.g. `60`s) since large files don't need the same urgency as config.
7. Save.

On **Tablet B**: accept the `scanner_results` share request, confirm local path `/data/Projects`.

## 8. Verify sync is working

- On the Syncthing web UI, each folder card shows a status:
  - **Green "Up to Date"** → synced correctly.
  - **Blue "Syncing"** → in progress.
  - **Red** → error, click into the folder for details (permission issues, path not found, etc.).
- Quick manual test: create a test file in `/home/jacon/intelijet_v2/intelijet_v2_ws/src/config/` on Tablet A, confirm it appears on Tablet B within a few seconds.

```bash
# On Tablet A
echo "test" > /home/jacon/intelijet_v2/intelijet_v2_ws/src/config/test.txt

# On Tablet B, after a few seconds
cat /home/jacon/intelijet_v2/intelijet_v2_ws/src/config/test.txt   # (adjust path to scanner_config)
```

## 9. Notes on conflict handling

- If both tablets modify the same file at nearly the same time, Syncthing does **not** silently overwrite — it creates a `filename.sync-conflict-<date>-<device>.ext` copy so no data is lost. Review these files manually when they appear.
- Time sync between the two tablets (`chrony` or `ntpd`) is still good practice, though Syncthing relies less on wall-clock time than plain `rsync --update` does.

```bash
sudo apt install chrony -y
sudo systemctl enable chrony --now
```

## 10. Firewall reminder

Syncthing uses:
- TCP/UDP port `22000` for sync traffic (device-to-device)
- UDP port `21027` for local discovery broadcast
- TCP port `8384` for the web GUI (local/LAN only — do not expose to WAN without a password)

If a firewall (ufw, iptables) is active on the tablets, make sure these ports are allowed on the LAN interface.