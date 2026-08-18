# 11. USB Data Copier

## Daily use (operators)

1. Plug in a USB drive.
2. A "USB Data Copier" window opens automatically.
3. Browse into the job/project folders shown and select what you want to copy.
4. Copy the selected items to the USB drive.
5. Once copying is finished, safely remove the USB drive.

## Setup (technicians)

1. Copy the `intelijet_usbcopier` folder into the target user's home directory.
2. Install the service:
   ```bash
   cd ~/intelijet_usbcopier
   chmod +x install.sh
   ./install.sh
   ```
3. Verify it's running:
   ```bash
   sudo systemctl status usb-copier.service
   ```
   Should show `active (running)`.

## Managing the service

```bash
sudo systemctl restart usb-copier.service
sudo systemctl stop usb-copier.service
sudo systemctl start usb-copier.service
journalctl -u usb-copier.service -f    # view logs
```

## If the window doesn't appear when a USB drive is plugged in

```bash
xhost +local:
sudo systemctl restart usb-copier.service
```

## Uninstalling

```bash
cd ~/intelijet_usbcopier
chmod +x uninstall.sh
./uninstall.sh
```
This stops the service, disables it from starting automatically, and removes it.

## Running without the service (technicians, for testing)

```bash
python3 usb_copier.py
python3 usb_copier.py /path/to/source/data   # to copy from a specific folder instead of the default
```

> ⚠️ Note: the default source folder is set inside `usb_copier.py` on each machine and may differ between installs — if the copier isn't showing the expected data on a given machine, check that setting there rather than assuming it matches another machine.
