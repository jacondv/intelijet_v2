# 13. USB Data Copier

## 13.1 Daily use

1. Plug a USB stick into the tablet.
2. The **USB Data Copier** window opens by itself, showing the `data/Projects` folders.
3. Open the project/job folders and tick the items you want (whole projects, jobs, or single PDF/`.ply` files).
4. Start the copy and wait for it to finish.
5. Remove the stick.

The copier runs as a background service; it does not need the Intelijet app.

## 13.2 Set-up (technicians)

```bash
cp -r intelijet_usbcopier ~/          # from the repo
cd ~/intelijet_usbcopier && chmod +x install.sh && ./install.sh
sudo systemctl status usb-copier.service    # expect: active (running)
```

Service control: `sudo systemctl start|stop|restart usb-copier.service`, logs: `journalctl -u usb-copier.service -f`.

If the window does not appear when a stick is inserted:

```bash
xhost +local:
sudo systemctl restart usb-copier.service
```

The source folder is set at the top of `usb_copier.py` (`SOURCE_DATA_PATH`) — check it points at this tablet's `data/Projects/`.

Uninstall: `./uninstall.sh` in the same folder.
