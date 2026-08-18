# 10. Data Sync Between Tablets (Syncthing)

The two tablets keep two folders in sync with each other automatically, using a background service called Syncthing: the configuration folder and the job/report data folder.

## For operators — checking sync status day to day

Open the Syncthing web page on the tablet (a technician can show you where) to see the status of each synced folder:
- **Green "Up to Date"** — synced correctly, nothing to do.
- **Blue "Syncing"** — currently transferring changes, wait for it to finish.
- **Red** — an error; open that folder's details for more information, or contact a technician.

**If you see a file with `sync-conflict` in its name:** this means both tablets changed the same file at nearly the same time. Syncthing does **not** silently overwrite either version — it keeps both and adds this extra copy so nothing is lost. Do not delete it without checking which version is correct first; ask a technician if unsure.

## For technicians — setup

Install Syncthing on **both** tablets:

```bash
curl -s https://syncthing.net/release-key.txt | sudo apt-key add -
echo "deb https://apt.syncthing.net/ syncthing stable" | sudo tee /etc/apt/sources.list.d/syncthing.list
sudo apt update
sudo apt install syncthing -y
sudo systemctl enable syncthing@$USER --now
```

Then, on each tablet:
1. Open the Syncthing web UI (`http://127.0.0.1:8384` from that tablet itself). If you need to reach it remotely over the LAN, change the GUI listen address to `0.0.0.0:8384` in Settings — and **set a GUI username/password immediately** when you do, since an unprotected `0.0.0.0` address is reachable by anyone on the network.
2. Under **Actions → Show ID**, get this tablet's Device ID, and pair the two tablets with each other (**Remote Devices → Add Remote Device**, paste the other tablet's ID) — do this on both sides.
3. Add the two shared folders on both tablets (folder paths depend on this specific machine's installation — check what path is already configured on each tablet rather than assuming a fixed path):
   - **Config folder** — small files, should sync quickly. Use **Simple File Versioning** (keep a handful of old versions) and a short rescan interval (a few seconds) so config changes propagate fast.
   - **Results folder** — larger scan/report files, sync speed is less urgent. Versioning is optional here.
4. Confirm both folders show **Connected** (Remote Devices) and **Up to Date** (Folders) on both tablets.

**Firewall:** allow these ports on the LAN interface if a firewall is active:
- TCP/UDP `22000` — sync traffic between the tablets.
- UDP `21027` — local discovery.
- TCP `8384` — web GUI (LAN/local only — never expose this to the internet without a password).
