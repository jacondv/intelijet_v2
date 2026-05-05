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