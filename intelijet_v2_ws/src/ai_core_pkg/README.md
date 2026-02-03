# Python Virtual Environment (venv) Setup Guide

This document describes how to create a Python virtual environment (venv)
and install all required dependencies using `requirements.txt`.

The environment is intended for projects using:
- Kornia (feature matching, LoFTR, DISK)
- OpenCV
- YAML configuration files
- ROS Python tools (`rospkg`)

---

## 1. System Requirements

- Ubuntu 18.04 / 20.04 / 22.04
- Python 3.8
- python3-venv installed

Check Python version:
```bash
python3 --version
```

## 2. Create Virtual Environment (venv)

```bash
sudo apt update
sudo apt install python3-venv -y

cd ~/intelijet_v2_ws/src/ai_core_pkg
python3 -m venv venv

source venv/bin/activate


pip install --upgrade pip
```
## 3. Install Dependencies from

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

## 4. Verify Installation

```bash
python -c "import kornia; print(kornia.__version__)"
```