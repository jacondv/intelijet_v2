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

# Config loaded
```bash
pip3 install watchdog
```
