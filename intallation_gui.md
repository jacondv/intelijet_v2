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
pip install git+https://github.com/eric-wieser/ros_numpy.git
pip install python-box

```

# Build project
```bash


