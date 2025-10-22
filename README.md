# 🚀 Unmanned-Aerial-Vehicle-3D-Radar

[![Stars](https://img.shields.io/github/stars/HLiu-uOttawa/Unmanned-Aerial-Vehicle-3D-Radar?style=social)](https://github.com/HLiu-uOttawa/Unmanned-Aerial-Vehicle-3D-Radar/stargazers)
[![Issues](https://img.shields.io/github/issues/HLiu-uOttawa/Unmanned-Aerial-Vehicle-3D-Radar)](https://github.com/HLiu-uOttawa/Unmanned-Aerial-Vehicle-3D-Radar/issues)
[![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10%2B-brightgreen)](https://www.python.org/)
[![License](https://img.shields.io/github/license/HLiu-uOttawa/Unmanned-Aerial-Vehicle-3D-Radar)](./LICENSE)

A **ROS 2–based 3D-Radar system** for [3D radar: DK-sR-14MPc](https://radar-sensor.com/products/developer-kits/dk-sr-14mpc.html).

---

## Table of Contents
- [Features](#features)
- [Requirements](#requirements)
- [Repository Structure](#repository-structure)
- [Configuration](#configuration)
- [Documentation](#documentation)
- [Roadmap](#roadmap)
- [License](#license)

---

## Features
- ✅ Real-time **radar**
- ✅ Live visualization with **RViz2 / Foxglove**
- ✅ Designed for **Jetson Orin NX** (CUDA) and desktop Linux

---

## Requirements
- **OS:** Ubuntu 22.04 (Jammy)
- **ROS 2:** Humble Hawksbill
- **Python:** 3.10+
- **CUDA:** NVIDIA CUDA (on Jetson/desktop GPU)  
- Optional tools: **Foxglove Studio**, **RViz2**

> If you use a fresh machine, make sure ROS 2 is installed and sourced before building.

---

## Repository Structure
```
.
├── sensors/
│   ├── radar3d/           # Camera driver nodes
├── bringup/              # Launch files & configs
├── docs/                 # Project docs (MkDocs-ready)
└── README.md
```

---

## Configuration
- Default parameter files live under `bringup/config/`.
- Typical overrides:
  - **Model paths** (YOLO weights)
  - **Topic names / QoS profiles**
  - **Sensor calibration** (intrinsics/extrinsics, time offsets)
  - **Tracking** thresholds (gating, birth/death rates)

Example (passing a params file):
```bash
ros2 launch bringup demo_launch.py params_file:=bringup/config/demo_params.yaml
```

---

## Documentation

This project uses **MkDocs** for documentation.  
To build and serve the docs locally:

```bash
# Enter the documentation directory
cd ~/ws_ros2/MultiSense-UAV-Detection/mydocs

# Create an isolated virtual environment (do NOT create it after sourcing ROS)
python3 -m venv .venv
source .venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Start the local documentation server
mkdocs serve
```
Then open the browser at http://127.0.0.1:8000
 to preview the site.
Use Ctrl+C to stop the server.

---

## Roadmap
- [ ] Multi-camera support
- [ ] ROS 2 bag record/playback integration
- [ ] Enhanced Foxglove dashboards
- [ ] Swarm/multi-UAV scenarios and stress testing

---

## License
This project is licensed under the [MIT License](./LICENSE).



