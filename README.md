<div align="center">

# 🛸 Autonomous Swarm UAV Formation Navigation & APF Guidance

**Decentralized Multi-UAV Swarm Avionics: Leader-Follower Guidance, Artificial Potential Field (APF) Collision Avoidance, ESP-Mesh Telemetry & OpenCV ArUco Vision**

[![Python 3.10](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![MAVLink 2.0](https://img.shields.io/badge/Protocol-PyMAVLink_2.0-blue)](https://mavlink.io/)
[![DroneKit](https://img.shields.io/badge/Avionics-DroneKit_Python-orange)](https://dronekit.netlify.app/)
[![Computer Vision](https://img.shields.io/badge/Vision-OpenCV_ArUco-5C3EE8?logo=opencv&logoColor=white)](https://opencv.org/)
[![GUI: PyQt5](https://img.shields.io/badge/GCS-PyQt5_Ground_Control-41CD52?logo=qt&logoColor=white)](gcs/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

[Architecture](#-system-architecture) • [Formation Simulator](sim/swarm_visualizer.py) • [Leader Flight Core](core/leader_fnav.py) • [Follower Core](core/follower1_fnav.py) • [GCS Ground Station](gcs/)

</div>

---

## 📌 Executive Summary

Autonomous multi-agent UAV operations require high-bandwidth coordination without relying entirely on centralized cloud infrastructure. In dynamic GPS-degraded or adversarial airspace, swarm agents must compute deterministic trajectory vectors to maintain tactical geometry while avoiding static obstacles and neighboring drones.

This repository implements a **complete multi-UAV swarm guidance stack**:
1. **Leader-Follower Formation Kinematics:** Computes dynamic NED (North-East-Down) velocity vectors ($V_x, V_y, V_z$) at $10\,\text{Hz}$ via **PyMAVLink 2.0 / DroneKit** to maintain rigid V-formation and Line-formation patterns.
2. **Artificial Potential Field (APF) Avoidance:** Real-time vector field math calculating attractive forces toward formation slots and strong non-linear repulsive fields around obstacles.
3. **Ad-Hoc Inter-UAV Telemetry:** Lightweight binary broadcast via ESP32 **PainlessMesh** allowing resilient inter-drone state sharing without cellular towers.
4. **GPS-Denied Relative Localization:** Integrated **OpenCV ArUco marker pose estimation** allowing followers to visually lock onto the leader's airframe.
5. **Live Offline Swarm Simulator (`sim/swarm_visualizer.py`):** Standalone visualizer testing APF vectors, obstacle avoidance, and follower slot retention without physical hardware.

---

## 🏛️ System Architecture

```mermaid
graph TD
    subgraph LeaderUAV [Leader UAV (Drone 0)]
        L_NAV[Waypoint Mission Planner] -->|Global NED Pos| L_MAV[PyMAVLink Flight Core]
        L_MAV -->|Broadcast Telemetry| MESH[ESP-Mesh Ad-Hoc Radio]
        ARUCO_TAG[Rear ArUco Marker]
    end

    subgraph Follower1 [Follower UAV 1 (Port Wing)]
        MESH -->|Leader State Frame| F1_APF[APF Vector Controller]
        CAM1[Downward/Forward Camera] -->|Marker Pose Estimation| ARUCO_EST1[ArUco Visual Tracker]
        ARUCO_EST1 & F1_APF --> F1_CORE[PyMAVLink 10Hz NED Velocity Vectors]
    end

    subgraph Follower2 [Follower UAV 2 (Starboard Wing)]
        MESH -->|Leader State Frame| F2_APF[APF Vector Controller]
        CAM2[Downward/Forward Camera] -->|Marker Pose Estimation| ARUCO_EST2[ArUco Visual Tracker]
        ARUCO_EST2 & F2_APF --> F2_CORE[PyMAVLink 10Hz NED Velocity Vectors]
    end

    subgraph GroundStation [Ground Control Station]
        GCS[PyQt5 Swarm Cockpit GCS] -.->|Telemetry & Abort| LeaderUAV & Follower1 & Follower2
    end
```

---

## 📐 Artificial Potential Field (APF) Mathematics

The steering force $\vec{F}_{\text{total}}$ acting on each follower drone is the superposition of an attractive slot-seeking force and repulsive obstacle forces:

$$\vec{F}_{\text{total}} = \vec{F}_{\text{att}} + \sum_{i} \vec{F}_{\text{rep}, i}$$

### 1. Attractive Potential ($F_{\text{att}}$):
Directs the follower toward its calculated formation slot relative to the leader:
$$\vec{F}_{\text{att}} = k_{\text{att}} \cdot (\vec{P}_{\text{slot}} - \vec{P}_{\text{drone}})$$

### 2. Repulsive Obstacle Potential ($F_{\text{rep}}$):
Activates only when distance to an obstacle $d_i$ falls within influence threshold $d_{\text{influence}}$:
$$\vec{F}_{\text{rep}, i} = k_{\text{rep}} \left( \frac{1}{d_i} - \frac{1}{d_{\text{influence}}} \right) \frac{1}{d_i^2} \cdot \hat{n}_i$$

---

## 📁 Repository Structure

```
swarm-uav-formation-nav/
├── sim/                                 # ══ Offline Swarm Simulator ══
│   ├── swarm_visualizer.py              # APF kinematic trajectory visualizer
│   └── tests/
│       └── test_apf_vectors.py          # Vector physics & APF unit tests
│
├── core/                                # ══ Multi-Agent Autonomy Flight Cores ══
│   ├── leader_fnav.py                   # Leader waypoint navigator & mesh broadcaster
│   ├── follower1_fnav.py                # Follower 1 APF slot controller
│   └── follower2_fnav.py                # Follower 2 APF slot controller
│
├── gcs/                                 # ══ Ground Control Station (GCS) ══
│   ├── ui_formation_navigation.py       # Full-scale PyQt5 swarm flight cockpit
│   └── ui_aruco.py                      # Computer vision relative tracking UI
│
├── vision/                              # ══ OpenCV ArUco Tracking ══
│   ├── drone0_aruco.py                  # Leader camera & marker detector
│   ├── drone1_aruco.py                  # Follower 1 visual tracking pipeline
│   └── drone2_aruco.py                  # Follower 2 visual tracking pipeline
│
├── comms/                               # ══ Inter-Drone Mesh Telemetry ══
│   └── esp_mesh.py                      # ESP32 PainlessMesh serial bridge
│
├── .github/workflows/
│   └── python-lint.yml                  # Automated unit test CI
│
├── LICENSE                              # MIT License
└── README.md
```

---

## ⚙️ Quick Start

### 1. Run Offline Swarm Formation Simulator
Test V-formation navigation and real-time obstacle avoidance without physical drones:
```bash
python sim/swarm_visualizer.py --steps 120
```

### 2. Run Kinematic Physics Tests
```bash
python -m unittest discover -s sim/tests
```

### 3. Launch PyQt5 Ground Control Station (GCS)
```bash
python gcs/ui_formation_navigation.py
```

---

## 👨‍💻 Engineering Attribution

Engineered by **[Kadir Mercan](https://www.linkedin.com/in/kadir-mercan-/)**  
*Senior Embedded Systems, Hardware & IoT Engineer*
