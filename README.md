<div align="center">

# 🛸 Autonomous Swarm UAV Formation Navigation & APF Guidance

**Decentralized Multi-UAV Swarm Avionics: Leader-Follower Guidance, Artificial Potential Field (APF) Collision Avoidance, ESP-Mesh Telemetry & OpenCV ArUco Vision**

[![Python 3.10](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![MAVLink 2.0](https://img.shields.io/badge/Protocol-PyMAVLink_2.0-blue)](https://mavlink.io/)
[![DroneKit](https://img.shields.io/badge/Avionics-DroneKit_Python-orange)](https://dronekit.netlify.app/)
[![Computer Vision](https://img.shields.io/badge/Vision-OpenCV_ArUco-5C3EE8?logo=opencv&logoColor=white)](https://opencv.org/)
[![Web Simulator](https://img.shields.io/badge/Simulator-60_FPS_Interactive_Web-brightgreen.svg)](simulator/)
[![Math Formulation](https://img.shields.io/badge/Math-APF_Vector_Field-red.svg)](docs/APF_MATHEMATICS.md)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

[Interactive Web Simulator](simulator/) • [Mathematical Formulation](docs/APF_MATHEMATICS.md) • [Architecture](#-system-architecture) • [Leader Flight Core](core/leader_fnav.py) • [Follower Core](core/follower1_fnav.py) • [GCS Ground Station](gcs/)

</div>

---

## 🎮 Interactive Web Swarm Simulator (`simulator/`)

An out-of-the-box, 60 FPS interactive tactical radar simulator located in `simulator/index.html`. It demonstrates the full APF multi-UAV guidance stack directly in any modern browser without needing Python or SITL installations:
- **Real-Time Force Vector Visualizer:** Renders designated attractive vectors ($\mathbf{F}_{att}$ in green), non-linear obstacle repulsion ($\mathbf{F}_{rep}$ in red), and resultant velocity ($\mathbf{V}_{ned}$ in cyan).
- **Interactive Obstacle Spawning:** Left-click on the radar screen to drop or remove dynamic radar threats / no-fly zones; observe follower drones fluidly evading obstacles and re-converging to formation slots.
- **Tactical Geometry Reconfiguration:** Instant in-flight switching between **V-Shape (Wedge)**, **Column (Trail)**, **Echelon (Line)**, and **Diamond Wing** formations.
- **Flight Mission Modes:** Circle patrol, figure-8 recon, or direct cursor tracking.
- **Avionics Cockpit HUD:** Real-time speed, slot error ($\Delta x, \Delta y$), and collision avoidance status indicators.

👉 **Launch Simulator:** Open `simulator/index.html` in your web browser.

---

## 📌 Executive Summary

Autonomous multi-agent UAV operations require high-bandwidth coordination without relying entirely on centralized cloud infrastructure. In dynamic GPS-degraded or adversarial airspace, swarm agents must compute deterministic trajectory vectors to maintain tactical geometry while avoiding static obstacles and neighboring drones.

This repository implements a **complete multi-UAV swarm guidance stack**:
1. **Leader-Follower Formation Kinematics:** Computes dynamic NED (North-East-Down) velocity vectors ($V_x, V_y, V_z$) at $10\,\text{Hz}$ via **PyMAVLink 2.0 / DroneKit** to maintain rigid V-formation and Line-formation patterns.
2. **Artificial Potential Field (APF) Avoidance:** Real-time vector field math calculating attractive forces toward formation slots and strong non-linear repulsive fields around obstacles (see [Mathematical Formulation](docs/APF_MATHEMATICS.md)).
3. **Interactive 60 FPS Simulator (`simulator/`):** Instant browser-based visualizer for force vectors and dynamic geometries.
4. **Ad-Hoc Inter-UAV Telemetry:** Lightweight binary broadcast via ESP32 **PainlessMesh** allowing resilient inter-drone state sharing without cellular towers.
5. **GPS-Denied Relative Localization:** Integrated **OpenCV ArUco marker pose estimation** allowing followers to visually lock onto the leader's airframe.

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
