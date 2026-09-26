# Multi-UAV Swarm Formation Navigation & Vision-Based Relative Guidance

[![License: Proprietary Showcase](https://img.shields.io/badge/License-Proprietary_Portfolio_Showcase-red.svg)](LICENSE)
[![Autopilot: ArduPilot / PX4](https://img.shields.io/badge/Autopilot-ArduPilot%20%2F%20PX4-green.svg)](https://ardupilot.org/)
[![Protocol: MAVLink 2.0](https://img.shields.io/badge/Protocol-MAVLink%202.0-blue.svg)](https://mavlink.io/)
[![Mesh: ESP32 PainlessMesh](https://img.shields.io/badge/Inter--UAV%20Mesh-ESP32%20PainlessMesh-orange.svg)](https://gitlab.com/painlessMesh/painlessMesh)
[![Vision: OpenCV ArUco](https://img.shields.io/badge/Computer%20Vision-OpenCV%20ArUco-yellow.svg)](https://opencv.org/)
[![GUI: PyQt5 Ground Station](https://img.shields.io/badge/GCS-PyQt5%20Ground%20Station-informational.svg)](gcs/)

> [!WARNING]
> **PROPRIETARY PORTFOLIO SHOWCASE NOTICE**  
> This repository is published strictly for technical evaluation, code review, and algorithmic demonstration as part of **Kadir Mercan's (Mercan Studio)** engineering portfolio.  
> **All rights reserved.** Reproduction, deployment on flight hardware, commercial exploitation, or reverse engineering of this software without prior written authorization is strictly prohibited under the terms of the included [LICENSE](LICENSE).

---

## 🎯 Executive Summary

A real-time, decentralized **Autonomous Swarm UAV Formation Navigation System** deployed on multi-rotor unmanned aerial vehicles. The architecture integrates **PyMAVLink / DroneKit** companion computers, an ad-hoc **ESP32 PainlessMesh inter-UAV telemetry bus**, **Artificial Potential Field (APF)** collision avoidance, and **OpenCV ArUco marker vision tracking** for relative pose estimation.

Designed for synchronized multi-agent tactical maneuvers, including dynamic Crescent (*Hilal*), V-Formation, and Line patrol configurations with heading compensation and velocity vectoring.

---

## 🏛️ System Architecture

```
                                  ┌──────────────────────────────┐
                                  │      Ground Control Station   │
                                  │    (PyQt5 Formation GCS)     │
                                  └──────────────┬───────────────┘
                                                 │ USB / RF Telemetry
                                                 ▼
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                   ESP32 PAINLESS-MESH INTER-UAV TELEMETRY BUS                          │
│        (JSON broadcast: Formation Vectors, Heading Commands, Health Diagnostics)        │
└───────────────┬────────────────────────────────┬───────────────────────────────┬───────┘
                │                                │                               │
                ▼                                ▼                               ▼
   ┌─────────────────────────┐      ┌─────────────────────────┐     ┌─────────────────────────┐
   │      DRONE 0 (LEADER)   │      │   DRONE 1 (FOLLOWER)    │     │   DRONE 2 (FOLLOWER)    │
   ├─────────────────────────┤      ├─────────────────────────┤     ├─────────────────────────┤
   │ • Companion Computer    │      │ • Companion Computer    │     │ • Companion Computer    │
   │ • Streaming Global Nav  │      │ • APF Collision Avoidance│    │ • APF Collision Avoidance│
   │ • ArUco Optical Tracking│      │ • NED Velocity Control  │     │ • NED Velocity Control  │
   │ • Smart Yaw Rate Limiter│      │ • Path Smoothing Filter │     │ • Path Smoothing Filter │
   │ • Pixhawk / MAVLink     │      │ • Pixhawk / MAVLink     │     │ • Pixhawk / MAVLink     │
   └─────────────────────────┘      └─────────────────────────┘     └─────────────────────────┘
```

---

## ⚙️ Core Algorithmic & Engineering Modules

### 1. Artificial Potential Field (APF) Collision Avoidance
Follower UAVs execute high-rate velocity vectoring in the local North-East-Down (NED) frame. To ensure inter-vehicle safety while maintaining tight tactical formations, the navigation engine computes resultant force vectors:

$$F_{\text{total}} = F_{\text{att}} + \sum F_{\text{rep}}$$

* **Attractive Force ($F_{\text{att}}$):** Drives the follower toward the dynamic formation slot calculated from the leader's GPS origin and orientation.
* **Repulsive Force ($F_{\text{rep}}$):** Activates dynamically when neighboring drones breach the $5.0\,\text{m}$ influence threshold ($d < d_{\text{influence}}$), with aggressive gain scaling inside the $3.0\,\text{m}$ critical collision boundary:
  $$F_{\text{rep}} = \eta \left( \frac{1}{d} - \frac{1}{d_0} \right) \frac{1}{d^2} \hat{r}$$

### 2. High-Frequency PyMAVLink NED Velocity Control
Instead of slow global waypoint commands (`NAV_WAYPOINT`), follower trajectories are modulated at **10 Hz** using native MAVLink velocity commands:
* Command: `MAV_CMD_DO_SET_POSITION_TARGET_LOCAL_NED`
* Velocity smoothing factor ($0.8$) and trajectory exponential filtering prevent abrupt motor current spikes and actuator chatter.
* Dynamic approach deceleration curves smoothly reduce cruise speed ($3.0\,\text{m/s}$) to landing precision ($0.5\,\text{m/s}$) within $5.0\,\text{m}$ of target convergence.

### 3. Smart Yaw Management & Deadband Limiter
Prevents compass hunting and continuous yaw spinning during formation turns:
* Minimal shortest-path angle wrapping (`wrap180`, `wrap360`).
* Deadband threshold ($3.0^\circ$) suppresses negligible oscillations.
* Rate-bounded yaw slew limit ($25^\circ/\text{s}$) maintains camera gimbal and sensor stability.

### 4. ESP32 PainlessMesh Ad-Hoc Inter-Drone Telemetry
* Autonomous self-healing RF mesh eliminating single-point-of-failure Wi-Fi access points.
* Transmits compact JSON payloads containing state estimation, target offsets, sequence numbers, and formation heading vectors.

### 5. Computer Vision: ArUco Marker Relative Guidance
* Onboard camera pipelines run OpenCV ArUco dictionary recognition to calculate relative azimuth, elevation, and range ($X, Y, Z$).
* Serves as an optical fallback when GPS signals undergo denial or multipath degradation in urban/GPS-denied zones.

---

## 📁 Repository Structure

```
swarm-uav-formation-nav/
├── core/
│   ├── leader_fnav.py            # Drone 0: Master path streamer & formation heading director
│   ├── follower1_fnav.py         # Drone 1: Crescent (Hilal) formation with APF vectoring
│   └── follower2_fnav.py         # Drone 2: Crescent wing-follower with velocity smoothing
├── comms/
│   └── esp_mesh.py               # Asynchronous threaded serial bridge to ESP32 PainlessMesh
├── vision/
│   ├── drone0_aruco.py           # Optical detection & relative position tracking
│   ├── drone1_aruco.py           # Vision-assisted follower orientation
│   └── drone2_aruco.py           # Multi-marker spatial calibration
├── gcs/
│   ├── ui_formation_navigation.py# Full-featured PyQt5 Ground Station with live telemetry
│   └── ui_aruco.py               # Vision telemetry visualizer & target diagnostic GUI
├── requirements.txt              # Python runtime dependencies (DroneKit, MAVLink, PyQt5)
├── LICENSE                       # Strict Proprietary Portfolio Showcase License
└── README.md                     # Technical specification & mathematical foundation
```

---

## 🚀 Execution & Simulation (SITL)

1. **Install Dependencies:**
   ```bash
   pip install -r requirements.txt
   ```
2. **Launch with ArduPilot Software-In-The-Loop (SITL):**
   ```bash
   # Terminal 1: Launch Ground Control Station
   python gcs/ui_formation_navigation.py

   # Terminal 2: Run Leader Node
   python core/leader_fnav.py --connect 127.0.0.1:14550

   # Terminal 3: Run Follower 1
   python core/follower1_fnav.py --connect 127.0.0.1:14560
   ```

---

## 👨‍💻 Engineering & Contact

**Kadir Mercan**  
*Senior Embedded Systems, Hardware (PCB) & IoT Engineer*  
- **GitHub:** [@kadirmercan00](https://github.com/kadirmercan00)  
- **Brand:** Mercan Studio  
- **Core Competencies:** UAV Avionics, Flight Control Firmware, Multi-Agent Swarm Algorithms, RF Embedded Hardware, Robotics Edge Vision.
