# Mathematical Formulation: Artificial Potential Field (APF) & Formation Kinematics

This document provides the rigorous mathematical foundation for the **Leader-Follower Swarm Guidance and Obstacle Avoidance Stack** implemented in this repository.

---

## 1. Coordinate Frame Transformations

The swarm operates under two reference coordinate frames:
1. **Inertial NED Frame ($\mathcal{F}_{NED}$):** Global coordinate system with axes pointing North ($X$), East ($Y$), and Down ($Z$).
2. **Leader Body Frame ($\mathcal{F}_{B}$):** Local coordinate system fixed to the Leader UAV, where $+X_B$ points along the Leader's heading vector $\psi_L$, and $+Y_B$ points to starboard (right wing).

### Rotation Matrix (Body to Global NED)
To project relative formation offset positions $\mathbf{p}_{slot}^B = [\Delta x^B, \Delta y^B]^T$ into the global inertial target coordinates $\mathbf{p}_{target}^{NED}$:

$$\mathbf{R}(\psi_L) = \begin{bmatrix} \cos\psi_L & -\sin\psi_L \\ \sin\psi_L & \cos\psi_L \end{bmatrix}$$

$$\mathbf{p}_{target}^{NED} = \mathbf{p}_L^{NED} + \mathbf{R}(\psi_L) \cdot \mathbf{p}_{slot}^B$$

---

## 2. Artificial Potential Field (APF) Mechanics

The motion of each follower drone is governed by an artificial energy potential field $U_{total}(\mathbf{q})$ composed of two antagonist potential fields:

$$U_{total}(\mathbf{q}) = U_{att}(\mathbf{q}) + \sum_{i=1}^{M} U_{rep, i}(\mathbf{q})$$

The total steering force $\mathbf{F}_{total}(\mathbf{q})$ acting on the UAV is the negative gradient of the total potential function:

$$\mathbf{F}_{total}(\mathbf{q}) = -\nabla U_{total}(\mathbf{q}) = \mathbf{F}_{att}(\mathbf{q}) + \sum_{i=1}^{M} \mathbf{F}_{rep, i}(\mathbf{q})$$

---

### 2.1. Attractive Potential ($U_{att}$)
The attractive field pulls the follower UAV toward its assigned geometric slot $\mathbf{q}_{target}$:

$$U_{att}(\mathbf{q}) = \frac{1}{2} k_{att} \|\mathbf{q} - \mathbf{q}_{target}\|^2$$

Taking the negative gradient yields the linear attractive force vector:

$$\mathbf{F}_{att}(\mathbf{q}) = -\nabla U_{att}(\mathbf{q}) = -k_{att} (\mathbf{q} - \mathbf{q}_{target}) = k_{att} (\mathbf{q}_{target} - \mathbf{q})$$

Where:
- $k_{att} > 0$ is the positive attractive scaling gain.
- The force scales proportionally with distance to the formation slot.

---

### 2.2. Repulsive Potential ($U_{rep}$)
To prevent mid-air collisions with static/dynamic obstacles and neighboring swarm agents, a non-linear repulsive potential is activated only within an influence distance $d_0$:

$$U_{rep}(\mathbf{q}) = \begin{cases} 
\frac{1}{2} k_{rep} \left( \frac{1}{d(\mathbf{q})} - \frac{1}{d_0} \right)^2 & \text{if } d(\mathbf{q}) \le d_0 \\ 
0 & \text{if } d(\mathbf{q}) > d_0 
\end{cases}$$

Where $d(\mathbf{q}) = \|\mathbf{q} - \mathbf{q}_{obs}\| - R_{obs}$ is the Euclidean distance from the UAV boundary to the obstacle perimeter.

Differentiating yields the repulsive force:

$$\mathbf{F}_{rep}(\mathbf{q}) = -\nabla U_{rep}(\mathbf{q}) = \begin{cases} 
k_{rep} \left( \frac{1}{d(\mathbf{q})} - \frac{1}{d_0} \right) \frac{1}{d^2(\mathbf{q})} \cdot \frac{\mathbf{q} - \mathbf{q}_{obs}}{\|\mathbf{q} - \mathbf{q}_{obs}\|} & \text{if } d(\mathbf{q}) \le d_0 \\ 
\mathbf{0} & \text{if } d(\mathbf{q}) > d_0 
\end{cases}$$

---

## 3. MAVLink 2.0 Velocity Command Injection

In standard multi-agent autopilot firmware (ArduPilot / PX4), trajectory tracking is controlled via direct velocity vector streaming at $10\,\text{Hz}$ using the `SET_POSITION_TARGET_LOCAL_NED` message:

```c
mavlink_msg_set_position_target_local_ned_pack(
    system_id,
    component_id,
    &msg,
    time_boot_ms,
    target_system,
    target_component,
    MAV_FRAME_LOCAL_NED,
    0b0000111111000111, // Type mask: ignore pos/accel, use Vx, Vy, Vz
    0, 0, 0,            // x, y, z (ignored)
    vx, vy, vz,         // Velocity vector (m/s)
    0, 0, 0,            // afx, afy, afz (ignored)
    yaw, yaw_rate
);
```

### Speed Saturation
To preserve aerodynamic stability and motor headroom, the computed velocity vector is saturated:

$$\mathbf{V}_{command} = \begin{cases} 
\mathbf{F}_{total} & \text{if } \|\mathbf{F}_{total}\| \le V_{max} \\ 
V_{max} \cdot \frac{\mathbf{F}_{total}}{\|\mathbf{F}_{total}\|} & \text{if } \|\mathbf{F}_{total}\| > V_{max} 
\end{cases}$$

---

## 4. Local Minima Mitigation (Virtual Sub-Goal Method)

A classical failure mode of APF is the **local minima problem**, where $\mathbf{F}_{att} + \mathbf{F}_{rep} = \mathbf{0}$ before reaching the target slot (e.g. when an obstacle lies directly collinear between the UAV and its slot).

### Remediation Strategy
When the resultant force $\|\mathbf{F}_{total}\| < \epsilon$ while $\|\mathbf{q} - \mathbf{q}_{target}\| > \delta$:
1. A tangential perturbation vector $\mathbf{F}_{tangent} = \mathbf{F}_{rep} \times \hat{\mathbf{z}}$ is injected perpendicular to the obstacle vector.
2. The agent breaks symmetry and follows the circular equipotential contour around the obstacle until the collinear dead-lock resolves.
