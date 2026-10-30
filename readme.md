# ROS 2 — Technical Benchmarks & Practical Evaluations


> **Project Scope & Attribution:**  
> The challenge specifications, system requirements, and expected outputs in this document were generated with AI to serve as practice benchmarks.  
> 
> All ROS 2 packages, node implementations, and mathematical logic were independently authored by me, with explicit notes wherever AI assistance was referenced.
>
> Topic name, executable name of nodes might not match the original question but the implementation is as described in the question
---

## Technical Scope & Architecture Overview

This repository documents hands-on engineering challenges designed to bridge the gap between basic, isolated documentation tutorials and production-ready, distributed robotics software. 

The primary goal is to build a foundation in core ROS 2 communication and node architecture **before transitioning to higher level frameworks like `tf2` coordinate transforms, URDF modeling and Gazebo simulation.**

### Core ROS 2 Concepts Practiced
* **Communication:** Publishers & Subscribers (Topics), Services (Client/Server), and Actions (Goal/Feedback/Result).
* **Interfaces & Configuration:** Custom IDL schemas (`.msg`, `.srv`, `.action`), Parameter declarations, and YAML configs.
* **Orchestration & Scale:** Multi-node launch files, dynamic parameter validation callbacks, and multi-package layouts.
* **Concurrency:** `MultiThreadedExecutor` and `ReentrantCallbackGroup` to prevent callback starvation.

---

### Applied Systems Paradigms
1. **Computational Concurrency:** Mitigating single-threaded callback deadlocks and starvation when long-running action loops, timers, and service requests share the same node.
2. **Interface Contracts:** Defining strict, clear boundaries between continuous telemetry (topics), synchronous state overrides (services), and preemptible physical maneuvers (actions).
3. **Kinematics & Control:** Implementing forward differential-drive kinematics, continuous angle-wrapping normalization, and closed-loop proportional steering controllers.
4. **Safety & Fault Handling:** Dynamic runtime parameter boundary enforcement, geofenced keep-out zones, and asynchronous emergency braking.

---

## Index of Engineering Challenges

| Index | Project | Complexity | Primary Topics Covered | Status |
| :--- | :--- | :--- | :--- | :--- |
| **01** | [Power & Charging Management Subsystem](#project-1-robot-power--charging-manager) | Beginner | Pub/Sub, Custom Interfaces, Services, Actions, Parameters | **COMPLETED** |
| **02** | [AGV Waypoint Dispatcher & Geofence Monitor](#project-2-agv-waypoint-dispatcher--energy-monitor) | Intermediate | Multi-threading, Launch Files, Dynamic Energy Modeling, Geofencing | **COMPLETED** |
| **03** | [Differential-Drive Kinematics & Path Tracker](#project-3-2d-differential-drive-kinematics--path-tracker) | Advanced | Closed-Loop Kinematics, Angle Wrapping, Emergency Brake, Param Validation | **COMPLETED** |

---

# Project 1: Robot Power & Charging Manager
**Status:** `COMPLETED` | **Level:** `Beginner / Foundations`  
**Packages:** `robot_interfaces` (`ament_cmake`), `robot_py_app` (`ament_python`)       
**Files:**['Robot App'](src/robot_py_app) ['Robot Interface'](src/robot_interface)

### System Architecture
A simulated autonomous base tracks its internal battery depletion, offers state-reset capabilities through an asynchronous Service, and handles continuous charging sequences through a feedback-enabled Action Server. A supervisory monitor node drives operational states and asserts low-power warnings.

### Architectural Constraints
* **`robot_interfaces` (`ament_cmake`):** Houses custom interface definitions (`.msg`, `.srv`, `.action`).
* **`robot_py_app` (`ament_python`):**
  * `robot_node.py`:
    * Declares parameters: `robot_name` (string, default: `"Robby"`), `drain_rate` (int, default: `1`).
    * Publisher: Emits `robot_interfaces/msg/BatteryStatus` at 1.0 Hz.
    * Service Server: Implements `/robot/reset_battery` (`robot_interfaces/srv/ResetBattery`) to forcefully override battery state.
    * Action Server: Implements `/robot/charge_battery` (`robot_interfaces/action/ChargeBattery`), incrementing battery at 5% per second until the target threshold is reached.
  * `monitor_node.py`:
    * Declares parameters: `charger_available` (int, default: `1`).
    * Subscriber: Monitors `/robot/battery_status`, triggers warning logs when state falls below 90%.
    * Sequence Controller: Invokes `/robot/reset_battery` (to 50%) after a 3.0-second delay, then executes a `/robot/charge_battery` Action Goal to 65%.

### Interface Contracts
- **`msg/BatteryStatus.msg`**
  ```text
  string robot_name
  int32 battery_level
  bool is_charging
  ```
- **`srv/ResetBattery.srv`**
  ```text
  int32 new_level
  ---
  bool success
  string message
  ```
- **`action/ChargeBattery.action`**
  ```text
  int32 target_level
  ---
  bool complete
  int32 final_level
  ---
  int32 current_level
  ```

### Acceptance Verification

#### Terminal 1 — Primary Node Execution
```bash
ros2 run robot_py_app robot_node --ros-args -p robot_name:="Apollo" -p drain_rate:=2
```
```text
[INFO] [robot_node]: Node started. Robot: Apollo, Drain rate: 2
[INFO] [robot_node]: Published Status -> Battery: 98%, Charging: False
[INFO] [robot_node]: Published Status -> Battery: 96%, Charging: False
[INFO] [robot_node]: Service request received: Reset to 50%
[INFO] [robot_node]: Battery reset successful.
[INFO] [robot_node]: Received charge goal to: 65%
[INFO] [robot_node]: Charging... Current: 55%
[INFO] [robot_node]: Charging... Current: 60%
[INFO] [robot_node]: Charging... Current: 65%
[INFO] [robot_node]: Charge goal succeeded.
[INFO] [robot_node]: Published Status -> Battery: 63%, Charging: False
```

#### Terminal 2 — Monitor & Client Dispatcher
```bash
ros2 run robot_py_app monitor_node
```
```text
[INFO] [monitor_node]: Listening for battery status...
[INFO] [monitor_node]: Status from Apollo: 98%
[INFO] [monitor_node]: Status from Apollo: 96%
[INFO] [monitor_node]: Sending reset request to set battery to 50%...
[INFO] [monitor_node]: Service response: Success! Message: Battery set to 50
[INFO] [monitor_node]: Sending Charge goal to reach 65%...
[INFO] [monitor_node]: [Feedback] Current charge level: 55%
[INFO] [monitor_node]: [Feedback] Current charge level: 60%
[INFO] [monitor_node]: [Feedback] Current charge level: 65%
[INFO] [monitor_node]: [Result] Charging complete! Final level: 65%
[WARN] [monitor_node]: Low Battery Warning! Apollo is at 63%
```

---

# Project 2: AGV Waypoint Dispatcher & Energy Monitor
**Status:** `COMPLETED` | **Level:** `Intermediate`  
**Packages:** `agv_interfaces` (`ament_cmake`), `agv_controller` (`ament_python`)     
**Files:** [AGV Controller](src/agv_controller), [AGV Interfaces](src/agv_interfaces)

### System Architecture
An Automated Guided Vehicle (AGV) deployed in an indoor warehouse navigates in a continuous 2D Euclidean coordinate frame starting from origin `(0.0, 0.0)`. Destination waypoints dispatched by a fleet supervisor must undergo spatial clearance checking against a circular geofenced hazard zone prior to trajectory initiation. Energy consumption is modeled dynamically as a function of travel distance and cargo load.

### Architectural Specifications
* **Package Structure:**
  ```text
  ros2_ws/src/
  ├── agv_interfaces/
  │   ├── CMakeLists.txt
  │   ├── package.xml
  │   ├── msg/AgvTelemetry.msg
  │   ├── srv/ValidateWaypoint.srv
  │   └── action/NavigateToWaypoint.action
  └── agv_controller/
      ├── config/agv_params.yaml
      ├── launch/agv_mission.launch.py
      └── agv_controller/
          ├── agv_node.py
          └── dispatcher_node.py
  ```

* **Interface Definitions:**
  * `msg/AgvTelemetry.msg`: `float64 current_x`, `float64 current_y`, `float64 current_battery`, `string status`
  * `srv/ValidateWaypoint.srv`: Request: `float64 target_x`, `float64 target_y` | Response: `bool is_valid`, `float64 estimated_distance`, `string reason`
  * `action/NavigateToWaypoint.action`: Goal: `target_x`, `target_y`, `payload_weight` | Result: `reached`, `total_distance_traveled`, `remaining_battery` | Feedback: `distance_to_goal`, `current_battery`

* **Concurrency Management:**
  The `agv_node` must support simultaneous telemetry publishing (2 Hz), service processing, and action execution loops without thread starvation. Both the action server and service callbacks are assigned to a `ReentrantCallbackGroup()` and driven by a `MultiThreadedExecutor()`.

### Acceptance Verification

#### Validation Case 1: Permissible Trajectory Dispatch (Launch Script)
```bash
ros2 launch agv_controller agv_mission.launch.py mission_cargo_weight:=10.0
```
```text
[INFO] [agv_node-1]: AGV Online. Initial Pose: (0.00, 0.00) | Battery: 100.0%
[INFO] [agv_node-1]: Hazard Zone Configured -> Center: (5.00, 5.00), Radius: 2.00m
[INFO] [dispatcher_node-2]: Dispatcher initialized. Target: (3.00, 4.00) | Cargo: 10.0kg
[INFO] [dispatcher_node-2]: Validating waypoint (3.00, 4.00) with AGV Service...
[INFO] [agv_node-1]: SERVICE: Received waypoint validation request -> (3.00, 4.00)
[INFO] [agv_node-1]: SERVICE: Distance to hazard: 2.24m. Target is CLEAR. Validation: APPROVED.
[INFO] [dispatcher_node-2]: Service Response -> Valid: True | Est. Distance: 5.00m | Reason: Clear
[INFO] [dispatcher_node-2]: Action Goal Accepted by AGV. Monitoring progress...
[INFO] [dispatcher_node-2]: [Feedback] Dist to Goal: 4.00m | Current Battery: 99.0%
[INFO] [dispatcher_node-2]: [Feedback] Dist to Goal: 3.00m | Current Battery: 98.0%
[INFO] [dispatcher_node-2]: [Feedback] Dist to Goal: 2.00m | Current Battery: 97.0%
[INFO] [dispatcher_node-2]: [Feedback] Dist to Goal: 1.00m | Current Battery: 96.0%
[INFO] [dispatcher_node-2]: [Feedback] Dist to Goal: 0.00m | Current Battery: 95.0%
[INFO] [dispatcher_node-2]: [RESULT] Mission Complete! Total Traveled: 5.00m | Final Battery: 95.0%
```

#### Validation Case 2: Geofence Hazard Rejection
```bash
ros2 run agv_controller dispatcher_node --ros-args -p goal_x:=5.5 -p goal_y:=5.0
```
```text
[INFO] [dispatcher_node]: Validating waypoint (5.50, 5.00) with AGV Service...
[ERROR] [dispatcher_node]: Service Response -> Valid: False | Reason: Inside Hazard Zone (Dist to hazard: 0.50m <= 2.00m)
[WARN] [dispatcher_node]: MISSION ABORTED: Proposed destination is unsafe. Action will NOT be dispatched.
```

---

# Project 3: 2D Differential-Drive Kinematics & Path Tracker
**Status:** `IN PROGRESS` | **Level:** `Advanced / Systems Control`  
**Packages:** `amr_interfaces` (`ament_cmake`), `amr_controller` (`ament_python`)     
**Files:** [AMR Controller](src/amr_controller), [AMR Interfaces](src/amr_interfaces)

### System Architecture
This module implements a closed-loop trajectory tracker for a non-holonomic mobile robot in 2D continuous space. Rather than treating motion as abstracted translation, the system runs local heading control and forward kinematic integration over time increments $\Delta t$. Dynamic safety systems (real-time speed zones, emergency braking service, dynamic parameter clamping) override navigation commands at runtime.

```
       (x_goal, y_goal)
            *
           /
          /   Distance (rho)
         /
        * (x, y)  Heading (theta)
       / 
      v (Velocity vector)
```

### Architectural Specifications

* **Package Layout:**
  ```text
  ros2_ws/src/
  ├── amr_interfaces/
  │   ├── CMakeLists.txt
  │   ├── package.xml
  │   ├── msg/
  │   │   ├── Pose2DStamped.msg
  │   │   └── SpeedLimit.msg
  │   ├── srv/
  │   │   └── EmergencyBrake.srv
  │   └── action/
  │       └── NavigateToPose.action
  │
  └── amr_controller/
      ├── config/amr_params.yaml
      ├── launch/amr_system.launch.py
      └── amr_controller/
          ├── amr_node.py
          └── supervisor_node.py
  ```

* **Interface Contracts:**
  * **`msg/Pose2DStamped.msg`:**
    ```text
    float64 x
    float64 y
    float64 theta
    float64 linear_velocity
    float64 angular_velocity
    ```
  * **`msg/SpeedLimit.msg`:**
    ```text
    float64 max_linear_speed
    ```
  * **`srv/EmergencyBrake.srv`:**
    ```text
    bool engage_brake
    ---
    bool success
    string message
    ```
  * **`action/NavigateToPose.action`:**
    ```text
    # Goal
    float64 goal_x
    float64 goal_y
    float64 tolerance
    ---
    # Result
    bool reached
    float64 final_x
    float64 final_y
    float64 final_theta
    string termination_reason
    ---
    # Feedback
    float64 current_x
    float64 current_y
    float64 current_theta
    float64 distance_remaining
    ```

---

### Node Engineering Requirements

#### Node 1: `amr_node.py`
* **Internal Registers:** $x=0.0$, $y=0.0$, $\theta=0.0$, $v=0.0$, $\omega=0.0$, `brake_engaged = False`.
* **Parameter Validation Callback:**
  * Declare: `default_linear_speed` (0.6, bounds: `[0.1, 1.5]`), `max_angular_speed` (1.2, bounds: `[0.2, 3.14]`), `kp_alpha` (1.5, bounds: `[0.5, 5.0]`).
  * Attach `add_on_set_parameters_callback` to reject out-of-bounds parameter configurations dynamically.
* **Telemetry Streaming:** 10 Hz Timer publishing `/amr/odometry` (`Pose2DStamped`).
* **Emergency Service:** `/amr/emergency_brake` flips internal brake latch and immediately commands zero velocity.
* **Action Server Engine:**
  * Action: `/amr/navigate_to_pose`.
  * Runs closed-loop integration loop at 10 Hz ($\Delta t = 0.1$).
  * Aborts if `brake_engaged == True` (`termination_reason: "EMERGENCY_BRAKE"`).
  * Cancels cleanly if `goal_handle.is_cancel_requested`.
  * Succeeds when $\rho \le \text{tolerance}$ (`termination_reason: "GOAL_REACHED"`).

#### Node 2: `supervisor_node.py`
* **Telemetry Listener:** Subscribes to `/amr/odometry`.
* **Speed Zone Regulation:** Publishes `/amr/speed_limit` messages dynamically.
* **Mission Dispatch Client:**
  * Mode `"NORMAL"`: Monitors goal completion.
  * Mode `"TRIGGER_BRAKE"`: Monitors feedback; when `distance_remaining < 1.5m`, asynchronously fires `/amr/emergency_brake`.

---

### Acceptance Verification Scenarios

#### Scenario A: Nominal Waypoint Navigation
```bash
ros2 launch amr_controller amr_system.launch.py mode:=NORMAL
```
```text
[amr_node-1] [INFO]: AMR Kinematics Online. Pose: (x=0.00, y=0.00, theta=0.00 rad)
[supervisor_node-2] [INFO]: Dispatching Goal -> Target: (2.00, 2.00), Tolerance: 0.10m
[amr_node-1] [INFO]: Nav Goal Accepted. Initial Distance: 2.83m
[supervisor_node-2] [INFO]: [Feedback] Rem: 2.45m | Pose: (0.28, 0.26, theta: 0.74 rad)
[supervisor_node-2] [INFO]: [Feedback] Rem: 1.82m | Pose: (0.71, 0.70, theta: 0.78 rad)
[supervisor_node-2] [INFO]: [Feedback] Rem: 1.15m | Pose: (1.18, 1.18, theta: 0.79 rad)
[supervisor_node-2] [INFO]: [Feedback] Rem: 0.48m | Pose: (1.66, 1.65, theta: 0.78 rad)
[supervisor_node-2] [INFO]: [Feedback] Rem: 0.08m | Pose: (1.95, 1.94, theta: 0.78 rad)
[amr_node-1] [INFO]: Target within tolerance (0.08m <= 0.10m). Stopping robot.
[supervisor_node-2] [INFO]: [RESULT] Success: True | Final Pose: (1.95, 1.94, 0.78 rad) | Reason: GOAL_REACHED
```

#### Scenario B: In-Flight Emergency Brake Interruption
```bash
ros2 launch amr_controller amr_system.launch.py mode:=TRIGGER_BRAKE
```
```text
[amr_node-1] [INFO]: Nav Goal Accepted. Initial Distance: 2.83m
[supervisor_node-2] [INFO]: [Feedback] Rem: 2.21m | Pose: (0.44, 0.42, theta: 0.76 rad)
[supervisor_node-2] [INFO]: [Feedback] Rem: 1.48m | Pose: (0.95, 0.94, theta: 0.78 rad)
[supervisor_node-2] [WARN]: Distance < 1.50m! Calling /amr/emergency_brake service...
[amr_node-1] [WARN]: SERVICE: Emergency Brake Engaged! Cutting motor velocities.
[amr_node-1] [ERROR]: ACTION: Aborting navigation due to active emergency brake!
[supervisor_node-2] [ERROR]: [RESULT] Action Aborted! Reason: EMERGENCY_BRAKE
```

#### Scenario C: Dynamic Parameter Boundary Enforcement
```bash
ros2 param set /amr_node default_linear_speed 5.0
```
```text
Setting parameter failed: Parameter 'default_linear_speed' value 5.0 exceeds allowed range [0.1, 1.5] m/s
```

---

# Project 4: Pan-Tilt Active Target Tracker & Frame Resolver
**Status:** `OPEN` | **Level:** `Beginner / Foundations of Spatial Transforms`  
**Packages:** `tracker_interfaces` (`ament_cmake`), `pantilt_tracker` (`ament_python`)

### System Architecture
An industrial inspection station utilizes a 2-DOF pan-tilt camera head mounted on a riser stand (`riser_link` $\rightarrow$ `pan_link` $\rightarrow$ `tilt_link` $\rightarrow$ `camera_optical_frame`). A static conveyor frame exists in the workspace. An external vision detector broadcasts dynamic detections of a moving part relative to the conveyor.

The tracker node must dynamically compute the target's position **relative to the camera's optical frame** using TF2 buffer transformations, calculate angular tracking errors, and execute an Action to slew the pan and tilt joints until the target is centered along the optical boresight ($Z$-axis).

```
   [world]
      │
      ├── (Static) ──> [riser_link] ──> [pan_link] (yaw: psi) ──> [tilt_link] (pitch: theta) ──> [camera_optical_frame]
      │
      └── (Static) ──> [conveyor_frame] ──> (Dynamic) ──> [target_object]
```

### Mathematical Foundations
1. **Coordinate Frame Transformation Chain:**
   Given the target pose in conveyor coordinates ${}^{\text{conv}}\mathbf{P}_{\text{obj}}$, the pose with respect to the camera optical frame is obtained via the TF2 transformation tree:
   $${}^{\text{cam}}\mathbf{P}_{\text{obj}} = \left({}^{\text{world}}\mathbf{T}_{\text{cam}}\right)^{-1} \cdot {}^{\text{world}}\mathbf{T}_{\text{conv}} \cdot {}^{\text{conv}}\mathbf{P}_{\text{obj}} = {}^{\text{cam}}\mathbf{T}_{\text{conv}} \cdot {}^{\text{conv}}\mathbf{P}_{\text{obj}}$$
   Where ${}^{A}\mathbf{T}_B \in SE(3)$ represents the homogeneous transformation matrix:
   $${}^{A}\mathbf{T}_B = \begin{bmatrix} \mathbf{R}_{3\times3} & \mathbf{p}_{3\times1} \\ \mathbf{0}_{1\times3} & 1 \end{bmatrix}$$

2. **Optical Boresight Error Calculation:**
   In standard camera optical conventions ($Z$ forward, $X$ right, $Y$ down):
   $$\text{Azimuth Error (Pan): } \Delta\alpha = \text{atan2}(x_{\text{cam}}, z_{\text{cam}})$$
   $$\text{Elevation Error (Tilt): } \Delta\beta = \text{atan2}(-y_{\text{cam}}, \sqrt{x_{\text{cam}}^2 + z_{\text{cam}}^2})$$

---

### Architectural Specifications

* **Package Structure:**
  ```text
  ros2_ws/src/
  ├── tracker_interfaces/
  │   ├── CMakeLists.txt
  │   ├── package.xml
  │   ├── msg/
  │   │   └── TrackingError.msg
  │   ├── srv/
  │   │   └── SetTrackingMode.srv
  │   └── action/
  │       └── CenterTarget.action
  └── pantilt_tracker/
      ├── config/tracker_params.yaml
      └── pantilt_tracker/
          ├── pan_tilt_broadcaster.py
          ├── target_simulator.py
          └── track_controller_node.py
  ```

* **Interface Contracts:**
  - **`msg/TrackingError.msg`:**
    ```text
    builtin_interfaces/Time stamp
    float64 pan_error_deg
    float64 tilt_error_deg
    float64 distance_to_target
    ```
  - **`srv/SetTrackingMode.srv`:**
    ```text
    string mode     # "IDLE", "CONTINUOUS", "HOLD"
    ---
    bool success
    string message
    ```
  - **`action/CenterTarget.action`:**
    ```text
    # Goal
    float64 angular_tolerance_deg
    float64 timeout_seconds
    ---
    # Result
    bool centered
    float64 final_pan_error_deg
    float64 final_tilt_error_deg
    ---
    # Feedback
    float64 current_pan_error_deg
    float64 current_tilt_error_deg
    ```

---

### Node Engineering Requirements

#### 1. `pan_tilt_broadcaster.py`
* Broadcasts static transforms: `world` $\rightarrow$ `riser_link` ($z = 0.5\text{m}$) and `world` $\rightarrow$ `conveyor_frame` ($x = 1.5\text{m}, y = 0.2\text{m}, z = 0.0\text{m}$).
* Maintains internal joint states: $\psi$ (pan), $\theta$ (tilt).
* Subscribes to `/joint_states` or internal state updates and publishes dynamic transforms at 30 Hz:
  * `riser_link` $\rightarrow$ `pan_link` (Rotation about $Z$: $\psi$)
  * `pan_link` $\rightarrow$ `tilt_link` (Rotation about $Y$: $\theta$)
  * `tilt_link` $\rightarrow$ `camera_optical_frame` (Standard camera optical orientation offset: $-90^\circ$ yaw, $+90^\circ$ pitch).

#### 2. `target_simulator.py`
* Publishes a dynamic transform `conveyor_frame` $\rightarrow$ `target_object` moving in a harmonic pattern along $X$ and $Y$ at 20 Hz ($x(t) = 0.3\sin(0.5t), y(t) = 0.1\cos(0.5t)$).

#### 3. `track_controller_node.py`
* Creates a `tf2_ros.Buffer` and `tf2_ros.TransformListener`.
* Lookups `${}^{\text{camera\_optical\_frame}}\mathbf{T}_{\text{target\_object}}$` using `buffer.lookup_transform`.
* Telemetry: Publishes `tracker_interfaces/msg/TrackingError` at 10 Hz.
* Service: `/tracker/set_mode` (`SetTrackingMode`).
* Action Server: `/tracker/center_target` (`CenterTarget`). Controls $\psi$ and $\theta$ to drive error within `angular_tolerance_deg`.

---

### Acceptance Verification

```bash
# Terminal 1: Broadcaster
ros2 run pantilt_tracker pan_tilt_broadcaster
```
```bash
# Terminal 2: Target Simulator
ros2 run pantilt_tracker target_simulator
```
```bash
# Terminal 3: Tracker Controller
ros2 run pantilt_tracker track_controller_node
```
```bash
# Terminal 4: Trigger Slew Alignment Action
ros2 action send_goal /tracker/center_target tracker_interfaces/action/CenterTarget "{angular_tolerance_deg: 0.5, timeout_seconds: 5.0}" --feedback
```
```text
[INFO] [track_controller]: Received goal to center target within 0.50 deg.
[INFO] [track_controller]: [Feedback] Pan Error: 14.32 deg | Tilt Error: -8.12 deg
[INFO] [track_controller]: [Feedback] Pan Error: 7.15 deg  | Tilt Error: -3.80 deg
[INFO] [track_controller]: [Feedback] Pan Error: 1.20 deg  | Tilt Error: -0.65 deg
[INFO] [track_controller]: [Feedback] Pan Error: 0.28 deg  | Tilt Error: 0.14 deg
[INFO] [track_controller]: Goal Succeeded. Target aligned within tolerance.
```

---

# Project 5: Eye-in-Hand Tool Re-Calibration & Visual Servoing
**Status:** `OPEN` | **Level:** `Intermediate / Sensor Calibration & Dynamic Kinematic Trees`  
**Packages:** `manipulator_interfaces` (`ament_cmake`), `manipulator_servoing` (`ament_python`)

### System Architecture
A 6-axis articulated arm carries a wrist-mounted inspection camera ("eye-in-hand") and an interchangeable end-effector tool (e.g., dispensing nozzle). The camera detects fiducial markers on workpieces in camera-centric 3D coordinates.

However, the tool's physical offset from the mounting flange shifts when tools are changed or bumped. The system must dynamically re-calibrate the tool frame (`flange` $\rightarrow$ `tool0`) using an asynchronous Service, maintain the entire spatial tree, and guide the arm via closed-loop velocity commands such that **the tip of the active tool** aligns with the workpiece marker.

```
       [world]
          │
      [base_link]
          │
         ... (Kinematic joints)
          │
       [flange]
       ├─── (Static/Calibrated) ───> [camera_color_optical_frame]
       │
       └─── (Dynamic via Service) ─> [tool0]  <── (Servo Goal: Align tool0 to target)
                                       │
                                       ▼
                             [workpiece_fiducial]
```

### Mathematical Foundations
1. **Kinematic Loop Closure for Tool Control:**
   To align `tool0` with `workpiece_fiducial`, the error vector must be resolved directly in the tool's coordinate system:
   $${}^{\text{tool0}}\mathbf{T}_{\text{fiducial}} = \left({}^{\text{flange}}\mathbf{T}_{\text{tool0}}\right)^{-1} \cdot {}^{\text{flange}}\mathbf{T}_{\text{cam}} \cdot {}^{\text{cam}}\mathbf{T}_{\text{fiducial}}$$

2. **Quaternion Error Formulation (Orientation Alignment):**
   Given current tool orientation quaternion $\mathbf{q}_{\text{curr}}$ and target orientation $\mathbf{q}_{\text{des}}$, the orientation error quaternion $\mathbf{q}_{e} = [w_e, \mathbf{v}_e]^T$ is:
   $$\mathbf{q}_{e} = \mathbf{q}_{\text{des}} \otimes \mathbf{q}_{\text{curr}}^{-1}$$
   The spatial angular error vector $\mathbf{e}_O$ used for proportional control:
   $$\mathbf{e}_O = 2 \cdot \text{sgn}(w_e) \cdot \mathbf{v}_e$$

3. **Cartesian Decoupled Velocity Proportional Controller:**
   $$\mathbf{v}_{\text{cmd}} = K_p \cdot {}^{\text{tool0}}\mathbf{p}_{\text{fiducial}}$$
   $$\boldsymbol{\omega}_{\text{cmd}} = K_o \cdot \mathbf{e}_O$$
   Velocity magnitudes must be clamped against dynamic safety limits:
   $$\mathbf{v}_{\text{clamped}} = \mathbf{v}_{\text{cmd}} \cdot \min\left(1.0, \frac{v_{\text{max}}}{\|\mathbf{v}_{\text{cmd}}\|}\right)$$

---

### Architectural Specifications

* **Package Structure:**
  ```text
  ros2_ws/src/
  ├── manipulator_interfaces/
  │   ├── CMakeLists.txt
  │   ├── package.xml
  │   ├── msg/
  │   │   └── ToolPoseDiagnostics.msg
  │   ├── srv/
  │   │   └── UpdateToolOffset.srv
  │   └── action/
  │       └── AlignToolToMarker.action
  └── manipulator_servoing/
      ├── config/servoing_params.yaml
      └── manipulator_servoing/
          ├── arm_kinematics_mock.py
          ├── vision_detector_mock.py
          └── servo_controller_node.py
  ```

* **Interface Contracts:**
  - **`srv/UpdateToolOffset.srv`:**
    ```text
    float64 offset_x
    float64 offset_y
    float64 offset_z
    float64 roll
    float64 pitch
    float64 yaw
    ---
    bool success
    string message
    ```
  - **`action/AlignToolToMarker.action`:**
    ```text
    string marker_frame_id     # e.g., "marker_42"
    float64 linear_tolerance   # in meters (e.g., 0.002)
    float64 angular_tolerance  # in radians (e.g., 0.02)
    ---
    bool aligned
    float64 final_linear_error
    float64 final_angular_error
    ---
    float64 current_linear_error
    float64 current_angular_error
    ```

---

### Node Engineering Requirements

#### 1. `arm_kinematics_mock.py`
* Broadcasts dynamic TF: `world` $\rightarrow$ `base_link` $\rightarrow$ `flange` at 50 Hz.
* Listens to Cartesian command twists `/arm/cartesian_velocity_cmd` (`geometry_msgs/msg/Twist`) and integrates forward flange motion.
* Dynamic parameter: `speed_limit` (clamped between $0.01$ and $0.5$ m/s via `add_on_set_parameters_callback`).

#### 2. `servo_controller_node.py`
* Contains `StaticTransformBroadcaster` publishing `flange` $\rightarrow$ `camera_color_optical_frame`.
* Manages dynamic broadcast of `flange` $\rightarrow$ `tool0`.
* Provides Service `/servoing/update_tool_offset` (`UpdateToolOffset`): dynamically updates the `flange` $\rightarrow$ `tool0` transform in memory and re-broadcasts it immediately without restarting nodes.
* Action Server: `/servoing/align_tool` (`AlignToolToMarker`).
  * Concurrency: Driven by `MultiThreadedExecutor` and `ReentrantCallbackGroup`.
  * Looks up `${}^{\text{tool0}}\mathbf{T}_{\text{marker}}$` continuously.
  * Emits Cartesian velocity commands until both linear and angular errors satisfy tolerance.
  * If the transform drops or timeout occurs, aborts and commands zero velocity.

---

### Acceptance Verification

#### Calibration Update via Service
```bash
ros2 service call /servoing/update_tool_offset manipulator_interfaces/srv/UpdateToolOffset \
  "{offset_x: 0.0, offset_y: 0.05, offset_z: 0.22, roll: 0.0, pitch: 0.0, yaw: 0.0}"
```
```text
requester: making request: manipulator_interfaces.srv.UpdateToolOffset_Request(offset_x=0.0, offset_y=0.05, offset_z=0.22, roll=0.0, pitch=0.0, yaw=0.0)
response:
manipulator_interfaces.srv.UpdateToolOffset_Response(success=True, message="TF flange -> tool0 updated successfully.")
```

#### Verification via TF Echo
```bash
ros2 run tf2_ros tf2_echo flange tool0
```
```text
At time 1711928341.200159
- Translation: [0.000, 0.050, 0.220]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
```

#### Action Servoing Execution
```bash
ros2 action send_goal /servoing/align_tool manipulator_interfaces/action/AlignToolToMarker \
  "{marker_frame_id: 'marker_42', linear_tolerance: 0.005, angular_tolerance: 0.05}" --feedback
```
```text
[INFO] [servo_controller]: Received alignment goal for marker_42.
[INFO] [servo_controller]: [Feedback] Lin Err: 0.145m | Ang Err: 0.35 rad
[INFO] [servo_controller]: [Feedback] Lin Err: 0.082m | Ang Err: 0.19 rad
[INFO] [servo_controller]: [Feedback] Lin Err: 0.012m | Ang Err: 0.04 rad
[INFO] [servo_controller]: [Feedback] Lin Err: 0.004m | Ang Err: 0.02 rad
[INFO] [servo_controller]: [Result] Tool aligned successfully.
```

---

# Project 6: Delayed Non-Inertial Interception & Historical TF Time-Travel
**Status:** `OPEN` | **Level:** `Advanced / Non-Inertial Kinematics & Temporal State Estimation`  
**Packages:** `interceptor_interfaces` (`ament_cmake`), `interceptor_core` (`ament_python`)

### System Architecture
In high-speed autonomous tracking (e.g., dual quadrotors or a mobile manipulator tracking a swinging crane load), visual perception pipelines introduce a non-negligible processing latency $\tau$ (e.g., 100ms – 250ms). Detections of the moving target arrive **time-stamped in the past**. 

Attempting to resolve target locations using the *latest* TF transform results in fatal geometric projection errors due to the observer's own high-speed motion during the processing window.

You must implement an Interceptor System capable of:
1. Handling target measurements with historical timestamps $t_{\text{stamp}} = t_{\text{current}} - \tau$.
2. Performing **historical time-travel lookups** in the TF2 buffer at $t_{\text{stamp}}$ to compute the historical target in the non-inertial tracking frame.
3. Propagating the target state forward to $t_{\text{current}}$ using spatial velocity integration.
4. Operating an Action Server that commands pursuit in a rotating, non-inertial reference frame subject to centrifugal and Coriolis kinematic effects.

```
Time Axis:
 t_stamp = (t_now - tau)                          t_now
    |                                                |
    ▼                                                ▼
 [Target detected in camera at t_stamp]        [Where is target NOW?]
    |                                                ▲
    └──── TF2 Historical Transform ────┐             │
          (base_link @ t_stamp)        │             │
                                       ▼             │
                        [Reconstruct Target @ t_stamp]
                                       │
                                       └── Numerical Extrapolation (Kinematics) ──┘
```

### Mathematical Foundations

1. **Temporal Spatial Transformations (Historical Time Lookup):**
   Let $t_s = t_{\text{curr}} - \tau$. The sensor returns target position ${}^{\text{sensor}}\mathbf{P}_{\text{target}}(t_s)$.
   To establish its position in the inertial world frame at that instant:
   $${}^{\text{world}}\mathbf{P}_{\text{target}}(t_s) = {}^{\text{world}}\mathbf{T}_{\text{sensor}}(t_s) \cdot {}^{\text{sensor}}\mathbf{P}_{\text{target}}(t_s)$$
   *Critical ROS 2 Concept:* You **cannot** use `tf2_ros.Time()`. You must invoke:
   `buffer.lookup_transform("world", sensor_frame, Time.from_msg(msg.header.stamp))`

2. **Kinematic Extrapolation to Current Epoch ($t_{\text{now}}$):**
   If the target has estimated velocity ${}^{\text{world}}\mathbf{v}_{\text{target}}$, its position at current time $t_{\text{now}}$ is:
   $${}^{\text{world}}\mathbf{P}_{\text{target}}(t_{\text{now}}) = {}^{\text{world}}\mathbf{P}_{\text{target}}(t_s) + \int_{t_s}^{t_{\text{now}}} {}^{\text{world}}\mathbf{v}_{\text{target}}(t) \, dt$$
   Assuming near-constant velocity over latency window $\tau$:
   $${}^{\text{world}}\mathbf{P}_{\text{target}}(t_{\text{now}}) \approx {}^{\text{world}}\mathbf{P}_{\text{target}}(t_s) + \tau \cdot {}^{\text{world}}\mathbf{v}_{\text{target}}$$

3. **Motion in a Non-Inertial Tracking Frame ($B$):**
   The interceptor tracks the target within a moving coordinate frame $B$ rotating with angular velocity $\boldsymbol{\omega}_{B}$ and linear acceleration $\mathbf{a}_B$. The apparent relative acceleration $\mathbf{a}_{\text{rel}}$ of the target seen by frame $B$ is:
   $$\mathbf{a}_{\text{rel}} = \mathbf{a}_{\text{inertial}} - \mathbf{a}_B - 2(\boldsymbol{\omega}_B \times \mathbf{v}_{\text{rel}}) - \boldsymbol{\omega}_B \times (\boldsymbol{\omega}_B \times \mathbf{r}) - \dot{\boldsymbol{\omega}}_B \times \mathbf{r}$$
   Where $2(\boldsymbol{\omega}_B \times \mathbf{v}_{\text{rel}})$ is the **Coriolis acceleration** and $\boldsymbol{\omega}_B \times (\boldsymbol{\omega}_B \times \mathbf{r})$ is the **Centrifugal acceleration**.

---

### Architectural Specifications

* **Package Structure:**
  ```text
  ros2_ws/src/
  ├── interceptor_interfaces/
  │   ├── CMakeLists.txt
  │   ├── package.xml
  │   ├── msg/
  │   │   ├── DelayedDetection.msg
  │   │   └── InterceptorState.msg
  │   ├── srv/
  │   │   └── TriggerEvasion.srv
  │   └── action/
  │       └── InterceptTarget.action
  └── interceptor_core/
      ├── config/interceptor_params.yaml
      └── interceptor_core/
          ├── target_evader_node.py
          ├── delayed_sensor_node.py
          └── interceptor_node.py
  ```

* **Interface Contracts:**
  - **`msg/DelayedDetection.msg`:**
    ```text
    std_msgs/Header header       # Stamped with historical time (now - tau)
    geometry_msgs/Point position # Position in sensor frame at stamp time
    geometry_msgs/Vector3 velocity_estimate # Estimated velocity at stamp time
    float64 latency_applied_sec
    ```
  - **`srv/TriggerEvasion.srv`:**
    ```text
    float64 evasion_acceleration_magnitude
    float64 duration_sec
    ---
    bool accepted
    string trajectory_type
    ```
  - **`action/InterceptTarget.action`:**
    ```text
    # Goal
    float64 capture_radius       # Distance to declare intercept (m)
    float64 max_pursuit_time_sec
    ---
    # Result
    bool captured
    float64 mission_time
    float64 min_distance_achieved
    string termination_status    # "CAPTURED", "TIMED_OUT", "ACCELERATION_LIMIT_EXCEEDED"
    ---
    # Feedback
    float64 current_relative_distance
    float64 time_to_impact_estimate
    float64 current_lateral_acceleration
    ```

---

### Node Engineering Requirements

#### 1. `target_evader_node.py`
* Moves a simulated high-speed target in 3D space (`world` $\rightarrow$ `target_frame`) at 50 Hz.
* Provides Service `/evader/trigger_evasion` (`TriggerEvasion`), which commands an instantaneous aggressive orthogonal acceleration step maneuver.

#### 2. `delayed_sensor_node.py`
* Monitors the true pose of `target_frame` relative to `sensor_optical_frame`.
* Injects an intentional, parameter-configurable transport and processing delay $\tau \in [0.10, 0.30]\text{ s}$.
* Buffers transforms and emits `interceptor_interfaces/msg/DelayedDetection` stamped with historical timestamp $t_{\text{stamp}} = t_{\text{curr}} - \tau$.

#### 3. `interceptor_node.py`
* Instantiates `tf2_ros.Buffer(cache_time=Duration(seconds=10.0))` with a high-capacity buffer.
* Maintains a dynamic non-inertial body frame `world` $\rightarrow$ `interceptor_base`.
* Subscribes to `/sensor/delayed_detection`:
  * Extracts $t_{\text{stamp}}$ from the incoming header.
  * Uses historical lookup: `lookup_transform("world", msg.header.frame_id, msg.header.stamp, timeout=Duration(seconds=0.05))`.
  * Computes the current extrapolated position of the target at $t_{\text{now}}$.
* Action Server: `/interceptor/intercept` (`InterceptTarget`):
  * Runs a 50 Hz control loop driving the interceptor's velocity toward the predicted current position.
  * Monitors commanded lateral acceleration: $\mathbf{a}_{\text{cmd}} = \frac{d\mathbf{v}}{dt}$.
  * Safety Violation Rule: If lateral acceleration exceeds the dynamic parameter `max_lateral_accel` ($5.0\text{ m/s}^2$), abort the action with `termination_status: "ACCELERATION_LIMIT_EXCEEDED"`.
  * If distance to extrapolated target $< \text{capture\_radius}$, succeed with `termination_status: "CAPTURED"`.

---

### Acceptance Verification

#### Terminal 1 — Delayed Sensor Emitter ($\tau = 200\text{ ms}$)
```bash
ros2 run interceptor_core delayed_sensor_node --ros-args -p simulated_latency:=0.20
```
```text
[INFO] [delayed_sensor]: Sensor pipeline online. Artificially injecting 0.200s latency.
[INFO] [delayed_sensor]: Published detection | Stamp: 1711929000.104 | Current Wall: 1711929000.304
```

#### Terminal 2 — Historical Interceptor Node
```bash
ros2 run interceptor_core interceptor_node
```
```text
[INFO] [interceptor]: Buffer initialized (10.0s window). Ready for historical transformations.
[INFO] [interceptor]: Received historical packet (delta: 0.200s). Querying TF buffer at stamp 1711929000.104...
[INFO] [interceptor]: Historical lookup SUCCESS. Extrapolated displacement over 0.200s: dx=+0.30m, dy=+0.12m
```

#### Terminal 3 — Mission Dispatch & Mid-Flight Evasion Stress Test
```bash
ros2 action send_goal /interceptor/intercept interceptor_interfaces/action/InterceptTarget \
  "{capture_radius: 0.15, max_pursuit_time_sec: 10.0}" --feedback
```
```text
[INFO] [client]: Dispatching interception goal...
[INFO] [client]: [Feedback] Rel Dist: 4.82m | Est. TTI: 2.41s | Lat Accel: 0.42 m/s²
[INFO] [client]: [Feedback] Rel Dist: 2.91m | Est. TTI: 1.45s | Lat Accel: 0.88 m/s²
```

*(While goal is active, trigger an aggressive evasion maneuver in another terminal)*
```bash
ros2 service call /evader/trigger_evasion interceptor_interfaces/srv/TriggerEvasion \
  "{evasion_acceleration_magnitude: 12.0, duration_sec: 1.5}"
```

```text
[WARN] [interceptor]: High-g maneuver detected in target frame!
[INFO] [client]: [Feedback] Rel Dist: 1.40m | Est. TTI: 0.70s | Lat Accel: 4.85 m/s²
[ERROR] [interceptor]: Commanded lateral acceleration (5.32 m/s²) breached threshold (5.00 m/s²)! Aborting for vehicle safety.
[WARN] [client]: [RESULT] Pursuit Aborted! min_distance_achieved: 1.22m | Reason: ACCELERATION_LIMIT_EXCEEDED
```

---

## Comparison Matrix of Challenges

| Dimension | Project 4 (Beginner) | Project 5 (Intermediate) | Project 6 (Advanced) |
| :--- | :--- | :--- | :--- |
| **Domain** | Stationary Pan-Tilt Tracker | Industrial Manipulator Arm | High-Speed Pursuit / Aerial Interception |
| **TF2 Complexity** | Static & Dynamic Broadcast, Simple Transform Resolution | Dynamic Kinematic Tree, Runtime Frame Recalibration | Non-Inertial Frames, Historical Time-Travel Lookups (`lookup_transform` @ `stamp`) |
| **Math Scope** | Spherical / Boresight Projection, 2D Vector Angles | Quaternion Error ($\mathbf{q}_e$), Transform Loop Inverses ($SE(3)$) | Historical Extrapolation, Rotating Reference Frames, Coriolis/Centrifugal Forces |
| **System Failure Mode** | Target out of pan/tilt reach | Marker occlusion / transform drop | Dynamics limits violation (g-force clamping), Buffer starvation |