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