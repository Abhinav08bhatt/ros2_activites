# This repo is dedicated for general practice of ros2 as i learn

- ## Project 1: **Robot Power & Charging Manager** [STATUS : **COMPLETED**]
    **Goal:** A robot simulates battery drainage, allows manual battery resets via a Service, charges up to a target battery level via an Action, and exposes customizable configuration via Parameters—all monitored and commanded by a second node.
    - **Level:** beginner
    - **Topics included:** pub, sub, ser, cli, custom interface, parameter and actions 
    - **Packages included:** ['robot_py_app'](src/robot_py_app) ['robot_interface'](src/robot_interface)

    ---

    ### Rules & Architecture

    1. **Only 2 Packages allowed**:
    * `robot_interfaces` (`ament_cmake`): Contains custom `.msg`, `.srv`, and `.action`.
    * `robot_py_app` (`ament_python`): Contains two Python nodes:
        * `robot_node.py`:
            * **Parameters:** `robot_name` (string), `drain_rate` (int)
            * **Publisher:** Publishes custom battery status
            * **Service Server:** Resets battery level
            * **Action Server:** Performs a simulated charge cycle
        * `monitor_node.py`:
            * **Subscriber:** Listens to battery status
            * **Service Client:** Requests a battery reset
            * **Action Client:** Sends a goal to charge the robot and tracks feedback

    ---

    #### `robot_node.py`
    1. **Parameters**:
        * Declares `robot_name` (default: `"Robby"`).
        * Declares `drain_rate` (default: `1`).
    2. **Publisher**:
        * Runs a 1.0-second timer.
        * If not charging, decrements internal `battery_level` (starting at 100) by `drain_rate`.
        * Publishes `BatteryStatus` to `/robot/battery_status`.
    3. **Service Server**:
        * Advertises `/robot/reset_battery`.
        * When requested, sets `battery_level = request.new_level`.
    4. **Action Server**:
        * Advertises `/robot/charge_battery`.
        * When a goal is accepted: marks `is_charging = True`, increments battery by `5%` every second until `target_level` is reached, publishing feedback each second. Once done, returns the final result and sets `is_charging = False`.

    #### `monitor_node.py`
    1. **Parameters**:
        * Declares `charger_available` (default: `1`).
    2. **Subscriber**:
        * Subscribes to `/robot/battery_status` and logs warnings if the battery drops below 90%.
    3. **Service Client**:
        * Calls `/robot/reset_battery` (e.g., reset to `50%`) 3 seconds after startup.
    4. **Action Client**:
        * After the reset is confirmed, immediately sends a goal to `/robot/charge_battery` with `target_level: 65`.
        * Prints feedback as the battery climbs up to 65%.

    ---

    ### Expected Output

    #### Terminal 1: Build & Launch `robot_node`
    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ros2 run robot_py_app robot_node --ros-args -p robot_name:="Apollo" -p drain_rate:=2
    ```
    **Expected Output (Terminal 1):**
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

    ---

    #### Terminal 2: Launch `monitor_node`
    ```bash
    cd ~/ros2_ws
    source install/setup.bash
    ros2 run robot_py_app monitor_node
    ```
    **Expected Output (Terminal 2):**
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

    #### Terminal 3: Verification Commands
    ```bash
    source install/setup.bash

    # Check Custom Interface types
    ros2 interface show robot_interfaces/msg/BatteryStatus
    ros2 interface show robot_interfaces/srv/ResetBattery
    ros2 interface show robot_interfaces/action/ChargeBattery

    # Check Parameters dynamically
    ros2 param get /robot_node robot_name
    # Output: String value is: Apollo

    # Check active topics/services/actions
    ros2 topic list
    ros2 service list
    ros2 action list
    ```

---

- ## Project 2: **AGV Waypoint Dispatcher & Energy Monitor**
    - **Level:** intermediate
    - **Topics included:** pub, sub, service, custom interface, parameter, actions and launch files 
    - **Packages included:** [not specified]

    ### Scenario Brief
        You are building an Automated Guided Vehicle (AGV) system for an indoor warehouse. 
        * The **AGV** spawns at `(0.0, 0.0)` in 2D space.
        * The **Fleet Dispatcher** wants to send the AGV to target waypoints `(x, y)` to deliver payloads.
        * However, before sending a goal, the Dispatcher must verify via a **Service** whether the target waypoint is inside an active **Hazard Zone (Geofence)**.
        * If safe, the Dispatcher requests navigation through an **Action Server**, which moves the robot step-by-step toward the target while tracking 2D distance and calculating battery/energy drain.
        * The AGV broadcasts its live 2D telemetry continuously via a **Custom Message**.

    ---

    ## Architecture Rules

    * **Allowed Packages:** Strictly **2**
        1. `agv_interfaces` (`ament_cmake`): For all `.msg`, `.srv`, `.action` files.
        2. `agv_controller` (`ament_python`): For nodes, config, and launch files.

    ---

    ## Pen & Paper Math You Will Need

    > Before writing code, calculate these with a pen and notebook:

    1. **Euclidean Distance & Remaining Distance:**
        $$\Delta x = x_{\text{target}} - x_{\text{current}}, \quad \Delta y = y_{\text{target}} - y_{\text{current}}$$
        $$D = \sqrt{\Delta x^2 + \Delta y^2}$$

    2. **Direction Vector (Normalized Step):**
        If speed is $v$ meters/sec and update interval is $\Delta t$ seconds, step distance is $S = v \times \Delta t$.
        If remaining distance $D > S$:
        $$x_{\text{next}} = x_{\text{current}} + S \times \frac{\Delta x}{D}$$
        $$y_{\text{next}} = y_{\text{current}} + S \times \frac{\Delta y}{D}$$
        *(If $D \le S$, snap the robot directly to $x_{\text{target}}, y_{\text{target}}$).*

    3. **Hazard Zone Circle Test (Geofence Service):**
        Given a circular hazard zone centered at $(X_h, Y_h)$ with radius $R_h$:
        $$\text{Inside Hazard if: } (x_{\text{target}} - X_h)^2 + (y_{\text{target}} - Y_h)^2 \le R_h^2$$

    4. **Dynamic Energy Consumption Formula:**
        For every step traveled:
        $$\text{Energy Consumed} = S \times (\text{base\_drain} + 0.05 \times \text{payload\_weight})$$

    ---

    ## File Structure

        ```text
        ros2_ws/
        └── src/
            ├── agv_interfaces/
            │   ├── CMakeLists.txt
            │   ├── package.xml
            │   ├── msg/
            │   │   └── AgvTelemetry.msg
            │   ├── srv/
            │   │   └── ValidateWaypoint.srv
            │   └── action/
            │       └── NavigateToWaypoint.action
            │
            └── agv_controller/
                ├── package.xml
                ├── setup.py
                ├── setup.cfg
                ├── config/
                │   └── agv_params.yaml
                ├── launch/
                │   └── agv_mission.launch.py
                └── agv_controller/
                    ├── __init__.py
                    ├── agv_node.py
                    └── dispatcher_node.py
        ```
    ---

    ### Custom Interfaces Specification

    ### 1. `msg/AgvTelemetry.msg`
        ```text
        float64 current_x
        float64 current_y
        float64 current_battery
        string status          # "IDLE", "NAVIGATING", "HAZARD_STOP"
        ```

    ### 2. `srv/ValidateWaypoint.srv`
        ```text
        # Request: proposed coordinates
        float64 target_x
        float64 target_y
        ---
        # Response: is it allowed?
        bool is_valid
        float64 estimated_distance
        string reason
        ```

    ### 3. `action/NavigateToWaypoint.action`
        ```text
        # Goal
        float64 target_x
        float64 target_y
        float64 payload_weight   # in kg
        ---
        # Result
        bool reached
        float64 total_distance_traveled
        float64 remaining_battery
        ---
        # Feedback
        float64 distance_to_goal
        float64 current_battery
        ```
    ---

    ###  Node Specifications

    ### Node 1: `agv_node.py`

    * **Parameters:**
        * `linear_speed` (double, default: `1.0` m/s)
        * `hazard_center_x` (double, default: `5.0`)
        * `hazard_center_y` (double, default: `5.0`)
        * `hazard_radius` (double, default: `2.0`)
        * `base_energy_rate` (double, default: `0.5` % per meter)

    * **Publisher:**
        * Topic: `/agv/telemetry` (Type: `AgvTelemetry`)
        * Frequency: 2 Hz (every 0.5s)

    * **Service Server:**
        * Service: `/agv/validate_waypoint` (Type: `ValidateWaypoint`)
        * Calculates Euclidean distance from current position to target.
        * Checks if target falls inside the circular hazard zone. If inside, `is_valid = False` with `reason = "Inside Hazard Zone"`.

    * **Action Server:**
        * Action: `/agv/navigate_to_waypoint` (Type: `NavigateToWaypoint`)
        * Moves the robot toward the target in a loop at 1 Hz increments.
        * Updates battery using the energy formula.
        * Publishes action feedback each second.
        * Completes goal and returns the result once target is reached within 0.1m tolerance.

    ---

    ### Node 2: `dispatcher_node.py`

    * **Parameters:**
        * `goal_x` (double, default: `6.0`)
        * `goal_y` (double, default: `6.0`)
        * `cargo_weight` (double, default: `10.0` kg)

    * **Subscriber:**
        * Topic: `/agv/telemetry`
        * Logs position. If `current_battery < 20.0`, logs a severe warning.

    * **Sequential Execution Flow:**
        1. Spawns and logs current position from telemetry.
        2. Calls `/agv/validate_waypoint` with `(goal_x, goal_y)`.
        3. If validation fails: Logs error and aborts mission.
        4. If validation succeeds: Sends goal to Action Server `/agv/navigate_to_waypoint`.
        5. Prints Action feedback until the destination is reached.

    ---

    ## 🚀 Launch File Requirement: `agv_mission.launch.py`

    Your launch file must:
        1. Accept a Launch Argument `mission_cargo_weight` (default: `15.0`).
        2. Load parameters from `config/agv_params.yaml` into `agv_node`.
        3. Launch both `agv_node` and `dispatcher_node` together.
        4. Pass the launch argument into `dispatcher_node` as its parameter.

    ---

    ## ⚠️ The "Tears" Factor: What will bite you if you're not careful

    1. **Deadlock Trap:**
        The Action Server runs a loop while updating state. If you try to run an Action Server and a Timer or Service in the default Single-Threaded Executor, **callbacks can starve each other and freeze the node**. 
        * *Hint:* You will need to think about **`MultiThreadedExecutor`** and **`ReentrantCallbackGroup`** for the first time.
    2. **Division by Zero:**
        When calculating the unit direction vector $(\Delta x / D, \Delta y / D)$, make sure $D \ne 0$ once the robot reaches the target.
    3. **Action Rejection Handling:**
        If a client disconnects or if a target is out of range, the action server must reject or abort cleanly without crashing the node.

    ---

    ### Case 1: Manual Run (Two Terminals — Happy Path)

    In this scenario:
    * Target: `(3.0, 4.0)`
    * Cargo Weight: `10.0 kg`
    * Euclidean distance: $\sqrt{3^2 + 4^2} = 5.0\text{ m}$
    * Target distance to hazard center `(5.0, 5.0)` is $\sqrt{(3-5)^2 + (4-5)^2} = \sqrt{5} \approx 2.24\text{ m} > 2.0\text{ m}$ (**Safe / Valid!**)
    * Drain formula: $1.0\text{ m} \times (0.5 + 0.05 \times 10) = 1.0\%$ per second.

    ---

    #### Terminal 1: `agv_node`
    ```bash
    ros2 run agv_controller agv_node
    ```
    **Expected Output:**
    ```text
    [INFO] [agv_node]: AGV Online. Initial Pose: (0.00, 0.00) | Battery: 100.0%
    [INFO] [agv_node]: Hazard Zone Configured -> Center: (5.00, 5.00), Radius: 2.00m
    [INFO] [agv_node]: [Telemetry] Pos: (0.00, 0.00) | Batt: 100.0% | Status: IDLE
    [INFO] [agv_node]: [Telemetry] Pos: (0.00, 0.00) | Batt: 100.0% | Status: IDLE
    [INFO] [agv_node]: SERVICE: Received waypoint validation request -> (3.00, 4.00)
    [INFO] [agv_node]: SERVICE: Distance to hazard: 2.24m. Target is CLEAR. Validation: APPROVED.
    [INFO] [agv_node]: ACTION: Received navigation goal -> Target: (3.00, 4.00), Cargo: 10.0kg
    [INFO] [agv_node]: ACTION: Goal accepted. Beginning trajectory execution.
    [INFO] [agv_node]: Navigating... Pos: (0.60, 0.80) | Remaining: 4.00m | Batt: 99.0%
    [INFO] [agv_node]: [Telemetry] Pos: (0.60, 0.80) | Batt: 99.0% | Status: NAVIGATING
    [INFO] [agv_node]: Navigating... Pos: (1.20, 1.60) | Remaining: 3.00m | Batt: 98.0%
    [INFO] [agv_node]: Navigating... Pos: (1.80, 2.40) | Remaining: 2.00m | Batt: 97.0%
    [INFO] [agv_node]: Navigating... Pos: (2.40, 3.20) | Remaining: 1.00m | Batt: 96.0%
    [INFO] [agv_node]: Navigating... Pos: (3.00, 4.00) | Remaining: 0.00m | Batt: 95.0%
    [INFO] [agv_node]: ACTION: Destination reached within tolerance! Goal SUCCEEDED.
    [INFO] [agv_node]: [Telemetry] Pos: (3.00, 4.00) | Batt: 95.0% | Status: IDLE
    ```

    ---

    #### Terminal 2: `dispatcher_node`
    ```bash
    ros2 run agv_controller dispatcher_node --ros-args -p goal_x:=3.0 -p goal_y:=4.0 -p cargo_weight:=10.0
    ```
    **Expected Output:**
    ```text
    [INFO] [dispatcher_node]: Dispatcher initialized. Target: (3.00, 4.00) | Cargo: 10.0kg
    [INFO] [dispatcher_node]: Subscribed to telemetry. Current AGV at: (0.00, 0.00)
    [INFO] [dispatcher_node]: Validating waypoint (3.00, 4.00) with AGV Service...
    [INFO] [dispatcher_node]: Service Response -> Valid: True | Est. Distance: 5.00m | Reason: Clear
    [INFO] [dispatcher_node]: Waypoint approved. Dispatching Action Goal...
    [INFO] [dispatcher_node]: Action Goal Accepted by AGV. Monitoring progress...
    [INFO] [dispatcher_node]: [Feedback] Dist to Goal: 4.00m | Current Battery: 99.0%
    [INFO] [dispatcher_node]: [Feedback] Dist to Goal: 3.00m | Current Battery: 98.0%
    [INFO] [dispatcher_node]: [Feedback] Dist to Goal: 2.00m | Current Battery: 97.0%
    [INFO] [dispatcher_node]: [Feedback] Dist to Goal: 1.00m | Current Battery: 96.0%
    [INFO] [dispatcher_node]: [Feedback] Dist to Goal: 0.00m | Current Battery: 95.0%
    [INFO] [dispatcher_node]: [RESULT] Mission Complete!
    [INFO] [dispatcher_node]: Total Traveled: 5.00m | Final Battery: 95.0% | Success: True
    ```

    ---

    ### Case 2: Hazard Detection / Geofence Trigger (Rejection Path)

    If the Dispatcher requests target `(5.5, 5.0)` (which is inside the radius `2.0m` of center `(5.0, 5.0)`):

    #### Terminal 2 (`dispatcher_node`):
    ```bash
    ros2 run agv_controller dispatcher_node --ros-args -p goal_x:=5.5 -p goal_y:=5.0
    ```
    **Expected Output:**
    ```text
    [INFO] [dispatcher_node]: Dispatcher initialized. Target: (5.50, 5.00) | Cargo: 10.0kg
    [INFO] [dispatcher_node]: Validating waypoint (5.50, 5.00) with AGV Service...
    [ERROR] [dispatcher_node]: Service Response -> Valid: False | Reason: Inside Hazard Zone (Dist to hazard: 0.50m <= 2.00m)
    [WARN] [dispatcher_node]: MISSION ABORTED: Proposed destination is unsafe. Action will NOT be dispatched.
    ```

    ---

    ### Case 3: Running via the Unified Launch File

    When you run everything from a single command:
    ```bash
    ros2 launch agv_controller agv_mission.launch.py mission_cargo_weight:=20.0
    ```
    **Expected Combined Output:**
    ```text
    [INFO] [launch]: All log files can be found below /home/user/.ros/log/...
    [INFO] [launch]: Default logging verbosity is set to INFO
    [INFO] [agv_node-1]: process started with pid [12345]
    [INFO] [dispatcher_node-2]: process started with pid [12346]
    [INFO] [agv_node-1]: AGV Online. Initial Pose: (0.00, 0.00) | Battery: 100.0%
    [INFO] [dispatcher_node-2]: Dispatcher initialized with Cargo Weight: 20.0kg
    [INFO] [dispatcher_node-2]: Validating waypoint (6.00, 6.00)...
    [INFO] [agv_node-1]: SERVICE: Validating (6.00, 6.00)...
    [INFO] [dispatcher_node-2]: Action Goal Accepted.
    [INFO] [dispatcher_node-2]: [Feedback] Dist to Goal: 7.48m | Current Battery: 98.5%
    ...
    [INFO] [dispatcher_node-2]: [RESULT] Mission Complete!
    ```

    ---

    ### Testing Tips While Developing:
    * To test the Service independently using CLI:
    ```bash
    ros2 service call /agv/validate_waypoint agv_interfaces/srv/ValidateWaypoint "{target_x: 5.5, target_y: 5.0}"
    ```
    * To check live telemetry updates:
    ```bash
    ros2 topic echo /agv/telemetry
    ```
    * To send an Action goal manually without the dispatcher node:
    ```bash
    ros2 action send_goal /agv/navigate_to_waypoint agv_interfaces/action/NavigateToWaypoint "{target_x: 3.0, target_y: 4.0, payload_weight: 10.0}" --feedback
    ```

---

# 🎯 Advanced Project: 2D Differential-Drive Kinematics & Path Tracker

### The Mission
Before working with TF2 transforms and Gazebo physics, you must understand how a real 2D mobile robot calculates its own position and turns toward a goal. 

You will build a **Differential-Drive Kinematic Simulator & Tracker**:
* The robot starts at $(x=0.0,\; y=0.0,\; \theta=0.0\text{ rad})$.
* An **Action Client** sends a target waypoint: $(x_{\text{goal}},\; y_{\text{goal}})$.
* An **Action Server** runs a closed-loop controller: it calculates heading error, computes linear velocity $v$ and angular velocity $\omega$, and integrates forward kinematics over time $(\Delta t)$ to steer the robot to the goal.
* A **Subscriber** listens to dynamic speed-zone limits to cap $v_{\text{max}}$.
* A **Service Server** provides an immediate emergency brake that aborts the navigation action cleanly.
* A **Publisher** streams full $(x, y, \theta)$ odometry at 10 Hz.

---

## 📦 Package Rules

Strictly **2 packages**:
1. `amr_interfaces` (`ament_cmake`): Custom message, service, and action.
2. `amr_controller` (`ament_python`): Nodes, config YAML, and launch file.

---

## 📐 Pen & Paper Math (Work Through This Before Coding)

This is the exact mathematics used in real mobile robot local planners.

```
       (x_goal, y_goal)
            *
           /
          / 
         /   Distance rho
        /
       * (x, y)  Heading theta
      / 
     v (facing angle)
```

### 1. Position & Heading Error
At every control tick ($\Delta t = 0.1\text{ s}$):
$$\Delta x = x_{\text{goal}} - x, \quad \Delta y = y_{\text{goal}} - y$$
$$\rho = \sqrt{\Delta x^2 + \Delta y^2} \quad \text{(Euclidean distance to goal)}$$
$$\theta_{\text{goal}} = \text{atan2}(\Delta y, \Delta x) \quad \text{(Angle pointing directly at the goal)}$$

### 2. Angle Normalization (The Most Important Math in Robotics!)
Subtracting angles directly causes bugs (e.g., $0.1\text{ rad} - 6.2\text{ rad} = -6.1\text{ rad}$, which should just be $+0.18\text{ rad}$).
Always wrap angular error $\alpha$ into $[-\pi, +\pi]$ using:
$$\alpha = \text{atan2}(\sin(\theta_{\text{goal}} - \theta), \; \cos(\theta_{\text{goal}} - \theta))$$

### 3. Proportional Control Law $(v, \omega)$
* **Angular velocity:** Turn toward the target:
  $$\omega = K_{\alpha} \times \alpha$$
  Clamp $\omega$ so that $-\omega_{\text{max}} \le \omega \le \omega_{\text{max}}$.
* **Linear velocity:** Drive forward, but slow down if facing the wrong way:
  $$v = v_{\text{max}} \times \max(0.0, \; \cos(\alpha))$$
  *(If $\cos(\alpha) < 0$, the goal is behind the robot—rotate in place before driving forward!)*

### 4. Forward Kinematic Integration (Euler Step)
Update the robot's pose for the next time-step ($\Delta t$):
$$x_{k+1} = x_k + v \cos(\theta_k) \Delta t$$
$$y_{k+1} = y_k + v \sin(\theta_k) \Delta t$$
$$\theta_{k+1} = \text{atan2}(\sin(\theta_k + \omega \Delta t), \; \cos(\theta_k + \omega \Delta t))$$

---

### Worked Numerical Example (Do This on Paper)
* Current: $(x=0.0,\; y=0.0,\; \theta=0.0\text{ rad})$, Goal: $(x=2.0,\; y=2.0)$
* $\Delta x = 2.0, \; \Delta y = 2.0 \implies \rho = \sqrt{2^2 + 2^2} = 2.828\text{ m}$
* $\theta_{\text{goal}} = \text{atan2}(2.0, 2.0) = \frac{\pi}{4} \approx 0.785\text{ rad}$
* $\alpha = 0.785 - 0.0 = 0.785\text{ rad}$
* If $K_{\alpha} = 1.5, \; v_{\text{max}} = 0.5\text{ m/s}$:
  * $\omega = 1.5 \times 0.785 = 1.178\text{ rad/s}$
  * $v = 0.5 \times \cos(0.785) = 0.5 \times 0.707 = 0.354\text{ m/s}$
* After $\Delta t = 0.1\text{s}$:
  * $x_{\text{next}} = 0.0 + 0.354 \times \cos(0) \times 0.1 = 0.0354\text{ m}$
  * $y_{\text{next}} = 0.0 + 0.354 \times \sin(0) \times 0.1 = 0.0000\text{ m}$
  * $\theta_{\text{next}} = 0.0 + 1.178 \times 0.1 = 0.1178\text{ rad}$

---

## 🛠️ Workspace File Structure

```text
ros2_ws/
└── src/
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
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── config/
        │   └── amr_params.yaml
        ├── launch/
        │   └── amr_system.launch.py
        └── amr_controller/
            ├── __init__.py
            ├── amr_node.py
            └── supervisor_node.py
```

---

## 📜 Interface Definitions

#### 1. `msg/Pose2DStamped.msg`
```text
float64 x
float64 y
float64 theta
float64 linear_velocity
float64 angular_velocity
```

#### 2. `msg/SpeedLimit.msg`
```text
float64 max_linear_speed   # meters per second
```

#### 3. `srv/EmergencyBrake.srv`
```text
bool engage_brake
---
bool success
string message
```

#### 4. `action/NavigateToPose.action`
```text
# Goal
float64 goal_x
float64 goal_y
float64 tolerance        # e.g., 0.1 meters
---
# Result
bool reached
float64 final_x
float64 final_y
float64 final_theta
string termination_reason # "GOAL_REACHED", "EMERGENCY_BRAKE", "CANCELED"
---
# Feedback
float64 current_x
float64 current_y
float64 current_theta
float64 distance_remaining
```

---

## 🧩 Node Specifications

### Node 1: `amr_node.py` (Kinematics & Action Server)

* **Internal State:**
  * $x = 0.0$, $y = 0.0$, $\theta = 0.0$, $v = 0.0$, $\omega = 0.0$.
  * `brake_engaged = False`.
* **Parameters & Validation:**
  * `default_linear_speed` (default: `0.6` m/s, valid range: `[0.1, 1.5]`)
  * `max_angular_speed` (default: `1.2` rad/s, valid range: `[0.2, 3.14]`)
  * `kp_alpha` (default: `1.5`, valid range: `[0.5, 5.0]`)
  * **Dynamic Validation:** Implement `add_on_set_parameters_callback`. Reject invalid parameter updates with informative messages.
* **Publisher:**
  * Topic: `/amr/odometry` (`Pose2DStamped`) at **10 Hz** (timer-driven).
* **Subscriber:**
  * Topic: `/amr/speed_limit` (`SpeedLimit`). Updates current speed cap $v_{\text{max}}$.
* **Service Server:**
  * Service: `/amr/emergency_brake` (`EmergencyBrake`).
  * If `engage_brake == True`: sets `self.brake_engaged = True`, drops $v=0, \omega=0$.
* **Action Server:**
  * Action: `/amr/navigate_to_pose` (`NavigateToPose`).
  * Loops at $\Delta t = 0.1\text{ s}$ (10 Hz):
    1. Check `if goal_handle.is_cancel_requested`: set result `"CANCELED"`, call `goal_handle.canceled()`, return.
    2. Check `if self.brake_engaged`: set result `"EMERGENCY_BRAKE"`, call `goal_handle.abort()`, return.
    3. Calculate $\rho, \theta_{\text{goal}}, \alpha, \omega, v$.
    4. Update $(x, y, \theta)$ via forward kinematics equations.
    5. Publish Action Feedback.
    6. If $\rho \le \text{goal.tolerance}$: set $v=0, \omega=0$, call `goal_handle.succeed()`, return.

---

### Node 2: `supervisor_node.py` (Command & Mission Control)

* **Subscriber:**
  * Listens to `/amr/odometry` and logs coordinates every 1 second.
* **Publisher:**
  * Publishes `/amr/speed_limit` (allows simulating slow zones).
* **Action Client:**
  * Sends target goal: e.g. `(goal_x=2.0, goal_y=2.0, tolerance=0.1)`.
  * Logs feedback (distance remaining).
* **Parameter `mission_mode`:**
  * `"NORMAL"`: Lets robot navigate to destination.
  * `"TRIGGER_BRAKE"`: When distance remaining drops below $1.5\text{ m}$, asynchronously calls the `/amr/emergency_brake` Service.

---

## ⚡ The Concurrency Requirement (Why Multi-threading Matters Here)

Your `amr_node` must run a 10 Hz kinematic loop inside the Action Server, **while simultaneously**:
1. Handling the 10 Hz odometry publisher timer.
2. Handling incoming `/amr/speed_limit` messages.
3. Responding to the `/amr/emergency_brake` Service call.

If you use the default Single-Threaded Executor, calling the emergency service or publishing odometry while inside an action loop will block or deadlock.
* **Must use:** `MultiThreadedExecutor` and assign all callbacks to a `ReentrantCallbackGroup()`.

---

## 🚀 Launch File: `amr_system.launch.py`

* Declares launch argument: `mode` (default: `'NORMAL'`).
* Loads `config/amr_params.yaml` into `amr_node`.
* Passes `mission_mode = LaunchConfiguration('mode')` to `supervisor_node`.
* Launches both nodes together.

---

## 🖥️ Expected Outputs

### Scenario A: NORMAL Navigation (Reaching Goal)
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

---

### Scenario B: Dynamic Emergency Brake Trigger
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

---

### Scenario C: Dynamic Parameter Validation Rejection (In another terminal)
```bash
ros2 param set /amr_node default_linear_speed 5.0
```
**Expected Terminal Output:**
```text
Setting parameter failed: Parameter 'default_linear_speed' value 5.0 exceeds allowed range [0.1, 1.5] m/s
```

---

Save this as `advanced_challenge.md`. It directly teaches you the mathematical foundations (`atan2`, heading, angular wrapping, local velocity commands) that TF2 and Gazebo build upon.