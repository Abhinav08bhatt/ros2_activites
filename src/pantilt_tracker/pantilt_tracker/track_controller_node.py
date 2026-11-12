""" track_controller_node [2 pub] [service ser] [action ser] [tf listener]

pub : /pan_tilt/joint_commands | Vector3 (intended to pan_tilt)
    - Vector3: 
        x : 
        y : 
        z :
pub : /tracker/error | TrackingError (for anyone to hear it)
    - TrackingError:
        pan_error_deg : 
        tilt_error_deg :
        distance_to_target : 

service ser : /tracker/set_mode | SetTrackingMode
    - SetTRackingMode:
        - Request:
            mode : IDLE / TRACKING
        - Response:
            success : 
            message :

action ser : /tracker/center_target | CenterTarget
    - CenterTarget:
        - Goal:
            tolerance_deg :
        - Result:
            final_pan_deg : 
            final_tilt_deg :
        - Feedback:
            current_tilt_error_deg :
            current_pan_error_deg : 

            
Flow of the file :

Here is a clear, step-by-step mental model and architecture flow of what
track_controller_node.py is supposed to do and how all the parts talk to each
other.

1. The Big Picture (What is this node?)

This node is the Brain / Feedback Controller.

  - Its Eyes: TF2 Listener (it constantly looks up where the target_object is
    relative to the camera_optical_frame).
  - Its Voice:
    1.  /tracker/error: "Hey everyone, here is how far off-center the object
        currently is."
    2.  /pan_tilt/joint_commands: "Hey pan_tilt_broadcaster, move your joints by
        this amount!"
  - Its Controls:
    1.  Service (SetTrackingMode): A master switch (e.g., enable/disable
        continuous following).
    2.  Action Server (CenterTarget): A goal-driven mission ("Turn until the
        target is within X degrees of the center, keep updating me on progress,
        and tell me when you're done").

2. The Two Publishers: Why two, and what do they do?

1.  Publisher A (/tracker/error - Type: TrackingError):

      - Purpose: Telemetry / Diagnostics.
      - It tells the outside world (like a dashboard, terminal, or plotter):
        "The object is currently 10° right, 5° down, and 2.1 meters away."
      - It does not move the robot; it just reports the state.

2.  Publisher B (/pan_tilt/joint_commands - Type: Vector3):

      - Purpose: Actuation / Control.
      - It sends desired joint positions (or joint updates) to your
        pan_tilt_broadcaster.py.
      - This is what actually moves the camera.

3. When and How TF2 is Listened To

TF2 is not a traditional callback topic; it is an internal lookup table that
updates automatically in the background.

  - You run a periodic timer loop (e.g., at 20 Hz or 30 Hz).
  - In every tick of that loop:
    1.  Query the TF Buffer: lookup_transform("camera_optical_frame",
        "target_object").
    2.  Extract (x, y, z).
    3.  Calculate the math:
          - Pan error = \text{atan2}(x, z)
          - Tilt error = \text{atan2}(-y, z)
          - Distance = \sqrt{x^2 + y^2 + z^2}
    4.  Publish these values to /tracker/error (Publisher A).

4. How the Service, Action, and Movement Interact

This is the core architectural question: Who commands the robot to move?

There are two common ways to handle this in ROS 2, but the cleanest and standard
design for this project is:

Role 1: The Service (/tracker/set_mode)

  - It's just a state switcher.
  - It takes a string ("IDLE" or "TRACKING") and sets an internal variable in
    the node: self.mode = "IDLE" or self.mode = "TRACKING".
  - If in "IDLE": the node only calculates and publishes /tracker/error, but
    never commands movements.
  - If in "TRACKING": the periodic timer loop itself could continuously send
    /pan_tilt/joint_commands to follow the target non-stop.

Role 2: The Action Server (/tracker/center_target)

  - The Action Server is designed to execute a specific task to completion:
    "Align until errors are under tolerance_deg."
  - When the Action goal is received:
    1.  The Action execution loop begins.
    2.  It looks at the latest pan/tilt errors (or does a fresh TF lookup).
    3.  Check condition: Are both errors \le \text{tolerance}?
          - If NO:
              - Publish feedback (Feedback: current_pan_error_deg,
                current_tilt_error_deg).
              - Calculate new pan/tilt target angles to cancel out the error.
              - Publish the new command to /pan_tilt/joint_commands (Publisher
                B).
              - Sleep slightly (e.g., 50ms) to allow TF to update.
              - Repeat the loop.
          - If YES:
              - Target is centered!
              - Fill out the Action Result (centered = True, final angles).
              - Set the goal state to succeeded and finish.

5. Step-by-Step Flowchart of the Node

[ INITIALIZATION ]
├── Setup TF2 Buffer & Listener
├── Setup Publisher 1: /tracker/error
├── Setup Publisher 2: /pan_tilt/joint_commands
├── Setup Service Server: /tracker/set_mode
├── Setup Action Server: /tracker/center_target
└── Start Background Timer Loop (20 Hz)

=======================================================

[ BACKGROUND TIMER LOOP (Runs non-stop) ]
├── Try TF Lookup: camera_optical_frame -> target_object
│     └── Calculate: pan_error, tilt_error, distance
└── Publish to /tracker/error

=======================================================

[ SERVICE CALLBACK (/tracker/set_mode) ]
└── Request arrives ("IDLE" or "TRACKING")
      └── Update internal state flag: self.mode = request.mode

=======================================================

[ ACTION EXECUTE CALLBACK (/tracker/center_target) ]
└── Goal received: tolerance_deg
      │
      └── WHILE (error > tolerance_deg):
            ├── Check latest errors (pan_error, tilt_error)
            ├── Publish Action Feedback (current errors)
            ├── Publish /pan_tilt/joint_commands to step toward the target
            └── Wait for next iteration
      │
      └── Target is within tolerance:
            └── Mark Goal as Succeeded & return Result

Summary of Dependencies:

  - TF Listener feeds the math for everything.
  - Publisher 1 (/tracker/error) is your public reporting stream.
  - Publisher 2 (/pan_tilt/joint_commands) is driven by the Action Server (to
    reach centering) or your timer (if continuous tracking mode is active).
  - The Service acts as a safety/mode latch.

"""