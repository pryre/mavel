# mavel
A position controller for multirotors

## Parameters

#### Program Control
- `control_rate` (50.0): Update rate that mavel will run at
- `control_frame` ("map"): Frame ID to use on the output messages

#### Flight Settings & Limits
- `throttle_min` (0.0): Minimum throttle output that will be output
- `throttle_mid` (0.0): Hover throttle estimate (set low for initial tuning/testing)
  - **Warning:** setting this value too high may make your UAV take-off very quickly or not be able to perform a steady hover.
  - To tune:
    - Figure out your required hover throttle (take a reading of throttle position from manual flight).
    - Start with a value 5% less than this reading (i.e. if your reading was 40%, enter `throttle_mid` as 35%).
    - If take-off is too slow (e.g. > 5 seconds), increase this value in small increments (e.g. 2% steps).
  - If you cannot take a throttle reading for some reason:
    - Start with a value of 10-20%
    - Aim to see the UAV take-off after ~10 seconds.
    - Increment in mid-sized steps (e.g. 5% steps) until a 10 second take-off is achieved.
    - Decrease increment steps to small increments (e.g. 2%) until a successful takeoff is achieved.
  - **Note:** Most UAVs should have `throttle_mid` set in the range of 30-50%. If your UAV is outside of this range, it is likely over- or under-powered.
- `throttle_max` (0.9): Maximum throttle output that will be output
- `tilt_max` (0.39): Maximum attitude rotation from vertical (in radians)
- `failsafe_land_vel` (-0.2): Emergency landing velocity for soft failsafe (must be negative)
- `failsafe_output_fatal` (false): Enables all-low output if hard failsafe occurs (default is to disable all outputs)
- `allow_timeout_position` (false): Allows fallback to the last position received if a timeout occurs on the position/trajectory reference (can be abused to allow once-off position goals to be sent)

#### Data Stream Rates
Data stream minimum rates define the period of time that mavel will allow before declaring an input timeout. Default vaules for each stream are shown below:
- `min_rate/state/mav_state` (0.5)
- `min_rate/state/odometry` (20.0)
- `min_rate/reference/trajectory` (20.0)
- `min_rate/reference/triplet` (20.0)
- `min_rate/reference/position` (5.0)
- `min_rate/reference/velocity` (20.0)
- `min_rate/reference/acceleration` (40.0)

#### Controller Dynamic Reconfigure
Setting the PID controller parameters is done using Dynamic Reconfigure. These params can be loaded in using a params file (see launch folder for example)

## Inputs

#### State References
- `~/reference/odom` (`nav_msgs/Odometry`): Current odometry state
- `~/reference/state` (`mavros_msgs/State`): Current UAV status (for armed/disarm flag)

#### Setpoint References
The following setpoint reference inputs are processed in reverse order. If mavel is presented with a higher-level input, it will switch over to that as the current reference instead. Specifically, if a pose is setpoint is being tracked, and a twist reference is provided, the twist will be used instead.

1. `~/reference/path` (`nav_msgs/Path`): Path input used for a trajectory reference
2. `~/reference/traj` (`nav_msgs/Odometry`): Odometry input used for trajectory reference (with orientation tracking)
2. `~/reference/triplet` (`mavros_msgs/PositionTarget`): Linear triplet input used for trajectory, position, or velocity reference (with orientation or yaw rate tracking )
2. `~/reference/pose` (`geometry_msgs/PoseStamped`): Pose used for position reference input (with orientation tracking)
3. `~/reference/twist` (`geometry_msgs/TwistStamped`): Twist used for velocity reference input (with yaw rate tracking)
4. `~/reference/accel` (`geometry_msgs/AccelStamped`): Acceleration used for acceleration reference input (with yaw acceleration tracking)

## Outputs

#### Attitude Command
- `~/attitude` (`mavros_msgs/AttitudeTarget`): The calculated attitude and throttle command

#### Control Feedback
- `~/feedback/pose` (`geometry_msgs/PoseStamped`): The currently tracked pose reference
- `~/feedback/twist` (`geometry_msgs/TwistStamped`): The currently tracked twist reference
- `~/feedback/accel_norm` (`geometry_msgs/AccelStamped`): The currently tracked normalized acceleration reference
