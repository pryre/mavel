# mavel
A position controller for multirotors

## Inputs

#### State References
- `~/reference/odom` (`nav_msgs/Odometry`): Current odometry state
- `~/reference/state` (`mavros_msgs/State`): Current UAV status (for armed/disarm flag)

#### Setpoint References
The following setpoint reference inputs are processed in reverse order. If mavel is presented with a higher-level input, it will switch over to that as the current reference instead. Specifically, if a pose is setpoint is being tracked, and a twist reference is provided, the twist will be used instead.

1. `~/reference/path` (`nav_msgs/Path`): Path input used for a trajectory reference
2. `~/reference/traj` (`nav_msgs/Odometry`): Odometry input used for trajectory reference (with orientation tracking)
2. `~/reference/pose` (`geometry_msgs/PoseStamped`): Pose used for position reference input (with orientation tracking)
3. `~/reference/twist` (`geometry_msgs/TwistStamped`): Twist used for velocity reference input (with yaw velocity tracking)
4. `~/reference/accel` (`geometry_msgs/AccelStamped`): Acceleration used for acceleration reference input (with yaw acceleration tracking)

## Outputs

#### Attitude Command
- `~/attitude` (`mavros_msgs/AttitudeTarget`): The calculated attitude and throttle command

#### Control Feedback
- `~/feedback/pose` (`geometry_msgs/PoseStamped`): The currently tracked pose reference
- `~/feedback/twist` (`geometry_msgs/TwistStamped`): The currently tracked twist reference
- `~/feedback/accel` (`geometry_msgs/AccelStamped`): The currently tracked acceleration reference

## Controller Dynamic Reconfigure
Setting the PID controller parameters is done using Dynamic Reconfigure. These params can be loaded in using a params file (see launch folder for example)
