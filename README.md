# COMS Utilities

## `launch`

Launch files that can be run with `roslaunch path/to/launch/file`. Some launch
files read environment variables that allows you to tweak its behavior.

- `all.launch`: spins up all the nodes required for a basic autonomous
  driving. This includes: odometry, LIDAR, RTK-GPS, GPS Compass, velocity
  control.
  - `COMS_AUTONOMOUS_ENABLED` (default: `true`): set to false if you don't
    want to start up the steering and velocity control, as starting these will
    enable the actuators and lock the brake pedal and steering.
- `odom.launch`: starts up the nodes required to publish the `odom` topic.

## `scripts`

- `accel_vel.py`: records the accelerator percentage and velocity change. Make
  sure to start the `gear_a_b_mbed` and `/encoder` node before running this
  script.
- `check_dirty.bash`: checks whether the given directories have uncommited
  changes.
  - Usage: `check_dirty.bash ~/ros/src/*`
- `odom_to_speed.py`: receives the `odom` topic and shows the current speed at
  which the vehicle is moving.
- `pull_all.bash`: similar usage to `check_dirty.bash`, but runs `git pull
  origin` on all given repositories.
