robot_nerf_launcher
====================

Purpose
- ROS 2 package for the my_steel robot that encapsulates the Nerf launcher control interfaces (aim, flywheel, fire).

Where to put what
- Node code: `src/robot_nerf_launcher/robot_nerf_launcher/` (Python package)
- Launch: `src/robot_nerf_launcher/launch/nerf_launcher.launch.py`
- URDF: The physical model is already available at `src/robot_description/urdf/robot_xl/components/nerf_launcher.urdf.xacro`. Include it by launching the robot with `include_nerf_launcher:=true`.

Build
- colcon build --packages-select robot_nerf_launcher

Run
- ros2 launch robot_nerf_launcher nerf_launcher.launch.py

Topics
- `nerf_launcher/cmd/aim` (geometry_msgs/Vector3): x=pan(rad), y=tilt(rad)
- `nerf_launcher/cmd/flywheel` (std_msgs/Float32): normalized [0..1]
- `nerf_launcher/cmd/fire` (std_msgs/Bool): True triggers a shot

Integration tips
- URDF: enable via `include_nerf_launcher:=true` for `robot_description/urdf/robot_xl.urdf.xacro`.
- ros2_control: Add servo and ESC joint interfaces in your ros2_control xacro and spawn appropriate controllers. Alternatively, bridge these topics to your microcontroller layer.
