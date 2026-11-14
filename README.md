# MOCAP4ROS2 OptiTrack driver

[![rolling](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack/actions/workflows/rolling.yaml/badge.svg?branch=rolling)](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack/actions/workflows/rolling.yaml) [![lyrical](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack/actions/workflows/lyrical.yaml/badge.svg?branch=rolling)](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack/actions/workflows/lyrical.yaml) [![kilted](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack/actions/workflows/kilted.yaml/badge.svg?branch=rolling)](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack/actions/workflows/kilted.yaml) [![jazzy](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack/actions/workflows/jazzy.yaml/badge.svg?branch=rolling)](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack/actions/workflows/jazzy.yaml) [![humble](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack/actions/workflows/humble.yaml/badge.svg?branch=rolling)](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack/actions/workflows/humble.yaml) [![natnet_sdk](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack/actions/workflows/natnet_sdk.yaml/badge.svg?branch=rolling)](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack/actions/workflows/natnet_sdk.yaml)

`mocap4r2_optitrack_driver` streams OptiTrack Motive into
[MOCAP4ROS2](https://mocap4ros2-project.github.io): markers, rigid bodies and skeletons.

It talks NatNet through the **official NatNet SDK**, when installed
(`ros2 run mocap4r2_optitrack_driver install_natnet_sdk.sh --accept-eula` and build from
source), or through an **open implementation** of NatNet 3.0–4.5 included in the package
(what the ROS binary packages use, on any architecture). Both produce the same messages.
It also includes `natnet_simulator`, a stand-in for Motive to test without the lab.

Documentation: https://mocap4ros2-project.github.io/latest/systems/optitrack.html

## Acknowledgements

MOCAP4ROS2 started as a Focused Technical Project of [ROSIN](https://www.rosin-project.eu/),
which received funding from the European Union’s Horizon 2020 research and innovation
programme under grant agreement no. 732287. Developed at the
[Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/), Rey Juan Carlos University.
