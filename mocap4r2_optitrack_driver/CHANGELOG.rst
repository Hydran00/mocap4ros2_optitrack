^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mocap4r2_optitrack_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2026-11-14)
------------------
* Rewritten on MocapSystemNode; the prebuilt libNatNet.so is removed.
* Two NatNet backends with the same output: the official NatNet SDK, when the package is built with it (install_natnet_sdk.sh), and an open implementation of NatNet 3.0-4.5, used by the binary packages on every architecture.
* Asset and marker names from the data descriptions, tracking validity and occlusion, Y-up to Z-up, local or global skeletons, latency compensation from the camera timestamps, reload of names when Motive changes the assets.
* Fixed use after free on shutdown, ignored connection errors, data races between NatNet and ROS threads and stale state across reconfigurations.
* natnet_simulator: a stand-in for Motive to test without the lab.
* Contributors: Francisco Martín Rico, Max Polzin, Aarsh Thakker, Christian Llanes, José Miguel Guerrero, Lorena Bajo Rebollo, pariaspe
