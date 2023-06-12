This package provides 3 exposure control methods: Active Exposure Control(AEC), Gamma-based Exposure Control(GEC), and default. To use the one you want, simpy set the corresponding bool value to true, and the rest to false in the  "changeParam.launch" under launch folder.


TO adjust other parameters like step length and upper bound: change the "changeParam.launch" in the launch folder.



please copy the launch file stag_and_fisheye_with_exposure_control.launch
to the "launch" folder under spinnaker_sdk_camera_driver. The line starting node rosbag recording is also in this launch file.

To run the project:

roslaunch stag_ros stag_and_fisheye_with_exposure_control.launch
