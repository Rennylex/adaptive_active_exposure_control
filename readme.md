

# Active Adaptive Exposure Control package
Active Adaptive Exposure Control (AAEC) aims to provide robust, constant, and accurate fiducial marker detection in the field for monocular camera.

## 1. Overview
  ### Task
  For robots operating in the field, visual input is a significant source for positioning and locating. One of the main stream solution is to use fiducial marker to provide a fixed anchor point for the robots as reference. However, the harsh lighting condition (e.g.  underwater caustics, advarsarial lighting, etc), will sometimes make it difficult for robot to get a precise reading of the markers location. In order to overcome this challenge, AAEC is proposed.
  
  ### Features
  AAEC uses 2 features to ensure its robustness and efficiency:
  1. Region of interest: the AAEC algorithm will automatically lock on a specific image region of the fiducial marker (in this case, STAG), which will siginificantly reduce the computational cost.
  2. Momentum-based gradient descending: This ensures the extraodinary exposure time convergence speed of AAEC

## 2. Tutorial
  ### aaec_exposure_control node:
  

  ### ChangeParams node:
  This package provides 3 exposure control methods: Active Exposure Control(AEC), Gamma-based Exposure Control(GEC), and default. To use the one you want, simpy set the corresponding bool value to true, and the rest to false in the  "changeParam.launch" under launch folder.


TO adjust other parameters like step length and upper bound: change the "changeParam.launch" in the launch folder.



please copy the launch file stag_and_fisheye_with_exposure_control.launch
to the "launch" folder under spinnaker_sdk_camera_driver. The line starting node rosbag recording is also in this launch file.

To run the project:

roslaunch stag_ros stag_and_fisheye_with_exposure_control.launch






