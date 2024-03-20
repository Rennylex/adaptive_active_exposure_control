

# Active Adaptive Exposure Control package
Active Adaptive Exposure Control (AAEC) aims to provide robust, constant, and accurate fiducial marker detection in the field for monocular camera.

(Below: underwater robot trying to identify the fiducial markers. Caustics is a challenge to the traditonal exposure control method, under-exposure and over-exposure will both lead to unideal imaging)
![Alt text](/images/auv.gif)


![Alt text](/images/sight_underwater.gif)
\

[![Watch the video](/images/youtube-video-screenshot.png)](https://youtu.be/5XBcN29iltI)

## 1. Overview
  ### Task
  For robots operating in the field, visual input is a significant source for positioning and locating. One of the main stream solution is to use fiducial marker to provide a fixed anchor point for the robots as reference. However, the harsh lighting condition (e.g.  underwater caustics, advarsarial lighting, etc), will sometimes make it difficult for robot to get a precise reading of the markers location. In order to overcome this challenge, AAEC is proposed.



  
  ### Features
  AAEC uses 2 features to ensure its robustness and efficiency:
  1. Region of interest: the AAEC algorithm will automatically lock on a specific image region of the fiducial marker (in this case, STAG), which will siginificantly reduce the computational cost.
  2. Momentum-based gradient descending: This ensures the extraodinary exposure time convergence speed of AAEC

(below: fast converging thanks to the 2 features of AAEC)
![Alt text](/images/converging.gif)


## 2. Tutorial
  ### aaec_exposure_control node:
  The aaec_exposure_control_standalone.launch file, along with the aaec_exposure_control.cpp source code, is designed to use AAEC to automatically adjust the exposure time of a camera based on the image's gradient info. The code uses ROS (Robot Operating System) for communication and OpenCV for image processing.



  #### Launch file parameters explained
  **`run_camera_node`**: If set to true, this parameter will also start the camera node as part of the launch. Default value is false.  
  **`image_topic`**: The ROS topic on which raw images from the camera are published. Default is `/camera_array/cam1/image_raw`.  
  **`verbose_mode`**: Enables detailed logging if set to true. Useful for debugging. Default is false.  
  **`publish_debug_images`**: When enabled, publishes images with diagnostic information overlay. Default is true.  
  **`exposure_upper_bound`**: The maximum allowable exposure time in **microseconds**. Default is 20000.  
  **`default_exposure_time`**: The initial exposure time in **microseconds** before any adjustments. Default is 5000.  
  **`step_length_aec`**: Defines the step size for AAEC's exposure correction. Smaller values mean finer adjustments.  


  #### Source code key variables and functions
  **`image_transport::Publisher image_publisher`**: Publishes images with diagnostic overlays if publish_debug_images is enabled.  
  **`ros::Publisher exposure_time_publisher`**: Publishes the current exposure time for monitoring or use by other systems.  
  **`void set_exposure_dynamic_reconfigure_param(double new_exposure_time)`**: Adjusts the camera's exposure time dynamically.  
  **`Mat get_image_gradient(Mat img)`**: Computes the image gradient, an essential component in determining the need for exposure adjustment.  
  **`void region_callback(const exposure_control::ExposureRegion& msg)`**: Callback for setting a region of interest within the image for exposure analysis. This is useful when focusing on specific parts of the image.  
  **`void update_exposure(Mat image)`**: Main logic for adjusting the exposure based on the image content and set parameters. 
  **`void image_callback(const sensor_msgs::ImageConstPtr& msg)`**: Callback for processing incoming images from the specified ROS topic.  

  ### ChangeParams node:
  This node provides 3 exposure control methods: Active Exposure Control(AEC), Gamma-based Exposure Control(GEC), and default. 
  
  
  #### how to run
  To use the one you want, simpy set the corresponding bool value to true, and the rest to false in the  "changeParam.launch" under launch folder.
  To adjust other parameters like step length and upper bound: change the "changeParam.launch" in the launch folder.


  please copy the launch file stag_and_fisheye_with_exposure_control.launch
  to the "launch" folder under spinnaker_sdk_camera_driver. The line starting node rosbag recording is also in this launch file.

  To run the project:
  
  `roslaunch stag_ros stag_and_fisheye_with_exposure_control.launch`






