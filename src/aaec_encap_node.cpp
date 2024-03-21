#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 

#include <ros/ros.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <exposure_control/ExposureRegion.h>
#include <aaec.h>

using namespace cv;

double last_exposure_time;

int min_x;
int min_y;
int max_x;
int max_y;

double step_length_aec;
double exposure_upper_bound;
bool verbose_mode = false;
bool region_is_set = false;
bool publish_debug_images = false;
double momentum_pre = 0;
double default_exposure_time = 0.0;

image_transport::Publisher image_publisher;
ros::Publisher exposure_time_publisher;

aaec aaec_instance(0.0000002, 20000.0, true); 


void set_exposure_dynamic_reconfigure_param(double new_exposure_time) {
    std_msgs::Float64 exposure_message;
    exposure_message.data = new_exposure_time;
    exposure_time_publisher.publish(exposure_message);

    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::IntParameter int_param;
    dynamic_reconfigure::Config conf;

    int_param.name = "exposure_time";
    int_param.value = new_exposure_time;
    conf.ints.push_back(int_param);

    srv_req.config = conf;
    ros::service::call("/acquisition_node/set_parameters", srv_req, srv_resp);
}


void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //update_exposure(cv_ptr->image);
    double exposure_time_;
    ros::param::get("/acquisition_node/exposure_time", exposure_time_);
    double new_exposure_time = aaec_instance.computeNewExposureTime(cv_ptr->image, exposure_time_);
    set_exposure_dynamic_reconfigure_param(new_exposure_time);

}


void region_callback(const exposure_control::ExposureRegion& msg) {
    int min_x_ = msg.min_x;
    int max_x_ = msg.max_x;
    int min_y_ = msg.min_y;
    int max_y_ = msg.max_y;

    if (max_x_ <= min_x_ || max_y_ <= min_y_) {
        ROS_WARN("Invalid region specified. Ignoring");
        region_is_set = false;
        momentum_pre = 0.0;
        return;
    }

    float padding_x = (max_x_ - min_x_) * 0.1;
    float padding_y = (max_y_ - min_y_) * 0.1;

    min_x_ = min_x_ - padding_x;
    min_y_ = min_y_ - padding_y;
    max_x_ = max_x_ + padding_x;
    max_y_ = max_y_ + padding_y;

    min_x = std::max(0, min_x_);
    min_y = std::max(0, min_y_);

    max_x = std::min(static_cast<int>(msg.image_width), max_x_);
    max_y = std::min(static_cast<int>(msg.image_height), max_y_);

    region_is_set = true;
    if (verbose_mode) {
        ROS_INFO_STREAM("Set region of interest to [ (" << min_x << "," << min_y << "), (" << max_x << "," << max_y << ") ]");
    }

    aaec_instance.setRegion(min_x, min_y, max_x, max_y);

}





int main(int argc, char** argv) {
    ros::init(argc, argv, "aaec_exposure_control");
    ros::NodeHandle node_handle;
    ros::NodeHandle private_node_handle("~");
    
    ROS_INFO("Node for exposure control is up and running.");

    std::string image_topic;
    private_node_handle.getParam("step_length_aec", step_length_aec);
    private_node_handle.getParam("exposure_upper_bound", exposure_upper_bound);
    private_node_handle.getParam("default_exposure_time", default_exposure_time);
    private_node_handle.getParam("image_topic", image_topic);
    private_node_handle.getParam("verbose_mode", verbose_mode);
    private_node_handle.getParam("publish_debug_images", publish_debug_images);

    //setter for the params above
    aaec_instance.setStepLengthAEC(step_length_aec);
    aaec_instance.setExposureUpperBound(exposure_upper_bound);
    aaec_instance.setDefaultExposureTime(default_exposure_time);
    aaec_instance.setVerboseMode(verbose_mode);


    ROS_INFO_STREAM("Exposure control using max upper_bound for exposure time: " << exposure_upper_bound);
    ROS_INFO_STREAM("Step length: " << step_length_aec);
    ROS_INFO_STREAM("Using verbose mode: " << verbose_mode);

    image_transport::ImageTransport it(node_handle);
    image_publisher = it.advertise("image_with_box", 1);

    ros::Subscriber image_subscriber = node_handle.subscribe(image_topic, 1, image_callback);
    ros::Subscriber region_subscriber = node_handle.subscribe("target_region", 1, region_callback);
    exposure_time_publisher = node_handle.advertise<std_msgs::Float64>("exposure_time", 1);

    ros::spin();
    return 0;
}


