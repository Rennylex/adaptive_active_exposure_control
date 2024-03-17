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

image_transport::Publisher image_publisher;
ros::Publisher exposure_time_publisher;

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


Mat get_image_gradient(Mat img) {
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_64F;
    Sobel(img, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
    
    abs_grad_x=abs( grad_x );
    Sobel(img, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
 
    abs_grad_y = abs(grad_y);
    Mat img_grad;
    img_grad = abs_grad_x.mul(abs_grad_x) + abs_grad_y.mul(abs_grad_y);

    img_grad = abs_grad_x;
    return img_grad;
}


Mat get_gradient_x(Mat img)
{
    Mat grad_x;
    Mat abs_grad_x;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_64F;
    Sobel(img, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );

    abs_grad_x = abs( grad_x );

    return grad_x;
}


Mat get_gradient_y(Mat img) {
    Mat grad_y;
    Mat abs_grad_y;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_64F;
    Sobel( img, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
 
    abs_grad_y=abs( grad_y );
    return grad_y;
}


void region_callback(const exposure_control::ExposureRegion& msg) {
    region_is_set = true;
    min_x = msg.min_x;
    max_x = msg.max_x;
    min_y = msg.min_y;
    max_y = msg.max_y;

    float padding_x = (max_x - min_x) * 0.1;
    float padding_y = (max_y - min_y) * 0.1;

    min_x=min_x - padding_x;
    min_y=min_y - padding_y;
    max_x=max_x + padding_x;
    max_y=max_y + padding_y;

    min_x = std::max(0, min_x);
    min_y = std::max(0, min_y);

    max_x = std::min(static_cast<int>(msg.image_height), max_x);
    max_y = std::min(static_cast<int>(msg.image_width), max_y);
}


void update_exposure(Mat image) {
    image.convertTo(image, CV_64FC1);

    double exposure_time_;
    ros::param::get("/acquisition_node/exposure_time", exposure_time_);

    if (verbose_mode) {
        ROS_INFO("exposure_time_ is %f", exposure_time_);
    }

    if(!region_is_set){
        set_exposure_dynamic_reconfigure_param(0);

        if (verbose_mode) {
            ROS_INFO("Region is not set. Using default exposure control.");
        }
    } else {
        image = image(
            Rect(
                min_x,
                min_y,
                max_x-min_x,
                max_y-min_y
            )
        );

        if (publish_debug_images) {
            Mat image_with_box = image.clone();
            cvtColor(image_with_box, image_with_box, CV_GRAY2RGB);
            rectangle(image_with_box, Point(min_x,min_y), Point(max_x,max_y), Scalar(255,0,0), 2, 8, 0);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image_with_box).toImageMsg();
            image_publisher.publish(msg);
        }

        double maxVal;
        double deltaT = exposure_time_/1000000;

        Mat gradient_x = get_gradient_x(image);
        Mat gradient_y = get_gradient_y(image);
        if(verbose_mode) {
            ROS_INFO("Gradient x and y calculated");
        }

        std::vector<double> g_prime {
            2.23648003e-28,
            -4.23843983e-25,
            3.61455838e-22,
            -1.83243015e-19,
            6.14568309e-17,
            -1.43597629e-14,
            2.39781444e-12,
            -2.88855261e-10,
            2.50378889e-08,
            -1.54014489e-06,
            6.54831886e-05,
            -1.84372346e-03,
            3.21098127e-02,
            -3.08446340e-01,
            1.30761122e+00
        };

        Mat gradient_result = get_image_gradient(image);

        if(verbose_mode) {
            ROS_INFO("Gradient calculated");
        }
        
        cv::Mat flat = gradient_result.reshape(1, gradient_result.rows*gradient_result.cols);
        std::vector<double> gradient_1d = flat;//gradient_result.isContinuous()? flat : flat.clone();

        std::vector<std::pair<double, std::pair<int, int>>> gradient_1d_with_location;

        for (int i=0;i<gradient_1d.size();i++)
        {
            gradient_1d_with_location.push_back(
                std::make_pair(
                    gradient_1d[i],
                    std::make_pair(
                        i / gradient_result.cols,
                        i % gradient_result.cols
                    )
                )
            );
        }

        //step 2: sort the gradient magnitude in ascending order
        std::sort(gradient_1d.begin(), gradient_1d.end());
        std::sort(gradient_1d_with_location.begin(), gradient_1d_with_location.end());
        //after sorting, I also want to get the location of the element in the original Matrix
        // //assume p=0.5

        // //step 3: Then calculate M_softperc. M_softperc=sum(W[i]*Gradient[i])
        //     //step 3.1: calculate the weight
        double p = 0.9;
        std::vector<double> W;

        int k = 5;
        int len = gradient_result.rows*gradient_result.cols;
        int scale_down = len * p;
        if(verbose_mode) {
            ROS_INFO("len is %d", len);
        }

        double wsum = 0;
        double lambda = 10;
        double sigma = 0.11;
        double N = log(lambda*(1-sigma)+1);

        for (int i = 0; i < len; i++) {
            if (i <= scale_down) {
                W.push_back(pow(sin((CV_PI * i) / (2 * scale_down)), k));
                wsum += W[i];
            } else {
                W.push_back(pow(sin(CV_PI / 2 - (CV_PI * (i-scale_down)) / (2 * (len - scale_down))), k));
                wsum += W[i];
            }
        }

        if(verbose_mode) {
            ROS_INFO("Weighting DONE");
        }

        for (int i = 0; i < len; i++) {
            W[i] = W[i] / wsum;
        }

        double M_softperc = 0;
        for (int i = 0; i < len; i++) {
            M_softperc += (W[i] * (gradient_1d[i]));
        }

        if(verbose_mode) {
            ROS_INFO("M_softperc is %f", M_softperc);
        }

        //size should be the same as image
        Mat g_prime_I = Mat(image.size(),CV_64FC1,Scalar(0,0,0));
        Mat g_accum = Mat(image.size(),CV_64FC1,Scalar(1,1,1));
        Mat image_cvt;
        image.convertTo(image_cvt, CV_64FC1);
            
        minMaxLoc(image_cvt, NULL, &maxVal);

        for (int i = 0; i < 15; i++) {
            g_prime_I += g_accum.mul(g_prime[14-i]);
            g_accum = g_accum.mul(image);
        }

        Mat temp=1/(g_prime_I.mul(deltaT));
        temp.convertTo(temp, CV_64FC1);
        Mat gradient_x_temp = get_gradient_x(temp);
        Mat gradient_y_temp = get_gradient_y(temp);
        Mat pGpdT = 2 * (gradient_x.mul(gradient_x_temp) + gradient_y.mul(gradient_y_temp));

        //FIND gradient_I max
        double minVal1;
        double maxVal1;
        minMaxLoc(pGpdT, &minVal1, &maxVal1);
        double pMpdT = 0;
        for (int i = 0; i < len; i++) {
            pMpdT = pMpdT + W[i] * pGpdT.at<double>(gradient_1d_with_location[i].second.first, gradient_1d_with_location[i].second.second);
        }
        
        if (verbose_mode) {
            ROS_INFO("pMpdT is %f", pMpdT);
        }
        // //step 5: update deltaT using the gradient descent method
        double alpha = step_length_aec;
        double nextdeltaT = deltaT;

        //MOMENTUM VERSION
        double momentum = 0.9*momentum_pre+alpha*pMpdT;
        if(verbose_mode) {
            ROS_INFO_STREAM("momentum is "<<momentum*1000000);
        }

        if(abs(momentum)*1000000>1){
            momentum_pre = momentum;
            nextdeltaT = deltaT + momentum;
        }
        //MOMENTUM VERSION ENDS
        if (verbose_mode) {
            ROS_INFO_STREAM("Attempted Exposure time: " << nextdeltaT * 1000000);
        }

        exposure_time_ = nextdeltaT * 1000000;
        if(exposure_time_ <= 12) {
            exposure_time_ = 13;
        }
        
        if(exposure_time_ > exposure_upper_bound) {
            exposure_time_ = exposure_upper_bound;
        }

        set_exposure_dynamic_reconfigure_param(exposure_time_);
    }
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

    update_exposure(cv_ptr->image);
}


int main(int argc, char** argv) {
    int result = 0;

    ros::init(argc, argv, "changeParam");
    ros::NodeHandle node_handle;
    
    ROS_INFO("Node for exposure control is up and running.");

    std::string image_topic = "";

    node_handle.getParam("step_length_aec", step_length_aec);
    node_handle.getParam("exposure_upper_bound", exposure_upper_bound);
    node_handle.getParam("image_topic", image_topic);
    node_handle.getParam("verbose_mode", verbose_mode);
    node_handle.getParam("publish_debug_images", publish_debug_images);

    ROS_INFO_STREAM("Exposure control using max upper_bound for exposure time: " << exposure_upper_bound);

    image_transport::ImageTransport it(node_handle);
    image_publisher = it.advertise("image_with_box", 1);

    ros::Subscriber image_subscriber = node_handle.subscribe(image_topic, 1, image_callback);
    ros::Subscriber region_subscriber = node_handle.subscribe("target_region", 1, region_callback);
    exposure_time_publisher = node_handle.advertise<std_msgs::Float64>("exposure_time", 1);

    ros::spin();
    return 0;
}