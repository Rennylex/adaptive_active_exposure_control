using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 

#include <ros/ros.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <stag_ros/StagMarkers.h>


using namespace cv;

double deltaT_now;

int min_x;
int min_y;
int max_x;
int max_y;

int height;
int width;

int count_skip = 0;
double step_len_aec;//=0.000000001;
double step_len_gec;
bool active_aec;// = false;
bool active_gec;// = false;
bool active_aec_ori="default";
double max_bound;//=70000;
int target_id;
bool id_detected=false;
bool info_mode=false;
string camera_name="";
double momentum_pre=0;
string chosen_method;
image_transport::Publisher pub_image;
ros::Publisher exposure_time_publisher;
ros::Publisher active_algorithm_publisher;
ros::Publisher momentum_publisher;
//publish the four corners of the bounding box in one message
ros::Publisher bounding_box_publisher;

// ros::NodeHandle n;

void set_exposure_param(double new_deltaT){
    std_msgs::Float64 exposure_message;
    exposure_message.data = new_deltaT;
    exposure_time_publisher.publish(exposure_message);

    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    //dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::IntParameter double_param;
    dynamic_reconfigure::Config conf;

    double_param.name = "exposure_time";
    double_param.value = new_deltaT;
    //conf.doubles.push_back(double_param);
    conf.ints.push_back(double_param);

    srv_req.config = conf;
    ros::service::call("/acquisition_node/set_parameters", srv_req, srv_resp);
}


Mat gradient(Mat img)
{
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_64F;
    // Gradient X
    Sobel( img, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
    //Scharr( img, grad_x, ddepth, 1, 0, 3, scale, delta);//, BORDER_DEFAULT );

    //use prewitt to get the gradient
    // Mat kernel = (Mat_<double>(3,3) << -1, -1, -1,
    //                                     0, 0, 0,
    //                                     1, 1, 1);

    // filter2D(img, grad_x, ddepth, kernel);
    
    abs_grad_x=abs( grad_x );
    // Gradient Y
    Sobel( img, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    //Scharr( img, grad_y, ddepth, 0, 1, 3, scale, delta);//, BORDER_DEFAULT );

    //use prewitt to get the gradient
    // Mat kernel2 = (Mat_<double>(3,3) << -1, 0, 1,
    //                                     -1, 0, 1,
    //                                     -1, 0, 1);
    // filter2D(img, grad_y, ddepth, kernel2);
 
    abs_grad_y=abs( grad_y );
    //return the square sum
    Mat img_grad;
    //square sum! 
    img_grad = abs_grad_x.mul(abs_grad_x) + abs_grad_y.mul(abs_grad_y);


    img_grad=abs_grad_x;
    return img_grad;
}


Mat gradient_x(Mat img)
{
    Mat grad_x;
    Mat abs_grad_x;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_64F;
    // Gradient X
    Sobel( img, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
    //Scharr( img, grad_x, ddepth, 1, 0, 3, scale, delta);//, BORDER_DEFAULT );

    //use prewitt to get the gradient
    //Sobel(img, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
    // Mat kernel = (Mat_<double>(3,3) << -1,0,1,-1,0,1,-1,0,1);
    // filter2D(img, grad_x, ddepth, kernel);

    //normalize(grad_x, grad_x, -128, 127, NORM_MINMAX);

    abs_grad_x=abs( grad_x );

    return grad_x;
}


Mat gradient_y(Mat img)
{
    Mat grad_y;
    Mat abs_grad_y;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_64F;
    // Gradient Y
    Sobel( img, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    //Scharr( img, grad_y, ddepth, 0, 1, 3, scale, delta);//, BORDER_DEFAULT );

    //use prewitt to get the gradient
    // Mat kernel = (Mat_<double>(3,3) << -1, -1, -1, 0, 0, 0, 1, 1, 1);
    // filter2D(img, grad_y, ddepth, kernel);


    //normalize the grad_y
   // normalize(grad_y, grad_y, -128, 127, NORM_MINMAX);
 
    abs_grad_y=abs( grad_y );
    return grad_y;
}


Mat gradient_64F(Mat img){
  Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_64F;
    // Gradient X
    Sobel( img, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
    //Scharr( img, grad_x, ddepth, 1, 0, 3, scale, delta);//, BORDER_DEFAULT );

    //use prewitt to get the gradient
    // Mat kernel = (Mat_<double>(3,3) << -1, -1, -1,
    //                                     0, 0, 0,
    //                                     1, 1, 1);

    // filter2D(img, grad_x, ddepth, kernel);
    
    //abs_grad_x=abs( grad_x );
    convertScaleAbs(grad_x, abs_grad_x);
    // Gradient Y
    Sobel( img, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    //Scharr( img, grad_y, ddepth, 0, 1, 3, scale, delta);//, BORDER_DEFAULT );

    //use prewitt to get the gradient
    // Mat kernel2 = (Mat_<double>(3,3) << -1, 0, 1,
    //                                     -1, 0, 1,
    //                                     -1, 0, 1);
    // filter2D(img, grad_y, ddepth, kernel2);
    

    convertScaleAbs(grad_y, abs_grad_y);
    //abs_grad_y=abs( grad_y );
    //return the square sum
    Mat img_grad;
    //square sum! 
    img_grad = abs_grad_x.mul(abs_grad_x) + abs_grad_y.mul(abs_grad_y);
    return img_grad;

}


double grad_score(Mat grad){
    double lambda = 1000;
    double sigma=0.04;
    double N=log(lambda*(1-sigma)+1);
    double N_inv=1/N;
    double score=0;

    for (int i=0; i<grad.rows; i++){
        for (int j=0; j<grad.cols; j++){
        double val = grad.at<uchar>(i,j);
            if(val>=sigma){
                score+=N_inv*log(lambda*(val-sigma)+1);
                //ROS_INFO("score: %f",val);
            }  
        }
    }

    return score;

}

void debug_check_mat(Mat target){
    for(int i=0;i<target.rows;i++){
        for(int j=0;j<target.cols;j++){
            double val=target.at<double>(i,j);
            if(info_mode) {
                ROS_INFO("val: %f",val);
            }
        }
    }
}


double get_gamma(Mat img){
    //search gamma from 1/1.9 to 1.9
    //double gamma_anchor[7]={1/1.9, 1/1.5, 1/1.2, 1, 1.2, 1.5, 1.9};
    double gamma_anchor[11]={1/1.9, 1/1.5, 1/1.3, 1/1.2, 1/1.1, 1, 1.1, 1.2, 1.3, 1.5, 1.9};
    double gamma=1;
    cv::Mat img_powed1;
    cv::pow(img,1, img_powed1);
    //normalize(img_powed1, img_powed1, 0, 1, NORM_MINMAX);
    //compute the gradient
    Mat grad1=gradient_64F(img_powed1);
    //normalize(grad1, grad1, 0, 1, NORM_MINMAX);
    //double max_score=grad_score2(grad1);
    double max_score=grad_score(grad1);
    double second_max_score=0;


    for(int i=0;i<11;i++){
        cv::Mat img_powed;
        
        cv::pow(img, gamma_anchor[i], img_powed);
        //convert to 64F
        //img_powed.convertTo(img_powed, CV_64F);
        //normalize(img_powed, img_powed, 0, 1, NORM_MINMAX);
        //compute the gradient
        Mat grad=gradient_64F(img_powed);
        //normalize(grad, grad, 0, 1, NORM_MINMAX);
        //double score=grad_score(grad);
        double score=grad_score(grad);
        //ROS_INFO("score: %f, %d",score,i);
        if(score>max_score){
            second_max_score=max_score;
            max_score=score;
            //ROS_INFO("max_score: %f",max_score);
            gamma=gamma_anchor[i];
        }
    }
    if(info_mode){
        ROS_INFO("max_score: %f",max_score);
        ROS_INFO("second_max_score: %f",second_max_score);
    }
    return gamma;
}


double update_exposure_linear(double step_length, double exposure_time, double gamma){
    double alpha=0;
    if(gamma>=1){
        alpha=0.5;
    }
    else{
        alpha=1;
    }
    double new_exposure_time = exposure_time *(1+alpha*step_length*(1-gamma));
    return new_exposure_time;
}


double update_exposure_nonlinear(double step_length, double exposure_time, double gamma,double d){
    double alpha=0;
    if(gamma>=1){
        alpha=0.5;
    }
    else{
        alpha=1;
    }

    double R=d*tan((2-gamma)*atan2(1,d)-atan2(1,d))+1;
    
    double new_exposure_time = exposure_time *(1+alpha*step_length*(R-1));
    if (info_mode) {
        ROS_INFO("New exposure time: %f",new_exposure_time*1000000);
    }
    return new_exposure_time;
}

//the call back function for the ar tag

//stag_ros/StagMarkers:
    //Header header    
    //StagMarker[] markers    

//stag_ros/StagMarker
    //Header header
    //uint32 id
    //float32 reprojection_error # sum of the squared reprojection error of each corner
    //geometry_msgs/PoseStamped pose # pose from solvePnP
    //geometry_msgs/Point[] corners # corners from marker detection in image coordinates
void tag_callback(const stag_ros::StagMarkers::ConstPtr& msg)
{
    int idx=0;
    //id_detected=false;
    for(int i=0;i<msg->markers.size();i++){
        uint32_t id=msg->markers[i].id;
        if (id==target_id)//change to 1
        {
            id_detected=true;
            idx=i;

            float upperleft_x=msg->markers[idx].corners[0].x;
            float upperleft_y=msg->markers[idx].corners[0].y;

            float upperright_x=msg->markers[idx].corners[1].x;
            float upperright_y=msg->markers[idx].corners[1].y;

            float lowerleft_x=msg->markers[idx].corners[2].x;
            float lowerleft_y=msg->markers[idx].corners[2].y;

            float lowerright_x=msg->markers[idx].corners[3].x;
            float lowerright_y=msg->markers[idx].corners[3].y;
            //ROS_INFO("In the call back!");
            // ROS_INFO("callback upper_left: [%f]", upperleft_x);

            //find the min and max of the x and y
            //make sure it's int
            min_x = (int)std::min(std::min(upperleft_x,lowerleft_x),std::min(upperright_x,lowerright_x));//-200;//-padding_x/8;//10;//25;
            min_y = (int)std::min(std::min(upperleft_y,upperright_y),std::min(lowerleft_y,lowerright_y));//-200;//-padding_y/8;//10;//25;
            max_x = (int)std::max(std::max(upperright_x,lowerright_x),std::max(lowerleft_x,upperleft_x));//+200;//+padding_x/8;//10;//25;
            max_y = (int)std::max(std::max(upperright_y,lowerright_y),std::max(lowerleft_y,upperleft_y));//+200;//+padding_y/8;//10;//25;

            //ROS_INFO("MAX_X: [%d]", max_x);
            //ROS_INFO("MIN_X: [%d]", min_x);

            //padding
            float padding_x=(max_x-min_x)*0.1;
            float padding_y=(max_y-min_y)*0.1;

            min_x=min_x-padding_x;
            min_y=min_y-padding_y;
            max_x=max_x+padding_x;
            max_y=max_y+padding_y;

            min_x=std::max(0,min_x);
            min_y=std::max(0,min_y);

            max_x=std::min(height,max_x);
            max_y=std::min(width,max_y);

            // ROS_INFO("MAX_X: [%d]", max_x);
            // ROS_INFO("MIN_X: [%d]", min_x);
            // ROS_INFO("MAX_Y: [%d]", max_y);
            // ROS_INFO("MIN_Y: [%d]", min_y);
        }   
    }
}

//  rostopic info /camera_array/cam0/image_raw
// Type: sensor_msgs/Image

// Publishers: 
//  * /acquisition_node (http://rlab-ubuntu:41519/)

// Subscribers: 
//  * /bluerov_nodelet_manager (http://rlab-ubuntu:39017/)
 Mat img;



//publish the image
void publish_image(Mat image, ros::Publisher pub_image, ros::Time timestamp){
    //convert image to CV_8UC1
    image.convertTo(image, CV_8UC1);
    //convert image to ros msg
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
    //set the timestamp
    msg->header.stamp = timestamp;
    //publish the image
    pub_image.publish(msg);
}


void update_exposure(double deltaT){

    Mat image=img;

//convert image to CV64FC1
    image.convertTo(image, CV_64FC1);

    //Mat image = frames_[0];
    // min_x = 843-50;
    // min_y = 485-50;
    // max_x = 929+50;
    // max_y = 574+50;

    //ros::Rate ros_rate(soft_framerate_);
            // //
        //     // //update the exposure time
    double next_exposure_time = 0;
    bool active_exposure_control=active_aec;//false;
    bool active_exposure_control_ori=active_aec_ori;
    bool grad_exposure_control=active_gec;//true;

    double exposure_time_ = deltaT_now;

  //
    count_skip++; // TODO remove this?

    if(!active_exposure_control && !grad_exposure_control && !active_aec_ori){
        set_exposure_param(0);
        ROS_INFO("Current exposure time: [%f]", exposure_time_);
    }
    else if(active_exposure_control && count_skip > 10 && id_detected){
    // TODO guaranteed update min and max according to current image
    // min_x = 0;
    // min_y = 0;
    // max_x = image.cols;
    // max_y = image.rows;
    //Mat image0 = frames_[0];
    // //print the size of image
    // ROS_INFO("I heard image.cols: [%d]", image.cols);
    // ROS_INFO("I heard image.rows: [%d]", image.rows);
    
    // ROS_INFO("min_x: [%d]", min_x);
    // ROS_INFO("min_y: [%d]", min_y);
    // ROS_INFO("max_x: [%d]", max_x);
    // ROS_INFO("max_y: [%d]", max_y);
    //crop the image
    Mat image_with_box = img.clone();

   // image_with_box.convertTo(image_with_box, CV_8UC3);
    //convert to rgb
    cvtColor(image_with_box, image_with_box, CV_GRAY2RGB);


    image = image(Rect(min_x,min_y,max_x-min_x,max_y-min_y));

    //draw the bounding box using red color
    rectangle(image_with_box, Point(min_x,min_y), Point(max_x,max_y), Scalar(255,0,0), 2, 8, 0);
    //publish the bounding box topic
    std_msgs::Int32MultiArray msg_box;
    msg_box.data.push_back(min_x);
    msg_box.data.push_back(min_y);
    msg_box.data.push_back(max_x);
    msg_box.data.push_back(max_y);
    bounding_box_publisher.publish(msg_box);
    //publish the image with bounding box
    //define a publisher
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image_with_box).toImageMsg();
    pub_image.publish(msg);
    //print the min and max
    // ROS_INFO("I heard min_x: [%d]", min_x);
    // ROS_INFO("I heard min_y: [%d]", min_y);
    // ROS_INFO("I heard max_x: [%d]", max_x);
    // ROS_INFO("I heard max_y: [%d]", max_y);

    //compress the image in half
    //resize(image, image, Size(), 0.5, 0.5, INTER_LINEAR);

    //new approach
    //get the largest value in the image
    double maxVal;
    double deltaT = exposure_time_/1000000;
    //Mat Gradient_I = gradient(image);
    Mat Gradient_x = gradient_x(image);
    Mat Gradient_y = gradient_y(image);
    if(info_mode) {
        ROS_INFO("Gradient x and y calculated");
    }
// //step 0: read the coefficients for 10th order polynomial g_prime.
    vector<double> g_prime;
    g_prime.push_back(2.23648003e-28);
    g_prime.push_back(-4.23843983e-25);
    g_prime.push_back(3.61455838e-22);
    g_prime.push_back(-1.83243015e-19);
    g_prime.push_back(6.14568309e-17);
    g_prime.push_back(-1.43597629e-14);
    g_prime.push_back(2.39781444e-12);
    g_prime.push_back(-2.88855261e-10);
    g_prime.push_back(2.50378889e-08);
    g_prime.push_back(-1.54014489e-06);
    g_prime.push_back(6.54831886e-05);
    g_prime.push_back(-1.84372346e-03);
    g_prime.push_back(3.21098127e-02);
    g_prime.push_back(-3.08446340e-01);
    g_prime.push_back(1.30761122e+00);

    //from order 14 to 0
// // //step 1: get the gradient magnitude
     Mat gradient_result = gradient(image);
    if(info_mode) {
        ROS_INFO("Gradient calculated");
    }
     
     //construct a vector to store the gradient magnitude
    //vector<double> gradient_magnitude_1d=vector<double>(gradient_result.rows);
    cv::Mat flat = gradient_result.reshape(1, gradient_result.rows*gradient_result.cols);
    // ROS_INFO("Gradient_result size: [%d]", gradient_result.rows*gradient_result.cols);
    // ROS_INFO("gradient_result number of rows: %d", gradient_result.rows);
    // ROS_INFO("gradient_result number of cols: %d", gradient_result.cols);
    // ROS_INFO("flat number of rows: %d", flat.rows);
    // ROS_INFO("flat number of cols: %d", flat.cols);
    // for(int i=0;i < gradient_result.rows*gradient_result.cols;i++)
    // {
    //    ROS_INFO("flat gradient_result[%d]: %f",i,flat.at<double>(i));
    // }
    //(1, gradient_result.total()*gradient_result.channels());
    std::vector<double> gradient_1d = flat;//gradient_result.isContinuous()? flat : flat.clone();

    // ROS_INFO("Gradient 1d calculated");

    std::vector<pair<double,pair<int,int>>> gradient_1d_with_location;

    // ROS_INFO("Gradient 1d size: %d", gradient_1d.size());

    for (int i=0;i<gradient_1d.size();i++)
    {
        gradient_1d_with_location.push_back(make_pair(gradient_1d[i],make_pair(i/gradient_result.cols,i%gradient_result.cols)));
        //ROS_INFO("gradient_1d[%d]: %f",i,gradient_1d[i]);
    }
    
    //get the minimum value in gradient_1d
    // ROS_INFO("GRADIENT1D MIN: %f", *min_element(gradient_1d.begin(),gradient_1d.end()));

// //step 2: sort the gradient magnitude in ascending order
    std::sort(gradient_1d.begin(),gradient_1d.end());
    std::sort(gradient_1d_with_location.begin(),gradient_1d_with_location.end());
//after sorting, I also want to get the location of the element in the original Matrix


// //assume p=0.5

// //step 3: Then calculate M_softperc. M_softperc=sum(W[i]*Gradient[i])
//     //step 3.1: calculate the weight
//         //if i<=scale_down(p*len(Gradient))
//             //W[i]=(1/N)*sin(pi*i/(2*scale_down(p*len(Gradient))))^k
//         //else
//             //W[i]=(1/N)*sin(pi/2-(pi*i)/(2*(len(Gradient)-scale_down(p*len(Gradient)))))^k
//         //k=5
    double p=0.9;
    vector<double> W;
    //int N = gradient_result.rows;
    //N normalize the sum of W to 1
    int k = 5;
    int len = gradient_result.rows*gradient_result.cols;
    int scale_down = len * p;
    if(info_mode) {
        ROS_INFO("len is %d", len);
    }

    double wsum = 0;
    //N=log(lambda(1-sigma)+1) is a normalization factor
    double lambda = 10;
    double sigma = 0.11;
    double N = log(lambda*(1-sigma)+1);
    for (int i = 0; i < len; i++) {
        if (i <= scale_down) {
            //W.push_back(0);
            W.push_back(pow(sin((CV_PI * i) / (2 * scale_down)), k));
            wsum+=W[i];
            //ROS_INFO("sum is %f", wsum);
        } else {
            //W.push_back(0);
            W.push_back(pow(sin(CV_PI / 2 - (CV_PI * (i-scale_down)) / (2 * (len - scale_down))), k));
            wsum+=W[i];
            //ROS_INFO("w is %f", W[i]);
        }
        //ROS_INFO("SUM is %f", wsum);
    }
    if(info_mode) {
        ROS_INFO("Weighting DONE");
    }

    for (int i = 0; i < len; i++) {
        W[i] = W[i] / wsum;
        //ROS_INFO("W[i] is %f", W[i]);
    }

//     //step 3.2: calculate the M_softperc
    double M_softperc = 0;
    for (int i = 0; i < len; i++) {
        // if(gradient_1d[i]!=0)
        //      ROS_INFO("gr is %f", gradient_1d[i]);
        M_softperc += (W[i] * (gradient_1d[i]));
    }
    if(info_mode) {
        ROS_INFO("M_softperc is %f", M_softperc);
    }
    Mat g_prime_I = Mat(image.size(),CV_64FC1,Scalar(0,0,0));//size should be the same as image
    Mat g_accum = Mat(image.size(),CV_64FC1,Scalar(1,1,1));//Mat::ones(image.size(), CV_32FC1);
    Mat image_cvt;
    image.convertTo(image_cvt, CV_64FC1);
        
    minMaxLoc(image_cvt, NULL, &maxVal);

    for (int i = 0; i < 15; i++) {
        g_prime_I += g_accum.mul(g_prime[14-i]);
        g_accum = g_accum.mul(image);
        //ROS_INFO("g_accum is %f", g_accum.at<double>(0,0));
        //ROS_INFO("g_prime is %f", g_prime[14-i]);
    }

    Mat temp=1/(g_prime_I.mul(deltaT));
    temp.convertTo(temp, CV_64FC1);
    Mat Gradient_x_temp=gradient_x(temp);
    Mat Gradient_y_temp=gradient_y(temp);
    Mat pGpdT = 2*(Gradient_x.mul(Gradient_x_temp)+Gradient_y.mul(Gradient_y_temp));


    //FIND gradient_I max
    double minVal1;
    double maxVal1;
    minMaxLoc(pGpdT, &minVal1, &maxVal1);
    //ROS_INFO_STREAM("minVal for gradient is "<<minVal1);

    //Mat pGpdT = 2*Gradient_I.t()*gradient(1/(g_prime_I*deltaT));
    //index is still the index of the sorted gradient magnitude
    //Mat pGpdT = gradient(g_prime_I.mul(deltaT));

    //pMpdT = sum(W[i]*pGpdT[index])
    double pMpdT = 0;
    for (int i = 0; i < len; i++) {
        //ROS_INFO("W[i] is %f", W[i]);
        //ROS_INFO("gradient_1d_with_location is %f", gradient_1d_with_location[i].first);
        //ROS_INFO("pGpdT is %f", pGpdT.at<double>(gradient_1d_with_location[i].second.first, gradient_1d_with_location[i].second.second));
        pMpdT = pMpdT + W[i] * pGpdT.at<double>(gradient_1d_with_location[i].second.first, gradient_1d_with_location[i].second.second);
        //ROS_INFO("pMpdT is %f", pMpdT);
    }

    
    if(info_mode) {
        ROS_INFO("pMpdT is %f", pMpdT);
    }
// //step 5: update deltaT using the gradient descent method
//     //deltaT = deltaT - alpha*pMpdT
    double alpha =step_len_aec;//0.000000001;//0.000001;//0.0000001; //0.00000000005;//0.0000000001;//0.000000005;//0.00000005
    double nextdeltaT = deltaT;
    // if(abs(pMpdT)>15000 && abs(alpha*pMpdT)*1000000>1){

    //     nextdeltaT = deltaT + alpha * pMpdT;

    // }

    //MOMENTUM VERSION
    double momentum = 0.9*momentum_pre+alpha*pMpdT;
    if(info_mode) {
        ROS_INFO_STREAM("momentum is "<<momentum*1000000);
    }
    //publish momentum
    std_msgs::Float64 msg_momentum;
    msg_momentum.data=momentum*1000000;
    momentum_publisher.publish(msg_momentum);


    if(abs(momentum)*1000000>1){

        momentum_pre=momentum;
        nextdeltaT = deltaT + momentum;
        //nextdeltaT = deltaT + alpha * pMpdT;
    }
    //MOMENTUM VERSION ENDS
    if (info_mode) {
        ROS_INFO_STREAM("Attempted Exposure time: " << nextdeltaT*1000000);
    }

    exposure_time_=nextdeltaT*1000000;
    if(exposure_time_<=12) {
        exposure_time_=13;
    }
    
    if(exposure_time_>max_bound) {
        exposure_time_=max_bound;
    }
    //cams[0].setFloatValue(exposure_time_);
    set_exposure_param(exposure_time_);
            //cams[0].setFloatValue("AutoExposureTargetGreyValue", 90);
    }//

    else if(active_exposure_control_ori && count_skip > 10 && id_detected){
    
        Mat image_with_box = img.clone();
        //convert to rgb
        cvtColor(image_with_box, image_with_box, CV_GRAY2RGB);

        //publish the image with bounding box
        //define a publisher
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image_with_box).toImageMsg();
        pub_image.publish(msg);

        //downsample the image
        //resize(image, image, Size(), 0.75, 0.75, INTER_LINEAR);

        //draw the bounding box using red color
        rectangle(image_with_box, Point(min_x,min_y), Point(max_x,max_y), Scalar(255,0,0), 2, 8, 0);
        //publish the bounding box topic
        std_msgs::Int32MultiArray msg_box;
        msg_box.data.push_back(min_x);
        msg_box.data.push_back(min_y);
        msg_box.data.push_back(max_x);
        msg_box.data.push_back(max_y);
        bounding_box_publisher.publish(msg_box);
        //get the largest value in the image
        double maxVal;
        double deltaT = exposure_time_/1000000;
        //Mat Gradient_I = gradient(image);
        Mat Gradient_x = gradient_x(image);
        Mat Gradient_y = gradient_y(image);
        if(info_mode) {
            ROS_INFO("Gradient x and y calculated");
        }
    // //step 0: read the coefficients for 10th order polynomial g_prime.
        vector<double> g_prime;
        g_prime.push_back(2.23648003e-28);
        g_prime.push_back(-4.23843983e-25);
        g_prime.push_back(3.61455838e-22);
        g_prime.push_back(-1.83243015e-19);
        g_prime.push_back(6.14568309e-17);
        g_prime.push_back(-1.43597629e-14);
        g_prime.push_back(2.39781444e-12);
        g_prime.push_back(-2.88855261e-10);
        g_prime.push_back(2.50378889e-08);
        g_prime.push_back(-1.54014489e-06);
        g_prime.push_back(6.54831886e-05);
        g_prime.push_back(-1.84372346e-03);
        g_prime.push_back(3.21098127e-02);
        g_prime.push_back(-3.08446340e-01);
        g_prime.push_back(1.30761122e+00);

        //from order 14 to 0
    // // //step 1: get the gradient magnitude
        Mat gradient_result = gradient(image);
        if(info_mode) {
            ROS_INFO("Gradient calculated");
        }
        
        //construct a vector to store the gradient magnitude
        //vector<double> gradient_magnitude_1d=vector<double>(gradient_result.rows);
        cv::Mat flat = gradient_result.reshape(1, gradient_result.rows*gradient_result.cols);
        std::vector<double> gradient_1d = flat;//gradient_result.isContinuous()? flat : flat.clone();

        // ROS_INFO("Gradient 1d calculated");

        std::vector<pair<double,pair<int,int>>> gradient_1d_with_location;

        // ROS_INFO("Gradient 1d size: %d", gradient_1d.size());

        for (int i=0;i<gradient_1d.size();i++)
        {
            gradient_1d_with_location.push_back(make_pair(gradient_1d[i],make_pair(i/gradient_result.cols,i%gradient_result.cols)));
            //ROS_INFO("gradient_1d[%d]: %f",i,gradient_1d[i]);
        }
        
        //get the minimum value in gradient_1d
        // ROS_INFO("GRADIENT1D MIN: %f", *min_element(gradient_1d.begin(),gradient_1d.end()));

    // //step 2: sort the gradient magnitude in ascending order
        std::sort(gradient_1d.begin(),gradient_1d.end());
        std::sort(gradient_1d_with_location.begin(),gradient_1d_with_location.end());
    //after sorting, I also want to get the location of the element in the original Matrix


    // //assume p=0.5

    // //step 3: Then calculate M_softperc. M_softperc=sum(W[i]*Gradient[i])
    //     //step 3.1: calculate the weight
    //         //if i<=scale_down(p*len(Gradient))
    //             //W[i]=(1/N)*sin(pi*i/(2*scale_down(p*len(Gradient))))^k
    //         //else
    //             //W[i]=(1/N)*sin(pi/2-(pi*i)/(2*(len(Gradient)-scale_down(p*len(Gradient)))))^k
    //         //k=5
        double p=0.9;
        vector<double> W;
        //int N = gradient_result.rows;
        //N normalize the sum of W to 1
        int k = 5;
        int len = gradient_result.rows*gradient_result.cols;
        int scale_down = len * p;
        if(info_mode) {
            ROS_INFO("len is %d", len);
        }
        double wsum = 0;
        //N=log(lambda(1-sigma)+1) is a normalization factor
        double lambda = 10;
        double sigma = 0.11;
        double N = log(lambda*(1-sigma)+1);
        for (int i = 0; i < len; i++) {
            if (i <= scale_down) {
                //W.push_back(0);
                W.push_back(pow(sin((CV_PI * i) / (2 * scale_down)), k));
                wsum+=W[i];
                //ROS_INFO("sum is %f", wsum);
            } else {
                //W.push_back(0);
                W.push_back(pow(sin(CV_PI / 2 - (CV_PI * (i-scale_down)) / (2 * (len - scale_down))), k));
                wsum+=W[i];
                //ROS_INFO("w is %f", W[i]);
            }
            //ROS_INFO("SUM is %f", wsum);
        }
        if(info_mode) {
            ROS_INFO("Weighting DONE");
        }

        for (int i = 0; i < len; i++) {
            W[i] = W[i] / wsum;
            //ROS_INFO("W[i] is %f", W[i]);
        }

    //     //step 3.2: calculate the M_softperc
        double M_softperc = 0;
        for (int i = 0; i < len; i++) {
            // if(gradient_1d[i]!=0)
            //      ROS_INFO("gr is %f", gradient_1d[i]);
            M_softperc += (W[i] * (gradient_1d[i]));
        }

        if(info_mode) {
            ROS_INFO("M_softperc is %f", M_softperc);
        }

        Mat g_prime_I = Mat(image.size(),CV_64FC1,Scalar(0,0,0));//size should be the same as image
        Mat g_accum = Mat(image.size(),CV_64FC1,Scalar(1,1,1));//Mat::ones(image.size(), CV_32FC1);
        Mat image_cvt;
        image.convertTo(image_cvt, CV_64FC1);
            
        minMaxLoc(image_cvt, NULL, &maxVal);

        for (int i = 0; i < 15; i++) {
            g_prime_I += g_accum.mul(g_prime[14-i]);
            g_accum = g_accum.mul(image);
            //ROS_INFO("g_accum is %f", g_accum.at<double>(0,0));
            //ROS_INFO("g_prime is %f", g_prime[14-i]);
        }

        Mat temp=1/(g_prime_I.mul(deltaT));
        temp.convertTo(temp, CV_64FC1);
        Mat Gradient_x_temp=gradient_x(temp);
        Mat Gradient_y_temp=gradient_y(temp);
        Mat pGpdT = 2*(Gradient_x.mul(Gradient_x_temp)+Gradient_y.mul(Gradient_y_temp));


        //FIND gradient_I max
        double minVal1;
        double maxVal1;
        minMaxLoc(pGpdT, &minVal1, &maxVal1);
        //ROS_INFO_STREAM("minVal for gradient is "<<minVal1);
        double pMpdT = 0;
        for (int i = 0; i < len; i++) {
            //ROS_INFO("W[i] is %f", W[i]);
            //ROS_INFO("gradient_1d_with_location is %f", gradient_1d_with_location[i].first);
            //ROS_INFO("pGpdT is %f", pGpdT.at<double>(gradient_1d_with_location[i].second.first, gradient_1d_with_location[i].second.second));
            pMpdT = pMpdT + W[i] * pGpdT.at<double>(gradient_1d_with_location[i].second.first, gradient_1d_with_location[i].second.second);
            //ROS_INFO("pMpdT is %f", pMpdT);
        }

        
        if(info_mode) {
            ROS_INFO("pMpdT is %f", pMpdT);
        }
    // //step 5: update deltaT using the gradient descent method
    //     //deltaT = deltaT - alpha*pMpdT
        double alpha =0.00000001;//step_len_aec;//0.000000001;//0.000001;//0.0000001; //0.00000000005;//0.0000000001;//0.000000005;//0.00000005
        double nextdeltaT = deltaT;

        if (info_mode) {
            ROS_INFO("alpha*pMpdT is %f", alpha*pMpdT);
        }

        if(abs(pMpdT)>15000 && abs(alpha*pMpdT)*1000000>0.1){

            nextdeltaT = deltaT + alpha * pMpdT;

        }

        if (info_mode) {
            ROS_INFO_STREAM("Attempted Exposure time: " << nextdeltaT*1000000);
        }

        exposure_time_=nextdeltaT*1000000;
        if(exposure_time_<=12)
            exposure_time_=13;
        
        if(exposure_time_>max_bound)
            exposure_time_=max_bound;
        //cams[0].setFloatValue(exposure_time_);
        set_exposure_param(exposure_time_);
                //cams[0].setFloatValue("AutoExposureTargetGreyValue", 90);
    }//

    else if(grad_exposure_control && count_skip>10){
        Mat image = img;

        //Mat image;

        //image0.convertTo(image, CV_64F);

        //crop the image
        // min_x=min_x-50;
        // min_y=min_y-50;
        // max_x+=50;
        // max_y+=50;

        min_x=max(0,min_x-25);
        min_y=max(0,min_y-25);
        max_x=min(max_x+25,image.cols);
        max_y=min(max_y+25,image.rows);
        //image = image(Rect(min_x,min_y,max_x-min_x,max_y-min_y));
        // //compress the image in half
        //resize(image, image, Size(), 1, 1, INTER_LINEAR);

        image.convertTo(image, CV_64F); 
        // //compute gradient

        //normalize.  comment out for gradscore2
        normalize(image, image, 0, 1, NORM_MINMAX);

        double gamma =0;
        double step_len=step_len_gec;
        
        gamma=get_gamma(image);
        if(info_mode) {
            ROS_INFO_STREAM("gamma is "<<gamma);
        }
        double deltaT = exposure_time_/1000000;
        //double nextdeltaT = update_exposure_linear(step_len, deltaT, gamma);

        double d=0.1;
        double nextdeltaT = update_exposure_nonlinear(step_len, deltaT, gamma,d);

        if (info_mode) {
            ROS_INFO_STREAM("Exposure time: " << nextdeltaT*1000000);
        }

        exposure_time_=nextdeltaT*1000000;
        if(exposure_time_<=12)
            exposure_time_=13;
        
        if(exposure_time_>max_bound)//70000)
            exposure_time_=max_bound;//70000;
        // cams[0].setFloatValue("ExposureTime", exposure_time_);
        
        set_exposure_param(exposure_time_);
    }

}


void img_callback(const sensor_msgs::ImageConstPtr& msg)
{
    //in this callback, we will convert the image to cv::Mat
    ros::param::get("/acquisition_node/exposure_time", deltaT_now);

    if (info_mode) {
        ROS_INFO("exposure_time_ is %f", deltaT_now);
    }

    std_msgs::String alg_message;
    if(active_aec==true){
        if (info_mode) {
            ROS_INFO("AAEC running");
        }
        alg_message.data = "aaec";
    }
    else if(active_gec==true){
        if (info_mode) {
            ROS_INFO("GEC running");
        }
        alg_message.data = "gec";
    }
    else if(active_aec_ori==true){
        if (info_mode) {
            ROS_INFO("AEC original running");
        }
        alg_message.data = "aec_orig";
    }
    else {
        if (info_mode) {
            ROS_INFO("Default Exposure Control running");
        }
        alg_message.data = "default";
    }

    active_algorithm_publisher.publish(alg_message);

    

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
        //change it into gray scale
        cv_ptr =cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        img=cv_ptr->image;
        height=img.cols;
        width=img.rows;
        //ROS_INFO("image intensity is "<< img.at<int>(0,0));
    
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    update_exposure(deltaT_now);
}


int main(int argc, char** argv) {
    
    int result = 0;

    // Initializing the ros node
    ros::init(argc, argv, "changeParam");
    //spinners
    ros::NodeHandle n;
    
    ROS_INFO("Node for exposure control is up and running.");

    // n.getParam("/changeParam/step_length_aec_", step_len_aec);
    // n.getParam("/changeParam/active_aec_", active_aec);
    // n.getParam("/changeParam/active_gec_", active_gec);
    // n.getParam("/changeParam/upper_bound_", max_bound);

    n.getParam("step_length_aec", step_len_aec);
    n.getParam("step_length_gec", step_len_gec);
    n.getParam("upper_bound", max_bound);
    n.getParam("target_id", target_id);
    n.getParam("cam_name",camera_name);
    n.getParam("info_mode", info_mode);
    n.getParam("chosen_method", chosen_method);
    ROS_INFO("Exposure control using max upper_bound for exposure time: %f",max_bound);

    //deal with chosen_method
    if(chosen_method=="aaec"){
        active_aec=true;
        active_gec=false;
        active_aec_ori=false;
    }
    else if(chosen_method=="gec"){
        active_aec=false;
        active_gec=true;
        active_aec_ori=false;
    }
    else if(chosen_method=="aec"){
        active_aec=false;
        active_gec=false;
        active_aec_ori=true;
    }
    else{
        active_aec=false;
        active_gec=false;
        active_aec_ori=false;
    }

    exposure_time_publisher = n.advertise<std_msgs::Float64>("exposure_time", 1);
    active_algorithm_publisher = n.advertise<std_msgs::String>("active_exposure_control_algorithm", 1);
    momentum_publisher = n.advertise<std_msgs::Float64>("momentum", 1);
    bounding_box_publisher = n.advertise<std_msgs::Int32MultiArray>("bounding_box", 1);
    image_transport::ImageTransport it(n);
    pub_image = it.advertise("image_with_box", 1);

// "/camera_array/"+camera_name+"/image_raw"
    //subscribe to /camera_array/cam0/image_raw
    ros::Subscriber sub_image = n.subscribe(camera_name, 1, img_callback);
    ros::Subscriber sub_ = n.subscribe("/bluerov_controller/ar_tag_detector_2", 1, tag_callback);

    ros::spin();

    return 0;
}
