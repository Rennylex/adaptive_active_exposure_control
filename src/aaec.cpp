//aaec.cpp
#include "aaec.h"
#include <opencv2/imgproc.hpp>
#include <iostream> // For logging, if verbose mode is true

aaec::aaec(double step_length, double upper_bound, bool verbose)
    : step_length_aec_(step_length), exposure_upper_bound_(upper_bound), verbose_mode_(verbose) {}

cv::Mat aaec::get_image_gradient(const cv::Mat& img) {
    // Compute image gradient
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_64F;
    Sobel(img, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
    
    abs_grad_x=abs( grad_x );
    Sobel(img, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
 
    abs_grad_y = abs(grad_y);
    cv::Mat img_grad;
    img_grad = abs_grad_x.mul(abs_grad_x) + abs_grad_y.mul(abs_grad_y);

    img_grad = abs_grad_x;
    return img_grad;
    //return cv::Mat(); // Placeholder return
}

cv::Mat aaec::get_gradient_x(const cv::Mat& img) {
    // Compute X gradient
    cv::Mat grad_x;
    cv::Mat abs_grad_x;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_64F;
    Sobel(img, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );

    abs_grad_x = abs( grad_x );

    return grad_x;
    //return cv::Mat(); // Placeholder return
}

cv::Mat aaec::get_gradient_y(const cv::Mat& img) {
    // Compute Y gradient
    cv::Mat grad_y;
    cv::Mat abs_grad_y;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_64F;
    Sobel( img, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
 
    abs_grad_y=abs( grad_y );
    return grad_y;
    //return cv::Mat(); // Placeholder return
}


double aaec::computeNewExposureTime(const cv::Mat& image, double current_exposure_time) {
    // Implementation of the exposure time computation algorithm
    auto image_original = image.clone();
    image.convertTo(image, CV_64FC1);
    double exposure_time_=current_exposure_time;

    if (exposure_time_ == 0.0) {
            exposure_time_ = default_exposure_time;
        }

    if(!region_is_set_){
        return 0;
    } else {
        cv::Mat image = image(
            cv::Rect(
                min_x_,
                min_y_,
                max_x_-min_x_,
                max_y_-min_y_
            )
        );

        double maxVal;
        double deltaT = exposure_time_/1000000.0;
        cv::Mat gradient_x = get_gradient_x(image);
        cv::Mat gradient_y = get_gradient_y(image);
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

        cv::Mat gradient_result = get_image_gradient(image);
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

        for (int i = 0; i < len; i++) {
            W[i] = W[i] / wsum;
        }

        double M_softperc = 0;
        for (int i = 0; i < len; i++) {
            M_softperc += (W[i] * (gradient_1d[i]));
        }

        //size should be the same as image
        cv::Mat g_prime_I = cv::Mat(image.size(),CV_64FC1,cv::Scalar(0,0,0));
        cv::Mat g_accum = cv::Mat(image.size(),CV_64FC1,cv::Scalar(1,1,1));
        cv::Mat image_cvt;
        image.convertTo(image_cvt, CV_64FC1);
        minMaxLoc(image_cvt, NULL, &maxVal);

        for (int i = 0; i < 15; i++) {
            g_prime_I += g_accum.mul(g_prime[14-i]);
            g_accum = g_accum.mul(image);
        }

        cv::Mat temp=1/(g_prime_I.mul(deltaT));
        temp.convertTo(temp, CV_64FC1);
        cv::Mat gradient_x_temp = get_gradient_x(temp);
        cv::Mat gradient_y_temp = get_gradient_y(temp);
        cv::Mat pGpdT = 2 * (gradient_x.mul(gradient_x_temp) + gradient_y.mul(gradient_y_temp));
        //FIND gradient_I max
        double minVal1;
        double maxVal1;
        minMaxLoc(pGpdT, &minVal1, &maxVal1);
        double pMpdT = 0;
        for (int i = 0; i < len; i++) {
            pMpdT = pMpdT + W[i] * pGpdT.at<double>(gradient_1d_with_location[i].second.first, gradient_1d_with_location[i].second.second);
        }

        // //step 5: update deltaT using the gradient descent method
        double alpha = step_length_aec_;
        double nextdeltaT = deltaT;
        //MOMENTUM VERSION
        double momentum = 0.9 * momentum_pre_ + alpha * pMpdT;

        if(abs(momentum) * 1000000.0 > 1){
            momentum_pre_ = momentum;
            nextdeltaT = deltaT + momentum;
        }

        exposure_time_ = nextdeltaT * 1000000.0;
        if(exposure_time_ <= 12) {
            exposure_time_ = 13;
        }
        
        if(exposure_time_ > exposure_upper_bound_) {
            exposure_time_ = exposure_upper_bound_;
        }

        return exposure_time_;
    }
}


void aaec::setStepLengthAEC(double step_length) {
    step_length_aec_ = step_length;
}

void aaec::setExposureUpperBound(double upper_bound) {
    exposure_upper_bound_ = upper_bound;
}

void aaec::setDefaultExposureTime(double default_exposure_time) {
    default_exposure_time = default_exposure_time;
}

void aaec::setVerboseMode(bool verbose) {
    verbose_mode_ = verbose;
}

void aaec::setRegion(int min_x, int min_y, int max_x, int max_y) {
    min_x_ = min_x;
    min_y_ = min_y;
    max_x_ = max_x;
    max_y_ = max_y;
    region_is_set_ = true;
}