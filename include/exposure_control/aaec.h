// aaec.h
#ifndef aaec_h
#define aaec_h

#include <opencv2/core.hpp>

class aaec {
public:
    aaec(double step_length, double upper_bound, bool verbose = false);

    double computeNewExposureTime(const cv::Mat& image, double current_exposure_time);
    // Setter
    void setStepLengthAEC(double step_length);
    void setExposureUpperBound(double upper_bound);
    void setDefaultExposureTime(double default_exposure_time);
    void setVerboseMode(bool verbose);
    void setRegion(int min_x, int min_y, int max_x, int max_y);


    
private:
    double step_length_aec_;
    double exposure_upper_bound_;
    bool verbose_mode_;

    double momentum_pre_ = 0.0;
    bool region_is_set_ = false;
    int min_x_, min_y_, max_x_, max_y_;
    double default_exposure_time = 0.0;

    cv::Mat get_image_gradient(const cv::Mat& img);
    cv::Mat get_gradient_x(const cv::Mat& img);
    cv::Mat get_gradient_y(const cv::Mat& img);
};

#endif // aaec_h