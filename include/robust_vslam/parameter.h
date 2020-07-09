#include "robust_vslam/common_include.h"
#include "robust_vslam/config.h"
#include "robust_vslam/sensors.h"
namespace robust_vslam
{
//double R_rl0_, R_rl1_, R_rl2_, R_rl3_, R_rl4_, R_rl5_, R_rl6_, R_rl7_, R_rl8_;
//double t_rl0_, t_rl1_, t_rl2_;
//double R_lm0_, R_lm1_, R_lm2_, R_lm3_, R_lm4_, R_lm5_, R_lm6_, R_lm7_, R_lm8_;
//double t_lm0_, t_lm1_, t_lm2_;

class Parameter
{
public:
    typedef std::shared_ptr<Parameter> Ptr;

    // Sensors
    double fx1_, fy1_, cx1_, cy1_, fx2_, fy2_, cx2_, cy2_;
    cv::Mat K1_ = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat K2_ = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat t_rl_ = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat R_rl_ = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat projMatr1_ = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat projMatr2_ = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat R_lm_ = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat t_lm_ = cv::Mat::zeros(3, 1, CV_64F);

    //
};
} // namespace robust_vslam
