// created by liuzhenbo in 2020/7/9
#pragma once

#ifndef robust_vslam_PARAMETER_H
#define robust_vslam_PARAMETER_H

#include "robust_vslam/common_include.h"
#include "robust_vslam/config.h"
namespace robust_vslam
{

class Parameter
{
public:
    typedef std::shared_ptr<Parameter> Ptr;
    Parameter();

    // Sensors
    // stereo camera
    double fx1_, fy1_, cx1_, cy1_, fx2_, fy2_, cx2_, cy2_;
    cv::Mat K1_ = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat K2_ = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat t_rl_ = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat R_rl_ = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat projMatr1_ = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat projMatr2_ = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat R_lm_ = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat t_lm_ = cv::Mat::zeros(3, 1, CV_64F);

    //Tracking
    int num_features_init_;
    int num_features_;
    int num_features_tracking_bad_;
    int num_features_needed_for_keyframe_;
    int init_landmarks_;
    double feature_match_error_;
    std::string track_mode_;
    int num_features_tracking_;
    double inlier_rate_;
    int iterationsCount_;
    float reprojectionError_;
    float confidence_;
    int display_scale_;
    int display_x_;
    int display_y_;
    double maxmove_;
    double minmove_;
    int GFTTDetector_num_;
    int nFeatures_;
    float fScaleFactor_;
    int nLevels_;
    int fIniThFAST_;
    int fMinThFAST_;
    std::string dataset_path_;
};
} // namespace robust_vslam
#endif // robust_vslam_PARAMETER_H
