
// created by liuzhenbo in 2020/7/9

#include "lzb_vio/parameter.h"
namespace lzb_vio
{
Parameter::Parameter()
{
    //Sensors
    fx1_ = Config::Get<double>("camera_l.fx");
    fy1_ = Config::Get<double>("camera_l.fy");
    cx1_ = Config::Get<double>("camera_l.cx");
    cy1_ = Config::Get<double>("camera_l.cy");
    fx2_ = Config::Get<double>("camera_r.fx");
    fy2_ = Config::Get<double>("camera_r.fy");
    cx2_ = Config::Get<double>("camera_r.cx");
    cy2_ = Config::Get<double>("camera_r.cy");
    K1_.at<double>(0, 0) = fx1_;
    K1_.at<double>(0, 2) = cx1_;
    K1_.at<double>(1, 1) = fy1_;
    K1_.at<double>(1, 2) = cy1_;
    K1_.at<double>(2, 2) = 1.0;

    K2_.at<double>(0, 0) = fx2_;
    K2_.at<double>(0, 2) = cx2_;
    K2_.at<double>(1, 1) = fy2_;
    K2_.at<double>(1, 2) = cy2_;
    K2_.at<double>(2, 2) = 1.0;

    t_rl_.at<double>(0, 0) = Config::Get<double>("t_lr0");
    t_rl_.at<double>(1, 0) = Config::Get<double>("t_lr1");
    t_rl_.at<double>(2, 0) = Config::Get<double>("t_lr2");
    R_rl_.at<double>(0, 0) = Config::Get<double>("R_lr0");
    R_rl_.at<double>(0, 1) = Config::Get<double>("R_lr1");
    R_rl_.at<double>(0, 2) = Config::Get<double>("R_lr2");
    R_rl_.at<double>(1, 0) = Config::Get<double>("R_lr3");
    R_rl_.at<double>(1, 1) = Config::Get<double>("R_lr4");
    R_rl_.at<double>(1, 2) = Config::Get<double>("R_lr5");
    R_rl_.at<double>(2, 0) = Config::Get<double>("R_lr6");
    R_rl_.at<double>(2, 1) = Config::Get<double>("R_lr7");
    R_rl_.at<double>(2, 2) = Config::Get<double>("R_lr8");
    cv::Mat R_ll = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat t_ll = cv::Mat::zeros(3, 1, CV_64F);
    cv::hconcat(K1_ * R_ll, K1_ * t_ll, projMatr1_);
    cv::hconcat(K2_ * R_rl_, K2_ * t_rl_, projMatr2_);

    //Tracking
    num_features_init_ = Config::Get<int>("num_features_init");
    num_features_ = Config::Get<int>("num_features");
    num_features_tracking_bad_ = Config::Get<int>("num_features_tracking_bad");
    num_features_needed_for_keyframe_ = Config::Get<int>("num_features_needed_for_keyframe");
    init_landmarks_ = Config::Get<int>("init_landmarks");
    feature_match_error_ = Config::Get<double>("feature_match_error");
    track_mode_ = Config::Get<std::string>("track_mode");
    num_features_tracking_ = Config::Get<int>("num_features_tracking");
    inlier_rate_ = Config::Get<double>("inlier_rate");
    iterationsCount_ = Config::Get<int>("iterationsCount");
    reprojectionError_ = Config::Get<float>("reprojectionError");
    confidence_ = Config::Get<float>("confidence");
    display_scale_ = Config::Get<double>("display_scale");
    display_x_ = Config::Get<int>("display_x");
    display_y_ = Config::Get<int>("display_y");
    maxmove_ = Config::Get<double>("maxmove");
    minmove_ = Config::Get<double>("minmove");
    GFTTDetector_num_ = Config::Get<int>("num_features");
    nFeatures_ = Config::Get<int>("nFeatures");
    fScaleFactor_ = Config::Get<float>("fScaleFactor");
    nLevels_ = Config::Get<int>("nLevels");
    fIniThFAST_ = Config::Get<int>("fIniThFAST");
    fMinThFAST_ = Config::Get<int>("fMinThFAST");
    dataset_path_ = Config::Get<std::string>("dataset_path");
}
} // namespace lzb_vio
