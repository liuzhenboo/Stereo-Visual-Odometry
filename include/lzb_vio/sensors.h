// created by liuzhenbo in 2020/7/9
#pragma once

#ifndef lzb_vio_SENSORS_H
#define lzb_vio_SENSORS_H
#include "lzb_vio/parameter.h"
#include "lzb_vio/common_include.h"

namespace lzb_vio
{
//class Parameter;
// Sensor platform
class Sensors
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Sensors> Ptr;
    Sensors(Parameter::Ptr parameter);
    //---------------------------------------------
    // stereo camera
    //---------------------------------------------
    // left camera intrinsics
    double fx1_ = 0, fy1_ = 0, cx1_ = 0, cy1_ = 0;
    cv::Mat K1_ = cv::Mat::zeros(3, 3, CV_64F);
    // right camera intrinsics
    double fx2_ = 0, fy2_ = 0, cx2_ = 0, cy2_ = 0;
    cv::Mat K2_ = cv::Mat::zeros(3, 3, CV_64F);
    // extrinsic
    cv::Mat t_rl_ = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat R_rl_ = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat projMatr1_ = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat projMatr2_ = cv::Mat::zeros(3, 4, CV_64F);

    // globalmap to left camera
    cv::Mat R_lm_ = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat t_lm_ = cv::Mat::zeros(3, 1, CV_64F);

    //---------------------------------------------
    // IMU
    //---------------------------------------------
    // ToDo

    //---------------------------------------------
    // Lidar
    //---------------------------------------------
    // ToDo

    // coordinate transform: world, camera, pixel
    cv::Mat world2camera(const cv::Mat &p_w, const cv::Mat &T_c_w);

    cv::Mat camera2world(const cv::Mat &p_c, const cv::Mat &T_c_w);

    cv::Mat camera2pixel(const cv::Mat &p_c);

    cv::Mat pixel2camera(const cv::Mat &p_p, double depth);

    cv::Mat pixel2world(const cv::Mat &p_p, const cv::Mat &T_c_w, double depth);

    cv::Mat world2pixel(const cv::Mat &p_w, const cv::Mat &T_c_w);
};

} // namespace lzb_vio
#endif // lzb_vio_SENSORS_H
