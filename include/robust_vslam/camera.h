#pragma once

#ifndef robust_vslam_CAMERA_H
#define robust_vslam_CAMERA_H

#include "robust_vslam/common_include.h"

namespace robust_vslam
{

// Pinhole stereo camera model
class Camera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Camera> Ptr;

    // Camera intrinsics
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
    cv::Mat K_;

    // Camera extrinsic
    double t_lr_ = 0;
    cv::Mat R_lr_;
    cv::Mat projMatr_;

    Camera();

    Camera(const double &fx, const double &fy, const double &cx, const double &cy, const double &t_lr,
           const cv::Mat &R_lr_)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), t_lr_(t_lr), R_lr_(R_lr)
    {
        K_ = (cv::Mat_<float>(3, 3) << fx, 0., cx, 0., fy, cy, 0., 0., 1.);
        projMatr_ = (cv::Mat_<float>(3, 4) << fx, 0., cx, -t_lr * fx, 0., fy, cy, 0., 0, 0., 1., 0.);
    }

    // coordinate transform: world, camera, pixel
    cv::Mat world2camera(const cv::Mat &p_w, const cv::Mat &T_c_w);

    cv::Mat camera2world(const cv::Mat &p_c, const cv::Mat &T_c_w);

    cv::Mat camera2pixel(const cv::Mat &p_c);

    cv::Mat pixel2camera(const cv::Mat &p_p, double depth = 1);

    cv::Mat pixel2world(const cv::Mat &p_p, const cv::Mat &T_c_w, double depth = 1);

    cv::Mat world2pixel(const cv::Mat &p_w, const cv::Mat &T_c_w);
};

} // namespace robust_vslam
#endif // robust_vslam_CAMERA_H
