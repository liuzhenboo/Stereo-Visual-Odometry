#include "robust_vslam/sensors.h"

namespace robust_vslam
{

Sensors::Sensors(Parameter::Ptr parameter)
{
    fx1_ = parameter->fx1_;
    fy1_ = parameter->fy1_;
    cx1_ = parameter->cx1_;
    cy1_ = parameter->cy1_;
    fx2_ = parameter->fx2_;
    fy2_ = parameter->fy2_;
    cx2_ = parameter->cx2_;
    cy2_ = parameter->cy2_;
    K1_ = parameter->K1_;
    K2_ = parameter->K2_;
    t_rl_ = parameter->t_rl_;
    R_rl_ = parameter->R_rl_;
    projMatr1_ = parameter->projMatr1_;
    projMatr2_ = parameter->projMatr2_;
}

cv::Mat Sensors::world2camera(const cv::Mat &p_w, const cv::Mat &T_c_w)
{
    cv::Mat R_c_w = T_c_w.rowRange(0, 3).colRange(0, 3);
    cv::Mat t_c_w = T_c_w.rowRange(0, 3).col(3);
    cv::Mat pw = p_w.rowRange(0, 3).col(0);
    return R_c_w * pw + t_c_w;
}

cv::Mat Sensors::camera2world(const cv::Mat &p_c, const cv::Mat &T_c_w)
{
    cv::Mat R_c_w = T_c_w.rowRange(0, 3).colRange(0, 3);
    cv::Mat R_w_c = R_c_w.inverse();
    cv::Mat t_w_c = -R_w_c * T_c_w.rowRange(0, 3).col(3);
    cv::Mat pc = p_c.rowRange(0, 3).col(0);
    return R_w_c * pc + t_w_c;
}

cv::Mat Sensors::camera2pixel(const cv::Mat &p_c)
{
    cv::Mat pc = p_c.rowRange(0, 3).col(0);
    cv::Mat camera = (cv::Mat_<double>(2, 1) << (fx_ * pc.at<double>(0, 0) / pc.at<double>(2, 0) + cx_),
                      (fy_ * pc.at<double>(1, 0) / pc.at<double>(2, 0) + cy_));
    return camera;
}

cv::Mat Sensors::pixel2camera(const cv::Mat &p_p, double depth)
{
    cv::Mat pixel = (cv::Mat_<double>(3, 1) << depth * (p_p.at<double>(0, 0) - cx_) / fx_, depth * (p_p.at<double>(1, 0) - cy_) / fy_, depth);

    return pixel;
}

cv::Mat Sensors::world2pixel(const cv::Mat &p_w, const cv::Mat &T_c_w)
{
    return camera2pixel(world2camera(p_w, T_c_w));
}

cv::Mat Sensors::pixel2world(const cv::Mat &p_p, const cv::Mat &T_c_w, double depth)
{
    return camera2world(pixel2camera(p_p, depth), T_c_w);
}

} // namespace robust_vslam
