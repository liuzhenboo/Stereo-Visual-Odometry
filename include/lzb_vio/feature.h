#pragma once

#ifndef lzb_vio_FEATURE_H
#define lzb_vio_FEATURE_H

#include <memory>
#include <opencv2/features2d.hpp>
#include "lzb_vio/common_include.h"

namespace lzb_vio
{

struct Frame;
struct MapPoint;

struct Feature
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    std::weak_ptr<Frame> frame_;   // 持有该feature的frame
    cv::KeyPoint position_;        // 2D提取位置
    cv::Mat Descriptor_;           //描述子
    bool is_outlier_ = false;      // 是否为异常点
    bool is_on_left_image_ = true; // 标识是否提在左图，false为右图

public:
    Feature() {}

    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
        : frame_(frame), position_(kp) {}
};
} // namespace lzb_vio

#endif // lzb_vio_FEATURE_H
