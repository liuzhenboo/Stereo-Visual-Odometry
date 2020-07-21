#pragma once
#ifndef lzb_vio_System_H
#define lzb_vio_System_H

#include "lzb_vio/tracking.h"
#include "lzb_vio/common_include.h"
#include "lzb_vio/parameter.h"
#include "lzb_vio/frame.h"

namespace lzb_vio
{
class Tracking;

enum class TrackingStatus;
class Sensors;
class Parameter;

class System
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    //对外接口
    System(std::string &config_path);

    void Run();

    bool Step();
    bool Step_ros(Frame::Ptr new_frame);

private:
    TrackingStatus GetFrontendStatus() const { return tracking_->GetStatus(); }
    Frame::Ptr NextFrame_kitti();

    void Shutdown();
    void Reset();

    // system parameter
    std::string config_file_path_;

    Parameter::Ptr init_parameter_ = nullptr;
    Sensors::Ptr sensors_ = nullptr;
    Tracking::Ptr tracking_ = nullptr;

    int current_image_index_ = 0;
    bool inited_ = false;
    std::string dataset_path_;
};
} // namespace lzb_vio

#endif // lzb_vio_System_H
