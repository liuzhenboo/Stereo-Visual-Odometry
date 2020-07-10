#pragma once
#ifndef robust_vslam_System_H
#define robust_vslam_System_H

#include "robust_vslam/tracking.h"
#include "robust_vslam/common_include.h"
#include "robust_vslam/parameter.h"
#include "robust_vslam/frame.h"

namespace robust_vslam
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
} // namespace robust_vslam

#endif // robust_vslam_System_H
