#pragma once
#ifndef lzbslam_VISUAL_ODOMETRY_H
#define lzbslam_VISUAL_ODOMETRY_H

#include "lzbslam/backend.h"
#include "lzbslam/common_include.h"
#include "lzbslam/dataset.h"
#include "lzbslam/frontend.h"
#include "lzbslam/viewer.h"
#include "lzbslam/frame.h"

namespace lzbslam
{
/**
 * VO 对外接口
 */
class VisualOdometry
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    //typedef std::shared_ptr<VisualOdometry> Ptr;
    Frame::Ptr current_frame2_ = nullptr; // 当前帧

    /// constructor with config file
    VisualOdometry(std::string &config_path);

    /**
     * do initialization things before run
     * @return true if success
     */
    bool Init();

    /**
     * do initialization(ros datasets) things before run
     * @return true if success
     */
    bool Init_StereoRos();

    /**
     * start vo in the dataset
     */
    void Run();

    /**
     * Make a step forward in dataset
     */
    bool Step();
    bool Step_ros(Frame::Ptr new_frame);
    void Shutdown();
    void Reset();
    /// 获取前端状态
    FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }

private:
    bool inited_ = false;
    std::string config_file_path_;

    Frontend::Ptr frontend_ = nullptr;
    Backend::Ptr backend_ = nullptr;
    Map::Ptr map_ = nullptr;
    Viewer::Ptr viewer_ = nullptr;

    // dataset
    Dataset::Ptr dataset_ = nullptr;
};
} // namespace lzbslam

#endif // lzbslam_VISUAL_ODOMETRY_H
