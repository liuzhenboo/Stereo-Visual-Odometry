#pragma once
#ifndef robust_vslam_System_H
#define robust_vslam_System_H

#include "robust_vslam/backend.h"
#include "robust_vslam/common_include.h"
#include "robust_vslam/dataset.h"
#include "robust_vslam/frontend.h"
#include "robust_vslam/viewer.h"
#include "robust_vslam/frame.h"

namespace robust_vslam
{
class System
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /// constructor with config file
    System(std::string &config_path);
    /**
     * 系统初始化 
     * @return true if success
     */
    bool Init_System();

    /**
     * 启动系统
     */
    void Run();

    /**
     * 读取新的一帧
     */
    bool Step();

    /**
    * ROS接口
    */
    bool Step_ros(Frame::Ptr new_frame);

    /**
    * 结束系统  
    */
    void Shutdown();

    /**
    * 重置系统
    */
    void Reset();

    /**
     * 获取前端状态
    */
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
} // namespace robust_vslam

#endif // robust_vslam_System_H
