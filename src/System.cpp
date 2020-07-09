#include <chrono>
#include "robust_vslam/config.h"
#include "robust_vslam/frontend.h"
#include "robust_vslam/System.h"

namespace robust_vslam
{

System::System(std::string &config_path)
    : config_file_path_(config_path)
{
    std::cout << std::endl
              << "robust-vslam, 2020-2021 liuzhenbo , NWPU" << std::endl
              << std::endl;
    if (Config::SetParameterFile(config_file_path_) == false)
    {
        std::cerr << "unable to open xx.yaml file！" << std::endl;
        return false;
    }
    else
    {
        std::cout << "be able to open xx.yaml file！" << std::endl;
    }

    init_parameter_ = Parameter::Ptr(new Parameter);
    sensors_ = Sensors::Ptr(new Sensors(init_parameter_));
    frontend_ = Frontend::Ptr(new Frontend(this, init_parameter_, sensors_));

    frontend_->Set_vo(this);
    frontend_->SetSensorss(camera_);
    return true;
}
void System::Run()
{
    while (1)
    {
        LOG(INFO) << "VO is running";
        // 主循环
        if (Step() == false)
        {
            break;
        }
    }
    Shutdown();
}

// 主程序
bool System::Step()
{
    Frame::Ptr new_frame = dataset_->NextFrame();
    if (new_frame == nullptr)
        return false;

    auto t1 = std::chrono::steady_clock::now();
    bool success = frontend_->AddFrame(new_frame);
    auto t2 = std::chrono::steady_clock::now();
    auto time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
    return 1;
}
bool System::Step_ros(Frame::Ptr new_frame)
{

    if (new_frame == nullptr)
        return false;
    auto t1 = std::chrono::steady_clock::now();

    bool success = frontend_->AddFrame(new_frame);

    auto t2 = std::chrono::steady_clock::now();
    auto time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
    return success;
}
void System::Shutdown()
{
    backend_->Stop();
    viewer_->Close();
    LOG(INFO) << "VO exit";
}

// 跟踪失败之后重置系统。
// 先重置地图，后端优化，显示，再给跟踪线程初始化标志位。
void System::Reset()
{
    map_->Reset_Map();
    backend_->Reset();
    viewer_->Reset();
    frontend_->SetStatus(FrontendStatus::INITING);
}

} // namespace robust_vslam
