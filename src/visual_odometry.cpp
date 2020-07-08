#include <chrono>
#include "myslam/config.h"
#include "myslam/frontend.h"
#include "myslam/visual_odometry.h"

namespace myslam
{

VisualOdometry::VisualOdometry(std::string &config_path)
    : config_file_path_(config_path) {}

bool VisualOdometry::Init()
{
    // read from config file
    if (Config::SetParameterFile(config_file_path_) == false)
    {
        return false;
    }

    dataset_ =
        Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir_kitti")));
    // CHECK_EQ 用来判断两个值是否相等；Init()读内外参到dataset_中
    CHECK_EQ(dataset_->Init(), true);

    frontend_ = Frontend::Ptr(new Frontend);
    backend_ = Backend::Ptr(new Backend);
    map_ = Map::Ptr(new Map);
    viewer_ = Viewer::Ptr(new Viewer);

    frontend_->SetBackend(backend_);
    frontend_->SetMap(map_);
    frontend_->SetViewer(viewer_);
    frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    backend_->SetMap(map_);
    backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    viewer_->SetMap(map_);

    return true;
}
bool VisualOdometry::Init_StereoRos()
{

    // read from config file
    if (Config::SetParameterFile(config_file_path_) == false)
    {
        std::cout << "不能打开配置文件！" << std::endl;
        return false;
    }
    else
    {
        std::cout << "可以打开配置文件，准备初始化类！" << std::endl;
    }

    dataset_ =
        Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir_kitti")));

    CHECK_EQ(dataset_->Stereo_Init(), true);

    // create components（组件） and links（相互访问的指针）
    frontend_ = Frontend::Ptr(new Frontend);
    backend_ = Backend::Ptr(new Backend);
    map_ = Map::Ptr(new Map);
    viewer_ = Viewer::Ptr(new Viewer);

    frontend_->Set_vo(this);
    frontend_->SetBackend(backend_);
    frontend_->SetMap(map_);
    frontend_->SetViewer(viewer_);
    frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    backend_->SetMap(map_);
    backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    viewer_->SetMap(map_);

    return true;
}
void VisualOdometry::Run()
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
bool VisualOdometry::Step()
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
bool VisualOdometry::Step_ros(Frame::Ptr new_frame)
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
void VisualOdometry::Shutdown()
{
    backend_->Stop();
    viewer_->Close();
    LOG(INFO) << "VO exit";
}

// 跟踪失败之后重置系统。
// 先重置地图，后端优化，显示，再给跟踪线程初始化标志位。
void VisualOdometry::Reset()
{
    map_->Reset_Map();
    backend_->Reset();
    viewer_->Reset();
    frontend_->SetStatus(FrontendStatus::INITING);
}

} // namespace myslam
