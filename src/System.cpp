// created by liuzhenbo in 2020/7/9

#include "lzb_vio/System.h"
#include <chrono>

namespace lzb_vio
{

System::System(std::string &config_path)
    : config_file_path_(config_path)
{
    std::cout << std::endl
              << "robust-vslam, 2020-2021 LiuZhenbo，ChengChangwei , NWPU" << std::endl
              << std::endl;
    if (Config::SetParameterFile(config_file_path_) == false)
    {
        std::cerr << "unable to open xx.yaml file！" << std::endl;
        exit(-1);
    }
    else
    {
        std::cout << "be able to open xx.yaml file！" << std::endl;
    }

    init_parameter_ = Parameter::Ptr(new Parameter);
    sensors_ = Sensors::Ptr(new Sensors(init_parameter_)); //init_parameter_));
    tracking_ = Tracking::Ptr(new Tracking(this, init_parameter_, sensors_));

    dataset_path_ = init_parameter_->dataset_path_;
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
    Frame::Ptr new_frame = NextFrame_kitti();
    if (new_frame == nullptr)
        return false;

    auto t1 = std::chrono::steady_clock::now();
    bool success = tracking_->AddFrame(new_frame);
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

    bool success = tracking_->AddFrame(new_frame);

    auto t2 = std::chrono::steady_clock::now();
    auto time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
    return success;
}
Frame::Ptr System::NextFrame_kitti()
{
    boost::format fmt("%s/image_%d/%06d.png");
    cv::Mat image_left, image_right;
    // read images
    image_left =
        cv::imread((fmt % dataset_path_ % 0 % current_image_index_).str(),
                   cv::IMREAD_GRAYSCALE);
    image_right =
        cv::imread((fmt % dataset_path_ % 1 % current_image_index_).str(),
                   cv::IMREAD_GRAYSCALE);
    std::cout << (fmt % dataset_path_ % 1 % current_image_index_).str() << std::endl;
    if (image_left.data == nullptr || image_right.data == nullptr)
    {
        LOG(WARNING) << "cannot find images at index " << current_image_index_;
        return nullptr;
    }

    cv::Mat image_left_resized, image_right_resized;
    cv::resize(image_left, image_left_resized, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);
    cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);

    auto new_frame = Frame::CreateFrame();
    new_frame->left_img_ = image_left;
    new_frame->right_img_ = image_right;
    current_image_index_++;
    return new_frame;
}
void System::Shutdown()
{
}

// 跟踪失败之后重置系统。
// 先重置地图，后端优化，显示，再给跟踪线程初始化标志位。
void System::Reset()
{
}

} // namespace lzb_vio
