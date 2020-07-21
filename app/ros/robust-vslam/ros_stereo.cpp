//
// Created by Liu Zhenbo on 2020-6-30.
//
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core/core.hpp>
#include <gflags/gflags.h>
#include "lzb_vio/System.h"

DEFINE_string(config_file, "/home/lzb/Projects/lzb_vio/config/default.yaml", "config file path");

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    ros::init(argc, argv, "ros_stereo");
    ros::start();
    // 初始化slam系统，传入config文件地址
    lzb_vio::System::Ptr vo(
        new lzb_vio::System(FLAGS_config_file));
    assert(vo->Init_ros() == true);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/cam0/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/cam1/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.registerCallback(boost::bind(&Get_stereo_data, _1, _2));

    ros::spinOnce();
    vo.Shutdown();

    return 0;
}
// 回调
void Get_stereo_data(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight)
{
    // Copy the ros image message to cv::Mat.
    // 左图
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // 右图
    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat image_left, image_right;
    image_left = cv_ptrLeft->image;
    image_right = cv_ptrRight->image;
    if (image_left.data == nullptr || image_right.data == nullptr)
    {
        LOG(WARNING) << "cannot get image data from rostopic! " << current_image_index_;
        return nullptr;
    }

    cv::Mat image_left_resized, image_right_resized;
    cv::resize(image_left, image_left_resized, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);
    cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);

    auto new_frame1 = Frame::CreateFrame();
    new_frame1->left_img_ = image_left_resized;
    new_frame1->right_img_ = image_right_resized;

    Frame::Ptr new_frame = new_frame1;
    if (new_frame == nullptr)
        return false;

    auto t1 = std::chrono::steady_clock::now();
    std::cout << "--------------------" << std::endl;
    bool success = frontend_->AddFrame(new_frame);
    std::cout << "222222222222" << std::endl;

    auto t2 = std::chrono::steady_clock::now();
    auto time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
    if (!success)
        return;
}
