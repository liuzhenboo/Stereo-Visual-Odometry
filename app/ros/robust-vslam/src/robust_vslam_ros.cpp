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
#include "myslam/visual_odometry.h"
#include "myslam/frame.h"
using namespace std;
DEFINE_string(config_file, "/home/lzb/Projects/robust-vslam/config/default.yaml", "config file path");
//void Get_stereo_data(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);
class ImageGrabber
{
public:
    ImageGrabber(myslam::VisualOdometry *pSLAM) : mp_vo(pSLAM) {}

    void Get_stereo_data(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight);

    myslam::VisualOdometry *mp_vo;
};

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    ros::init(argc, argv, "ros_stereo");
    ros::start();
    // 初始化slam系统，传入config文件地址
    myslam::VisualOdometry *vo(
        new myslam::VisualOdometry(FLAGS_config_file));
    vo->Init_StereoRos();
    //assert(vo->Init_StereoRos() == true);

    ImageGrabber igb(vo);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/zed/zed_node/left/image_rect_gray", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/zed/zed_node/right/image_rect_gray", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    //std::cout << "hello" <<std::endl;

    sync.registerCallback(boost::bind(&ImageGrabber::Get_stereo_data, &igb, _1, _2));

    ros::spin();
    //vo->Shutdown();

    return 0;
}
// 回调
void ImageGrabber::Get_stereo_data(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight)
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
        LOG(WARNING) << "cannot get image data from rostopic! ";
        return;
    }

    cv::Mat image_left_resized, image_right_resized;
    cv::resize(image_left, image_left_resized, cv::Size(), 0.6, 0.6,
               cv::INTER_NEAREST);
    cv::resize(image_right, image_right_resized, cv::Size(), 0.6, 0.6,
               cv::INTER_NEAREST);

    myslam::Frame::Ptr new_frame1 = myslam::Frame::CreateFrame(); //myslam::Frame::CreateFrame();
    new_frame1->left_img_ = image_left;
    new_frame1->right_img_ = image_right;

    if (!(mp_vo->Step_ros(new_frame1)))
    {
        std::cout << "return" << std::endl;
        return;
    }
}
