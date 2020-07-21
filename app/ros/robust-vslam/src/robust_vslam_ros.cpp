//
// Created by Liu Zhenbo on 2020-6-30.
//
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core/core.hpp>
#include "lzb_vio/System.h"
#include "lzb_vio/frame.h"
using namespace std;
class ImageGrabber
{
public:
    ImageGrabber(lzb_vio::System *pSLAM) : mp_slam(pSLAM) {}

    void Get_stereo_data(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight);

    lzb_vio::System *mp_slam;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_stereo");
    ros::start();
    std::string config_file_path = argv[1];

    // 初始化slam系统，传入config文件地址
    lzb_vio::System *slam(
        new lzb_vio::System(config_file_path));
    ImageGrabber igb(slam);

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

    // cv::Mat image_left_resized, image_right_resized;
    // cv::resize(image_left, image_left_resized, cv::Size(), 0.6, 0.6,
    //            cv::INTER_NEAREST);
    // cv::resize(image_right, image_right_resized, cv::Size(), 0.6, 0.6,
    //            cv::INTER_NEAREST);

    lzb_vio::Frame::Ptr new_frame = lzb_vio::Frame::CreateFrame(); //lzb_vio::Frame::CreateFrame();
    new_frame->left_img_ = image_left;
    new_frame->right_img_ = image_right;
    mp_slam->Step_ros(new_frame);
}
