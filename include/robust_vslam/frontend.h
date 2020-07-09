#pragma once
#ifndef robust_vslam_FRONTEND_H
#define robust_vslam_FRONTEND_H

#include <opencv2/features2d.hpp>
#include "robust_vslam/ORBextractor.h"

#include "robust_vslam/algorithm.h"
#include "robust_vslam/frame.h"

namespace robust_vslam
{
class System;

enum class FrontendStatus
{
   INITING,
   TRACKING_GOOD,
   TRACKING_BAD,
   LOST
};

/**
 * 前端
 * 估计当前帧Pose，在满足关键帧条件时向地图加入关键帧并触发优化
 */
class Frontend
{
public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
   typedef std::shared_ptr<Frontend> Ptr;
   //bool Reset();
   Frame::Ptr current_frame_ = nullptr;  // 当前帧
   Frame::Ptr current_frame1_ = nullptr; // 当前帧
   Vec3 t_;
   Frontend();
   cv::Mat frame_pose_ = cv::Mat::eye(4, 4, CV_64F);

   int display_scale_ = 1;
   int display_x_ = 1000;
   int display_y_ = 1000;
   double minmove_ = 0.01;
   double maxmove_ = 0.01;
   std::string orb_dir_;
   robust_vslam::ORBextractor *mpORBextractorLeft;
   cv::Mat trajectory_ = cv::Mat::zeros(600, 1200, CV_8UC3);
   //cv::Mat translation_;
   double Px_ = 0;
   double Py_ = 0;
   double Pz_ = 0;
   //cv::Mat rotation_ = (cv::Mat_<float>(3, 3) << 1.0, 0., 0., 0., 1.0, 0., 0,  0., 1.0);
   cv::Mat rotation_ = cv::Mat::eye(3, 3, CV_64F);
   /// 外部接口，添加一个帧并计算其定位结果
   bool AddFrame(Frame::Ptr frame);
   void displayTracking(cv::Mat &imageLeft_t1,
                        std::vector<cv::Point2f> &pointsLeft_t0,
                        std::vector<cv::Point2f> &pointsLeft_t1);

   /// Set函数
   void Set_vo(System *vo);
   void deleteUnmatchFeaturesCircle(std::vector<cv::Point2f> &points0, std::vector<cv::Point2f> &points1,
                                    std::vector<cv::Point2f> &points2, std::vector<cv::Point2f> &points3,
                                    std::vector<cv::Point2f> &points0_return,
                                    std::vector<uchar> &status0, std::vector<uchar> &status1,
                                    std::vector<uchar> &status2, std::vector<uchar> &status3);

   bool StereoInit_f2f();
   FrontendStatus GetStatus() const { return status_; }

   void SetCameras(Camera::Ptr left, Camera::Ptr right)
   {
      camera_left_ = left;
      camera_right_ = right;
   }

   int LK_Robust_Find_MuliImage_MatchedFeatures(
       std::vector<cv::Point2f> &points_t2_left,
       std::vector<cv::Point2f> &points_t2_right,
       std::vector<cv::Point2f> &points_t1_left,
       std::vector<cv::Point2f> &points_t1_right);
   int ORB_Robust_Find_MuliImage_MatchedFeatures(std::vector<cv::Point2f> &points_t1_left,
                                                 std::vector<cv::Point2f> &points_t2_left,
                                                 std::vector<cv::Point2f> &points_t2_right);

   bool triangulation_opencv(
       const std::vector<cv::Point2f> &p_1,
       const std::vector<cv::Point2f> &p_2,
       const cv::Mat &t,
       const cv::Mat &K1,
       const cv::Mat &K2,
       std::vector<cv::Point3f> &points);

   bool opencv_EstimatePose_PnP(cv::Mat &projMatrl,
                                std::vector<cv::Point2f> &pointsLeft_t2,
                                cv::Mat &points3D_t0,
                                cv::Mat &rotation,
                                cv::Mat &translation);

   bool slambook_EstimatePose_PnP(cv::Mat &projMatrl,
                                  std::vector<cv::Point2f> &pointsLeft_t2,
                                  cv::Mat &points3D_t0,
                                  cv::Mat &rotation,
                                  cv::Mat &translation);

   cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R);
   bool isRotationMatrix(cv::Mat &R);

private:
   /**
     * Track inference
     * @return true if success
     */
   bool Track();

   /**  
     * Reset when lost
     * @return true if success
     */
   bool Reset();

   /**
     * estimate current frame's pose
     * @return num of inliers
     */
   bool LK_StereoF2F_PnP_Track();
   bool ORB_StereoF2F_PnP_Track();

   /**
     * Detect features in left image in current_frame_
     * keypoints will be saved in current_frame_
     * @return
     */
   int DetectFeatures();

   // data
   FrontendStatus status_ = FrontendStatus::INITING;

   //Frame::Ptr current_frame_ = nullptr;  // 当前帧
   Frame::Ptr last_frame_ = nullptr;    // 上一帧
   Camera::Ptr camera_left_ = nullptr;  // 左侧相机
   Camera::Ptr camera_right_ = nullptr; // 右侧相机

   System *vo_ = nullptr;

   SE3 relative_motion_; // 当前帧与上一帧的相对运动，用于估计当前帧pose初值

   int tracking_inliers_ = 0; // inliers, used for testing new keyframes

   // params
   int num_features_ = 200;
   int num_features_init_ = 100;
   int num_features_tracking_ = 50;
   int num_features_tracking_bad_ = 20;
   int num_features_needed_for_keyframe_ = 80;
   int init_landmarks_ = 5;
   double feature_match_error_ = 10;
   double inlier_rate_ = 0.5;
   int iterationsCount_ = 500;
   float reprojectionError_ = 0.5;
   float confidence_ = 0.999;
   // utilities
   cv::Ptr<cv::GFTTDetector> gftt_; // feature detector in opencv

   // tracking mode
   std::string track_mode_ = "stereoicp_f2f";
};

} // namespace robust_vslam

#endif // robust_vslam_FRONTEND_H
