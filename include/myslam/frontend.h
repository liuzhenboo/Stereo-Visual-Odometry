#pragma once
#ifndef MYSLAM_FRONTEND_H
#define MYSLAM_FRONTEND_H

#include <opencv2/features2d.hpp>
#include "myslam/ORBextractor.h"

//#include "myslam/visual_odometry.h"
#include "myslam/algorithm.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam
{
class VisualOdometry;
class Backend;
class Viewer;

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
   std::string orb_dir_;
   myslam::ORBextractor *mpORBextractorLeft;
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
   void Set_vo(VisualOdometry *vo);
   void SetMap(Map::Ptr map) { map_ = map; }
   void deleteUnmatchFeaturesCircle(std::vector<cv::Point2f> &points0, std::vector<cv::Point2f> &points1,
                                    std::vector<cv::Point2f> &points2, std::vector<cv::Point2f> &points3,
                                    std::vector<cv::Point2f> &points0_return,
                                    std::vector<uchar> &status0, std::vector<uchar> &status1,
                                    std::vector<uchar> &status2, std::vector<uchar> &status3);

   void SetBackend(std::shared_ptr<Backend> backend)
   {
      backend_ = backend;
   }
   bool StereoInit_f2f();
   void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }
   void SetStatus(FrontendStatus status) { status_ = status; }
   FrontendStatus GetStatus() const { return status_; }

   void SetCameras(Camera::Ptr left, Camera::Ptr right)
   {
      camera_left_ = left;
      camera_right_ = right;
   }
   // ransac 剔除外点
   int RANSAC(std::vector<std::shared_ptr<Feature>> &Features_1, std::vector<std::shared_ptr<Feature>> &Features_2);

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
   void display(cv::Mat &trajectory, double px, double pz);
   /**
     * 跟踪局部地图模式
     * @return true if success
     */
   bool F2LocalMap_Track();

   /**
     * 双目帧间跟踪，先三角化每一帧的三维点，然后利用ICP计算位姿变换.
     * 只考虑运动，不考虑结构（地图结构）
     * @return true if sucess
     */
   bool ICP_SVD_EstimateCurrentPose(const std::vector<cv::Point3f> &pts1, const std::vector<cv::Point3f> &pts2, Eigen::Matrix3d &R, Eigen::Vector3d &t);
   /**  
     * Reset when lost
     * @return true if success
     */
   bool Reset();

   /**
     * Track with last frame
     * @return num of tracked points
     */
   int TrackLastFrame();

   /**
     * estimate current frame's pose，简单的滑动窗口
     * @return num of inliers
     */
   int G2O_F2LocalMap_EstimateCurrentPose();

   /**
     * estimate current frame's pose
     * @return num of inliers
     */
   bool LK_StereoF2F_PnP_Track();
   bool StereoF2F_PnP_Track();

   /**
     * set current frame as a keyframe and insert it into backend
     * @return true if success
     */
   bool InsertKeyframe();

   /**
     * Try init the frontend with stereo images saved in current_frame_
     * @return true if success
     */
   bool StereoInit();

   /**
     * Detect features in left image in current_frame_
     * keypoints will be saved in current_frame_
     * @return
     */
   int DetectFeatures();

   /**
     * Find the corresponding features in right image of current_frame_
     * @return num of features found
     */
   int FindFeaturesInRight();

   /**
     * Build the initial map with single image
     * @return true if succeed
     */
   bool BuildInitMap();

   /**
     * Triangulate the 2D points in current frame
     * @return num of triangulated points
     */
   int TriangulateNewPoints();

   /**
     * Set the features in keyframe as new observation of the map points
     */
   void SetObservationsForKeyFrame();

   // data
   FrontendStatus status_ = FrontendStatus::INITING;

   //Frame::Ptr current_frame_ = nullptr;  // 当前帧
   Frame::Ptr last_frame_ = nullptr;    // 上一帧
   Camera::Ptr camera_left_ = nullptr;  // 左侧相机
   Camera::Ptr camera_right_ = nullptr; // 右侧相机

   VisualOdometry *vo_ = nullptr;
   Map::Ptr map_ = nullptr;
   std::shared_ptr<Backend> backend_ = nullptr;
   std::shared_ptr<Viewer> viewer_ = nullptr;

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

} // namespace myslam

#endif // MYSLAM_FRONTEND_H
