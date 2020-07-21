// created by liuzhenbo in 2020/7/9

#pragma once
#ifndef lzb_vio_TRACKING_H
#define lzb_vio_TRACKING_H

#include <opencv2/features2d.hpp>
#include "lzb_vio/ORBextractor.h"

#include "lzb_vio/parameter.h"
#include "lzb_vio/sensors.h"
#include "lzb_vio/algorithm.h"
#include "lzb_vio/frame.h"
//#include "lzb_vio/System.h"
#include "lzb_vio/feature.h"

namespace lzb_vio
{
class System;
class Sensors;
class Parameter;
enum class TrackingStatus
{
  INITING,
  TRACKING_GOOD,
  LOST
};

class Tracking
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Tracking> Ptr;

  //接口函数
  Tracking(System *system, Parameter::Ptr parameter, Sensors::Ptr sensors);
  void Set_vo(System *vo);
  bool AddFrame(Frame::Ptr frame);
  TrackingStatus GetStatus() const { return status_; }

private:
  bool StereoInit_f2f();

  bool Track();

  bool Reset();
  void Readparameter();

  bool LK_StereoF2F_PnP_Track();
  bool ORB_StereoF2F_PnP_Track();

  int LK_Robust_Find_MuliImage_MatchedFeatures(
      std::vector<cv::Point2f> &points_t2_left,
      std::vector<cv::Point2f> &points_t2_right,
      std::vector<cv::Point2f> &points_t1_left,
      std::vector<cv::Point2f> &points_t1_right);
  int ORB_Robust_Find_MuliImage_MatchedFeatures(std::vector<cv::Point2f> &points_t1_left,
                                                std::vector<cv::Point2f> &points_t2_left,
                                                std::vector<cv::Point2f> &points_t2_right);

  bool OpenCV_Triangulation(
      const std::vector<cv::Point2f> &p_1,
      const std::vector<cv::Point2f> &p_2,
      const cv::Mat &t,
      const cv::Mat &K1,
      const cv::Mat &K2,
      std::vector<cv::Point3f> &points);
  bool Robust_Triangulation(
      const std::vector<cv::Point2f> &p_1,
      const std::vector<cv::Point2f> &p_2,
      const cv::Mat &t,
      const cv::Mat &K1,
      const cv::Mat &K2,
      std::vector<cv::Point3f> &points);

  bool OpenCV_EstimatePose_PnP(cv::Mat &projMatrl,
                               std::vector<cv::Point2f> &pointsLeft_t2,
                               cv::Mat &points3D_t0,
                               cv::Mat &rotation,
                               cv::Mat &translation);

  bool G2O_EstimatePose_PnP(cv::Mat &projMatrl,
                            std::vector<cv::Point2f> &pointsLeft_t2,
                            cv::Mat &points3D_t0,
                            cv::Mat &rotation,
                            cv::Mat &translation);

  int Detect_OpenCVFASTFeatures();
  int Detect_MyORBFeatures();

  void deleteBadmatchFeatures(std::vector<cv::Point2f> &points0, std::vector<cv::Point2f> &points1,
                              std::vector<cv::Point2f> &points2, std::vector<cv::Point2f> &points3,
                              std::vector<cv::Point2f> &points0_return,
                              std::vector<uchar> &status0, std::vector<uchar> &status1,
                              std::vector<uchar> &status2, std::vector<uchar> &status3);
  void displayTracking(cv::Mat &imageLeft_t1,
                       std::vector<cv::Point2f> &pointsLeft_t0,
                       std::vector<cv::Point2f> &pointsLeft_t1);
  void display(cv::Mat &trajectory, double p1, double p2);

  cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R);
  bool isRotationMatrix(cv::Mat &R);

  //跟踪状态
  TrackingStatus status_ = TrackingStatus::INITING;

  // 前后帧传感器数据
  Frame::Ptr current_frame_ = nullptr; // 当前帧
  Frame::Ptr last_frame_ = nullptr;    // 上一帧

  //数据共享
  Sensors::Ptr sensors_ = nullptr; // 传感器平台
  System *system_ = nullptr;
  Parameter::Ptr parameter_ = nullptr;

  //当前定位结果
  cv::Mat frame_pose_ = cv::Mat::eye(4, 4, CV_64F);
  cv::Mat rotation_ = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat trajectory_ = cv::Mat::zeros(600, 1200, CV_8UC3);
  double Px_ = 0;
  double Py_ = 0;
  double Pz_ = 0;

  // 特征检测与描述，匹配
  cv::Ptr<cv::GFTTDetector> gftt_;   // feature detector in opencv
  ORBextractor *mpORBextractorLeft_; // feature detector and descriptor from orbslam2

  // 以下变量用Parameter参数类来初始化
  // Readparameter()
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
  int GFTTDetector_num_ = 100;
  double display_scale_ = 1;
  int display_x_ = 1000;
  int display_y_ = 1000;
  double minmove_ = 0.01;
  double maxmove_ = 0.01;
  std::string track_mode_ = "stereoicp_f2f";
  int nFeatures_;
  float fScaleFactor_;
  int nLevels_;
  int fIniThFAST_;
  int fMinThFAST_;
};

} // namespace lzb_vio

#endif // lzb_vio_TRACKING_H
