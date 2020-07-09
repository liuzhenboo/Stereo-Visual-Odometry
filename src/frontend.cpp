// created by liuzhenbo in 2020/7/9

#include <opencv2/opencv.hpp>
#include "robust_vslam/algorithm.h"
#include "robust_vslam/backend.h"
#include "robust_vslam/config.h"
#include "robust_vslam/feature.h"
#include "robust_vslam/System.h"

#include "robust_vslam/g2o_types.h"
#include "robust_vslam/map.h"
#include "robust_vslam/viewer.h"
#include "robust_vslam/frontend.h"

namespace robust_vslam
{

Frontend::Frontend(System *system, Parameter::Ptr parameter, Sensors::Ptr sensors)
{

    sensors_ = sensors;
    system_ = system;
    parameter_ = parameter_;
    Readparameter();
    //----------------------------------------------------------------------------------
    gftt_ = cv::GFTTDetector::create(GFTTDetector_num_, 0.01, 20);

    mpORBextractorLeft_ = new ORBextractor(nFeatures_, fScaleFactor_, nLevels_, fIniThFAST_, fMinThFAST_);
}
void Frontend::Readparameter()
{
    num_features_init_ = parameter_->num_features_init_;
    num_features_ = parameter_->num_features_;
    num_features_tracking_bad_ = parameter_->num_features_tracking_bad_;
    num_features_needed_for_keyframe_ = parameter_->num_features_needed_for_keyframe_;
    init_landmarks_ = parameter_->init_landmarks_;
    feature_match_error_ = parameter_->feature_match_error_;
    track_mode_ = parameter_->track_mode_;
    num_features_tracking_ = parameter_->num_features_tracking_;
    inlier_rate_ = parameter_->inlier_rate_;
    iterationsCount_ = parameter_->iterationsCount_;
    reprojectionError_ = parameter_->reprojectionError_;
    confidence_ = parameter_->confidence_;
    display_scale_ = parameter_->display_scale_;
    display_x_ = parameter_->display_x_;
    display_y_ = parameter_->display_y_;
    maxmove_ = parameter_->maxmove_;
    minmove_ = parameter_->minmove_;
    GFTTDetector_num_ = parameter_->GFTTDetector_num_;
    nFeatures_ = parameter_->nFeatures_;
    fScaleFactor_ = parameter_->fScaleFactor_;
    nLevels_ = parameter_->nLevels_;
    fIniThFAST_ = parameter_->fIniThFAST_;
    fMinThFAST_ = parameter_->fMinThFAST_;
}
// 向系统输入新的图片数据
bool Frontend::AddFrame(robust_vslam::Frame::Ptr frame)
{
    current_frame_ = frame;

    switch (status_)
    {
    case FrontendStatus::INITING:
        StereoInit_f2f();
        break;
    case FrontendStatus::TRACKING_GOOD:
    case FrontendStatus::TRACKING_BAD:
        if (Track())
        {
            last_frame_ = current_frame_;
            return true;
        }
        else
        {
            last_frame_ = current_frame_;
            return false;
        } //{Reset();}
        break;
    case FrontendStatus::LOST:
        //Reset();
        break;
    }

    //last_frame_ = current_frame_;
    return true;
}
bool Frontend::StereoInit_f2f()
{
    int num_features_left = DetectFeatures();
    // 设置初始位姿
    Vec3 t;
    t << 0.0, 0.0, 0.0;
    current_frame_->SetPose(SE3(SO3(), t));
    std::cout << "初始位置：" << t << std::endl;

    last_frame_ = current_frame_;
    status_ = FrontendStatus::TRACKING_GOOD;
    return true;
}

bool Frontend::StereoInit()
{
    //std::cout << "num_features_left" << std::endl;

    int num_features_left = DetectFeatures();
    //std::cout << "num_features_left" << std::endl;
    int num_coor_features = FindFeaturesInRight();
    //int inliers = CV_RANSAC();

    if (num_coor_features < num_features_init_)
    {
        LOG(INFO) << "初始化地图时，左右图像匹配点较少...Try Again!";
        return false;
    }

    bool build_map_success = BuildInitMap();
    if (build_map_success)
    {
        status_ = FrontendStatus::TRACKING_GOOD;
        std::cout << "双目初始化成功，开始跟踪！！" << std::endl;
        if (viewer_)
        {
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }
        return true;
    }
    return false;
}

int Frontend::DetectFeatures()
{
    /*cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
    for (auto &feat : current_frame_->features_left_)
    {
        cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                      feat->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
    }

    std::vector<cv::KeyPoint> keypoints;
    gftt_->detect(current_frame_->left_img_, keypoints, mask);

    int cnt_detected = 0;
    for (auto &kp : keypoints)
    {
        current_frame_->features_left_.push_back(
            Feature::Ptr(new Feature(current_frame_, kp)));
        cnt_detected++;
    }

    LOG(INFO) << "Detect " << cnt_detected << " new features";
    return cnt_detected;*/
    //------------------------------------------------------------------fast feature
    int cnt_detected = 0;

    std::vector<cv::KeyPoint> keypoints;
    int fast_threshold = 20;
    bool nonmaxSuppression = true;
    cv::FAST(current_frame_->left_img_, keypoints, fast_threshold, nonmaxSuppression);
    std::vector<cv::Point2f> points;
    //cv::KeyPoint::convert(keypoints, points, std::vector<int>());
    for (auto &kp : keypoints)
    {
        current_frame_->features_left_.push_back(
            Feature::Ptr(new Feature(current_frame_, kp)));
        cnt_detected++;
    }

    LOG(INFO) << "Detect " << cnt_detected << " new features";
    return cnt_detected;
}

bool Frontend::Track()
{
    if (track_mode_ == "LK_stereof2f_pnp")
    {
        std::cout << "跟踪模式为：LK_stereof2f_pnp" << std::endl;
        return LK_StereoF2F_PnP_Track();
    }
    else if (track_mode_ == "ORB_stereof2f_pnp")
    {
        std::cout << "跟踪模式为：ORB_stereof2f_pnp" << std::endl;
        return ORB_StereoF2F_PnP_Track();
    }
}
bool Frontend::OpenCV_Triangulation(
    const std::vector<cv::Point2f> &p_1,
    const std::vector<cv::Point2f> &p_2,
    const cv::Mat &t,
    const cv::Mat &K1,
    const cv::Mat &K2,
    std::vector<cv::Point3f> &points)
{
    cv::Mat T1 = (cv::Mat_<float>(3, 4) << 1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1, 0);
    cv::Mat T2 = (cv::Mat_<float>(3, 4) << 1, 0, 0, -0.12,
                  0, 1, 0, 0,
                  0, 0, 1, 0);
    std::vector<cv::Point2f> pts_1, pts_2;
    for (size_t i = 0; i < p_1.size(); i++)
    {
        // 将像素坐标转换至相机坐标
        pts_1.push_back(cv::Point2f((p_1[i].x - K1.at<double>(0, 2)) / K1.at<double>(0, 0), (p_1[i].y - K1.at<double>(1, 2)) / K1.at<double>(1, 1)));
        pts_2.push_back(cv::Point2f((p_2[i].x - K2.at<double>(0, 2)) / K2.at<double>(0, 0), (p_2[i].y - K2.at<double>(1, 2)) / K2.at<double>(1, 1)));
    }
    cv::Mat pts_4d;

    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);
    //std::cout << pts_4d << std::endl;

    // 转换成非齐次坐标
    for (int i = 0; i < pts_4d.cols; i++)
    {
        cv::Mat x = pts_4d.col(i);
        x /= x.at<float>(3, 0); // 归一化
        cv::Point3d p(
            x.at<float>(0, 0),
            x.at<float>(1, 0),
            x.at<float>(2, 0));
        points.push_back(p);
    }
    return true;
}
bool Frontend::ORB_StereoF2F_PnP_Track()
{
    int num_features_left = DetectFeatures();

    // 光流跟踪前后帧
    std::vector<cv::Point2f> matched_t1_left; // 光流跟踪到的像素点
    std::vector<cv::Point2f> matched_t2_left;
    std::vector<cv::Point2f> matched_t2_right;
    std::vector<cv::Point2f> matched_t1_right;
    //std::cout << "LK_Robust_Find_MuliImage_MatchedFeatures" << std::endl;
    int num_f2f_trackedfeatures = ORB_Robust_Find_MuliImage_MatchedFeatures(matched_t2_left, matched_t1_left, matched_t1_right);
    if (num_f2f_trackedfeatures < num_features_tracking_)
    {
        std::cout << "前后帧跟踪orb点少，忽略此次估计值！" << std::endl;
        return false;
    }
    cv::Mat points3D_t0, points4D_t0;
    cv::Mat projMatrl = camera_left_->projMatr_;
    cv::Mat projMatrr = camera_right_->projMatr_;

    cv::triangulatePoints(projMatrl, projMatrr, matched_t1_left, matched_t1_right, points4D_t0);

    cv::convertPointsFromHomogeneous(points4D_t0.t(), points3D_t0);

    //---------------------------
    //PnP估计姿态；3D->2D
    //-----------------------------
    cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat translation = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat delta_translation = cv::Mat::zeros(3, 1, CV_64F);
    bool valid_pose = G2O_EstimatePose_PnP(projMatrl, matched_t2_left, points3D_t0, rotation, translation);
    if (!valid_pose)
        return false;
    std::cout << "f2f rotation:" << std::endl
              << rotation << std::endl;
    std::cout << "f2f translation:" << std::endl
              << translation << std::endl;

    //显示轨迹
    displayTracking(current_frame_->left_img_, matched_t1_left, matched_t2_left);

    // 5cm,100cm，保证帧之间的运动合适
    cv::Vec3f rotation_euler = rotationMatrixToEulerAngles(rotation);
    if (abs(rotation_euler[1]) < 0.1 && abs(rotation_euler[0]) < 0.1 && abs(rotation_euler[2]) < 0.1)
    {
        double pose_norm2 = (std::pow(translation.at<double>(0), 2) + std::pow(translation.at<double>(1), 2) + std::pow(translation.at<double>(2), 2));
        if (pose_norm2 < maxmove_ * maxmove_ && pose_norm2 > minmove_ * minmove_)
        {
            cv::Mat addup = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);

            cv::Mat rigid_body_transformation;
            cv::hconcat(rotation, translation, rigid_body_transformation);
            cv::vconcat(rigid_body_transformation, addup, rigid_body_transformation);
            rigid_body_transformation = rigid_body_transformation.inv();
            frame_pose_ = frame_pose_ * rigid_body_transformation;
        }
        else
        {
            return false;
        }
    }
    else
    {

        return false;
    }
    cv::Mat xyz = frame_pose_.col(3).clone();
    Px_ = xyz.at<double>(0);
    Pz_ = xyz.at<double>(2);
    Py_ = xyz.at<double>(1);

    std::cout << "global translation:" << std::endl
              << Px_ << ", " << Py_ << ", " << Pz_ << std::endl;
    display(trajectory_, Px_, Pz_);
    return true;
}

bool Frontend::LK_StereoF2F_PnP_Track()
{
    int num_features_left = DetectFeatures();
    if (num_features_left < 30)
        return false;

    std::vector<cv::Point2f> matched_t2_left, matched_t1_left, matched_t2_right, matched_t1_right;
    for (auto &feature : last_frame_->features_left_)
    {
        matched_t1_left.push_back(feature->position_.pt);
    }

    //std::cout << "LK_Robust_Find_MuliImage_MatchedFeatures" << std::endl;
    int num_f2f_trackedfeatures = LK_Robust_Find_MuliImage_MatchedFeatures(matched_t2_left, matched_t2_right, matched_t1_left, matched_t1_right);
    if (num_f2f_trackedfeatures < num_features_tracking_)
    {
        std::cout << "跟踪点较少：" << num_f2f_trackedfeatures << std::endl;
        //last_frame_ = current_frame_;
        return false;
    }
    else
    {
        std::cout << "跟踪了" << num_f2f_trackedfeatures << "个点" << std::endl;
    }
    cv::Mat points3D_t0, points4D_t0;
    cv::Mat projMatrl = camera_left_->projMatr_;
    cv::Mat projMatrr = camera_right_->projMatr_;
    cv::triangulatePoints(projMatrl, projMatrr, matched_t1_left, matched_t1_right, points4D_t0);
    //std::cout << "triangulatePoints" << std::endl;

    cv::convertPointsFromHomogeneous(points4D_t0.t(), points3D_t0);
    //std::cout << "3D点坐标：" << std::endl << points3D_t0 << std::endl;

    //---------------------------
    //PnP估计姿态；3D->2D
    //-----------------------------
    cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat translation = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat delta_translation = cv::Mat::zeros(3, 1, CV_64F);
    bool enough_inlier = OpenCV_EstimatePose_PnP(projMatrl, matched_t2_left, points3D_t0, rotation, translation);
    if (!enough_inlier)
    {
        return false;
    }
    std::cout << "f2f rotation:" << std::endl
              << rotation << std::endl;
    std::cout << "f2f translation:" << std::endl
              << translation << std::endl;
    displayTracking(current_frame_->left_img_, matched_t1_left, matched_t2_left);

    // 5cm,100cm，保证帧之间的运动合适
    cv::Vec3f rotation_euler = rotationMatrixToEulerAngles(rotation);

    if (abs(rotation_euler[1]) < 0.1 && abs(rotation_euler[0]) < 0.1 && abs(rotation_euler[2]) < 0.1)
    //if (1)
    {
        double pose_norm2 = (std::pow(translation.at<double>(0), 2) + std::pow(translation.at<double>(1), 2) + std::pow(translation.at<double>(2), 2));
        if (pose_norm2 < 100 && pose_norm2 > 0.0005 * 0.0005)
        {
            cv::Mat addup = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);

            cv::Mat rigid_body_transformation;
            cv::hconcat(rotation, translation, rigid_body_transformation);
            cv::vconcat(rigid_body_transformation, addup, rigid_body_transformation);
            rigid_body_transformation = rigid_body_transformation.inv();
            frame_pose_ = frame_pose_ * rigid_body_transformation;
        }
        else
        {
            return false;
        }
    }
    else
    {

        return false;
    }
    cv::Mat xyz = frame_pose_.col(3).clone();
    Px_ = xyz.at<double>(0);
    Pz_ = xyz.at<double>(2);
    Py_ = xyz.at<double>(1);

    std::cout << "global translation:" << std::endl
              << Px_ << ", " << Py_ << ", " << Pz_ << std::endl;
    display(trajectory_, Px_, Pz_);
    return true;
} // namespace robust_vslam

void Frontend::displayTracking(cv::Mat &imageLeft_t1,
                               std::vector<cv::Point2f> &pointsLeft_t0,
                               std::vector<cv::Point2f> &pointsLeft_t1)
{
    // -----------------------------------------
    // Display feature racking
    // -----------------------------------------
    int radius = 2;
    cv::Mat vis;

    cv::cvtColor(imageLeft_t1, vis, cv::COLOR_GRAY2BGR, 3);

    for (int i = 0; i < pointsLeft_t0.size(); i++)
    {
        cv::circle(vis, cv::Point(pointsLeft_t0[i].x, pointsLeft_t0[i].y), radius, CV_RGB(0, 255, 0));
    }

    for (int i = 0; i < pointsLeft_t1.size(); i++)
    {
        cv::circle(vis, cv::Point(pointsLeft_t1[i].x, pointsLeft_t1[i].y), radius, CV_RGB(255, 0, 0));
    }

    for (int i = 0; i < pointsLeft_t1.size(); i++)
    {
        cv::line(vis, pointsLeft_t0[i], pointsLeft_t1[i], CV_RGB(0, 255, 0));
    }

    cv::imshow("robust_vslam ", vis);
}

bool Frontend::G2O_EstimatePose_PnP(cv::Mat &projMatrl,
                                    std::vector<cv::Point2f> &pointsLeft_t2,
                                    cv::Mat &points3D_t0,
                                    cv::Mat &rotation,
                                    cv::Mat &translation)
{

    // ------------------------------------------------
    // Translation (t) estimation by use solvePnPRansac
    // ------------------------------------------------
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat intrinsic_matrix = (cv::Mat_<float>(3, 3) << projMatrl.at<float>(0, 0), projMatrl.at<float>(0, 1), projMatrl.at<float>(0, 2),
                                projMatrl.at<float>(1, 0), projMatrl.at<float>(1, 1), projMatrl.at<float>(1, 2),
                                projMatrl.at<float>(1, 1), projMatrl.at<float>(1, 2), projMatrl.at<float>(1, 3));

    int iterationsCount = iterationsCount_;       // number of Ransac iterations.
    float reprojectionError = reprojectionError_; // maximum allowed distance to consider it an inlier.
    float confidence = confidence_;               // RANSAC successful confidence.
    bool useExtrinsicGuess = true;
    int flags = cv::SOLVEPNP_ITERATIVE;

    cv::Mat inliers;
    //cv::solvePnP(points3D_t0, pointsLeft_t2, intrinsic_matrix, Mat(), rvec, translation, false);
    cv::solvePnPRansac(points3D_t0, pointsLeft_t2, intrinsic_matrix, distCoeffs, rvec, translation,
                       useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
                       inliers, flags);
    cv::Rodrigues(rvec, rotation);
    //std::cout << "三角化结果：" << points3D_t0 << std::endl;
    std::cout << "其中估计位姿的内点数量为: " << inliers.size[0] << std::endl;
    //std::cout << "inliers.size()" << inliers.size[0] << ", " << inliers.size[1] << ", " << inliers.size[2] << std::endl;
    if ((double)inliers.size[0] / (double)pointsLeft_t2.size() < inlier_rate_)
    {
        std::cout << "inliers / tracking point = " << (double)inliers.size[0] / (double)pointsLeft_t2.size() << ",  姿态估计时候内点过少!" << std::endl;
        last_frame_ = current_frame_;
        return false;
    }
    else
    {
        std::cout << "inliers / tracking point: " << (double)inliers.size[0] / (double)pointsLeft_t2.size() << std::endl;
        return true;
    }
}

bool Frontend::isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

    return norm(I, shouldBeIdentity) < 1e-6;
}
// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
cv::Vec3f Frontend::rotationMatrixToEulerAngles(cv::Mat &R)
{

    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = atan2(-R.at<double>(2, 0), sy);
        z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    }
    else
    {
        x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);
}
bool Frontend::OpenCV_EstimatePose_PnP(cv::Mat &projMatrl,
                                       std::vector<cv::Point2f> &pointsLeft_t2,
                                       cv::Mat &points3D_t0,
                                       cv::Mat &rotation,
                                       cv::Mat &translation)
{

    // ------------------------------------------------
    // Translation (t) estimation by use solvePnPRansac
    // ------------------------------------------------
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat intrinsic_matrix = (cv::Mat_<float>(3, 3) << projMatrl.at<float>(0, 0), projMatrl.at<float>(0, 1), projMatrl.at<float>(0, 2), projMatrl.at<float>(1, 0), projMatrl.at<float>(1, 1), projMatrl.at<float>(1, 2),
                                projMatrl.at<float>(2, 0), projMatrl.at<float>(2, 1), projMatrl.at<float>(2, 2));

    int iterationsCount = iterationsCount_;       // number of Ransac iterations.
    float reprojectionError = reprojectionError_; // maximum allowed distance to consider it an inlier.
    float confidence = confidence_;               // RANSAC successful confidence.
    bool useExtrinsicGuess = true;
    int flags = cv::SOLVEPNP_ITERATIVE;

    cv::Mat inliers;
    cv::solvePnPRansac(points3D_t0, pointsLeft_t2, intrinsic_matrix, distCoeffs, rvec, translation,
                       useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
                       inliers, flags);
    cv::Rodrigues(rvec, rotation);
    std::cout << "其中估计位姿的内点数量为: " << inliers.size[0] << std::endl;
    //std::cout << "inliers.size()" << inliers.size[0] << ", " << inliers.size[1] << ", " << inliers.size[2] << std::endl;
    if ((double)inliers.size[0] / (double)pointsLeft_t2.size() < inlier_rate_)
    {
        std::cout << "inliers / tracking point = " << (double)inliers.size[0] / (double)pointsLeft_t2.size() << ",  姿态估计时候内点过少！，跟踪失败" << std::endl;
        return false;
    }
    else
    {
        std::cout << "inliers / tracking point: " << (double)inliers.size[0] / (double)pointsLeft_t2.size() << std::endl;
        return true;
    }
}

int Frontend::ORB_Robust_Find_MuliImage_MatchedFeatures(std::vector<cv::Point2f> &points_t2_left,
                                                        std::vector<cv::Point2f> &points_t1_left,
                                                        std::vector<cv::Point2f> &points_t1_right)
{
    // 当前帧图像中提取的特征点集合
    std::vector<cv::KeyPoint> mvKeys1, mvKeys2, mvKeys3;
    // 特征点对应的描述子

    cv::Mat mDescriptors1, mDescriptors2, mDescriptors3;
    (*mpORBextractorLeft)(last_frame_->left_img_, cv::Mat(), mvKeys1, mDescriptors1);
    (*mpORBextractorLeft)(last_frame_->right_img_, cv::Mat(), mvKeys2, mDescriptors2);
    (*mpORBextractorLeft)(current_frame_->left_img_, cv::Mat(), mvKeys3, mDescriptors3);
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    std::vector<cv::DMatch> match1, match2;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match(mDescriptors1, mDescriptors2, match1);
    matcher->match(mDescriptors1, mDescriptors3, match2);
    int des_index = std::min(std::min(mDescriptors2.rows, mDescriptors3.rows), mDescriptors1.rows);
    double min_dist = 10000,
           max_dist = 0;

    for (int i = 0; i < des_index; i++)
    {
        double dist = std::max(match1[i].distance, match2[i].distance);
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < des_index; i++)
    {
        if ((match1[i].distance <= std::max(2 * min_dist, 30.0)) && (match2[i].distance <= std::max(2 * min_dist, 30.0)) && (std::abs((mvKeys1[match1[i].queryIdx].pt.y) - (mvKeys2[match1[i].trainIdx].pt.y)) < feature_match_error_))
        {
            cv::Point2f t1_l = mvKeys1[match1[i].queryIdx].pt;
            cv::Point2f t1_r = mvKeys2[match1[i].trainIdx].pt;
            cv::Point2f t2_l = mvKeys3[match2[i].trainIdx].pt;
            //std::cout << "-----------------------" << std::endl;
            //std::cout << match1[i].queryIdx << ", " << match2[i].queryIdx << std::endl;
            points_t2_left.push_back(t2_l);
            points_t1_left.push_back(t1_l);
            points_t1_right.push_back(t1_r);
            //std::cout << "good双目匹配:" << t1_l.x << ", " << t1_l.y << "-----" << t1_r.x << ", " << t1_r.y << std::endl;
        }
    }

    std::cout << "跟踪到上一帧" << points_t2_left.size() << "个特征点" << std::endl;
    return points_t2_left.size();
}

int Frontend::LK_Robust_Find_MuliImage_MatchedFeatures(std::vector<cv::Point2f> &points_t2_left,
                                                       std::vector<cv::Point2f> &points_t2_right,
                                                       std::vector<cv::Point2f> &points_t1_left,
                                                       std::vector<cv::Point2f> &points_t1_right)
{
    std::vector<cv::Point2f> points_final;

    // 4次LK光流跟踪
    std::vector<uchar> status1, status2, status3, status4;
    cv::Mat error1, error2, error3, error4;
    //std::cout << "calcOpticalFlowPyrLK" <<  std::endl;
    cv::calcOpticalFlowPyrLK(
        last_frame_->left_img_, last_frame_->right_img_, points_t1_left,
        points_t1_right, status1, error1, cv::Size(21, 21), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        0, 0.001);
    //std::cout << "calcOpticalFlowPyrLK" <<  std::endl;

    // 当前时刻左图与上一时刻右图
    cv::calcOpticalFlowPyrLK(
        last_frame_->right_img_, current_frame_->right_img_, points_t1_right,
        points_t2_right, status2, error2, cv::Size(21, 21), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        0, 0.001);
    //std::cout << "calcOpticalFlowPyrLK" <<  std::endl;

    // 上一时刻右图与上一时刻左图
    cv::calcOpticalFlowPyrLK(
        current_frame_->right_img_, current_frame_->left_img_, points_t2_right,
        points_t2_left, status3, error3, cv::Size(21, 21), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        0, 0.001);
    cv::calcOpticalFlowPyrLK(
        current_frame_->left_img_, last_frame_->left_img_, points_t2_left,
        points_final, status4, error4, cv::Size(21, 21), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        0, 0.001);
    deleteUnmatchFeaturesCircle(points_t1_left, points_t1_right, points_t2_right, points_t2_left, points_final,
                                status1, status2, status3, status4);
    return points_t1_left.size();
}
void Frontend::deleteUnmatchFeaturesCircle(std::vector<cv::Point2f> &points0, std::vector<cv::Point2f> &points1,
                                           std::vector<cv::Point2f> &points2, std::vector<cv::Point2f> &points3,
                                           std::vector<cv::Point2f> &points0_return,
                                           std::vector<uchar> &status0, std::vector<uchar> &status1,
                                           std::vector<uchar> &status2, std::vector<uchar> &status3)
{
    //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
    int indexCorrection = 0;
    for (int i = 0; i < status3.size(); i++)
    {
        cv::Point2f pt0 = points0.at(i - indexCorrection);
        cv::Point2f pt1 = points1.at(i - indexCorrection);
        cv::Point2f pt2 = points2.at(i - indexCorrection);
        cv::Point2f pt3 = points3.at(i - indexCorrection);
        cv::Point2f pt0_r = points0_return.at(i - indexCorrection);

        if ((status3.at(i) == 0) || (pt3.x < 0) || (pt3.y < 0) ||
            (status2.at(i) == 0) || (pt2.x < 0) || (pt2.y < 0) ||
            (status1.at(i) == 0) || (pt1.x < 0) || (pt1.y < 0) ||
            (status0.at(i) == 0) || (pt0.x < 0) || (pt0.y < 0))
        {
            if ((pt0.x < 0) || (pt0.y < 0) || (pt1.x < 0) || (pt1.y < 0) || (pt2.x < 0) || (pt2.y < 0) || (pt3.x < 0) || (pt3.y < 0))
            {
                status3.at(i) = 0;
            }
            points0.erase(points0.begin() + (i - indexCorrection));
            points1.erase(points1.begin() + (i - indexCorrection));
            points2.erase(points2.begin() + (i - indexCorrection));
            points3.erase(points3.begin() + (i - indexCorrection));
            points0_return.erase(points0_return.begin() + (i - indexCorrection));

            indexCorrection++;
        }
    }
}

bool Frontend::Reset()
{
    LOG(INFO) << "跟踪丢失，重新初始化！. ";
    //status_ = FrontendStatus::INITING;
    vo_->Reset();
    current_frame_ = nullptr;
    last_frame_ = nullptr;
    t_[0] = 0;
    t_[1] = 0;
    t_[2] = 0;
    return true;
}
void Frontend::Set_vo(System *vo)
{
    vo_ = vo;
}

} // namespace robust_vslam